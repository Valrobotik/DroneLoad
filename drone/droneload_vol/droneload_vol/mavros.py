"""Interface vers l'autopilote (ArduPilot) via MAVROS : remplace l'ancien DroneController pymavlink.

Aucune méthode ne bloque : les services MAVROS sont appelés en asynchrone et l'état du drone
(mode, armement, distance lidar) est mis à jour par les callbacks. Les missions appellent ces
méthodes à chaque tick et lisent le résultat aux ticks suivants.
"""
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval, SetMode
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

# Messages MAVLink demandés à l'autopilote (id, Hz) : ArduPilot ne les envoie pas tous par défaut
STREAMS = {
    173: 20.0,   # RANGEFINDER (lidar brut) -> /mavros/rangefinder/rangefinder
    1: 1.0,      # SYS_STATUS (batterie) -> /mavros/battery
}

# Masque de PositionTarget : seules les vitesses et la vitesse de lacet sont prises en compte
VELOCITY_ONLY = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                 | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                 | PositionTarget.IGNORE_YAW)


class MavrosDrone:
    def __init__(self, node, origin):
        """origin : (latitude, longitude, altitude) de l'origine EKF fictive. Sans GPS, ArduPilot
        en a besoin pour passer en GUIDED ; elle est envoyée tant que le drone n'est pas armé."""
        self.node = node
        self.log = node.get_logger()
        self.origin = origin
        self.state = State()
        self.altitude = None          # distance lidar (m), None tant qu'aucune mesure n'est arrivée
        self._streams_requested = False

        node.create_subscription(State, '/mavros/state', self._on_state, 10)
        node.create_subscription(Range, '/mavros/rangefinder/rangefinder', self._on_range,
                                 qos_profile_sensor_data)
        self.setpoint_pub = node.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.origin_pub = node.create_publisher(GeoPointStamped, '/mavros/global_position/set_gp_origin', 10)

        self.set_mode_client = node.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = node.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = node.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.interval_client = node.create_client(MessageInterval, '/mavros/set_message_interval')

        node.create_timer(1.0, self._housekeeping)

    # --- État ---
    @property
    def connected(self):
        return self.state.connected

    @property
    def armed(self):
        return self.state.armed

    @property
    def mode(self):
        return self.state.mode

    def get_current_alt_brute(self):
        """Distance lidar (m), 0.0 tant qu'aucune mesure n'est arrivée."""
        return self.altitude if self.altitude is not None else 0.0

    # --- Commandes (non bloquantes) ---
    def set_mode(self, mode):
        self._call(self.set_mode_client, SetMode.Request(custom_mode=mode), f'mode {mode}')

    def arm(self):
        self._call(self.arming_client, CommandBool.Request(value=True), 'armement')

    def disarm(self):
        self._call(self.arming_client, CommandBool.Request(value=False), 'désarmement')

    def takeoff(self, altitude):
        self._call(self.takeoff_client, CommandTOL.Request(altitude=float(altitude)),
                   f'décollage à {altitude} m')

    def land(self):
        self.set_mode('LAND')

    def send_velocity_body(self, vx, vy, vz, yaw_rate):
        """Vitesses dans le repère du drone : vx vers l'avant, vy vers la droite, vz vers le bas
        (négatif pour monter), en m/s ; yaw_rate en rad/s."""
        msg = PositionTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_BODY_NED
        msg.type_mask = VELOCITY_ONLY
        msg.velocity.x, msg.velocity.y, msg.velocity.z = float(vx), float(vy), float(vz)
        msg.yaw_rate = float(yaw_rate)
        self.setpoint_pub.publish(msg)

    # --- Interne ---
    def _on_state(self, msg):
        if msg.connected != self.state.connected:
            self.log.info('Autopilote connecté' if msg.connected else 'Autopilote DÉCONNECTÉ')
            self._streams_requested = False
        self.state = msg

    def _on_range(self, msg):
        self.altitude = msg.range

    def _housekeeping(self):
        """Toutes les secondes : demande les flux MAVLink et envoie l'origine EKF tant que nécessaire."""
        if not self.connected:
            return
        if not self._streams_requested:
            self._streams_requested = all([
                self._call(self.interval_client, MessageInterval.Request(message_id=msg_id, message_rate=rate),
                           f'flux MAVLink {msg_id}')
                for msg_id, rate in STREAMS.items()])
        if not self.armed:
            msg = GeoPointStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.position.latitude, msg.position.longitude, msg.position.altitude = self.origin
            self.origin_pub.publish(msg)

    def _call(self, client, request, label):
        """Appel asynchrone ; l'échec éventuel est journalisé à l'arrivée de la réponse."""
        if not client.service_is_ready():
            self.log.warning(f'MAVROS indisponible ({label})', throttle_duration_sec=5.0)
            return False
        future = client.call_async(request)
        future.add_done_callback(lambda f: self._check(f, label))
        return True

    def _check(self, future, label):
        result = future.result()
        ok = result is not None and getattr(result, 'success', getattr(result, 'mode_sent', False))
        if not ok:
            self.log.warning(f'Refusé par l\'autopilote : {label}')
