"""Interface vers l'autopilote (ArduPilot) via MAVROS : remplace l'ancien DroneController pymavlink.

GUIDED ne s'obtient que par la radio (FLTMODE_GCSBLOCK de pixhawk.param) : la Pi n'y passe jamais
elle-même. Elle ne passe en LAND que depuis GUIDED (land()).

Aucune méthode ne bloque : les services MAVROS sont appelés en asynchrone et l'état du drone
(mode, armement, distance lidar) est mis à jour par les callbacks. Les missions appellent ces
méthodes à chaque tick et lisent le résultat aux ticks suivants.
"""
import math
import time

from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from mavros_msgs.msg import GPSRAW, PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, MessageInterval, SetMode
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

# Messages MAVLink demandés à l'autopilote (id, Hz) : ArduPilot ne les envoie pas tous par défaut
STREAMS = {
    173: 20.0,   # RANGEFINDER (lidar brut) -> /mavros/rangefinder/rangefinder
    1: 1.0,      # SYS_STATUS (batterie) -> /mavros/battery
    24: 1.0,     # GPS_RAW_INT (fix GPS ; jamais envoyé sans GPS configuré) -> /mavros/gpsstatus/gps1/raw
    32: 10.0,    # LOCAL_POSITION_NED (position EKF) -> /mavros/local_position/pose (get_position, carte, dashboard)
    31: 20.0,    # ATTITUDE_QUATERNION -> orientation de /mavros/local_position/pose et /mavros/imu/data
}
# Sans GPS_RAW_INT ce délai (s) après la demande des flux, l'autopilote n'a pas de GPS (config intérieure)
GPS_TIMEOUT = 3.0
# Sans mesure lidar depuis ce délai (s), les flux sont redemandés : un autopilote qui redémarre
# (SITL relancé, Pixhawk rebranchée) les oublie, parfois sans que MAVROS voie de déconnexion
STREAM_TIMEOUT = 1.0
# Âge maximal (s) d'une mesure lidar : au-delà, la hauteur est inconnue (None), jamais une valeur figée.
# Le lidar arrive à 20 Hz : 0,5 s = une dizaine de mesures manquées.
ALT_MAX_AGE = 0.5
# Plage valide du lidar (m), alignée sur RNGFND1_MIN / RNGFND1_MAX de config/ardupilot/pixhawk.param :
# hors plage (inf, nan, 0 quand le MTF-01 perd le sol), la hauteur est inconnue
LIDAR_MIN = 0.01
LIDAR_MAX = 8.0

# Position de l'EKF (/mavros/local_position/pose, 10 Hz) : au-delà de cet âge (s), position inconnue
POSE_MAX_AGE = 0.5
# GPS (ou Marvelmind branché à sa place sur GPS1) : position fiable avec un fix 3D reçu depuis moins de
# GPS_MAX_AGE s (GPS_RAW_INT demandé à 1 Hz)
GPS_FIX_MIN = 3
GPS_MAX_AGE = 3.0
# Source de position horizontale de l'EKF (EK3_SRC1_POSXY, jeu de sources 1) -> source surveillée
SOURCES_EKF = {6: 'uwb', 3: 'gps'}   # 6 ExternalNav (UWB, position_ext), 3 GPS ; autre (0) : flux seul

# Masque de PositionTarget : seules les vitesses et la vitesse de lacet sont prises en compte
VELOCITY_ONLY = (PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ
                 | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                 | PositionTarget.IGNORE_YAW)

# Numéros des modes d'ArduCopter (custom_mode de HEARTBEAT) -> noms de MAVROS
COPTER_MODES = {'0': 'STABILIZE', '2': 'ALT_HOLD', '4': 'GUIDED', '5': 'LOITER', '9': 'LAND',
                '16': 'POSHOLD', '22': 'FLOWHOLD'}


class MavrosDrone:
    def __init__(self, node, origin, delai_source=1.0, delai_recalage=3.0):
        """delai_source : âge maximal (s) de la position UWB envoyée à la Pixhawk (position_ext) pour que
        la position soit jugée fiable (get_position). delai_recalage : après le retour de la source, temps (s)
        laissé à l'EKF pour se recaler avant de juger la position de nouveau fiable.

        origin : (latitude, longitude, altitude) de l'origine EKF fictive. Sans GPS, ArduPilot
        en a besoin pour passer en GUIDED ; elle est envoyée tant que le drone n'est pas armé, et
        seulement si l'autopilote n'a aucun GPS configuré (GPS1_TYPE 0, config intérieure : pas de
        GPS_RAW_INT) : avec un GPS (config extérieure), même sans fix, l'origine vient
        du GPS et une origine fictive la fausserait."""
        self.node = node
        self.log = node.get_logger()
        self.origin = origin
        self.state = State()
        self.altitude = None          # distance lidar (m), None sans mesure valide
        self.gps = None               # GPS configuré sur l'autopilote ? None : pas encore su
        self._streams_requested = False
        self._range_time = 0.0          # time.monotonic() de la dernière mesure lidar
        self._streams_time = 0.0        # time.monotonic() de la dernière demande des flux
        self.delai_source = delai_source
        self.delai_recalage = delai_recalage
        self._source_perdue = False     # source absolue perdue pendant ce vol, pas encore recalée
        self._source_retour = None      # time.monotonic() du retour de la source (recalage en cours)
        self.connexions = 0             # (re)connexions de l'autopilote depuis le lancement (redémarrage possible)
        self._pose = None               # dernière /mavros/local_position/pose (PoseStamped)
        self._pose_time = 0.0
        self._uwb_time = 0.0            # dernière position UWB envoyée à la Pixhawk par position_ext
        self._gps_fix = 0
        self._gps_time = 0.0
        self.source = None              # source de position absolue figée à l'armement : uwb, gps, flux
        self._posxy = None              # EK3_SRC1_POSXY lu par MAVROS, None tant qu'inconnu
        self._posxy_demande = False

        node.create_subscription(State, '/mavros/state', self._on_state, 10)
        node.create_subscription(Range, '/mavros/rangefinder/rangefinder', self._on_range,
                                 qos_profile_sensor_data)
        node.create_subscription(GPSRAW, '/mavros/gpsstatus/gps1/raw', self._on_gps, qos_profile_sensor_data)
        node.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_pose, qos_profile_sensor_data)
        node.create_subscription(PoseWithCovarianceStamped, '/mavros/vision_pose/pose_cov', self._on_uwb, 10)
        self.setpoint_pub = node.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', 10)
        self.origin_pub = node.create_publisher(GeoPointStamped, '/mavros/global_position/set_gp_origin', 10)

        self.set_mode_client = node.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = node.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = node.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.interval_client = node.create_client(MessageInterval, '/mavros/set_message_interval')
        self.param_client = node.create_client(GetParameters, '/mavros/param/get_parameters')

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
        """Mode de l'autopilote, avec le nom ArduCopter même quand MAVROS ne l'a pas : il écrit CMODE(<n>)
        tant qu'il ne connaît pas le type d'autopilote (~10 s après une (re)connexion) et toujours pour
        les modes qu'il ignore (FlowHold = CMODE(22)). Sans ce nom, GUIDED passerait pour un autre mode
        pendant ces 10 s (LAND refusé, décollage en attente)."""
        mode = self.state.mode
        if mode.startswith('CMODE(') and mode.endswith(')'):
            return COPTER_MODES.get(mode[6:-1], mode)
        return mode

    def get_current_alt_brute(self):
        """Distance lidar (m), ou None sans mesure valide récente (moins de ALT_MAX_AGE).

        None veut dire « hauteur inconnue » : la mission attend ou s'arrête, elle ne devine pas.
        """
        if self.altitude is None or time.monotonic() - self._range_time > ALT_MAX_AGE:
            self.log.warning('Hauteur lidar indisponible (pas de mesure valide récente)',
                             throttle_duration_sec=1.0)
            return None
        return self.altitude

    def get_pose(self, age_max=POSE_MAX_AGE):
        """((x, y, z), (qx, qy, qz, qw)) de l'EKF (repère map, ENU), ou None si plus vieille que age_max.
        Ne juge pas la fiabilité (voir get_position)."""
        if self._pose is None or time.monotonic() - self._pose_time > age_max:
            return None
        p, o = self._pose.pose.position, self._pose.pose.orientation
        return (p.x, p.y, p.z), (o.x, o.y, o.z, o.w)

    @property
    def source_ekf(self):
        """Source de position de l'EKF d'après EK3_SRC1_POSXY : uwb, gps, flux ; None tant que pas lu."""
        return None if self._posxy is None else SOURCES_EKF.get(self._posxy, 'flux')

    def get_position(self):
        """Position (x, y) de l'EKF en m (repère map de MAVROS, ENU), ou None si elle n'est pas fiable :
        position EKF absente depuis POSE_MAX_AGE, ou source absolue présente à l'armement (UWB, GPS)
        qui s'est tue, ou revenue depuis moins de delai_recalage (l'EKF se recale). Sans source absolue
        à l'armement (flux optique seul), la position est relative au départ et jugée fiable.

        None veut dire « position inconnue » : la mission attend ou s'arrête, elle ne devine pas.
        """
        now = time.monotonic()
        if self._pose is None or now - self._pose_time > POSE_MAX_AGE:
            return None
        if self.source is not None:
            if not self._source_ok(self.source, now):
                self._source_perdue, self._source_retour = True, None
                return None
            if self._source_perdue:
                if self._source_retour is None:
                    self._source_retour = now
                    self.log.info(f'Source de position ({self.source}) revenue : {self.delai_recalage:.0f} s '
                                  'de recalage de l\'EKF avant de la juger fiable')
                if now - self._source_retour < self.delai_recalage:
                    return None
                self._source_perdue, self._source_retour = False, None
        p = self._pose.pose.position
        return p.x, p.y

    def _source_actuelle(self, now):
        """Source de position utilisée par l'EKF : d'après EK3_SRC1_POSXY (l'UWB peut être envoyé sans que
        l'EKF s'en serve, VISO_TYPE 0 sans uwb.param). Paramètre pas encore lu : d'après ce qui arrive."""
        if self._posxy is not None:
            return SOURCES_EKF.get(self._posxy, 'flux')
        self.log.warning('Source de position de l\'EKF inconnue (EK3_SRC1_POSXY pas encore lu) : '
                         'déduite des mesures reçues')
        if self._source_ok('uwb', now):
            return 'uwb'
        if self._source_ok('gps', now):
            return 'gps'
        return 'flux'

    def _source_ok(self, source, now):
        if source == 'uwb':
            return now - self._uwb_time <= self.delai_source
        if source == 'gps':
            return self._gps_fix >= GPS_FIX_MIN and now - self._gps_time <= GPS_MAX_AGE
        return True

    # --- Commandes (non bloquantes) ---
    # Chaque commande renvoie True si la requête est partie (MAVROS prêt) ; la réponse de l'autopilote
    # arrive plus tard (refus journalisé par _check)
    def set_mode(self, mode):
        return self._call(self.set_mode_client, SetMode.Request(custom_mode=mode), f'mode {mode}')

    def arm(self):
        return self._call(self.arming_client, CommandBool.Request(value=True), 'armement')

    def disarm(self):
        return self._call(self.arming_client, CommandBool.Request(value=False), 'désarmement')

    def takeoff(self, altitude):
        return self._call(self.takeoff_client, CommandTOL.Request(altitude=float(altitude)),
                   f'décollage à {altitude} m')

    def land(self):
        """Mode LAND, seulement depuis GUIDED : dans un autre mode, le pilote a la main à la radio et la Pi
        ne la lui reprend pas. GUIDED lui-même ne s'obtient que par la radio (FLTMODE_GCSBLOCK)."""
        if self.mode != 'GUIDED':
            self.log.warning(f'LAND refusé : autopilote en {self.mode or "?"}, pas en GUIDED (pilote aux commandes)',
                             throttle_duration_sec=5.0)
            return False
        return self.set_mode('LAND')

    @property
    def guided(self):
        """Drone armé en GUIDED : seul cas où les consignes de vitesse sont envoyées."""
        return self.armed and self.mode == 'GUIDED'

    def hover(self):
        """Vol stationnaire (vitesse nulle), seulement si le drone vole sous notre contrôle."""
        self.send_velocity_body(0, 0, 0, 0)

    def send_velocity_local(self, vx, vy, vz):
        """Vitesses dans le repère de l'EKF (map, ENU pour MAVROS : x, y, z vers le haut), en m/s ;
        cap inchangé. Rien n'est envoyé hors GUIDED armé (pilote aux commandes, drone posé)."""
        if not self.guided:
            return
        msg = PositionTarget()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = VELOCITY_ONLY | PositionTarget.IGNORE_YAW_RATE
        msg.velocity.x, msg.velocity.y, msg.velocity.z = float(vx), float(vy), float(vz)
        self.setpoint_pub.publish(msg)

    def send_velocity_body(self, vx, vy, vz, yaw_rate):
        """Vitesses dans le repère du drone : vx vers l'avant, vy vers la droite, vz vers le bas
        (négatif pour monter), en m/s ; yaw_rate en rad/s. Rien n'est envoyé hors GUIDED armé."""
        if not self.guided:
            return
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
            self.gps = None
            self._posxy = None   # autopilote (re)connecté : sa config a pu changer
            if msg.connected:
                self.connexions += 1
        if msg.armed and not self.state.armed:
            # Source de position figée pour tout le vol : sa perte rend la position non fiable
            self.source = self._source_actuelle(time.monotonic())
            self._source_perdue, self._source_retour = False, None
            noms = {'uwb': 'UWB', 'gps': 'GPS', 'flux': 'flux optique seul (relative au départ)'}
            self.log.info(f'Position : {noms[self.source]}')
        elif not msg.armed:
            self.source = None
        self.state = msg

    def _on_pose(self, msg):
        self._pose = msg
        self._pose_time = time.monotonic()

    def _on_uwb(self, msg):
        self._uwb_time = time.monotonic()

    def _on_range(self, msg):
        # L'heure est mise à jour même pour une mesure invalide : le flux arrive, inutile de le redemander
        self._range_time = time.monotonic()
        if math.isfinite(msg.range) and LIDAR_MIN <= msg.range <= LIDAR_MAX:
            self.altitude = msg.range
        else:
            self.altitude = None
            self.log.warning(f'Mesure lidar hors plage ignorée : {msg.range}', throttle_duration_sec=1.0)

    def _on_gps(self, msg):
        # ArduPilot n'envoie GPS_RAW_INT qu'avec un GPS configuré : tout message veut dire « GPS présent »,
        # même fix_type 0 (envoyé au démarrage, avant que le GPS soit détecté)
        self._gps_fix = msg.fix_type
        self._gps_time = time.monotonic()
        if self.gps is not True:
            self.log.info(f'GPS présent (fix_type {msg.fix_type}) : pas d\'origine EKF fictive')
        self.gps = True

    def _housekeeping(self):
        """Toutes les secondes : demande les flux MAVLink et envoie l'origine EKF tant que nécessaire."""
        if not self.connected:
            return
        now = time.monotonic()
        if self._streams_requested and now - max(self._range_time, self._streams_time) > STREAM_TIMEOUT:
            self.log.warning('Plus de mesure lidar : flux MAVLink redemandés', throttle_duration_sec=10.0)
            self._streams_requested = False
            self.gps = None   # autopilote redémarré : sa config a pu changer
        if not self._streams_requested:
            self._streams_time = now   # laisse STREAM_TIMEOUT au flux pour arriver
            self._streams_requested = all([
                self._call(self.interval_client, MessageInterval.Request(message_id=msg_id, message_rate=rate),
                           f'flux MAVLink {msg_id}')
                for msg_id, rate in STREAMS.items()])
        # Origine fictive seulement si l'autopilote n'a aucun GPS : pas de GPS_RAW_INT GPS_TIMEOUT après
        # la demande des flux (ArduPilot ne l'envoie pas sans GPS configuré)
        if self.gps is None and self._streams_requested and now - self._streams_time > GPS_TIMEOUT:
            self.log.info('Pas de GPS_RAW_INT : autopilote sans GPS, origine EKF fictive')
            self.gps = False
        if self._posxy is None and not self._posxy_demande and self.param_client.service_is_ready():
            # Paramètres lus par MAVROS après la connexion : redemandé chaque seconde jusqu'à la réponse
            self._posxy_demande = True
            future = self.param_client.call_async(GetParameters.Request(names=['EK3_SRC1_POSXY']))
            future.add_done_callback(self._on_posxy)
        if not self.armed and self.gps is False:
            msg = GeoPointStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.position.latitude, msg.position.longitude, msg.position.altitude = self.origin
            self.origin_pub.publish(msg)

    def _on_posxy(self, future):
        self._posxy_demande = False
        result = future.result()
        if result is None or not result.values or result.values[0].type != ParameterType.PARAMETER_INTEGER:
            return
        self._posxy = result.values[0].integer_value
        self.log.info(f'Source de position de l\'EKF : EK3_SRC1_POSXY {self._posxy} '
                      f'({SOURCES_EKF.get(self._posxy, "flux optique seul")})')

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
