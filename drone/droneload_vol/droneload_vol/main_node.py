"""Nœud principal du drone : lit le mode demandé (depuis le dashboard, via mqtt_server) et fait
avancer la mission correspondante d'un pas à chaque tick (20 Hz).

Modes : attente, takeoff, land, epreuve1, epreuve2. À chaque changement de mode, la mission est
réinitialisée : une 2e demande de décollage ou d'atterrissage repart du début.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray

from .mavros import MavrosDrone
from .missions import Atterrissage, Decollage, Epreuve1, run_epreuve2

MODES = ('attente', 'takeoff', 'land', 'epreuve1', 'epreuve2')
DETECTIONS_MAX_AGE = 0.5   # s : au-delà, les détections sont considérées comme perdues

# Dernière valeur conservée pour un nœud qui (re)démarre après la commande
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('takeoff_altitude', 1.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('ekf_origin', [45.7838, 4.8722, 170.0])

        log = self.get_logger()
        self.drone = MavrosDrone(self, tuple(self.get_parameter('ekf_origin').value))
        self.decollage = Decollage(log, self.get_parameter('takeoff_altitude').value)
        self.atterrissage = Atterrissage(log)
        self.epreuve1 = Epreuve1(log)
        self.image_width = self.get_parameter('image_width').value

        self.mode = 'attente'
        self.target_class = None
        self.detections = []
        self.detections_time = 0.0
        self.published_state = None

        self.create_subscription(String, '/mission/mode', self._on_mode, LATCHED)
        self.create_subscription(String, '/mission/target_class', self._on_target_class, LATCHED)
        self.create_subscription(Detection2DArray, '/detections', self._on_detections,
                                 qos_profile_sensor_data)
        self.state_pub = self.create_publisher(String, '/mission/state', LATCHED)
        self.hook_pub = self.create_publisher(Bool, '/hook/cmd', 10)

        self.create_timer(1.0 / self.get_parameter('rate').value, self._tick)
        log.info('Nœud principal prêt, mode attente')

    def _on_mode(self, msg):
        if msg.data not in MODES:
            self.get_logger().warning(f'Mode inconnu ignoré : {msg.data}')
            return
        if msg.data == self.mode:
            return
        self.mode = msg.data
        for mission in (self.decollage, self.atterrissage, self.epreuve1):
            mission.reset()
        self.get_logger().info(f'Mode : {self.mode}')

    def _on_target_class(self, msg):
        self.target_class = msg.data
        self.epreuve1.reset()

    def _on_detections(self, msg):
        self.detections_time = self.get_clock().now().nanoseconds / 1e9
        self.detections = []
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            w, h = det.bbox.size_x, det.bbox.size_y
            self.detections.append({'class': hyp.class_id, 'conf': hyp.score,
                                    'box': (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)})
        if self.mode == 'epreuve1' and self.target_class:
            self.epreuve1.add_frame({d['class'] for d in self.detections}, self.target_class)

    def _tick(self):
        now = self.get_clock().now().nanoseconds / 1e9
        drone = self.drone

        if self.mode == 'takeoff':
            self.decollage.run(drone, now)
            state = self.decollage.state
        elif self.mode == 'land':
            self.atterrissage.run(drone, now)
            state = self.atterrissage.state
        elif self.mode == 'epreuve1':
            if self.target_class and self.epreuve1.run(now):
                self.get_logger().info(f'Cible trouvée : {self.target_class}, ouverture du crochet')
                self.hook_pub.publish(Bool(data=True))
            state = self.epreuve1.state if self.target_class else 'SANS_CIBLE'
        elif self.mode == 'epreuve2':
            fresh = now - self.detections_time < DETECTIONS_MAX_AGE
            run_epreuve2(drone, self.detections if fresh else [], self.image_width)
            state = 'SUIVI'
        else:
            # Attente : vol stationnaire, seulement si le drone vole sous notre contrôle
            if drone.armed and drone.mode == 'GUIDED':
                drone.send_velocity_body(0, 0, 0, 0)
            state = 'ATTENTE'

        full_state = f'{self.mode}:{state}'
        if full_state != self.published_state:
            self.published_state = full_state
            self.state_pub.publish(String(data=full_state))


def main():
    rclpy.init()
    node = MainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
