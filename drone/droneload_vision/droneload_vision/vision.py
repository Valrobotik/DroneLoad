"""Détection d'objets (YOLO) : /camera/image_raw -> /detections + /camera/annotated.

Le calcul est confié à un backend choisi par le paramètre `backend` :
- `ultralytics` : modèle .pt ou export NCNN, sur CPU (simulation, Pi sans AI HAT) ;
- `hailo` : AI HAT de la Pi, modèle .hef (pas encore disponible, issue #27).
Le reste du nœud (messages publiés, dessin des boîtes) est commun aux deux.
"""
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

STATS_PERIOD = 10.0   # s entre deux lignes de log sur le temps d'inférence


class UltralyticsBackend:
    def __init__(self, model_path):
        from ultralytics import YOLO
        self.model = YOLO(model_path, task='detect')

    def detect(self, frame, confidence):
        """Liste de (classe, confiance, x1, y1, x2, y2) en pixels."""
        result = self.model(frame, conf=confidence, verbose=False)[0]
        return [(result.names[int(cls)], float(conf), *map(float, box))
                for cls, conf, box in zip(result.boxes.cls.tolist(), result.boxes.conf.tolist(),
                                          result.boxes.xyxy.tolist())]


BACKENDS = {'ultralytics': UltralyticsBackend}


class Vision(Node):
    def __init__(self):
        super().__init__('vision')
        self.declare_parameter('backend', 'ultralytics')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence', 0.5)
        backend = self.get_parameter('backend').value
        model_path = self.get_parameter('model_path').value
        self.confidence = self.get_parameter('confidence').value

        if backend not in BACKENDS:
            raise RuntimeError(f'Backend de vision inconnu ou pas encore disponible : {backend}')
        self.get_logger().info(f'Chargement du modèle {model_path} (backend {backend})...')
        self.backend = BACKENDS[backend](model_path)

        self.bridge = CvBridge()
        self.target_class = None
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/mission/target_class', self._on_target_class, latched)
        self.create_subscription(Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', qos_profile_sensor_data)
        self.img_pub = self.create_publisher(Image, '/camera/annotated', qos_profile_sensor_data)

        self.stats_start = time.monotonic()
        self.stats_frames = 0
        self.stats_time = 0.0
        self.get_logger().info('Vision prête')

    def _on_target_class(self, msg):
        self.target_class = msg.data

    def _on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        start = time.monotonic()
        boxes = self.backend.detect(frame, self.confidence)
        self._stats(time.monotonic() - start)

        detections = Detection2DArray(header=msg.header)
        for name, conf, x1, y1, x2, y2 in boxes:
            det = Detection2D(header=msg.header)
            det.bbox.center.position.x = (x1 + x2) / 2
            det.bbox.center.position.y = (y1 + y2) / 2
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = name
            hyp.hypothesis.score = conf
            det.results.append(hyp)
            detections.detections.append(det)

            # Vert pour la classe cible, rouge pour les autres
            color = (0, 255, 0) if name == self.target_class else (0, 0, 255)
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            cv2.putText(frame, f'{name} {conf:.2f}', (int(x1), max(int(y1) - 10, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self.det_pub.publish(detections)
        annotated = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        annotated.header = msg.header
        self.img_pub.publish(annotated)

    def _stats(self, duration):
        self.stats_frames += 1
        self.stats_time += duration
        elapsed = time.monotonic() - self.stats_start
        if elapsed >= STATS_PERIOD:
            self.get_logger().info(
                f'Inférence : {1000 * self.stats_time / self.stats_frames:.0f} ms par image, '
                f'{self.stats_frames / elapsed:.1f} images/s')
            self.stats_start, self.stats_frames, self.stats_time = time.monotonic(), 0, 0.0


def main():
    rclpy.init()
    node = Vision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
