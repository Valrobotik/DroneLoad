"""Caméra Module 3 (vers le sol), Pi seulement : libcamerasrc (GStreamer) -> /camera/image_raw,
avec sa calibration sur /camera/camera_info (fichier config/camera/, format camera_info de ROS).

En simulation, ces topics sont publiés par le bridge Gazebo : ce nœud n'est pas lancé.

Caméra absente ou occupée (nappe, autre programme) : le nœud reste lancé, le signale sur /rosout (donc sur
le dashboard) et réessaie. Ouverture bloquée (OPEN_TIMEOUT) ou plus d'image (FREEZE_TIMEOUT) : le nœud
s'arrête et le launch le relance (OpenCV peut bloquer sans fin, seul un nouveau processus en sort).
"""
import os
import subprocess
import threading
import time

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

RETRY_PERIOD = 5.0     # s entre deux tentatives d'ouverture
ERROR_THROTTLE = 30.0  # s entre deux rappels de la même erreur
FREEZE_TIMEOUT = 3.0   # s sans image avant de relancer le nœud
OPEN_TIMEOUT = 10.0    # s d'ouverture bloquée avant de relancer le nœud


def load_camera_info(path):
    """CameraInfo depuis un fichier de calibration au format camera_info de ROS."""
    with open(path) as f:
        calib = yaml.safe_load(f)
    info = CameraInfo()
    info.width = calib['image_width']
    info.height = calib['image_height']
    info.distortion_model = calib['distortion_model']
    info.k = calib['camera_matrix']['data']
    info.d = calib['distortion_coefficients']['data']
    info.r = calib['rectification_matrix']['data']
    info.p = calib['projection_matrix']['data']
    return info


class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('calibration_file', '')
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.size = f'{width}x{height} à {fps} images/s'
        self.pipeline = (
            f'libcamerasrc ! '
            f'video/x-raw,format=NV12,width={width},height={height},framerate={fps}/1 ! '
            f'videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1'
        )

        calibration_file = self.get_parameter('calibration_file').value
        self.info = load_camera_info(calibration_file)
        if (self.info.width, self.info.height) != (width, height):
            raise RuntimeError(f'Calibration {calibration_file} faite en {self.info.width}x{self.info.height}, '
                               f'caméra réglée en {width}x{height}')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
        self.cap = None
        self.thread = None
        self.running = True
        self.opening_since = None   # ouverture en cours (dans le thread)
        self.last_frame = None
        self._open()
        self.create_timer(RETRY_PERIOD, self._watchdog)

    def _open(self):
        # Avec un élément GStreamer absent, cv2.VideoCapture bloque sans fin (OpenCV 4.6) : vérifié avant
        if subprocess.run(['gst-inspect-1.0', '--exists', 'libcamerasrc']).returncode != 0:
            self.get_logger().error('Caméra INUTILISABLE : élément GStreamer libcamerasrc absent '
                                    '(paquet gstreamer1.0-libcamera, ./install/apply.sh)',
                                    throttle_duration_sec=ERROR_THROTTLE)
            return
        self.opening_since = time.monotonic()
        # Ouverture et lecture bloquantes dans un thread : le rythme est donné par la caméra
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()

    def _watchdog(self):
        now = time.monotonic()
        if self.opening_since is not None:
            if now - self.opening_since > OPEN_TIMEOUT:
                self._restart(f'Ouverture de la caméra bloquée depuis {OPEN_TIMEOUT:.0f} s')
        elif self.cap is None:
            self._open()
        elif now - self.last_frame > FREEZE_TIMEOUT:
            self._restart(f'Plus d\'image caméra depuis {FREEZE_TIMEOUT:.0f} s')

    def _restart(self, reason):
        self.get_logger().error(f'{reason} : nœud relancé')
        # Laisse le message partir sur /rosout ; os._exit car le thread est bloqué dans OpenCV
        threading.Timer(0.5, os._exit, args=(1,)).start()

    def _capture_loop(self):
        cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            cap.release()
            self.get_logger().error('Caméra impossible à ouvrir (libcamerasrc) : nappe, caméra prise par un '
                                    f'autre programme ? Nouvel essai toutes les {RETRY_PERIOD:.0f} s',
                                    throttle_duration_sec=ERROR_THROTTLE)
            self.opening_since = None
            return
        self.last_frame = time.monotonic()
        self.cap = cap
        self.opening_since = None   # après self.cap : le watchdog ne relance pas d'ouverture
        self.get_logger().info(f'Caméra ouverte : {self.size}')
        while self.running and rclpy.ok():
            ok, frame = cap.read()
            if not ok:
                self.get_logger().warning('Image caméra manquée', throttle_duration_sec=5.0)
                time.sleep(0.01)   # read() peut échouer aussitôt en boucle (flux arrêté)
                continue
            self.last_frame = time.monotonic()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.info.header = msg.header
            self.pub.publish(msg)
            self.info_pub.publish(self.info)

    def destroy_node(self):
        self.running = False
        if self.cap is not None:
            self.thread.join(timeout=1.0)
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = Camera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
