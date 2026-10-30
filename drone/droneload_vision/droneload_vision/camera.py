"""Caméra Module 3 (vers le sol), Pi seulement : libcamerasrc (GStreamer) -> /camera/image_raw,
avec sa calibration sur /camera/camera_info (fichier config/camera/, format camera_info de ROS).

En simulation, ces topics sont publiés par le bridge Gazebo : ce nœud n'est pas lancé.
"""
import threading

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


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

        pipeline = (
            f'libcamerasrc ! '
            f'video/x-raw,format=NV12,width={width},height={height},framerate={fps}/1 ! '
            f'videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1'
        )
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError('Impossible d\'ouvrir la caméra via GStreamer (libcamerasrc)')

        calibration_file = self.get_parameter('calibration_file').value
        self.info = load_camera_info(calibration_file)
        if (self.info.width, self.info.height) != (width, height):
            raise RuntimeError(f'Calibration {calibration_file} faite en {self.info.width}x{self.info.height}, '
                               f'caméra réglée en {width}x{height}')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', qos_profile_sensor_data)
        self.running = True
        # Lecture bloquante dans un thread : le rythme est donné par la caméra
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f'Caméra ouverte : {width}x{height} à {fps} images/s')

    def _capture_loop(self):
        while self.running and rclpy.ok():
            ok, frame = self.cap.read()
            if not ok:
                self.get_logger().warning('Image caméra manquée', throttle_duration_sec=5.0)
                continue
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            self.info.header = msg.header
            self.pub.publish(msg)
            self.info_pub.publish(self.info)

    def destroy_node(self):
        self.running = False
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
