"""Flux vidéo vers le PC de pilotage : /camera/annotated -> GStreamer RTP/JPEG en UDP.

Reçu sur le PC par ground/launch.py (port 5000 par défaut).
"""
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class GStream(Node):
    def __init__(self):
        super().__init__('gstream')
        self.declare_parameter('pc_ip', '127.0.0.1')
        self.declare_parameter('port', 5000)
        self.declare_parameter('fps', 15)
        self.host = self.get_parameter('pc_ip').value
        self.port = self.get_parameter('port').value
        self.fps = self.get_parameter('fps').value

        self.bridge = CvBridge()
        self.writer = None
        self.size = None
        self.create_subscription(Image, '/camera/annotated', self._on_image, qos_profile_sensor_data)
        self.get_logger().info(f'Flux vidéo vers {self.host}:{self.port}')

    def _open(self, size):
        # La taille de l'image n'est connue qu'à la première image reçue
        pipeline = (
            f'appsrc ! videoconvert ! video/x-raw,format=I420 ! '
            f'jpegenc ! rtpjpegpay ! '
            f'udpsink host={self.host} port={self.port} sync=false'
        )
        self.writer = cv2.VideoWriter(pipeline, cv2.CAP_GSTREAMER, 0, self.fps, size, True)
        if not self.writer.isOpened():
            self.get_logger().error('Impossible d\'ouvrir le flux GStreamer de sortie')
        self.size = size

    def _on_image(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        size = (frame.shape[1], frame.shape[0])
        if size != self.size:
            if self.writer is not None:
                self.writer.release()
            self._open(size)
        self.writer.write(frame)

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = GStream()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
