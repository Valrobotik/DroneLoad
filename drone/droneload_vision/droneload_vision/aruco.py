"""Détection des ArUco et pose 3D : /camera/image_raw + /camera/camera_info -> /aruco/detections.

Reprend la détection de 2026 (drone/vision/aruco.py : DICT_4X4_50, centre en pixels) et ajoute la
pose de chaque marqueur par cv2.solvePnP (remplace aruco.estimatePoseSingleMarkers, obsolète),
avec la calibration publiée sur /camera/camera_info (nœud camera sur la Pi, bridge Gazebo en simu).

Pose dans le repère optique de la caméra (x à droite, y vers le bas, z dans l'axe), en mètres.
Caméra vers le sol : z = distance verticale au marqueur quand le drone est à plat (sinon voir #69).
"""
import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


class ArucoProcessor:
    """Détection des marqueurs (code 2026) et pose 3D de chacun."""

    def __init__(self, dictionary, marker_size):
        self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, dictionary))
        if hasattr(aruco, 'ArucoDetector'):   # OpenCV >= 4.7
            self.parameters = aruco.DetectorParameters()
        else:                                 # OpenCV 4.6 d'Ubuntu 24.04 (Pi et simu)
            self.parameters = aruco.DetectorParameters_create()
        # Sur un sol clair, la marge blanche du marqueur forme un 2e carré à quelques pixels du
        # carré noir ; avec la valeur par défaut (0,05), le détecteur ne garde que le carré blanc
        # et rejette le marqueur. 0,02 garde le carré noir (une seule détection par marqueur).
        self.parameters.minMarkerDistanceRate = 0.02
        self.detector = (aruco.ArucoDetector(self.aruco_dict, self.parameters)
                         if hasattr(aruco, 'ArucoDetector') else None)
        # Coins du marqueur dans son repère, dans l'ordre des coins détectés (requis par IPPE_SQUARE)
        s = marker_size / 2
        self.object_points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)
        self.marker_size = marker_size

    def detect(self, gray):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids

    def process(self, frame, canvas, camera_matrix, dist_coeffs):
        """Analyse 'frame' et dessine sur 'canvas'.

        Renvoie {id: (centre en pixels (x, y), rvec, tvec)} ; rvec et tvec valent None sans calibration.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect(gray)

        detected_arucos = {}

        if ids is not None:
            # Dessine les bordures sur le canvas
            aruco.drawDetectedMarkers(canvas, corners, ids)

            for i, aruco_id in enumerate(ids.flatten()):
                c = corners[i][0]
                center_x = int((c[0][0] + c[2][0]) / 2)
                center_y = int((c[0][1] + c[2][1]) / 2)
                rvec = tvec = None

                if camera_matrix is not None:
                    ok, rvec, tvec = cv2.solvePnP(self.object_points, c, camera_matrix, dist_coeffs,
                                                  flags=cv2.SOLVEPNP_IPPE_SQUARE)
                    if ok:
                        cv2.drawFrameAxes(canvas, camera_matrix, dist_coeffs, rvec, tvec,
                                          self.marker_size / 2)
                        x, y, z = (100 * tvec.flatten()).tolist()
                        cv2.putText(canvas, f'{aruco_id}: {x:.0f} {y:.0f} {z:.0f} cm',
                                    (center_x + 10, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                    (0, 255, 255), 2)
                    else:
                        rvec = tvec = None

                detected_arucos[int(aruco_id)] = ((center_x, center_y), rvec, tvec)

                # Centre en rouge
                cv2.circle(canvas, (center_x, center_y), 5, (0, 0, 255), -1)

        return canvas, detected_arucos


def rotation_to_quaternion(rvec):
    """Vecteur de rotation (Rodrigues) -> quaternion (x, y, z, w)."""
    m, _ = cv2.Rodrigues(rvec)
    w = np.sqrt(max(0.0, 1.0 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    x = np.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2])) / 2
    y = np.sqrt(max(0.0, 1.0 - m[0, 0] + m[1, 1] - m[2, 2])) / 2
    z = np.sqrt(max(0.0, 1.0 - m[0, 0] - m[1, 1] + m[2, 2])) / 2
    x = np.copysign(x, m[2, 1] - m[1, 2])
    y = np.copysign(y, m[0, 2] - m[2, 0])
    z = np.copysign(z, m[1, 0] - m[0, 1])
    return float(x), float(y), float(z), float(w)


class Aruco(Node):
    def __init__(self):
        super().__init__('aruco')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.20)   # m, côté du carré noir
        self.marker_size = self.get_parameter('marker_size').value
        self.processor = ArucoProcessor(self.get_parameter('dictionary').value, self.marker_size)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.create_subscription(CameraInfo, '/camera/camera_info', self._on_camera_info,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/camera/image_raw', self._on_image, qos_profile_sensor_data)
        self.det_pub = self.create_publisher(Detection3DArray, '/aruco/detections', qos_profile_sensor_data)
        self.img_pub = self.create_publisher(Image, '/aruco/annotated', qos_profile_sensor_data)
        self.get_logger().info(f'ArUco prêt (marqueurs de {100 * self.marker_size:.1f} cm)')

    def _on_camera_info(self, msg):
        if self.camera_matrix is None:
            self.get_logger().info(f'Calibration reçue : fx = {msg.k[0]:.1f}, fy = {msg.k[4]:.1f} px')
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def _on_image(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warning('Pas de calibration sur /camera/camera_info : pas de pose',
                                      throttle_duration_sec=5.0)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        canvas, arucos = self.processor.process(frame, frame.copy(), self.camera_matrix, self.dist_coeffs)

        detections = Detection3DArray(header=msg.header)
        for aruco_id, (_, rvec, tvec) in arucos.items():
            if tvec is None:
                continue
            det = Detection3D(header=msg.header)
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(aruco_id)
            hyp.hypothesis.score = 1.0
            pose = hyp.pose.pose
            pose.position.x, pose.position.y, pose.position.z = tvec.flatten().tolist()
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w) = rotation_to_quaternion(rvec)
            det.results.append(hyp)
            det.bbox.center = pose
            det.bbox.size.x = det.bbox.size.y = self.marker_size
            detections.detections.append(det)

        self.det_pub.publish(detections)
        annotated = self.bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        annotated.header = msg.header
        self.img_pub.publish(annotated)


def main():
    rclpy.init()
    node = Aruco()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
