import cv2
import cv2.aruco as aruco


class ArucoProcessor:
    def __init__(self):
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.parameters)

    def process(self, frame, canvas):
        """
        Analyse 'frame' mais dessine sur 'canvas'.
        Si canvas est noir, on n'envoie que les traits sur le réseau.
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detector.detectMarkers(gray)

        detected_arucos = {}

        if ids is not None:
            # Dessine les bordures sur le canvas
            aruco.drawDetectedMarkers(canvas, corners, ids)

            for i, aruco_id in enumerate(ids.flatten()):
                c = corners[i][0]
                center_x = int((c[0][0] + c[2][0]) / 2)
                center_y = int((c[0][1] + c[2][1]) / 2)
                detected_arucos[aruco_id] = (center_x, center_y)

                # Centre en rouge
                cv2.circle(canvas, (center_x, center_y), 5, (0, 0, 255), -1)

        return canvas, detected_arucos