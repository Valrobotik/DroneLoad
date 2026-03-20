#!/usr/bin/env python3
import cv2
import cv2.aruco as aruco
import numpy as np
import head as h
import mavlink
import time

# ====== PARAMÈTRES À ADAPTER ======
MARKER_SIZE = 4.3  # taille réelle du marqueur (ex: cm). tvec sera dans la même unité.

cameraMatrix = np.array([
    [800.0,   0.0, 320.0],
    [  0.0, 800.0, 240.0],
    [  0.0,   0.0,   1.0]
], dtype=np.float64)

distCoeffs = np.array([0.1, -0.05, 0.001, 0.0, 0.0], dtype=np.float64)
# ==================================

def draw_bottom_point(frame: np.ndarray, marker_corners: np.ndarray) -> tuple[int, int]:
    """
    marker_corners: shape (4,2) float32/float64
    Retourne le point (x,y) du coin ayant le y le plus grand (le plus bas à l'écran).
    """
    pts = marker_corners.reshape(4, 2)
    idx = int(np.argmax(pts[:, 1]))  # max y
    x, y = pts[idx]
    p = (int(round(x)), int(round(y)))
    cv2.circle(frame, p, 6, (0, 0, 255), thickness=2)
    return p

def put_text(frame: np.ndarray, text: str, org: tuple[int, int]) -> None:
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

def camera_init():

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Impossible d'ouvrir la caméra (VideoCapture(0)).")

    # Optionnel: fixer une résolution (si supportée)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    return cap

#this fonction get the x,y,z values from the camera reference to the drone reference
def reference_rectifier(x, y, z):
    return x + h.DX, y + h.DY, z + h.DZ


def window_center_from_known_markers(ids, tvecs):
    if ids is None:
        return None

    pts = []

    for i in range(len(ids)):
        marker_id = int(ids[i][0])
        if marker_id in h.WINDOW_IDS:
            x, y, z = tvecs[i].reshape(3)
            pts.append([x, y, z])

    if len(pts) != 4:
        return None

    pts = np.array(pts, dtype=np.float64)
    center = np.mean(pts, axis=0)

    return tuple(center)

def win_coordinates(cap):

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
    parameters = aruco.DetectorParameters()

    ret, frame = cap.read()
    if not ret:
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is None or len(ids) < 4:
        cv2.imshow("Detection ArUco", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return None
        return None

    aruco.drawDetectedMarkers(frame, corners, ids)

    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
        corners, MARKER_SIZE, cameraMatrix, distCoeffs
    )

    center = window_center_from_known_markers(ids, tvecs)
    if center is None:
        cv2.imshow("Detection ArUco", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            return None
        return None

    x, y, z = center

    put_text(frame, f"Window center: x={x:.2f} y={y:.2f} z={z:.2f}", (20, 30))
    cv2.imshow("Detection ArUco", frame)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        return None

    return (x, y, z)

def correct(xyz0: list, xyzr: list) -> list:
    err = [0 , 0, 0]
    for i , el in enumerate(xyzr):
        err[i] = el - xyz0[i]

    return err


if __name__ == "__main__":
    #need to test the return value None if the initialisation fail
    cap = camera_init()
    master, target_position, target_component = mavlink.init_mavlink()
    while True:
        #this try test is for when we get None from the fonction That means that we've couldn't get a frame or the q button on the
        #keyboard was pressed
        try:
            x, y , z = reference_rectifier(*win_coordinates(cap))
            x_r, y_r, z_r = mavlink.drone_position(master)
            u = correct([x, y, z], [x_r, y_r, z_r])
            mavlink.move(master, target_position, target_component,u, h.SPEED, h.ACC)
            time.sleep(1)

        except:
            break

    cap.release()
    cv2.destroyAllWindows()