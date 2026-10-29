import time
import cv2
import numpy as np
import glob
import os
from typing import Optional


def calibrate_camera(
        images_folder: str,
        chessboard_size: tuple = (9, 6),
        square_size_mm: float = 27.0
) -> Optional[np.ndarray]:
    """
    Calibre la caméra à partir de photos d'un damier.
    Retourne la cameraMatrix (3x3) ou None si calibration impossible.
    """
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[
        0:chessboard_size[0],
        0:chessboard_size[1]
    ].T.reshape(-1, 2)
    objp *= square_size_mm

    obj_points = []
    img_points = []

    images  = glob.glob(os.path.join(images_folder, "*.jpg"))
    images += glob.glob(os.path.join(images_folder, "*.png"))

    if len(images) == 0:
        print(f"Aucune image trouvée dans : {images_folder}")
        return None

    print(f"{len(images)} images trouvées — recherche du damier...")

    img_shape    = None
    success_count = 0

    for path in images:
        img  = cv2.imread(path)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_shape = gray.shape[::-1]

        found, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(objp)
            img_points.append(corners_refined)
            success_count += 1
            print(f"  OK : {os.path.basename(path)}")
        else:
            print(f"  ECHEC : {os.path.basename(path)}")

    print(f"\n{success_count}/{len(images)} images utilisées")

    if success_count < 10:
        print("Pas assez d'images valides (minimum 10)")
        return None

    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, img_shape, None, None
    )

    print(f"Erreur de reprojection : {ret:.4f} px")
    if ret > 1.0:
        print("Erreur elevee — reprendre les photos")

    print("\ncameraMatrix :")
    print(camera_matrix)

    return camera_matrix


def take_photos(cap: cv2.VideoCapture,
                folder_name: str = "photos",
                n_photos: int = 80,
                delay: float = 1.0) -> str:
    """
    Prend n_photos photos et les enregistre dans un dossier.
    """
    os.makedirs(folder_name, exist_ok=True)
    print(f"Dossier : {os.path.abspath(folder_name)}")

    count = 0
    while count < n_photos:
        ret, frame = cap.read()
        if not ret:
            print(f"Erreur frame {count + 1} — retry")
            time.sleep(0.05)
            continue

        filename = os.path.join(folder_name, f"photo_{count + 1:03d}.jpg")
        cv2.imwrite(filename, frame)
        print(f"[{count + 1}/{n_photos}] {filename}")
        count += 1
        time.sleep(delay)

    print(f"\n{n_photos} photos dans : {os.path.abspath(folder_name)}")
    return os.path.abspath(folder_name)


if __name__ == "__main__":

    # ── Initialisation caméra ──────────────────────────────────────────────
    width = 640
    height = 480

        # --- 1. ENTRÉE VIDÉO ---
    pipeline_in = (
        f"libcamerasrc ! "
        f"video/x-raw,format=NV12,width={width},height={height},framerate=15/1 ! "
        f"videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=1"
    )

    print("Initialisation de la caméra...")
    cap = cv2.VideoCapture(pipeline_in, cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        raise RuntimeError("Impossible d'ouvrir la caméra")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # ── Prise de photos ────────────────────────────────────────────────────
    take_photos(cap, folder_name="photos", n_photos=80, delay=1.0)
    cap.release()

    # ── Attente avant calibration ──────────────────────────────────────────
    print("\nAttente 5 secondes avant calibration...")
    time.sleep(5)

    # ── Calibration ────────────────────────────────────────────────────────
    camera_matrix = calibrate_camera(
        images_folder="photos",
        chessboard_size=(9, 6),
        square_size_mm=25.0
    )

    if camera_matrix is not None:
        print("\nCalibration OK !")
        print(camera_matrix)
