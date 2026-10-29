"""Régénère les textures ArUco de la simulation (DICT_4X4_50, ids 0 à 15, cadre blanc).

Usage, depuis la racine du repo : python sim/tools/generate_aruco.py
"""
from pathlib import Path

import cv2

OUT_DIR = Path(__file__).resolve().parent.parent / "models" / "aruco_markers" / "materials" / "textures"
MARKER_PX = 200   # marqueur noir
BORDER_PX = 20    # cadre blanc autour, nécessaire à la détection
IDS = range(16)


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for marker_id in IDS:
        img = cv2.aruco.generateImageMarker(dictionary, marker_id, MARKER_PX)
        img = cv2.copyMakeBorder(img, BORDER_PX, BORDER_PX, BORDER_PX, BORDER_PX,
                                 cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(str(OUT_DIR / f"aruco_{marker_id}.png"), img)
    print(f"{len(IDS)} ArUco générés dans {OUT_DIR}")


if __name__ == "__main__":
    main()
