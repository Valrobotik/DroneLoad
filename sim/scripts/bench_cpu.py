"""Benchmark CPU pour régler PI_CPUS (compose.pi.yml) : même charge sur la Pi et dans la simu.

Charge par image (640×480, comme la caméra) : conversion en gris, flou, détection ArUco, redimensionnement.
Deux mesures :
  - 1 cœur  : vitesse d'un cœur (ne se règle pas avec PI_CPUS, donnée à titre indicatif) ;
  - N cœurs : débit total avec N processus en parallèle (N = 4, les cœurs de la Pi), c'est lui qu'on aligne.

Usage :
  sur la Pi                    : python3 sim/scripts/bench_cpu.py
  dans la simu (depuis sim/)   : docker compose exec companion bash -lc 'python3 sim/scripts/bench_cpu.py'
  comparer à la Pi             : ... bench_cpu.py --ref <images/s N cœurs mesurées sur la Pi>
"""
import argparse
import multiprocessing
import os
import time

import cv2
import numpy as np

WIDTH, HEIGHT = 640, 480    # = drone/config.py
PI_CORES = 4
DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)


# API ArUco : OpenCV ≥ 4.7 (ArucoDetector) ou 4.6 d'Ubuntu 24.04 (fonctions)
if hasattr(cv2.aruco, "ArucoDetector"):
    draw_marker = cv2.aruco.generateImageMarker

    def make_detector():
        detector = cv2.aruco.ArucoDetector(DICT, cv2.aruco.DetectorParameters())
        return detector.detectMarkers
else:
    draw_marker = cv2.aruco.drawMarker

    def make_detector():
        params = cv2.aruco.DetectorParameters_create()
        return lambda gray: cv2.aruco.detectMarkers(gray, DICT, parameters=params)


def make_frame():
    """Image de test : bruit + 4 ArUco, pour que la détection ait du travail réel."""
    rng = np.random.default_rng(0)
    frame = rng.integers(60, 200, (HEIGHT, WIDTH, 3), dtype=np.uint8)
    for i, (x, y) in enumerate([(40, 40), (360, 60), (80, 280), (420, 300)]):
        marker = draw_marker(DICT, i, 120)
        frame[y:y + 120, x:x + 120] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    return frame


def process(frame, detect):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, ids, _ = detect(gray)
    cv2.resize(frame, (320, 240))
    return 0 if ids is None else len(ids)


def run(duration):
    """Traite des images pendant `duration` s sur un seul thread ; renvoie le nombre d'images."""
    cv2.setNumThreads(1)
    frame = make_frame()
    detect = make_detector()
    assert process(frame, detect) == 4, "ArUco de test non détectés"
    count = 0
    end = time.perf_counter() + duration
    while time.perf_counter() < end:
        process(frame, detect)
        count += 1
    return count


def cpu_limit():
    """Quota CPU du cgroup (conteneur limité par compose.pi.yml), sinon None."""
    try:
        quota, period = open("/sys/fs/cgroup/cpu.max").read().split()
        return None if quota == "max" else int(quota) / int(period)
    except (OSError, ValueError):
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=10.0, help="durée de chaque mesure (s)")
    parser.add_argument("--ref", type=float, help="images/s sur %d cœurs mesurées sur la Pi" % PI_CORES)
    args = parser.parse_args()

    limit = cpu_limit()
    print(f"CPU visibles : {os.cpu_count()}, limite cgroup : {limit if limit else 'aucune'}")

    single = run(args.duration) / args.duration
    print(f"1 cœur          : {single:7.1f} images/s ({1000 / single:.2f} ms/image)")

    with multiprocessing.Pool(PI_CORES) as pool:
        multi = sum(pool.map(run, [args.duration] * PI_CORES)) / args.duration
    print(f"{PI_CORES} cœurs        : {multi:7.1f} images/s")

    if args.ref:
        ratio = multi / args.ref
        print(f"Rapport simu / Pi : {ratio:.2f}")
        if limit:
            print(f"PI_CPUS conseillé : {limit / ratio:.2f} (actuel {limit:g})")
        else:
            print("Relancer avec compose.pi.yml pour obtenir un PI_CPUS conseillé.")


if __name__ == "__main__":
    main()
