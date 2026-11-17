"""Tests des changements de repère de la carte (droneload_vol/reperes.py), sans ROS.

Lancement : python3 tests/run.py unitaires (ou python3 -m pytest tests/unitaires, ou python3 <ce fichier>)
Géométrie de la fenêtre : celle de la volière 2027 simulée (modèle fenetre, ArUco 4 à 9).
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'drone' / 'src' / 'droneload_vol'))   # paquet droneload_vol
from droneload_vol.reperes import (CAMERA_SOL_ROTATION, Camera, Fenetre, cap_quaternion, direction_map,  # noqa: E402
                                   pixel_sol, point_map, quaternion_cap, rotate)

CAMERA_SOL = Camera((0.05, 0.0, -0.04), CAMERA_SOL_ROTATION)
K = (938.0, 0.0, 320.0, 0.0, 938.0, 240.0, 0.0, 0.0, 1.0)
FENETRE = Fenetre.depuis_parametres(['4 recto 0.43 -0.68', '5 recto 0.43 0.68', '6 recto -0.43 0.68',
                                     '7 recto -0.43 -0.68', '8 verso 0.1667 -0.68', '9 verso 0.1667 0.68'])


def proche(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def test_rotate_cap():
    q = quaternion_cap(math.pi / 2)
    assert proche(rotate(q, (1, 0, 0)), (0, 1, 0))
    assert abs(cap_quaternion(q) - math.pi / 2) < 1e-9


def test_camera_sol_drone_a_plat():
    # Drone en (1, 2, 1), cap nul ; marqueur 0,1 m vers le haut de l'image (y_opt = -0,1) à 0,96 m de la caméra
    pose = ((1.0, 2.0, 1.0), (0.0, 0.0, 0.0, 1.0))
    assert proche(point_map(pose, CAMERA_SOL, (0.0, -0.1, 0.96)), (1.15, 2.0, 0.0))
    # Droite de l'image = droite du drone (y négatif)
    assert proche(point_map(pose, CAMERA_SOL, (0.1, 0.0, 0.96)), (1.05, 1.9, 0.0))


def test_camera_sol_cap_et_tangage():
    # Cap de 90° : l'avant du drone est vers +y
    pose = ((0.0, 0.0, 1.0), quaternion_cap(math.pi / 2))
    assert proche(point_map(pose, CAMERA_SOL, (0.0, -0.1, 0.96)), (0.0, 0.15, 0.0))
    # Rotation de +10° autour de y (FLU) = nez vers le bas : l'axe de la caméra sol part vers l'arrière
    a = math.radians(10)
    pose = ((0.0, 0.0, 1.0), (0.0, math.sin(a / 2), 0.0, math.cos(a / 2)))
    d = direction_map(pose, CAMERA_SOL, (0.0, 0.0, 1.0))
    assert d[0] < 0 and d[2] < 0


def test_pixel_sol():
    pose = ((0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0))
    # Centre de l'image : sous la caméra (0,05 m devant le centre du drone)
    assert proche(pixel_sol(pose, CAMERA_SOL, K, 320, 240, 1.0), (0.05, 0.0, 0.0))
    # 93,8 px vers le haut de l'image = 0,1 rad vers l'avant, à 0,96 m de haut pour la caméra
    p = pixel_sol(pose, CAMERA_SOL, K, 320, 240 - 93.8, 1.0)
    assert proche(p, (0.05 + 0.096, 0.0, 0.0), 1e-3)


def test_fenetre_recto():
    # Fenêtre 2027 : modèle en (-1,68 ; 0,11), ouverture centrée à 1,30 m, recto vers -x, ArUco à 2,6 cm du plan
    c, cap = FENETRE.depuis_aruco(4, (-1.706, 0.11 - 0.43, 0.62), (-1.0, 0.0, 0.0))
    assert proche(c, (-1.706, 0.11, 1.30))
    assert abs(abs(cap) - math.pi) < 1e-9
    c, _ = FENETRE.depuis_aruco(6, (-1.706, 0.11 + 0.43, 1.98), (-1.0, 0.0, 0.0))
    assert proche(c, (-1.706, 0.11, 1.30))


def test_fenetre_verso():
    c, cap = FENETRE.depuis_aruco(8, (-1.654, 0.11 - 0.1667, 0.62), (1.0, 0.0, 0.0))
    assert proche(c, (-1.654, 0.11, 1.30))
    assert abs(abs(cap) - math.pi) < 1e-9   # cap = côté recto, même vu de derrière


def test_fenetre_tournee():
    # Fenêtre tournée : recto vers +y (cap 90°) ; vu du recto (on regarde vers -y), la droite est -x :
    # u = -n x z = (-1, 0), ArUco 4 (u = +0,43) en x = 2 - 0,43
    c, cap = FENETRE.depuis_aruco(4, (1.57, 1.0, 0.62), (0.0, 1.0, 0.0))
    assert proche(c, (2.0, 1.0, 1.30))
    assert abs(cap - math.pi / 2) < 1e-9


def test_fenetre_aruco_pas_vertical():
    assert FENETRE.depuis_aruco(4, (0.0, 0.0, 0.0), (0.0, 0.0, 1.0)) is None


if __name__ == '__main__':
    for nom, f in list(globals().items()):
        if nom.startswith('test_'):
            f()
            print('OK', nom)
