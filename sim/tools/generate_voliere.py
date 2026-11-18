"""Génère le monde Gazebo de la volière (sim/worlds/voliere.sdf), terrain du règlement 2027.

Usage, depuis la racine du repo : python3 sim/tools/generate_voliere.py
Ne pas modifier voliere.sdf à la main : changer les cotes ci-dessous et régénérer.

Disposition relevée sur la figure 1 du règlement 2027 (V1), qui n'est pas cotée : seules les tailles
du terrain (8 x 5 m), de la fenêtre (ouverture 1 x 1 m), des ArUco (20 cm) et des affiches (A3)
sont données ; le reste est mesuré sur le schéma, à recaler sur le vrai terrain.

Repère : origine au centre du terrain de jeu, x vers l'avant du drone au départ (de la zone A vers
la zone C), y vers la gauche, z vers le haut. Le poste de pilotage est derrière la zone A (x < -4) :
la droite du pilote est -y.
"""
import math
from pathlib import Path

OUT = Path(__file__).resolve().parent.parent / "worlds" / "voliere.sdf"

# Terrain de jeu : bande rouge vers l'intérieur, blanche vers l'extérieur
TERRAIN_X, TERRAIN_Y = 8.0, 5.0
LIGNE = 0.05              # largeur des lignes au sol

# Zone A (départ) : coin du terrain côté pilote, à sa droite, bordée de bleu côté terrain
ZONE_A_X = -2.9           # limite de la zone A vers la zone B
ZONE_A_Y = 0.0            # limite de la zone A vers la gauche du pilote
ARUCO_0 = (-3.45, -1.25)  # centre de la zone A, drone posé dessus au départ

# Limite B / C : blanc côté B, bleu côté C, sur toute la largeur du terrain
LIMITE_BC_X = 0.95

# Zone C : 4 affiches A3 au sol (zones de livraison ; images de l'annexe 3 pas encore publiées :
# couleurs du schéma en attendant), dans l'ordre 1, 3, 2, 4 de la droite vers la gauche du pilote
AFFICHES_X = 3.52
AFFICHES = [  # (numéro, y, couleur)
    (1, -1.875, (0.11, 0.62, 0.46)),
    (3, -0.625, (0.85, 0.35, 0.19)),
    (2, 0.625, (0.83, 0.33, 0.49)),
    (4, 1.875, (0.59, 0.77, 0.35)),
]
A3 = (0.297, 0.42)

# Zone G (coopération) : robot côté B du cache visuel, ArUco 2 et 3 au sol de l'autre côté
CACHE_X = 1.98
CACHE_Y = (-1.76, 1.86)
CACHE_H = 0.45            # non donnée ; < 50 cm (seule la zone B a des objets plus hauts)
ROBOT = (1.40, -0.27)     # se déplace selon y ; ArUco 1 sur le dessus
ROBOT_TAILLE = (0.30, 0.25, 0.15)  # non donnée
ARUCOS_ALIGNEMENT_X = 2.72
ARUCOS_ALIGNEMENT = [(2, -0.10), (3, 1.08)]

# Fenêtre E : ouverture 1 x 1 m, hauteur aléatoire au concours, traversée selon x
FENETRE = (-1.68, 0.11)
FENETRE_BAS = 0.8         # hauteur du bas de l'ouverture
FENETRE_CADRE = 0.05      # largeur et épaisseur des montants (non donnée)
# Recto (face vers A) : un ArUco à chaque coin, au-dessus et en dessous du cadre (pas sur les
# diagonales) ; droite / gauche vues du pilote, droite = -y
ARUCOS_RECTO = [(4, "droite", "bas"), (5, "droite", "haut"), (6, "gauche", "haut"), (7, "gauche", "bas")]
# Verso (face vers C) : deux ArUco alignés verticalement, au-dessus et en dessous du cadre, pas au
# centre : au tiers de la largeur de l'ouverture
FENETRE_VERSO_Y = -1 / 6
ARUCOS_VERSO = [(8, "bas"), (9, "haut")]

# Obstacles F (totems triangulaires d'environ 2 m de l'annexe 6, approchés par des boîtes),
# à plus de 50 cm du passage sûr
OBSTACLES = [(-1.50, -1.85, 0.3), (0.50, 1.50, -0.4)]  # (x, y, lacet)
OBSTACLE_TAILLE = (0.40, 0.40, 2.2)

# Passage sûr D : ligne verte à droite du pilote (à droite en allant de A vers C), jaune à gauche,
# accolées. Points de passage de l'axe, de la sortie de la zone A jusqu'à la limite B / C.
PASSAGE = [(-3.55, 0.05), (-3.545, 1.08), (-3.03, 1.76), (-1.89, 2.04), (-0.81, 1.86),
           (-0.34, 1.22), (-0.22, 0.08), (-0.16, -1.05), (0.14, -1.83), (0.62, -2.02),
           (LIMITE_BC_X - LIGNE, -2.02)]
PAS_PASSAGE = 0.05

ROUGE = (1, 0, 0)
BLANC = (1, 1, 1)
BLEU = (0, 0.3, 1)
VERT = (0, 0.7, 0.1)
JAUNE = (1, 0.85, 0)
Z_LIGNE = 0.003           # lignes au sol : boîtes de 2 mm d'épaisseur
Z_ARUCO = 0.012           # lue par tests/simu/mission.py (position_sdf de commun.py)


def fmt(valeurs):
    return " ".join(f"{v:.4g}" for v in valeurs)


def couleur(c, emissive=False):
    rgba = f"{fmt(c)} 1"
    em = f"<emissive>{rgba}</emissive>" if emissive else ""
    return f"<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>{em}</material>"


def boite(nom, pose, taille, c, collision=False, emissive=False):
    geo = f"<geometry><box><size>{fmt(taille)}</size></box></geometry>"
    xml = f'        <visual name="{nom}"><pose>{fmt(pose)}</pose>{geo}{couleur(c, emissive)}</visual>'
    if collision:
        xml += f'\n        <collision name="{nom}_c"><pose>{fmt(pose)}</pose>{geo}</collision>'
    return xml


def aruco(nom, ident, pose):
    """Boîte de 24 cm (carré noir de 20 cm + marge blanche), texture vue sur la face +z de la boîte."""
    tex = f"model://aruco_markers/materials/textures/aruco_{ident}.png"
    return (f'        <visual name="{nom}"><pose>{fmt(pose)}</pose><geometry><box><size>0.24 0.24 0.001</size>'
            f'</box></geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>'
            f'<emissive>1 1 1 1</emissive><pbr><metal><albedo_map>{tex}</albedo_map>'
            f'<emissive_map>{tex}</emissive_map></metal></pbr></material></visual>')


def segment(nom, a, b, c):
    """Ligne au sol de a à b."""
    (xa, ya), (xb, yb) = a, b
    longueur = math.hypot(xb - xa, yb - ya)
    lacet = math.atan2(yb - ya, xb - xa)
    return boite(nom, ((xa + xb) / 2, (ya + yb) / 2, Z_LIGNE, 0, 0, lacet), (longueur, LIGNE, 0.002), c)


def modele(nom, corps, pose=None):
    p = f"\n      <pose>{fmt(pose)}</pose>" if pose else ""
    return (f'    <model name="{nom}">\n      <static>true</static>{p}\n'
            f'      <link name="link">\n{corps}\n      </link>\n    </model>\n\n')


def marquages_sol():
    hx, hy = TERRAIN_X / 2, TERRAIN_Y / 2
    v = []
    for nom, c, d in (("rouge", ROUGE, -LIGNE / 2), ("blanc", BLANC, LIGNE / 2)):
        ex, ey = hx + d, hy + d   # rouge vers l'intérieur, blanc vers l'extérieur
        v.append(segment(f"{nom}_gauche", (-ex, ey), (ex, ey), c))
        v.append(segment(f"{nom}_droite", (-ex, -ey), (ex, -ey), c))
        v.append(segment(f"{nom}_fond", (ex, -ey), (ex, ey), c))
        v.append(segment(f"{nom}_pilote", (-ex, -ey), (-ex, ey), c))

    # Zone A : bleu le long de ses deux côtés intérieurs au terrain
    v.append(segment("zone_a_x", (ZONE_A_X, -hy), (ZONE_A_X, ZONE_A_Y), BLEU))
    v.append(segment("zone_a_y", (-hx, ZONE_A_Y), (ZONE_A_X, ZONE_A_Y), BLEU))
    v.append(aruco("aruco_0", 0, (*ARUCO_0, Z_ARUCO, 0, 0, 0)))

    # Limite B / C : blanc côté B, bleu côté C
    v.append(segment("bc_blanc", (LIMITE_BC_X - LIGNE / 2, -hy), (LIMITE_BC_X - LIGNE / 2, hy), BLANC))
    v.append(segment("bc_bleu", (LIMITE_BC_X + LIGNE / 2, -hy), (LIMITE_BC_X + LIGNE / 2, hy), BLEU))

    for num, y, c in AFFICHES:
        v.append(boite(f"affiche_{num}", (AFFICHES_X, y, Z_LIGNE, 0, 0, 0), (*A3, 0.002), c, emissive=True))

    for ident, y in ARUCOS_ALIGNEMENT:
        v.append(aruco(f"aruco_{ident}", ident, (ARUCOS_ALIGNEMENT_X, y, Z_ARUCO, 0, 0, 0)))
    return "\n".join(v)


def catmull_rom(points, pas):
    """Courbe lisse passant par les points, échantillonnée environ tous les `pas` mètres."""
    pts = [points[0]] + list(points) + [points[-1]]
    sortie = []
    for i in range(1, len(pts) - 2):
        p0, p1, p2, p3 = pts[i - 1], pts[i], pts[i + 1], pts[i + 2]
        n = max(1, round(math.dist(p1, p2) / pas))
        for k in range(n):
            t = k / n
            sortie.append(tuple(
                0.5 * (2 * b + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t * t
                       + (-a + 3 * b - 3 * c + d) * t ** 3)
                for a, b, c, d in zip(p0, p1, p2, p3)))
    sortie.append(points[-1])
    return sortie


def passage_sur():
    axe = catmull_rom(PASSAGE, PAS_PASSAGE)
    # Normale à droite du sens de parcours (A vers C) : (dy, -dx)
    droite = []
    for i in range(len(axe)):
        xa, ya = axe[max(i - 1, 0)]
        xb, yb = axe[min(i + 1, len(axe) - 1)]
        n = math.hypot(xb - xa, yb - ya)
        droite.append(((yb - ya) / n, -(xb - xa) / n))
    v = []
    for nom, c, cote in (("vert", VERT, 1), ("jaune", JAUNE, -1)):
        d = cote * LIGNE / 2
        courbe = [(x + d * nx, y + d * ny) for (x, y), (nx, ny) in zip(axe, droite)]
        for i in range(len(courbe) - 1):
            (xa, ya), (xb, yb) = courbe[i], courbe[i + 1]
            # léger recouvrement pour ne pas laisser de trou dans les virages
            e = 0.005 / math.hypot(xb - xa, yb - ya)
            ux, uy = (xb - xa) * e, (yb - ya) * e
            v.append(segment(f"{nom}_{i}", (xa - ux, ya - uy), (xb + ux, yb + uy), c))
    return "\n".join(v)


def fenetre():
    o, f = 1.0, FENETRE_CADRE
    zc = FENETRE_BAS + o / 2               # centre de l'ouverture
    h_montant = zc + o / 2 + f             # montants de côté posés au sol
    v = [
        boite("haut", (0, 0, zc + (o + f) / 2, 0, 0, 0), (f, o + 2 * f, f), BLANC, collision=True),
        boite("bas", (0, 0, zc - (o + f) / 2, 0, 0, 0), (f, o + 2 * f, f), BLANC, collision=True),
        boite("gauche", (0, (o + f) / 2, h_montant / 2, 0, 0, 0), (f, f, h_montant), BLANC, collision=True),
        boite("droite", (0, -(o + f) / 2, h_montant / 2, 0, 0, 0), (f, f, h_montant), BLANC, collision=True),
    ]
    # Panneaux blancs portant les ArUco, au-dessus et en dessous du cadre
    z = {"haut": zc + o / 2 + f + 0.13, "bas": zc - o / 2 - f - 0.13}
    y_coin = {"droite": -(o / 2 + f - 0.12), "gauche": o / 2 + f - 0.12}  # bord aligné sur le cadre
    panneaux = {(y_coin[c], z[n]) for _, c, n in ARUCOS_RECTO} | {(FENETRE_VERSO_Y, z[n]) for _, n in ARUCOS_VERSO}
    for i, (y, zp) in enumerate(sorted(panneaux)):
        v.append(boite(f"panneau_{i}", (0, y, zp, 0, 0, 0), (f, 0.26, 0.26), BLANC, collision=True))
    # ArUco sur les faces avant des panneaux, au même plan que les faces du cadre (sinon les montants,
    # plus épais, en masquent le bord)
    dx = f / 2 + 0.001
    # Recto : face -x (vers A) ; verso : face +x (vers C). Tangage de ∓90° : la face +z de la boîte,
    # qui porte la texture, est tournée vers -x ou +x, sans miroir.
    for ident, cote, niveau in ARUCOS_RECTO:
        v.append(aruco(f"aruco_{ident}", ident, (-dx, y_coin[cote], z[niveau], 0, -math.pi / 2, 0)))
    for ident, niveau in ARUCOS_VERSO:
        v.append(aruco(f"aruco_{ident}", ident, (dx, FENETRE_VERSO_Y, z[niveau], 0, math.pi / 2, 0)))
    return "\n".join(v)


def robot():
    lz = ROBOT_TAILLE[2]
    return "\n".join([
        boite("corps", (0, 0, lz / 2, 0, 0, 0), ROBOT_TAILLE, (0.6, 0.21, 0.34), collision=True),
        aruco("aruco_1", 1, (0, 0, lz + 0.001, 0, 0, 0)),
    ])


def cache():
    (y0, y1), h = CACHE_Y, CACHE_H
    return boite("cache", (CACHE_X, (y0 + y1) / 2, h / 2, 0, 0, 0), (0.02, y1 - y0, h), (0.37, 0.37, 0.35),
                 collision=True)


def obstacle():
    h = OBSTACLE_TAILLE[2]
    return boite("totem", (0, 0, h / 2, 0, 0, 0), OBSTACLE_TAILLE, (0.94, 0.58, 0.58), collision=True)


ENTETE = f"""<?xml version="1.0" ?>
<!--
  Volière DroneLoad, terrain du règlement 2027 (figure 1).
  FICHIER GÉNÉRÉ par sim/tools/generate_voliere.py : modifier les cotes du script et régénérer.
  Terrain de jeu {TERRAIN_X:g} x {TERRAIN_Y:g} m (rouge dedans, blanc dehors) ; zone A de départ (ArUco 0) ;
  zone B : fenêtre E (ouverture 1 x 1 m, ArUco 4-7 au recto, 8-9 au verso), obstacles F, passage sûr D
  (lignes verte et jaune) ; zone C : affiches de livraison 1-4, robot (ArUco 1 dessus), cache visuel,
  ArUco d'alignement 2 et 3.
  ArUco : carré noir de 20 cm (marker_size de drone.yaml) ; la texture a une marge blanche de 1/12
  de chaque côté, d'où des boîtes de 24 cm.
  Repère : origine au centre du terrain, x de la zone A vers la zone C (avant du drone au départ),
  z vers le haut ; poste de pilotage derrière la zone A, sa droite est -y.
-->
<sdf version="1.9">
  <world name="voliere">
    <!-- Pas de 1 ms, synchronisé avec le SITL (lock-step du plugin ArduPilot) -->
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Drone posé sur l'ArUco 0, face à la zone C (pose lue par sim/scripts/reset_drone.sh et les tests) -->
    <include>
      <uri>model://droneload_drone</uri>
      <name>drone</name>
      <pose>{ARUCO_0[0]:g} {ARUCO_0[1]:g} 0.195 0 0 0</pose>
    </include>

"""


def main():
    sdf = ENTETE
    sdf += modele("marquages_sol", marquages_sol())
    sdf += modele("passage_sur", passage_sur())
    sdf += modele("fenetre", fenetre(), pose=(*FENETRE, 0, 0, 0, 0))
    for i, (x, y, lacet) in enumerate(OBSTACLES, 1):
        sdf += modele(f"obstacle_{i}", obstacle(), pose=(x, y, 0, 0, 0, lacet))
    sdf += modele("cache_visuel", cache())
    # Robot : modèle à part pour pouvoir le déplacer (gz service set_pose) ; immobile pour l'instant
    sdf += modele("robot", robot(), pose=(*ROBOT, 0, 0, 0, 0))
    sdf += "  </world>\n</sdf>\n"
    OUT.write_text(sdf, encoding="utf-8")
    print(f"Monde écrit dans {OUT}")


if __name__ == "__main__":
    main()
