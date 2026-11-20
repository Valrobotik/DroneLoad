"""Carte de la volière en direct (vue de dessus) : position du drone et points qu'il a repérés.

Usage, depuis la racine du repo (ouverte aussi par ground/launch.py) :
    .venv/bin/python ground/carte.py --broker localhost --sim                 (simulation)
    .venv/bin/python ground/carte.py --broker <IP du Pi> --taille 8x5 --depart -3.45,-1.25

Reçoit par MQTT (mqtt_server du drone) :
- drone/position : {"ekf": {x, y, z, cap}, "verite": {...}} à 5 Hz ; ekf = position estimée par la
  Pixhawk (repère map de MAVROS, ENU), verite = position Gazebo (simu seulement), null si absente ;
- drone/carte (retained, même repère que ekf) : [{"type", "nom", "x", "y", "z", "cap" (degrés, null sans
  orientation), "vues", "age" (s depuis la dernière vue), "taille" ([largeur, hauteur] en m)}], format
  écrit par _on_carte de mqtt_server.

Terrain :
- --sim : tout est lu dans sim/worlds/voliere.sdf (bords, zones, piliers, fenêtres, ArUco attendus,
  départ du drone) : la carte suit le monde Gazebo sans fichier à tenir aligné ;
- sinon : --taille LxP (m, volière centrée sur l'origine du terrain) et --depart x,y (départ du drone).

Repère (--repere auto, par défaut : celui que le drone annonce dans drone/position) : sans UWB (flux),
l'origine de l'EKF est le point de départ du drone : la position est décalée du départ et tournée de
--rotation ; avec l'UWB (ou le GPS), le repère map est celui des ancres réglé par position_ext : affiché
tel quel. --repere depart ou uwb force l'un ou l'autre ; repère pas encore annoncé : comme depart.
"""
import argparse
import json
import math
import threading
import time
import xml.etree.ElementTree as ET
from collections import deque
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import paho.mqtt.client as mqtt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon, Rectangle

REPO = Path(__file__).resolve().parents[1]
SDF = REPO / 'sim' / 'worlds' / 'voliere.sdf'
TRACE_DUREE = 60.0     # s de trajectoire affichée
DONNEES_MAX_AGE = 2.0  # s sans drone/position : « pas de données »
TAILLE_DRONE = 0.45    # m, longueur du triangle du drone
COULEURS = {'aruco': 'tab:orange', 'robot': 'tab:cyan', 'fenetre': 'tab:purple', 'obstacle': 'tab:red',
            'image': 'tab:green', 'cible': 'tab:olive'}


# --- Lecture du monde Gazebo ---
def pose_sdf(element):
    """(position, matrice de rotation) de la balise <pose> d'un élément SDF (identité sans pose)."""
    texte = element.findtext('pose')
    x, y, z, roll, pitch, yaw = (float(v) for v in texte.split()) if texte else (0,) * 6
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]])
    return np.array([x, y, z]), rotation


def enveloppe(points):
    """Enveloppe convexe 2D (chaîne monotone d'Andrew)."""
    points = sorted(set(map(tuple, np.round(points, 4))))
    if len(points) < 3:
        return points

    def demi(pts):
        h = []
        for p in pts:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (p[1] - h[-2][1]) - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(p)
        return h[:-1]
    return demi(points) + demi(points[::-1])


def lire_sdf(chemin):
    """Formes vues de dessus, ArUco au sol attendus et départ du drone, depuis le monde Gazebo."""
    monde = ET.parse(chemin).getroot().find('world')
    formes, arucos, depart = [], {}, None
    for inclus in monde.findall('include'):
        if inclus.findtext('name') == 'drone':
            depart = pose_sdf(inclus)[0][:2]
    for modele in monde.findall('model'):
        p_m, r_m = pose_sdf(modele)
        for lien in modele.findall('link'):
            p_l, r_l = pose_sdf(lien)
            p_ml, r_ml = p_m + r_m @ p_l, r_m @ r_l
            for visuel in lien.findall('visual'):
                p_v, r_v = pose_sdf(visuel)
                p, r = p_ml + r_ml @ p_v, r_ml @ r_v
                geo = visuel.find('geometry')
                diffuse = visuel.findtext('material/diffuse')
                couleur = tuple(float(c) for c in diffuse.split()[:3]) if diffuse else (0.5, 0.5, 0.5)
                texture = visuel.findtext('material/pbr/metal/albedo_map') or ''
                if geo.find('box') is not None:
                    taille = np.array([float(v) for v in geo.findtext('box/size').split()])
                    coins = np.array([[sx, sy, sz] for sx in (-.5, .5) for sy in (-.5, .5) for sz in (-.5, .5)]) * taille
                    contour = enveloppe((p + coins @ r.T)[:, :2])
                    if 'aruco_' in texture:
                        ident = int(texture.rsplit('aruco_', 1)[1].split('.')[0])
                        if abs(r[2, 2]) > 0.9:   # à plat : vu par la caméra vers le sol
                            arucos[ident] = p[:2]
                        formes.append(('aruco', contour, (0.3, 0.3, 0.3)))
                    else:
                        formes.append(('boite', contour, couleur))
                elif geo.find('cylinder') is not None:
                    formes.append(('cercle', (p[:2], float(geo.findtext('cylinder/radius'))), couleur))
    if depart is None:
        depart = arucos.get(0, np.zeros(2))
    return formes, arucos, np.array(depart)


# --- Fenêtre ---
class Carte:
    def __init__(self, args):
        self.lock = threading.Lock()
        self.position = None        # dernier drone/position
        self.position_time = 0.0
        self.points = []            # dernier drone/carte
        self.trace = deque()        # (t, x, y) en repère terrain
        self.trace_verite = deque()
        self.repere_force = None if args.repere == 'auto' else args.repere
        self.repere_drone = None    # repère annoncé par le drone (uwb, gps, flux)
        self.rotation = math.radians(args.rotation)

        if args.sim:
            self.formes, self.arucos, depart = lire_sdf(args.sdf)
        else:
            longueur, largeur = (float(v) for v in args.taille.lower().split('x'))
            self.formes = [('boite', [(-longueur / 2, -largeur / 2), (longueur / 2, -largeur / 2),
                                      (longueur / 2, largeur / 2), (-longueur / 2, largeur / 2)], None)]
            self.arucos, depart = {}, np.zeros(2)
        if args.depart:
            depart = np.array([float(v) for v in args.depart.split(',')])
        self.depart = depart
        self.sim = args.sim

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, 'PC_Carte')
        self.client.on_connect = lambda c, *a: c.subscribe([('drone/position', 0), ('drone/carte', 0)])
        self.client.on_message = self._on_message
        self.client.connect_async(args.broker, 1883, 60)   # reconnexion automatique
        self.client.loop_start()
        self._figure()

    def absolu(self):
        """Vrai si le repère map de l'EKF est celui du terrain (UWB, GPS) : rien à décaler."""
        if self.repere_force is not None:
            return self.repere_force == 'uwb'
        return self.repere_drone in ('uwb', 'gps')

    def vers_terrain(self, x, y):
        """Repère map de l'EKF -> repère du terrain."""
        if self.absolu():
            return x, y
        c, s = math.cos(self.rotation), math.sin(self.rotation)
        return self.depart[0] + c * x - s * y, self.depart[1] + s * x + c * y

    def _on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload)
        except ValueError:
            return
        with self.lock:
            if msg.topic == 'drone/position':
                self.position, self.position_time = data, time.monotonic()
                self.repere_drone = data.get('repere') or self.repere_drone
                now = time.monotonic()
                if data.get('ekf'):
                    self.trace.append((now, *self.vers_terrain(data['ekf']['x'], data['ekf']['y'])))
                if data.get('verite'):
                    self.trace_verite.append((now, data['verite']['x'], data['verite']['y']))
                for trace in (self.trace, self.trace_verite):
                    while trace and now - trace[0][0] > TRACE_DUREE:
                        trace.popleft()
            elif msg.topic == 'drone/carte' and isinstance(data, list):
                self.points = data

    def _figure(self):
        self.fig, self.ax = plt.subplots(figsize=(9, 7))
        self.fig.canvas.manager.set_window_title('DroneLoad - carte de la volière')
        ax = self.ax
        tous = []
        for genre, forme, couleur in self.formes:
            if genre == 'cercle':
                centre, rayon = forme
                ax.add_patch(plt.Circle(centre, rayon, color=couleur, alpha=0.8))
                tous.append(centre)
            else:
                plein = genre != 'aruco' and couleur is not None
                bord = couleur or 'black'
                if couleur is not None and min(couleur) > 0.8:   # blanc (cadres des fenêtres) : invisible sur fond blanc
                    bord = 'dimgray'
                ax.add_patch(Polygon(forme, closed=True, fill=plein, facecolor=couleur if plein else 'none',
                                     edgecolor=bord, linewidth=1.5 if couleur is None else 0.8))
                tous.extend(forme)
        for ident, (x, y) in self.arucos.items():
            ax.add_patch(Rectangle((x - 0.1, y - 0.1), 0.2, 0.2, fill=False, edgecolor='black', linestyle='--'))
            ax.annotate(f'attendu {ident}', (x, y - 0.18), ha='center', va='top', fontsize=7, color='gray')
        ax.plot(*self.depart, marker='+', color='green', markersize=12)
        tous = np.array(tous) if tous else np.array([[-4, -3], [4, 3]])
        marge = 0.5
        ax.set_xlim(tous[:, 0].min() - marge, tous[:, 0].max() + marge)
        ax.set_ylim(tous[:, 1].min() - marge, tous[:, 1].max() + marge)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')

        self.ligne_trace, = ax.plot([], [], color='tab:blue', linewidth=1, alpha=0.6)
        self.ligne_verite, = ax.plot([], [], color='gray', linewidth=1, linestyle=':')
        self.drone = Polygon(np.zeros((3, 2)), closed=True, color='tab:blue', zorder=5)
        self.drone_verite = Polygon(np.zeros((3, 2)), closed=True, fill=False, edgecolor='gray',
                                    linestyle='--', zorder=4)
        ax.add_patch(self.drone)
        ax.add_patch(self.drone_verite)
        self.artistes_points = []
        self.titre = ax.set_title('En attente du drone...', fontsize=10, family='monospace')
        self.anim = FuncAnimation(self.fig, self._maj, interval=200, cache_frame_data=False)

    @staticmethod
    def triangle(x, y, cap):
        a = math.radians(cap)
        av = (x + TAILLE_DRONE * 0.6 * math.cos(a), y + TAILLE_DRONE * 0.6 * math.sin(a))
        g = (x + TAILLE_DRONE * 0.4 * math.cos(a + 2.5), y + TAILLE_DRONE * 0.4 * math.sin(a + 2.5))
        d = (x + TAILLE_DRONE * 0.4 * math.cos(a - 2.5), y + TAILLE_DRONE * 0.4 * math.sin(a - 2.5))
        return np.array([av, g, d])

    def _maj(self, _):
        with self.lock:
            position, age = self.position, time.monotonic() - self.position_time
            points = list(self.points)
            trace = [(x, y) for _, x, y in self.trace]
            trace_verite = [(x, y) for _, x, y in self.trace_verite]

        self.ligne_trace.set_data(*zip(*trace) if trace else ([], []))
        self.ligne_verite.set_data(*zip(*trace_verite) if trace_verite else ([], []))

        ekf = position.get('ekf') if position and age < DONNEES_MAX_AGE else None
        verite = position.get('verite') if position and age < DONNEES_MAX_AGE else None
        if ekf:
            x, y = self.vers_terrain(ekf['x'], ekf['y'])
            cap = ekf['cap'] + (0 if self.absolu() else math.degrees(self.rotation))
            self.drone.set_xy(self.triangle(x, y, cap))
            self.drone.set_color('tab:blue')
            repere = self.repere_force or self.repere_drone or '?'
            texte = f"EKF ({repere}) x {x:+.2f}  y {y:+.2f}  z {ekf['z']:+.2f} m  cap {cap:+.0f}°"
        else:
            self.drone.set_color('lightgray')
            texte = 'PAS DE DONNÉES (drone ou MQTT)' if age >= DONNEES_MAX_AGE else 'POSITION PERDUE (EKF)'
        if verite:
            self.drone_verite.set_xy(self.triangle(verite['x'], verite['y'], verite['cap']))
            self.drone_verite.set_visible(True)
            if ekf:
                texte += f"\nvérité Gazebo : écart {math.dist((x, y), (verite['x'], verite['y'])):.2f} m"
        else:
            self.drone_verite.set_visible(False)

        for artiste in self.artistes_points:
            artiste.remove()
        self.artistes_points = []
        for point in points:
            self.dessiner_point(point)
        texte += f"   |   {len(points)} point(s) repéré(s)"
        self.titre.set_text(texte)

    def dessiner_point(self, point):
        """Point de drone/carte : carré coloré par type ; fenêtre : trait de sa largeur, flèche côté recto."""
        genre, nom = point.get('type', '?'), point.get('nom', '?')
        couleur = COULEURS.get(genre, 'gray')
        px, py = self.vers_terrain(point['x'], point['y'])
        etiquette = f"{genre if nom == genre else f'{genre} {nom}'} ({point.get('vues')} vues)"
        if genre == 'fenetre' and point.get('cap') is not None:
            a = math.radians(point['cap']) + (0 if self.absolu() else self.rotation)
            ux, uy = -math.sin(a), math.cos(a)   # axe de la fenêtre (u = -n x z, au signe près)
            demi = (point.get('taille') or [1.0])[0] / 2   # largeur de l'ouverture (fenetre_taille du drone)
            self.artistes_points += self.ax.plot([px - demi * ux, px + demi * ux], [py - demi * uy, py + demi * uy],
                                                 color=couleur, linewidth=4, zorder=3)
            self.artistes_points.append(self.ax.annotate(
                '', (px + 0.4 * math.cos(a), py + 0.4 * math.sin(a)), (px, py),
                arrowprops={'arrowstyle': '->', 'color': couleur}, zorder=3))
            etiquette += f"\nhaut. {point['z']:.2f} m, recto"
        else:
            self.artistes_points.append(self.ax.add_patch(
                Rectangle((px - 0.1, py - 0.1), 0.2, 0.2, color=couleur, zorder=3)))
        if genre == 'robot' and point.get('age') is not None:
            etiquette += f"\nvu il y a {point['age']:.0f} s"
        attendu = self.arucos.get(int(nom)) if genre == 'aruco' and str(nom).isdigit() else None
        if attendu is not None:
            etiquette += f"\nécart {math.dist((px, py), attendu):.2f} m"
        self.artistes_points.append(self.ax.annotate(etiquette, (px, py + 0.15), ha='center', va='bottom',
                                                     fontsize=8, color=couleur, zorder=6))

    def lancer(self):
        try:
            plt.show()
        except KeyboardInterrupt:   # Ctrl-C dans le terminal : fermeture normale
            pass
        finally:
            self.client.loop_stop()


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--broker', default='192.168.2.3', help='IP du Raspberry Pi (broker MQTT)')
    parser.add_argument('--sim', action='store_true', help='terrain lu dans le monde Gazebo')
    parser.add_argument('--sdf', default=str(SDF), help='monde Gazebo (avec --sim)')
    parser.add_argument('--taille', default='8x5', help='terrain LxP en m, centré (sans --sim ; règlement 2027 : 8x5)')
    parser.add_argument('--depart', help='départ du drone x,y en m (repère terrain ; --sim : lu dans le monde)')
    parser.add_argument('--rotation', type=float, default=0.0,
                        help="degrés : axe x de l'EKF (est) par rapport à l'axe x du terrain, sens trigo (--repere depart)")
    parser.add_argument('--repere', choices=['auto', 'depart', 'uwb'], default='auto',
                        help="auto : annoncé par le drone ; depart : origine EKF au départ du drone (sans UWB) ; "
                             "uwb : repère map = terrain")
    Carte(parser.parse_args()).lancer()


if __name__ == '__main__':
    main()
