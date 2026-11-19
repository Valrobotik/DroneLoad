"""Carte du terrain : registre des points enregistrés par les missions, dans le repère map (ENU) de MAVROS.

Le nœud ne regarde aucune caméra : ce sont les missions (main_node, droneload_vol/carte_client.py) qui
décident quoi enregistrer et calculent la position (droneload_vol/reperes.py).

- /carte/ajouter (visualization_msgs/Marker) : ns = type (aruco, robot, fenetre, obstacle, image, cible...),
  text = nom du point (identifiant ArUco, classe YOLO, numéro...), pose = position (+ cap par l'orientation,
  fenêtre). action ADD : ajoute une vue ; DELETE : retire le point.
  Types de `remplacer` (robot qui bouge, point noté une fois par une mission) : dernière vue seule, publié
  dès la 1re vue. Autres types : moyenne des vues, publié à partir de vues_min vues ; une fois 2 x vues_min
  vues, une vue à plus de ecart_max de la moyenne est rejetée (reflet, pose ambiguë) : un point dont les
  premières vues étaient fausses est alors figé, seuls DELETE ou /carte/reset le corrigent.
- Position fiable : un ajout n'est accepté que si /position/fiable (main_node, même critère que la pause des
  missions : UWB présent et sans saut quand le vol a commencé avec l'UWB, et revenu depuis au moins
  delai_recalage de main_node) dit vrai et date de moins de age_fiable_max. Vaut pour tout nœud qui
  enregistre (main_node, caméra avant...). Sans UWB (flux optique seul), la position est toujours jugée
  fiable : la carte dérive avec l'EKF.
- Repère : les points d'une carte sont tous dans le même repère (celui de l'EKF au 1er point, /carte/repere).
  Si le repère de l'EKF change alors que la carte a des points (vol sans UWB après un vol avec), les ajouts
  sont refusés jusqu'à reset_carte : la carte ne mélange jamais deux repères.
- /carte/points (visualization_msgs/MarkerArray, transient local, 1 Hz et à chaque changement) : ns = type,
  id = numéro stable du point, text = JSON {"nom", "vues", "age"} (age : s depuis la dernière vue),
  orientation = cap (fenêtre : normale côté recto), scale = taille (fenêtre : épaisseur, largeur, hauteur).
- /carte/json (std_msgs/String, transient local, avec /carte/points) : les mêmes points au format de
  drone/carte du dashboard, relayé tel quel par mqtt_server : liste de {"type", "nom", "x", "y", "z",
  "cap" (degrés, null sans orientation), "vues", "age" (s), "taille" ([largeur, hauteur] en m)}.
- /carte/reset (std_msgs/Empty) : vide la carte (et le fichier).
- Sauvegarde : `fichier` (JSON) réécrit à chaque changement, avec le repère de l'EKF (/carte/repere de
  main_node : uwb, gps, flux). Au démarrage, par défaut (garder_carte false), la carte part vide et
  l'ancienne est mise de côté dans <fichier>_precedente.json ; /carte/charger (commande charger_carte du
  dashboard) la recharge. garder_carte true : l'ancienne carte est rechargée au démarrage.
  Sans UWB (ni GPS), l'origine de l'EKF est le point de décollage : une carte d'un autre vol n'est juste
  que si le drone est reparti du même endroit avec le même cap (avertissement au chargement).
"""
import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Empty, String
from visualization_msgs.msg import Marker, MarkerArray

from .reperes import cap_quaternion, quaternion_cap

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
REPERES_ABSOLUS = ('uwb', 'gps')   # repères identiques d'un vol à l'autre
DELAI_DEMARRAGE = 3.0   # s avant le démarrage de la carte : ses messages doivent arriver au dashboard (mqtt_server)
COULEURS = {'aruco': (1.0, 0.5, 0.0), 'robot': (0.0, 0.6, 1.0), 'fenetre': (0.6, 0.2, 0.8),
            'obstacle': (0.9, 0.1, 0.1), 'image': (0.1, 0.7, 0.2)}


class Point:
    """Position (moyenne ou dernière) d'un point enregistré."""

    def __init__(self, ident, genre, nom):
        self.ident = ident
        self.genre = genre
        self.nom = nom
        self.somme = [0.0, 0.0, 0.0]
        self.somme_cap = None      # (somme des cos, somme des sin) du cap, None sans cap
        self.vues = 0
        self.vu = time.monotonic()
        self.annonce = False       # « repéré » déjà journalisé

    @property
    def position(self):
        return tuple(s / self.vues for s in self.somme)

    @property
    def cap(self):
        return None if self.somme_cap is None else math.atan2(self.somme_cap[1], self.somme_cap[0])

    def ajouter(self, position, cap, remplacer):
        """Ajoute une vue ; remplacer : la position devient celle de cette vue (vues compte toutes les vues)."""
        self.vues += 1
        if remplacer:
            self.somme = [p * self.vues for p in position]
            if cap is not None:
                self.somme_cap = (math.cos(cap), math.sin(cap))
        else:
            self.somme = [s + p for s, p in zip(self.somme, position)]
            if cap is not None:
                c, s = self.somme_cap or (0.0, 0.0)
                self.somme_cap = (c + math.cos(cap), s + math.sin(cap))
        self.vu = time.monotonic()

    def json(self):
        return {'type': self.genre, 'nom': self.nom, 'position': [round(p, 4) for p in self.position],
                'cap': None if self.cap is None else round(self.cap, 4), 'vues': self.vues}

    @classmethod
    def depuis_json(cls, ident, d):
        point = cls(ident, d['type'], d['nom'])
        point.vues = max(1, int(d['vues']))
        point.annonce = True
        point.somme = [float(p) * point.vues for p in d['position']]
        if d.get('cap') is not None:
            point.somme_cap = (math.cos(d['cap']), math.sin(d['cap']))
        return point


class Carte(Node):
    def __init__(self):
        super().__init__('carte')
        self.declare_parameter('ecart_max', 0.5)       # m
        self.declare_parameter('vues_min', 3)
        self.declare_parameter('remplacer', ['robot', 'cible'])
        self.declare_parameter('fenetre_taille', [1.0, 1.0])   # m, largeur et hauteur de l'ouverture (règlement)
        self.declare_parameter('fichier', '')          # vide : pas de sauvegarde
        self.declare_parameter('garder_carte', False)
        self.declare_parameter('age_fiable_max', 0.5)  # s : /position/fiable plus vieux = non fiable
        self.ecart_max = self.get_parameter('ecart_max').value
        self.age_fiable_max = self.get_parameter('age_fiable_max').value
        self.vues_min = self.get_parameter('vues_min').value
        self.remplacer = set(self.get_parameter('remplacer').value)
        self.fenetre_taille = list(self.get_parameter('fenetre_taille').value)
        fichier = self.get_parameter('fichier').value
        self.fichier = Path(fichier) if fichier else None
        self.precedente = self.fichier.with_name(self.fichier.stem + '_precedente.json') if self.fichier else None

        self.points = {}           # (type, nom) -> Point
        self.prochain_ident = 0
        self.repere = None         # repère de l'EKF (main_node), None tant qu'inconnu
        self.repere_points = None  # repère des points de la carte, None si vide ou pas encore connu
        self.repere_refus = False  # ajouts refusés pour changement de repère (signalé une fois)
        self.modifiee = False
        self.fiable = False
        self.fiable_time = 0.0     # réception du dernier /position/fiable
        self.refuses = 0           # ajouts refusés depuis la dernière perte signalée
        # Démarrage (carte mise de côté ou rechargée) DELAI_DEMARRAGE après le lancement, pas dans le constructeur : ses
        # messages partiraient sur /rosout avant que mqtt_server ne découvre le nœud et n'arriveraient
        # jamais au dashboard
        self.demarre = False
        self.lancement = time.monotonic()

        self.pub = self.create_publisher(MarkerArray, '/carte/points', LATCHED)
        self.pub_json = self.create_publisher(String, '/carte/json', LATCHED)
        self.create_subscription(Marker, '/carte/ajouter', self._on_ajouter, 100)
        self.create_subscription(Empty, '/carte/reset', self._on_reset, 10)
        self.create_subscription(Empty, '/carte/charger', lambda _: self._charger(self.precedente), 10)
        self.create_subscription(String, '/carte/repere', self._on_repere, LATCHED)
        self.create_subscription(Bool, '/position/fiable', self._on_fiable, 10)
        self.create_timer(1.0, self._timer)

    # --- Sauvegarde ---
    def _demarrage(self):
        if self.fichier is None:
            self.get_logger().info('Carte sans sauvegarde (paramètre fichier vide)')
            return
        if not self.fichier.exists():
            return
        if self.get_parameter('garder_carte').value:
            self._charger(self.fichier)
            return
        try:
            os.replace(self.fichier, self.precedente)
            self.get_logger().info('Carte vide au démarrage ; carte précédente gardée de côté '
                                   '(commande charger_carte pour la reprendre)')
        except OSError as e:
            self.get_logger().error(f'Carte précédente non mise de côté : {e}')

    def _charger(self, chemin):
        if chemin is None or not chemin.exists():
            self.get_logger().warning(f'Pas de carte à charger ({chemin})')
            return
        try:
            data = json.loads(chemin.read_text())
            points = [Point.depuis_json(i, d) for i, d in enumerate(data['points'])]
        except (OSError, ValueError, KeyError, TypeError) as e:
            self.get_logger().error(f'Carte illisible ({chemin}) : {e}')
            return
        self.points = {(p.genre, p.nom): p for p in points}
        self.prochain_ident = len(points)
        repere = data.get('repere')
        self.repere_points = repere if points else None
        self.repere_refus = False
        self.get_logger().info(f'Carte chargée : {len(points)} point(s), repère {repere or "inconnu"}')
        if repere not in REPERES_ABSOLUS:
            self.get_logger().warning('Carte enregistrée sans UWB : juste seulement si le drone est reparti '
                                      'du même endroit avec le même cap')
        elif self.repere is not None and self.repere != repere:
            self.get_logger().warning(f'Carte enregistrée en repère {repere}, EKF actuel en {self.repere} : '
                                      'ajouts refusés jusqu\'à reset_carte')
        self.modifiee = True
        self._publier()

    def _sauver(self):
        if self.fichier is None:
            return
        data = {'repere': self.repere_points or self.repere, 'date': time.strftime('%Y-%m-%d %H:%M:%S'),
                'points': [p.json() for p in sorted(self.points.values(), key=lambda p: p.ident)]}
        try:
            self.fichier.parent.mkdir(parents=True, exist_ok=True)
            temporaire = self.fichier.with_suffix('.tmp')
            temporaire.write_text(json.dumps(data, indent=1))
            os.replace(temporaire, self.fichier)   # jamais de fichier à moitié écrit (coupure de batterie)
        except OSError as e:
            self.get_logger().error(f'Carte non sauvegardée : {e}', throttle_duration_sec=10.0)

    # --- Entrées ---
    def _on_repere(self, msg):
        if msg.data != self.repere:
            self.repere = msg.data
            if self.points and self.repere_points is None:
                self.repere_points = msg.data   # points enregistrés avant que le repère soit connu (au sol)
            self.modifiee = True

    def _on_fiable(self, msg):
        self.fiable = msg.data
        self.fiable_time = time.monotonic()

    def _position_fiable(self):
        return self.fiable and time.monotonic() - self.fiable_time <= self.age_fiable_max

    def _repere_ok(self):
        """Faux (et signalé une fois) si la carte a des points dans un autre repère que celui de l'EKF."""
        if not self.points or self.repere_points is None or self.repere is None or self.repere_points == self.repere:
            self.repere_refus = False
            return True
        if not self.repere_refus:
            self.repere_refus = True
            self.get_logger().error(f'Carte en repère {self.repere_points}, EKF en repère {self.repere} : '
                                    'points refusés jusqu\'à reset_carte (la carte ne mélange pas deux repères)')
        return False

    def _on_reset(self, msg):
        self.points.clear()
        self.repere_points = None
        self.repere_refus = False
        self.prochain_ident = 0
        self.get_logger().info('Carte effacée')
        self.modifiee = True
        self._publier()

    def _on_ajouter(self, msg):
        genre, nom = msg.ns, msg.text
        if not genre or not nom:
            self.get_logger().warning(f'Point sans type ou sans nom ignoré ({genre!r}, {nom!r})')
            return
        if msg.action == Marker.DELETE:
            if self.points.pop((genre, nom), None) is not None:
                self.get_logger().info(f'{genre} {nom} retiré de la carte')
                self.modifiee = True
            return
        if not self._position_fiable():
            if self.refuses == 0:
                self.get_logger().warning(f'Position du drone non fiable : points refusés ({genre} {nom}...)')
            self.refuses += 1
            return
        if self.refuses:
            self.get_logger().info(f'Position fiable : enregistrement repris ({self.refuses} vue(s) refusée(s))')
            self.refuses = 0
        if not self._repere_ok():
            return
        if not self.points:
            self.repere_points = self.repere
        p, o = msg.pose.position, msg.pose.orientation
        position = (p.x, p.y, p.z)
        if not all(math.isfinite(v) for v in position):
            return
        cap = None if (o.x, o.y, o.z, o.w) == (0.0, 0.0, 0.0, 0.0) else cap_quaternion((o.x, o.y, o.z, o.w))
        remplacer = genre in self.remplacer
        point = self.points.get((genre, nom))
        if point is None:
            point = self.points[(genre, nom)] = Point(self.prochain_ident, genre, nom)
            self.prochain_ident += 1
        elif not remplacer and point.vues >= 2 * self.vues_min:
            ecart = math.dist(point.position[:2], position[:2])
            if ecart > self.ecart_max:
                self.get_logger().warning(f'{genre} {nom} : estimation à {ecart:.2f} m de la moyenne, ignorée',
                                          throttle_duration_sec=10.0)
                return
        point.ajouter(position, cap, remplacer)
        self.modifiee = True
        if not point.annonce and point.vues >= (1 if remplacer else self.vues_min):
            point.annonce = True
            x, y, z = point.position
            self.get_logger().info(f'{genre} {nom} repéré en ({x:.2f}, {y:.2f}, {z:.2f})'
                                   + ('' if point.cap is None else f', cap {math.degrees(point.cap):.0f}°'))

    # --- Sortie ---
    def _timer(self):
        if not self.demarre and time.monotonic() - self.lancement >= DELAI_DEMARRAGE:
            self.demarre = True
            self._demarrage()
        if self.modifiee and self.demarre:   # jamais avant le démarrage : il écraserait l'ancienne carte
            self._sauver()
            self.modifiee = False
        self._publier()

    def _publier(self):
        markers = MarkerArray()
        efface = Marker(action=Marker.DELETEALL)   # remplace la carte précédente (points effacés compris)
        efface.header.frame_id = 'map'
        markers.markers.append(efface)
        liste = []   # /carte/json
        maintenant = time.monotonic()
        for point in sorted(self.points.values(), key=lambda p: p.ident):
            if point.genre not in self.remplacer and point.vues < self.vues_min:
                continue
            m = Marker(ns=point.genre, id=point.ident, type=Marker.CUBE, action=Marker.ADD)
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = point.position
            q = quaternion_cap(point.cap or 0.0)
            m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
            if point.genre == 'fenetre':
                m.scale.x, m.scale.y, m.scale.z = 0.05, self.fenetre_taille[0], self.fenetre_taille[1]
            else:
                m.scale.x = m.scale.y = 0.2
                m.scale.z = 0.01
            m.color.r, m.color.g, m.color.b = COULEURS.get(point.genre, (0.5, 0.5, 0.5))
            m.color.a = 1.0
            age = round(maintenant - point.vu, 1)
            m.text = json.dumps({'nom': point.nom, 'vues': point.vues, 'age': age, 'cap': point.cap is not None})
            markers.markers.append(m)
            x, y, z = point.position
            liste.append({'type': point.genre, 'nom': point.nom, 'x': round(x, 3), 'y': round(y, 3), 'z': round(z, 3),
                          'cap': None if point.cap is None else round(math.degrees(point.cap), 1),
                          'vues': point.vues, 'age': age, 'taille': [round(m.scale.y, 3), round(m.scale.z, 3)]})
        self.pub.publish(markers)
        self.pub_json.publish(String(data=json.dumps(liste)))


def main():
    rclpy.init()
    node = Carte()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):   # Ctrl-C, ou arrêt par le launch (SIGTERM)
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
