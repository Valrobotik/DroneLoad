"""Accès des missions à la carte (nœud carte) : lecture, ajout, et ce que voit la caméra dans le repère map.

Créé par main_node (self.carte) et passé aux missions qui en ont besoin. Ne bloque jamais.

Lecture (/carte/points) :
    carte.get('aruco', '0') -> {'type', 'nom', 'x', 'y', 'z', 'cap', 'vues', 'age'} ou None
    carte.get('fenetre')    -> premier point du type (fenêtre, robot : un seul)
    carte.liste('image')    -> tous les points du type
    cap : rad (fenêtre : normale côté recto), None si le point n'a pas d'orientation ; age : s depuis la
    dernière vue. Point absent : la mission attend ou cherche, elle ne devine pas.
Ajout (/carte/ajouter, une vue par appel ; le nœud carte fait la moyenne) :
    carte.ajouter('cible', 'pomme', (x, y, z))      carte.retirer('cible', 'pomme')
Ce que voit la caméra sol, dans le repère map (dernière image ; vide si la pose du drone a plus de
age_pose_max à la réception de l'image) :
    carte.arucos_vus()   -> [{'id', 'position', 'normale'}]   (+ carte.image_aruco : numéro de l'image)
    carte.images_vues()  -> [{'classe', 'position', 'confiance'}] (centre du cadre YOLO projeté au sol)
    carte.enregistrer_vus() : enregistre une fois par image tout ce qui est vu (mission releve) :
    ArUco de la fenêtre -> fenetre, ArUco du robot -> robot, autres ArUco -> aruco, objets YOLO -> image.

Pose prise à la réception de la détection (pas d'appariement par horodatage : horloges MAVROS et image
différentes). La carte ne vaut que ce que vaut la position de l'EKF (dérive du flux optique sans UWB).
"""
import json
import math

from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

from .reperes import Camera, Fenetre, cap_quaternion, direction_map, pixel_sol, point_map, quaternion_cap, rotate

LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


def declarer_parametres(node):
    """Paramètres du bloc main_node de drone.yaml utilisés par la carte."""
    node.declare_parameter('camera_sol_position', [0.05, 0.0, -0.04])   # m, repère du drone (avant, gauche, haut)
    node.declare_parameter('camera_sol_rotation', [0.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0])
    node.declare_parameter('age_pose_max', 0.2)      # s
    node.declare_parameter('distance_max', 4.0)      # m : ArUco plus loin ignoré (pose imprécise)
    node.declare_parameter('aruco_robot', 1)
    node.declare_parameter('fenetre_nom', '1')
    node.declare_parameter('fenetre_arucos', ['4 recto 0.43 -0.68', '5 recto 0.43 0.68', '6 recto -0.43 0.68',
                                              '7 recto -0.43 -0.68', '8 verso 0.1667 -0.68', '9 verso 0.1667 0.68'])


class CarteClient:
    def __init__(self, node, drone):
        self.node = node
        self.log = node.get_logger()
        self.drone = drone
        p = node.get_parameter
        self.camera_sol = Camera(p('camera_sol_position').value, p('camera_sol_rotation').value)
        self.age_pose_max = p('age_pose_max').value
        self.distance_max = p('distance_max').value
        self.aruco_robot = p('aruco_robot').value
        self.fenetre_nom = p('fenetre_nom').value
        self.fenetre = Fenetre.depuis_parametres(p('fenetre_arucos').value)

        self.points = {}           # (type, nom) -> dict, d'après /carte/points
        self.connexions = drone.connexions   # reconnexions de l'autopilote déjà vues
        self.reconnexion = False             # reconnexion à vérifier (source de l'EKF pas encore relue)
        self.k = None              # calibration de la caméra sol (camera_info)
        self._arucos = []
        self._images = []
        self.image_aruco = 0       # numéros des images analysées (une vue enregistrée par image)
        self.image_yolo = 0
        self._enregistre = (0, 0)  # dernières images enregistrées par enregistrer_vus()
        self._repere = None

        node.create_subscription(MarkerArray, '/carte/points', self._on_points, LATCHED)
        node.create_subscription(Detection3DArray, '/aruco/detections', self._on_aruco, qos_profile_sensor_data)
        node.create_subscription(Detection2DArray, '/detections', self._on_yolo, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, '/camera/camera_info', self._on_camera_info, qos_profile_sensor_data)
        self.ajout_pub = node.create_publisher(Marker, '/carte/ajouter', 100)
        self.repere_pub = node.create_publisher(String, '/carte/repere', LATCHED)
        node.create_timer(1.0, self._surveiller)

    # --- Lecture ---
    def get(self, genre, nom=None):
        if nom is not None:
            return self.points.get((genre, str(nom)))
        return next((p for (g, _), p in self.points.items() if g == genre), None)

    def liste(self, genre):
        return [p for (g, _), p in self.points.items() if g == genre]

    # --- Ajout ---
    def ajouter(self, genre, nom, position, cap=None):
        m = Marker(ns=genre, text=str(nom), action=Marker.ADD)
        m.header.frame_id = 'map'
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in position)
        if cap is not None:   # orientation nulle (0, 0, 0, 0) : point sans cap
            (m.pose.orientation.x, m.pose.orientation.y,
             m.pose.orientation.z, m.pose.orientation.w) = quaternion_cap(cap)
        else:
            m.pose.orientation.w = 0.0
        self.ajout_pub.publish(m)

    def retirer(self, genre, nom):
        self.ajout_pub.publish(Marker(ns=genre, text=str(nom), action=Marker.DELETE))

    # --- Ce que voit la caméra ---
    def arucos_vus(self):
        return list(self._arucos)

    def images_vues(self):
        return list(self._images)

    def enregistrer_vus(self):
        """Enregistre sur la carte ce que voit la caméra sol, une seule fois par image analysée."""
        if self._enregistre[0] != self.image_aruco:
            for a in self._arucos:
                ident = a['id']
                if self.fenetre.face(ident) is not None:
                    fenetre = self.fenetre.depuis_aruco(ident, a['position'], a['normale'])
                    if fenetre is not None:
                        self.ajouter('fenetre', self.fenetre_nom, fenetre[0], fenetre[1])
                elif ident == self.aruco_robot:
                    self.ajouter('robot', 'robot', a['position'])
                else:
                    self.ajouter('aruco', ident, a['position'])
        if self._enregistre[1] != self.image_yolo:
            for img in self._images:
                self.ajouter('image', img['classe'], img['position'])
        self._enregistre = (self.image_aruco, self.image_yolo)

    # --- Interne ---
    def _pose_recente(self):
        pose = self.drone.get_pose(self.age_pose_max)
        if pose is None:
            self.log.warning('Détection sans position récente du drone : pas placée sur la carte',
                             throttle_duration_sec=10.0)
        return pose

    def _on_points(self, msg):
        points = {}
        for m in msg.markers:
            if m.action != Marker.ADD:
                continue
            try:
                info = json.loads(m.text)
            except ValueError:
                continue
            p, o = m.pose.position, m.pose.orientation
            cap = cap_quaternion((o.x, o.y, o.z, o.w)) if info.get('cap') else None
            points[(m.ns, info['nom'])] = {'type': m.ns, 'nom': info['nom'], 'x': p.x, 'y': p.y, 'z': p.z,
                                          'cap': cap, 'vues': info.get('vues'), 'age': info.get('age')}
        self.points = points

    def _on_camera_info(self, msg):
        self.k = tuple(msg.k)

    def _on_aruco(self, msg):
        self.image_aruco += 1
        self._arucos = []
        pose = self._pose_recente() if msg.detections else None
        if pose is None:
            return
        for det in msg.detections:
            if not det.results:
                continue
            c = det.results[0].pose.pose
            v = (c.position.x, c.position.y, c.position.z)
            if math.sqrt(sum(x * x for x in v)) > self.distance_max:
                continue
            q = (c.orientation.x, c.orientation.y, c.orientation.z, c.orientation.w)
            self._arucos.append({'id': int(det.results[0].hypothesis.class_id),
                                 'position': point_map(pose, self.camera_sol, v),
                                 'normale': direction_map(pose, self.camera_sol, rotate(q, (0.0, 0.0, 1.0)))})

    def _on_yolo(self, msg):
        self.image_yolo += 1
        self._images = []
        if not msg.detections or self.k is None:
            return
        pose = self._pose_recente()
        hauteur = self.drone.get_current_alt_brute()
        if pose is None or hauteur is None:
            return
        for det in msg.detections:
            if not det.results:
                continue
            centre = det.bbox.center.position
            position = pixel_sol(pose, self.camera_sol, self.k, centre.x, centre.y, hauteur)
            if position is not None:
                hyp = det.results[0].hypothesis
                self._images.append({'classe': hyp.class_id, 'position': position, 'confiance': hyp.score})

    def _surveiller(self):
        """Chaque seconde : repère de l'EKF (source figée à l'armement, au sol celle d'EK3_SRC1_POSXY : uwb,
        gps, flux) pour le nœud carte et le PC,
        et avertissement si l'autopilote s'est reconnecté sans source absolue (redémarrage possible de la
        Pixhawk : l'origine de l'EKF, donc le repère de la carte, a pu changer)."""
        source = self.drone.source or self.drone.source_ekf   # au sol : source configurée de l'EKF
        if source is not None and source != self._repere:
            self._repere = source
            self.repere_pub.publish(String(data=source))
        if self.drone.connexions != self.connexions:
            self.reconnexion = self.connexions > 0   # la 1re connexion n'est pas une reconnexion
            self.connexions = self.drone.connexions
        if self.reconnexion and self.drone.source_ekf is not None:   # EK3_SRC1_POSXY relu après la reconnexion
            self.reconnexion = False
            if self.points and self.drone.source_ekf == 'flux':
                self.log.warning('Autopilote reconnecté sans UWB : si la Pixhawk a redémarré, l\'origine de '
                                 'l\'EKF a changé et la carte est décalée (reset_carte)')
