"""Nœud principal du drone : lit le mode demandé (depuis le dashboard, via mqtt_server) et fait
avancer la mission correspondante d'un pas à chaque tick (20 Hz).

Modes : attente, takeoff, land, epreuve1, epreuve2, deplacement, releve. À chaque demande de mode, même
identique au mode en cours, la mission est réinitialisée : une 2e demande de décollage ou d'atterrissage
repart du début. Les modes qui ne pilotent pas le drone (attente, epreuve1) le tiennent en stationnaire.

Missions : table self.missions (mode -> objet). Une mission standard a reset(), state et run(drone, now) ;
pour en ajouter une : l'écrire dans missions/, puis une ligne dans self.missions (rien d'autre à brancher).

Carte : self.carte (carte_client.py) donne aux missions la carte (get, liste), l'ajout de points
(ajouter) et ce que voit la caméra sol dans le repère map (arucos_vus, images_vues, enregistrer_vus).
Mission releve : enregistre tout ce qui est vu, en stationnaire ou vers un point envoyé pendant le relevé.

/position/fiable (std_msgs/Bool, chaque tick) : même critère que la pause des missions, lu par le nœud carte.

Perte de position : une mission qui en a besoin (besoin_position = True, ex. deplacement) n'est pas
appelée tant que drone.get_position() est None ; le drone reste en stationnaire (au flux optique) et l'état est
<mode>:POSITION_PERDUE. Position revenue avant delai_position : la mission reprend où elle était. Au-delà :
<mode>:ABANDON, stationnaire jusqu'à une nouvelle demande de mode ou la reprise en main par le pilote.
"""
import rclpy
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray

from .carte_client import CarteClient, declarer_parametres
from .mavros import MavrosDrone
from .missions import Atterrissage, Decollage, Deplacement, Epreuve1, Releve, run_epreuve2

MODES_SANS_MISSION = ('attente', 'epreuve2')   # modes traités directement par _step
# Modes que la reprise en main par le pilote n'arrête pas : takeoff passe lui-même en GUIDED, land en LAND
MODES_SANS_REPRISE = ('attente', 'takeoff', 'land')
DETECTIONS_MAX_AGE = 0.5   # s : au-delà, les détections sont considérées comme perdues

# Dernière valeur conservée pour un nœud qui (re)démarre après la commande
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class MainNode(Node):
    def __init__(self):
        super().__init__('main_node')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('takeoff_altitude', 1.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('ekf_origin', [45.7838, 4.8722, 170.0])
        self.declare_parameter('delai_source', 1.0)     # s : position UWB plus vieille = position non fiable
        self.declare_parameter('delai_recalage', 3.0)   # s après le retour de l'UWB avant de juger la position fiable
        self.declare_parameter('delai_position', 30.0)  # s de perte de position avant d'abandonner la mission
        self.declare_parameter('vitesse_max', 0.5)      # m/s, missions deplacement et releve (modifiable en vol)
        self.declare_parameter('precision', 0.1)        # m, missions deplacement et releve

        log = self.get_logger()
        self.drone = MavrosDrone(self, tuple(self.get_parameter('ekf_origin').value),
                                 self.get_parameter('delai_source').value,
                                 self.get_parameter('delai_recalage').value)
        self.delai_position = self.get_parameter('delai_position').value
        self.deplacement = Deplacement(log, self.get_parameter('vitesse_max').value,
                                       self.get_parameter('precision').value)
        self.perte_position = None   # heure du début de la perte de position (mission en pause)
        self.abandon = False
        declarer_parametres(self)
        self.carte = CarteClient(self, self.drone)
        self.releve = Releve(log, self.carte, self.get_parameter('vitesse_max').value,
                             self.get_parameter('precision').value)
        self.decollage = Decollage(log, self.get_parameter('takeoff_altitude').value)
        self.atterrissage = Atterrissage(log)
        self.epreuve1 = Epreuve1(log)
        self.image_width = self.get_parameter('image_width').value
        # Mode -> mission. Une nouvelle mission s'ajoute ici : reset() à chaque demande de mode, pause et
        # abandon automatiques si besoin_position, run(drone, now) à chaque tick, state publié
        self.missions = {
            'takeoff': self.decollage,
            'land': self.atterrissage,
            'epreuve1': self.epreuve1,
            'deplacement': self.deplacement,
            'releve': self.releve,
        }
        self.modes = MODES_SANS_MISSION + tuple(self.missions)

        self.mode = 'attente'
        self.guided_vu = False       # autopilote vu en GUIDED depuis la dernière demande de mode
        self.target_class = None
        self.detections = []
        self.detections_time = 0.0
        self.published_state = None

        self.create_subscription(String, '/mission/mode', self._on_mode, LATCHED)
        self.create_subscription(String, '/mission/target_class', self._on_target_class, LATCHED)
        self.create_subscription(Point, '/mission/target_point', self._on_target_point, LATCHED)
        self.create_subscription(Detection2DArray, '/detections', self._on_detections,
                                 qos_profile_sensor_data)
        self.state_pub = self.create_publisher(String, '/mission/state', LATCHED)
        self.hook_pub = self.create_publisher(Bool, '/hook/cmd', 10)
        # Position jugée fiable (get_position non None), à chaque tick : le nœud carte refuse tout point
        # enregistré sans (y compris par des nœuds hors de main_node, ex. caméra avant)
        self.fiable_pub = self.create_publisher(Bool, '/position/fiable', 10)

        self.add_on_set_parameters_callback(self._on_parametres)
        self.create_timer(1.0 / self.get_parameter('rate').value, self._tick)
        log.info('Nœud principal prêt, mode attente')

    def _on_mode(self, msg):
        if msg.data not in self.modes:
            self.get_logger().warning(f'Mode inconnu ignoré : {msg.data}')
            return
        # Une demande du mode en cours le relance aussi (ex. nouveau décollage après un ECHEC)
        relance = msg.data == self.mode
        self.mode = msg.data
        for mission in self.missions.values():
            mission.reset()
        self.perte_position = None
        self.abandon = False
        self.guided_vu = False
        # Arrête tout de suite la consigne de vitesse du mode précédent (ArduPilot la garde sinon
        # jusqu'à GUID_TIMEOUT). Pas en takeoff : une consigne de vitesse interrompt la montée GUIDED
        if self.mode != 'takeoff':
            self._hover()
        self.get_logger().info(f'Mode : {self.mode}' + (' (relancé)' if relance else ''))

    def _on_target_class(self, msg):
        self.target_class = msg.data
        self.epreuve1.reset()

    def _on_target_point(self, msg):
        self.deplacement.set_point(msg.x, msg.y)
        self.get_logger().info(f'Point visé : ({msg.x:.2f}, {msg.y:.2f})')
        if self.mode == 'deplacement':
            self.deplacement.reset()
        elif self.mode == 'releve':   # point envoyé pendant le relevé seulement (Releve.reset l'oublie)
            self.releve.set_point(msg.x, msg.y)

    def _position_ok(self, now):
        """Pause ou abandon d'une mission qui a besoin de la position ; vrai si elle peut avancer."""
        if self.abandon:
            self._hover()
            return False
        if self.drone.get_position() is not None:
            if self.perte_position is not None:
                self.get_logger().info(f'Position revenue après {now - self.perte_position:.1f} s : reprise de la mission')
                self.perte_position = None
            return True
        self._hover()
        if self.perte_position is None:
            self.perte_position = now
            self.get_logger().warning(f'Position perdue : stationnaire, reprise automatique si elle revient '
                                      f'dans {self.delai_position:.0f} s')
        elif now - self.perte_position > self.delai_position:
            self.abandon = True
            self.get_logger().error(f'Position perdue depuis {self.delai_position:.0f} s : mission abandonnée, '
                                    'stationnaire, reprendre la main à la radio')
        return False

    def _on_detections(self, msg):
        self.detections_time = self.get_clock().now().nanoseconds / 1e9
        self.detections = []
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            w, h = det.bbox.size_x, det.bbox.size_y
            self.detections.append({'class': hyp.class_id, 'conf': hyp.score,
                                    'box': (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)})
        if self.mode == 'epreuve1' and self.target_class:
            self.epreuve1.add_frame({d['class'] for d in self.detections}, self.target_class)

    def _hover(self):
        """Vol stationnaire, seulement si le drone vole sous notre contrôle (armé en GUIDED)."""
        self.drone.hover()

    def _on_parametres(self, parametres):
        """vitesse_max modifiable en vol (ros2 param set /main_node vitesse_max 1.0) : missions deplacement, releve."""
        for p in parametres:
            if p.name == 'vitesse_max':
                if not isinstance(p.value, (int, float)) or p.value <= 0:
                    return SetParametersResult(successful=False, reason='vitesse_max > 0 en m/s')
                for deplacement in (self.deplacement, self.releve.deplacement):
                    deplacement.vitesse_max = float(p.value)
                self.get_logger().info(f'Vitesse maximale des déplacements : {p.value:.2f} m/s')
        return SetParametersResult(successful=True)

    def _reprise_pilote(self):
        """Vrai si le pilote a repris la main à la radio (autopilote sorti de GUIDED, drone armé) pendant une
        mission : main_node passe en attente, et la mission ne repart pas seule au retour en GUIDED."""
        if self.drone.mode == 'GUIDED':
            self.guided_vu = True
            return False
        if not (self.guided_vu and self.drone.armed and self.mode not in MODES_SANS_REPRISE):
            return False
        self.get_logger().warning(f'Reprise en main par le pilote (mode {self.drone.mode}) : mission {self.mode} '
                                  'arrêtée, mode attente')
        self.mode = 'attente'
        self.guided_vu = False
        for mission in self.missions.values():
            mission.reset()
        return True

    def _tick(self):
        # Une erreur dans une mission ne doit pas tuer le nœud : le drone reste en stationnaire
        try:
            self.fiable_pub.publish(Bool(data=self.drone.get_position() is not None))
            self._step()
        except Exception as e:
            self.get_logger().error(f'Erreur dans la mission {self.mode} : {e!r}, stationnaire',
                                    throttle_duration_sec=1.0)
            self._hover()

    def _step(self):
        now = self.get_clock().now().nanoseconds / 1e9
        drone = self.drone

        self._reprise_pilote()
        mission = self.missions.get(self.mode)
        if mission is not None and getattr(mission, 'besoin_position', False) and not self._position_ok(now):
            # Mission en pause (stationnaire) ou abandonnée : pas appelée
            state = 'ABANDON' if self.abandon else 'POSITION_PERDUE'
        elif self.mode == 'epreuve1':
            self._hover()
            if self.target_class and self.epreuve1.run(now):
                self.get_logger().info(f'Cible trouvée : {self.target_class}, ouverture du crochet')
                self.hook_pub.publish(Bool(data=True))
            state = self.epreuve1.state if self.target_class else 'SANS_CIBLE'
        elif mission is not None:
            mission.run(drone, now)
            state = mission.state
        elif self.mode == 'epreuve2':
            fresh = now - self.detections_time < DETECTIONS_MAX_AGE
            run_epreuve2(drone, self.detections if fresh else [], self.image_width)
            state = 'SUIVI'
        else:
            self._hover()
            state = 'ATTENTE'

        full_state = f'{self.mode}:{state}'
        if full_state != self.published_state:
            self.published_state = full_state
            self.state_pub.publish(String(data=full_state))


def main():
    rclpy.init()
    node = MainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
