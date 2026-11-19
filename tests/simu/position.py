"""Test de la perte de position pendant une mission (main_node, mission deplacement, MavrosDrone.get_position).

Piloté par MQTT comme le dashboard (set_point, set_mode), observé par ROS 2 (/mission/state, vérité Gazebo
/sim/drone/pose), pannes d'UWB par le service uwb (paramètre actif) :
1. décollage, déplacement à 1,5 m du départ (y croissant, dans la zone A : la fenêtre est devant le
   départ en x) : ARRIVE, vraie position à moins de 0,2 m du point ;
2. retour au départ avec 5 s de panne d'UWB en route : POSITION_PERDUE en moins de 2 s, drone arrêté
   (vitesse réelle < 0,15 m/s) et toujours en vol ; UWB revenu : reprise après delai_recalage (3 s, l'EKF
   se recale), ARRIVE à moins de 0,2 m ;
3. nouveau déplacement avec 35 s de panne : POSITION_PERDUE puis ABANDON vers 30 s (delai_position),
   drone toujours en GUIDED et arrêté ; UWB revenu : l'état reste ABANDON (pas de reprise seule) ;
4. reprise en main par le pilote pendant un déplacement (radio simulée en FlowHold, gaz au neutre) :
   main_node passe en attente ; revenu en GUIDED, le drone ne repart pas vers le point ;
5. atterrissage.
Sans uwb.param (PARAM_UWB vide) : étape 1 seulement, au flux optique seul (position relative au départ).

Lancement : python3 tests/run.py position, qui prépare la simu (avec l'UWB). À la main : PARAM_UWB=uwb
docker compose --profile drone up -d, drone au départ (sim/scripts/reset_drone.sh), puis
docker compose exec companion bash -lc 'python3 tests/simu/position.py'
Code de sortie 0 si tout est OK, 1 sinon.
"""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

from commun import ClientMqtt, Etapes, Radio, attendre, entier, param_drone, verifier_uwb, xy

DISTANCE = 1.5          # m entre le départ et le point visé
PRECISION = 0.2         # m : vraie position au point visé
PRECISION_FLUX = 0.5    # m : au flux optique seul (sans UWB), la position de l'EKF dérive
VITESSE_ARRET = 0.15    # m/s : drone arrêté pendant la perte de position
DETECTION_MAX = 2.0     # s entre la coupure de l'UWB et POSITION_PERDUE (delai_source 1 s)
PANNE_COURTE = 5.0      # s
DELAI_POSITION = param_drone('main_node', 'delai_position')   # s (temps de simulation, comme main_node)
DELAI_RECALAGE = param_drone('main_node', 'delai_recalage')   # s : reprise de la mission après le retour de l'UWB
PANNE_LONGUE = DELAI_POSITION + 5.0   # s : panne plus longue que delai_position
VITESSE_RECALAGE = 0.3  # m/s : au retour de l'UWB, l'EKF se recale et le drone corrige la dérive de la panne
LATCHED = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class PositionTest(Node):
    def __init__(self):
        # Temps de simulation : main_node compte delai_position en temps simulé (la simu peut être plus lente)
        super().__init__('position_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.state = State()
        self.mission = ''
        self.ekf = None
        self.verite = None
        self.create_subscription(State, '/mavros/state', lambda m: setattr(self, 'state', m), 10)
        self.create_subscription(String, '/mission/state', self._on_mission, LATCHED)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', lambda m: setattr(self, 'ekf', m.pose),
                                 qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/sim/drone/pose', lambda m: setattr(self, 'verite', m.pose),
                                 qos_profile_sensor_data)
        self.uwb_params = self.create_client(SetParameters, '/uwb_sim/set_parameters')
        self.fcu_params = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.radio = Radio()   # pilote simulé : seule la radio passe en GUIDED (FLTMODE_GCSBLOCK)
        self.mqtt = ClientMqtt('position_test', afficher_mission=False)   # le drone vu du dashboard

    def _on_mission(self, msg):
        if msg.data != self.mission:
            print(f'    état mission : {msg.data}', flush=True)
        self.mission = msg.data

    @property
    def telemetry(self):
        return self.mqtt.telemetry

    @property
    def logs(self):
        return self.mqtt.logs

    def wait_for(self, condition, timeout):
        return attendre(condition, timeout, spin=lambda: rclpy.spin_once(self, timeout_sec=0.05))

    def attendre(self, duree):
        self.wait_for(lambda: False, duree)

    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        self.wait_for(future.done, timeout)
        return future.result() if future.done() else None

    def cmd(self, action, **champs):
        self.mqtt.commande(action, **champs)

    def aller(self, point):
        self.cmd('set_point', x=point[0], y=point[1])
        self.attendre(0.5)   # point reçu par main_node avant le mode
        self.cmd('set_mode', mode='deplacement')

    def pilote_flowhold(self):
        """Le pilote reprend la main à la radio : FlowHold, gaz au neutre (hauteur tenue)."""
        self.radio.envoyer('flowhold', gaz=1500)
        return self.wait_for(lambda: self.state.mode not in ('GUIDED', ''), 3)

    def pilote_guided(self):
        """Le pilote remet l'interrupteur de modes sur GUIDED (seule façon d'y passer)."""
        self.radio.guided()
        return self.wait_for(lambda: self.state.mode == 'GUIDED', 3)

    def set_uwb(self, actif):
        reponse = self.call(self.uwb_params, SetParameters.Request(
            parameters=[Parameter('actif', value=actif).to_parameter_msg()]))
        return reponse is not None and reponse.results[0].successful

    def avec_uwb(self):
        posxy = entier(self.call(self.fcu_params, GetParameters.Request(names=['EK3_SRC1_POSXY'])))
        return None if posxy is None else posxy == 6

    def sim_now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def ecart(self, point):
        x, y = xy(self.verite)
        return math.hypot(x - point[0], y - point[1])

    def vitesse_max(self, duree):
        """Vitesse réelle maximale (m/s, mesurée sur 0,5 s) pendant duree ; None si le drone n'est plus en vol."""
        vmax = 0.0
        start = time.monotonic()
        while time.monotonic() - start < duree:
            t0, p0 = time.monotonic(), xy(self.verite)
            self.attendre(0.5)
            p1 = xy(self.verite)
            vmax = max(vmax, math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / (time.monotonic() - t0))
            if not self.state.armed:
                return None
        return vmax


def main():
    rclpy.init()
    node = PositionTest()
    step = Etapes('TEST POSITION')

    try:
        if not step('télémétrie MQTT du drone reçue', node.wait_for(lambda: node.telemetry is not None, 30)):
            return 1
        if not step('MAVROS connecté', node.wait_for(lambda: node.state.connected, 60)):
            return 1
        if not step('vérité terrain et position EKF reçues',
                    node.wait_for(lambda: node.verite is not None and node.ekf is not None, 20)):
            return 1
        uwb = None
        deadline = time.monotonic() + 30
        while uwb is None and time.monotonic() < deadline:
            uwb = node.avec_uwb()
            node.attendre(0.5)
        if not step('paramètres de l\'autopilote lus (EK3_SRC1_POSXY)', uwb is not None):
            return 1
        if not verifier_uwb(step, uwb):
            return 1
        print('    EKF : ' + ('position UWB (uwb.param)' if uwb else 'flux optique seul : étape 1 seulement'),
              flush=True)

        node.cmd('set_mode', mode='takeoff')
        node.pilote_guided()   # le pilote passe en GUIDED : takeoff arme et décolle
        if not step('décollage', node.wait_for(lambda: node.mission == 'takeoff:STATIONNAIRE', 150)):
            return 1
        node.attendre(3.0)
        attendu = 'Position : UWB' if uwb else 'Position : flux optique seul'
        step(f'source de position de main_node : « {attendu} »', any(attendu in log for log in node.logs))
        # Points dans le repère de l'EKF (avec l'UWB : celui de Gazebo, sinon : relatif au départ)
        depart_ekf, depart_verite = xy(node.ekf), xy(node.verite)
        point_ekf = (depart_ekf[0], depart_ekf[1] + DISTANCE)
        point_verite = (depart_verite[0], depart_verite[1] + DISTANCE)

        # 1. Déplacement simple
        node.aller(point_ekf)
        ok = step('déplacement de 1,5 m : ARRIVE', node.wait_for(lambda: node.mission == 'deplacement:ARRIVE', 30))
        if ok:
            node.attendre(1.0)
            e = node.ecart(point_verite)
            print(f'    vraie position à {e:.2f} m du point', flush=True)
            precision = PRECISION if uwb else PRECISION_FLUX
            ok = step(f'vraiment au point (< {precision} m)', e < precision)

        # 2. Panne courte en route
        if uwb and ok:
            node.aller(depart_ekf)
            node.attendre(1.5)   # en route
            node.set_uwb(False)
            coupure = time.monotonic()
            perdue = node.wait_for(lambda: node.mission == 'deplacement:POSITION_PERDUE', DETECTION_MAX + 1)
            step(f'panne d\'UWB en route : POSITION_PERDUE en {time.monotonic() - coupure:.1f} s '
                 f'(< {DETECTION_MAX} s)', perdue and time.monotonic() - coupure < DETECTION_MAX)
            node.attendre(1.0)   # freinage
            v = node.vitesse_max(PANNE_COURTE - (time.monotonic() - coupure))
            if step('toujours en vol pendant la perte', v is not None):
                print(f'    vitesse réelle max {v:.2f} m/s', flush=True)
                step(f'drone arrêté (< {VITESSE_ARRET} m/s)', v < VITESSE_ARRET)
            node.set_uwb(True)
            retour = time.monotonic()
            repris = node.wait_for(lambda: node.mission == 'deplacement:EN_ROUTE', DELAI_RECALAGE + 5)
            step(f'UWB revenu : reprise après {time.monotonic() - retour:.1f} s de recalage de l\'EKF '
                 f'(>= {DELAI_RECALAGE - 0.5:.1f} s)', repris and time.monotonic() - retour >= DELAI_RECALAGE - 0.5)
            ok = step('puis ARRIVE',
                      node.wait_for(lambda: node.mission == 'deplacement:ARRIVE', 30))
            if ok:
                node.attendre(1.0)
                e = node.ecart(depart_verite)
                print(f'    vraie position à {e:.2f} m du départ', flush=True)
                ok = step(f'vraiment au départ (< {PRECISION} m)', e < PRECISION)

        # 3. Panne longue : abandon
        if uwb and ok:
            node.aller(point_ekf)
            node.attendre(1.0)
            node.set_uwb(False)
            node.wait_for(lambda: node.mission == 'deplacement:POSITION_PERDUE', DETECTION_MAX + 1)
            perte, perte_sim = time.monotonic(), node.sim_now()
            abandon = node.wait_for(lambda: node.mission == 'deplacement:ABANDON', 2 * DELAI_POSITION)
            duree = node.sim_now() - perte_sim
            step(f'panne longue : ABANDON après {duree:.1f} s simulées (≈ {DELAI_POSITION:.0f} s)',
                 abandon and DELAI_POSITION - 2 < duree < DELAI_POSITION + 3)
            step('toujours en vol et en GUIDED', node.state.armed and node.state.mode == 'GUIDED')
            node.attendre(max(0.0, PANNE_LONGUE - (time.monotonic() - perte)))
            node.set_uwb(True)
            v = node.vitesse_max(3.0)   # recalage de l'EKF sur l'UWB : petite correction de la dérive
            step('UWB revenu : toujours ABANDON (pas de reprise seule)', node.mission == 'deplacement:ABANDON')
            if v is not None:
                print(f'    vitesse réelle max après le retour de l\'UWB {v:.2f} m/s', flush=True)
                step(f'pas de reprise du déplacement, seulement le recalage (< {VITESSE_RECALAGE} m/s)',
                     v < VITESSE_RECALAGE)

        # 4. Reprise en main par le pilote (radio) pendant une mission
        if uwb and ok:
            node.aller(depart_ekf)
            en_route = node.wait_for(lambda: node.mission == 'deplacement:EN_ROUTE', DELAI_RECALAGE + 5)
            if step('nouveau déplacement en route', en_route):
                node.attendre(1.0)
                debut_logs = len(node.logs)
                step('pilote : radio en FlowHold', node.pilote_flowhold())
                step('main_node passe en attente (reprise en main signalée)',
                     node.wait_for(lambda: node.mission == 'attente:ATTENTE', 3)
                     and node.wait_for(lambda: any('Reprise en main' in log for log in node.logs[debut_logs:]), 3))
                node.attendre(2.0)
                step('retour en GUIDED à la radio', node.pilote_guided())
                node.attendre(1.0)
                v = node.vitesse_max(3.0)
                if step('toujours en vol', v is not None):
                    step(f'la mission ne repart pas seule ({v:.2f} m/s < {VITESSE_ARRET} m/s, attente:ATTENTE)',
                         v < VITESSE_ARRET and node.mission == 'attente:ATTENTE')

        node.cmd('set_mode', mode='land')
        step('atterrissage et désarmement', node.wait_for(lambda: not node.state.armed, 60))
        node.cmd('set_mode', mode='attente')
    finally:
        node.set_uwb(True)   # ne pas laisser l'UWB simulé en panne
        node.radio.fermer()
        node.mqtt.fermer()
        node.destroy_node()
        rclpy.shutdown()

    return step.fin()


if __name__ == '__main__':
    sys.exit(main())
