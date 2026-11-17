"""Test de la position UWB simulée fusionnée dans l'EKF (config/ardupilot/uwb.param, nœud position_ext).

Compare la position de l'EKF (/mavros/local_position/pose) à la vérité terrain de Gazebo (/sim/drone/pose,
même repère ENU que l'UWB simulé) :
1. décollage, stationnaire 20 s : le drone ne dérive pas et l'EKF suit la vérité (avec l'UWB : sans décalage) ;
2. sauts : 30 % des mesures UWB décalées de 2 m pendant 10 s (service uwb, paramètre taux_sauts) : rejetés
   par position_ext (message sur drone/logs), le drone ne bouge pas ;
3. panne d'UWB de 5 s (service uwb, paramètre actif) : le drone reste en vol au flux optique, puis l'EKF
   se recale sur l'UWB à son retour ;
4. panne d'UWB de 30 s : OBSERVATION, sans critère (état affiché toutes les 5 s, messages de la Pixhawk) ;
   recalage vérifié si le drone vole encore, sinon les étapes suivantes sont sautées. Le flux optique
   simulé est presque parfait : la dérive en simu est optimiste par rapport au vol réel ;
5. jeu de sources 2 de l'EKF (comme le switch RC9 : flux optique + lidar), stationnaire 10 s, retour au jeu 1 ;
6. atterrissage.
Sans uwb.param (PARAM_UWB= : EKF au flux optique seul, pour comparer) : étape 1 seulement.

Orientation : position_ext envoie une orientation à l'identité (PANS ne donne pas de cap). Pour vérifier
qu'ArduPilot n'en tire pas une rotation de la position, relancer ce test drone tourné de 90° au départ
(sim/scripts/reset_drone.sh 90) : l'étape 1 doit montrer le même décalage nul.

Prérequis : simu lancée avec le code du drone et l'UWB (PARAM_UWB=uwb docker compose --profile drone up -d),
drone au départ (sim/scripts/reset_drone.sh).
Usage : docker compose exec companion bash -lc 'python3 sim/scripts/uwb_test.py'
Code de sortie 0 si tout est OK, 1 sinon.
"""
import json
import math
import sys
import time

import paho.mqtt.client as mqtt
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, StatusText
from mavros_msgs.srv import CommandLong, MessageInterval
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

BROKER = ('127.0.0.1', 1883)
TAKEOFF_ALT = 1.0       # = takeoff_altitude de drone.yaml
ALT_TOLERANCE = 0.2     # m
HOVER_TIME = 20.0       # s
PANNE_TIME = 5.0        # s
PANNE_LONGUE = 30.0     # s
SAUTS_TIME = 10.0       # s
TAUX_SAUTS = 0.3        # proportion de mesures UWB décalées pendant l'étape des sauts
SRC2_TIME = 10.0        # s
DERIVE_MAX = 0.3        # m : déplacement réel maximal en stationnaire
DERIVE_PANNE_MAX = 0.5  # m : pendant la panne d'UWB (flux optique seul)
SUIVI_MAX = 0.3         # m : écart de l'EKF à la vérité, autour de son décalage moyen
DECALAGE_MAX = 0.3      # m : décalage moyen EKF - vérité avec l'UWB (même repère)
LOCAL_POSITION_NED = 32
MAV_CMD_SET_EKF_SOURCE_SET = 42007


def xy(pose):
    return pose.position.x, pose.position.y


class UwbTest(Node):
    def __init__(self):
        super().__init__('uwb_test')
        self.state = State()
        self.height = None
        self.ekf = None
        self.ekf_time = 0.0
        self.verite = None
        self.telemetry = None
        self.logs = []               # messages reçus sur drone/logs
        self.create_subscription(State, '/mavros/state', lambda m: setattr(self, 'state', m), 10)
        self.create_subscription(Range, '/mavros/rangefinder/rangefinder', self._on_range, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose', self._on_ekf, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/sim/drone/pose', lambda m: setattr(self, 'verite', m.pose),
                                 qos_profile_sensor_data)
        # Messages de la Pixhawk (failsafe EKF, changement de source...)
        self.create_subscription(StatusText, '/mavros/statustext/recv',
                                 lambda m: print(f'    Pixhawk : {m.text}', flush=True), qos_profile_sensor_data)
        self.interval = self.create_client(MessageInterval, '/mavros/set_message_interval')
        self.command = self.create_client(CommandLong, '/mavros/cmd/command')
        self.uwb_params = self.create_client(SetParameters, '/uwb_sim/set_parameters')
        # Paramètres de l'autopilote, lus par MAVROS à la connexion
        self.fcu_params = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.interval_requested = 0.0

        self.mqtt = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, 'uwb_test')
        self.mqtt.on_message = self._on_mqtt
        self.mqtt.connect(*BROKER)
        self.mqtt.subscribe([('drone/telemetry', 0), ('drone/logs', 0)])
        self.mqtt.loop_start()

    def _on_range(self, msg):
        self.height = msg.range

    def _on_ekf(self, msg):
        self.ekf = msg.pose
        self.ekf_time = time.monotonic()

    def _on_mqtt(self, client, userdata, msg):
        if msg.topic == 'drone/telemetry':
            self.telemetry = json.loads(msg.payload)
        else:
            self.logs.append(msg.payload.decode())
            print(f'    log : {self.logs[-1]}', flush=True)

    def spin(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        # La Pi ne reçoit que les messages demandés : position de l'EKF redemandée si elle se tait
        now = time.monotonic()
        if now - self.ekf_time > 2.0 and now - self.interval_requested > 2.0 and self.interval.service_is_ready():
            self.interval.call_async(MessageInterval.Request(message_id=LOCAL_POSITION_NED, message_rate=20.0))
            self.interval_requested = now

    def wait_for(self, condition, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.spin()
            if condition():
                return True
        return False

    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        self.wait_for(future.done, timeout)
        return future.result() if future.done() else None

    def set_mode(self, mode):
        self.mqtt.publish('drone/cmd', json.dumps({'action': 'set_mode', 'mode': mode}))

    def fcu_param(self, nom):
        """Valeur entière d'un paramètre de l'autopilote, None tant que MAVROS ne l'a pas lu."""
        reponse = self.call(self.fcu_params, GetParameters.Request(names=[nom]))
        if reponse is None or reponse.values[0].type != ParameterType.PARAMETER_INTEGER:
            return None
        return reponse.values[0].integer_value

    def set_uwb_sim(self, nom, valeur):
        """Règle un paramètre du capteur UWB simulé (actif, taux_sauts)."""
        reponse = self.call(self.uwb_params, SetParameters.Request(
            parameters=[Parameter(nom, value=valeur).to_parameter_msg()]))
        return reponse is not None and reponse.results[0].successful

    def set_uwb(self, actif):
        return self.set_uwb_sim('actif', actif)

    def panne_longue(self, duree):
        """Observe le drone sans UWB pendant duree, sans critère ; vrai s'il vole encore à la bonne hauteur."""
        depart = xy(self.verite) if self.verite else (0.0, 0.0)
        start = time.monotonic()
        prochain = start
        while time.monotonic() - start < duree:
            self.spin()
            if time.monotonic() >= prochain:
                prochain += 5.0
                vx, vy = xy(self.verite)
                ex, ey = xy(self.ekf)
                hauteur = f'{self.height:.2f} m' if self.height is not None else '?'
                print(f'    t = {time.monotonic() - start:4.1f} s : armé {self.state.armed}, mode {self.state.mode}, '
                      f'hauteur {hauteur}, dérive réelle {math.hypot(vx - depart[0], vy - depart[1]):.2f} m, '
                      f'écart EKF - vérité {math.hypot(ex - vx, ey - vy):.2f} m', flush=True)
        return self.state.armed and self.at_alt()

    def source_set(self, jeu):
        reponse = self.call(self.command, CommandLong.Request(command=MAV_CMD_SET_EKF_SOURCE_SET, param1=float(jeu)))
        return reponse is not None and reponse.success

    def at_alt(self):
        return self.height is not None and abs(self.height - TAKEOFF_ALT) < ALT_TOLERANCE

    def observer(self, duree):
        """Stationnaire observé pendant duree : (en vol à la bonne hauteur tout du long, statistiques)."""
        ecarts, positions = [], []
        start = time.monotonic()
        prochain = start
        while time.monotonic() - start < duree:
            self.spin()
            if not (self.state.armed and self.at_alt()):
                print(f'    hauteur {self.height} m après {time.monotonic() - start:.1f} s, '
                      f'armé : {self.state.armed}, mode : {self.state.mode}', flush=True)
                return False, None
            if time.monotonic() >= prochain and self.ekf is not None and self.verite is not None:
                prochain += 0.1
                (ex, ey), (vx, vy) = xy(self.ekf), xy(self.verite)
                ecarts.append((ex - vx, ey - vy))
                positions.append((vx, vy))
        if not ecarts:
            print('    ni position EKF ni vérité terrain reçues', flush=True)
            return False, None
        mx = sum(e[0] for e in ecarts) / len(ecarts)
        my = sum(e[1] for e in ecarts) / len(ecarts)
        stats = {
            'decalage': math.hypot(mx, my),
            'suivi': max(math.hypot(e[0] - mx, e[1] - my) for e in ecarts),
            'derive': max(math.hypot(p[0] - positions[0][0], p[1] - positions[0][1]) for p in positions),
        }
        print(f'    décalage EKF - vérité {stats["decalage"]:.2f} m ({mx:+.2f}, {my:+.2f}), '
              f'écart de suivi max {stats["suivi"]:.2f} m, dérive réelle max {stats["derive"]:.2f} m', flush=True)
        return True, stats


def main():
    rclpy.init()
    node = UwbTest()
    steps = []

    def step(name, ok):
        print(f"[{'OK' if ok else 'ÉCHEC'}] {name}", flush=True)
        steps.append(ok)
        return ok

    try:
        if not step('télémétrie MQTT du drone reçue', node.wait_for(lambda: node.telemetry is not None, 30)):
            return 1
        if not step('MAVROS connecté', node.wait_for(lambda: node.state.connected, 60)):
            return 1
        if not step('vérité terrain Gazebo reçue (/sim/drone/pose)', node.wait_for(lambda: node.verite is not None, 10)):
            return 1
        # UWB utilisé par l'EKF seulement avec config/ardupilot/uwb.param (PARAM_UWB de sim/.env)
        posxy = None
        deadline = time.monotonic() + 30
        while posxy is None and time.monotonic() < deadline:
            posxy = node.fcu_param('EK3_SRC1_POSXY')
            node.wait_for(lambda: False, 0.5)
        if not step('paramètres de l\'autopilote lus (EK3_SRC1_POSXY)', posxy is not None):
            return 1
        avec_uwb = posxy == 6
        if avec_uwb:
            print('    EKF : position UWB (uwb.param)', flush=True)
            if not step('mesures UWB reçues (télémétrie uwb_hz)', node.wait_for(lambda: node.telemetry['uwb_hz'] > 0, 10)):
                return 1
        else:
            print('    EKF sans UWB (PARAM_UWB vide) : test au flux optique seul (étape 1)', flush=True)
        if not step('position EKF reçue (/mavros/local_position/pose)', node.wait_for(lambda: node.ekf is not None, 15)):
            return 1

        node.set_mode('takeoff')
        if not step(f'décollage à {TAKEOFF_ALT} m', node.wait_for(lambda: node.state.armed and node.at_alt(), 150)):
            return 1
        node.wait_for(lambda: False, 3.0)   # le drone dépasse un peu la cible avant de se stabiliser
        node.set_mode('attente')

        ok, s = node.observer(HOVER_TIME)
        if step(f'stationnaire {HOVER_TIME:.0f} s', ok):
            step(f'dérive réelle < {DERIVE_MAX} m', s['derive'] < DERIVE_MAX)
            step(f'EKF suit la vérité (écart < {SUIVI_MAX} m)', s['suivi'] < SUIVI_MAX)
            if avec_uwb:
                step(f'EKF dans le repère de l\'UWB (décalage < {DECALAGE_MAX} m)', s['decalage'] < DECALAGE_MAX)

        if avec_uwb and ok:
            if step('sauts UWB simulés', node.set_uwb_sim('taux_sauts', TAUX_SAUTS)):
                debut_logs = len(node.logs)
                ok, s = node.observer(SAUTS_TIME)
                if step(f'{100 * TAUX_SAUTS:.0f} % de sauts de 2 m pendant {SAUTS_TIME:.0f} s : toujours en vol', ok):
                    step(f'dérive réelle < {DERIVE_MAX} m', s['derive'] < DERIVE_MAX)
                    step(f'EKF non tiré par les sauts (décalage < {DECALAGE_MAX} m)', s['decalage'] < DECALAGE_MAX)
                step('sauts rejetés par position_ext (drone/logs)',
                     any('Saut UWB rejeté' in log for log in node.logs[debut_logs:]))
                step('sauts arrêtés', node.set_uwb_sim('taux_sauts', 0.0))

        if avec_uwb and ok:
            if step('panne d\'UWB déclenchée', node.set_uwb(False)):
                ok, s = node.observer(PANNE_TIME)
                step(f'panne d\'UWB {PANNE_TIME:.0f} s : toujours en vol', ok)
                if ok:
                    step(f'dérive pendant la panne < {DERIVE_PANNE_MAX} m', s['derive'] < DERIVE_PANNE_MAX)
                step('UWB rétabli', node.set_uwb(True))
                node.wait_for(lambda: False, 5.0)   # l'EKF se recale sur l'UWB
                ok, s = node.observer(3.0)
                step('EKF recalé sur l\'UWB après la panne', ok and s['decalage'] < DECALAGE_MAX)

        if avec_uwb and ok:
            print(f'    panne d\'UWB de {PANNE_LONGUE:.0f} s : observation (pas de critère)', flush=True)
            if step('panne longue déclenchée', node.set_uwb(False)):
                en_vol = node.panne_longue(PANNE_LONGUE)
                step('UWB rétabli', node.set_uwb(True))
                if en_vol:
                    node.wait_for(lambda: False, 5.0)
                    ok, s = node.observer(3.0)
                    step('EKF recalé sur l\'UWB après la panne longue', ok and s['decalage'] < DECALAGE_MAX)
                else:
                    ok = False
                    print('    ArduPilot a interrompu le vol pendant la panne longue : étapes suivantes sautées',
                          flush=True)

        if avec_uwb and ok:
            if step('bascule sur le jeu de sources 2 (flux optique + lidar)', node.source_set(2)):
                ok, s = node.observer(SRC2_TIME)
                if step(f'stationnaire {SRC2_TIME:.0f} s sur le jeu 2', ok):
                    step(f'dérive réelle < {DERIVE_MAX} m', s['derive'] < DERIVE_MAX)
                step('retour au jeu de sources 1 (UWB)', node.source_set(1))
                node.wait_for(lambda: False, 5.0)
                ok, s = node.observer(3.0)
                step('EKF recalé sur l\'UWB au retour du jeu 1', ok and s['decalage'] < DECALAGE_MAX)

        node.set_mode('land')
        step('atterrissage et désarmement', node.wait_for(lambda: not node.state.armed, 60))
        node.set_mode('attente')
    finally:
        node.set_uwb(True)   # ne pas laisser l'UWB simulé en panne ni décalé
        node.set_uwb_sim('taux_sauts', 0.0)
        node.mqtt.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

    print('TEST UWB :', 'OK' if all(steps) else 'ÉCHEC')
    return 0 if all(steps) else 1


if __name__ == '__main__':
    sys.exit(main())
