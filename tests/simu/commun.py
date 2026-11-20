"""Code partagé des tests de simulation (tests/simu/) : étapes OK/ÉCHEC, client MQTT du dashboard, outils.

Lancés dans un conteneur de la simu (companion ou drone), par tests/run.py ou à la main :
    docker compose exec companion bash -lc 'python3 tests/simu/<test>.py'
Le dossier du script est dans sys.path : `from commun import ...` suffit. Écrire un test : docs/tests.md.
"""
import json
import math
import os
import socket
import struct
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import paho.mqtt.client as mqtt
import yaml

REPO = Path(__file__).resolve().parents[2]
SDF = REPO / 'sim' / 'worlds' / 'voliere.sdf'
DRONE_YAML = REPO / 'drone' / 'src' / 'droneload_bringup' / 'config' / 'drone.yaml'
BROKER = ('127.0.0.1', 1883)


def param_drone(noeud, nom):
    """Réglage du code du drone, lu dans drone.yaml (ex. param_drone('main_node', 'takeoff_altitude')) : les
    tests suivent les réglages au lieu d'en garder une copie."""
    with open(DRONE_YAML, encoding='utf-8') as f:
        return yaml.safe_load(f)[noeud]['ros__parameters'][nom]


TAKEOFF_ALT = param_drone('main_node', 'takeoff_altitude')   # m, hauteur visée par la mission takeoff
# Config UWB attendue par tests/run.py (variable DRONELOAD_UWB : '1' avec, '0' sans) ; None : test lancé à la
# main, il s'adapte à la config de la simu
UWB_ATTENDU = {'1': True, '0': False}.get(os.environ.get('DRONELOAD_UWB', ''))


def verifier_uwb(etape, avec_uwb):
    """Échec si la simu n'a pas la config UWB attendue par le lanceur (sinon un test qui s'adapte passerait
    en sautant ses étapes). Vrai si la config convient."""
    if UWB_ATTENDU is None:
        return True
    return etape(f"simu {'avec' if UWB_ATTENDU else 'sans'} UWB, comme attendu par tests/run.py "
                 f"(EKF : {'UWB' if avec_uwb else 'sans UWB'})", avec_uwb == UWB_ATTENDU)


class Etapes:
    """Étapes d'un test : etapes(nom, ok) affiche [OK]/[ÉCHEC] et renvoie ok ; fin() affiche le verdict
    (« <titre> : OK/ÉCHEC », lu par tests/run.py) et renvoie le code de sortie (0 si tout est OK)."""

    def __init__(self, titre):
        self.titre = titre
        self.resultats = []

    def __call__(self, nom, ok):
        ok = bool(ok)
        print(f"[{'OK' if ok else 'ÉCHEC'}] {nom}", flush=True)
        self.resultats.append(ok)
        return ok

    def fin(self):
        ok = all(self.resultats)
        print(f"{self.titre} : {'OK' if ok else 'ÉCHEC'}", flush=True)
        return 0 if ok else 1


def attendre(condition, timeout, pas=0.1, spin=None):
    """Vrai dès que condition() est vraie, faux après timeout s. spin : appelé à chaque tour à la place de
    time.sleep(pas) (ex. lambda: rclpy.spin_once(node, timeout_sec=0.05) pour un test ROS)."""
    deadline = time.monotonic() + timeout
    while True:
        if condition():
            return True
        if time.monotonic() >= deadline:
            return False
        if spin is None:
            time.sleep(pas)
        else:
            spin()


def depart_sdf(sdf=SDF):
    """Départ du drone (x, y) : <include> du drone dans le monde Gazebo (suit la volière, quelle qu'elle soit).
    Le drone part posé sur l'ArUco 0 (sim/scripts/reset_drone.sh)."""
    for inclus in ET.parse(sdf).getroot().find('world').findall('include'):
        if inclus.findtext('name') == 'drone':
            x, y = (float(v) for v in inclus.findtext('pose').split()[:2])
            return x, y
    raise RuntimeError(f'pas de drone dans {sdf}')


def position_sdf(visuel, sdf=SDF):
    """Position (x, y, z) d'un visuel du monde Gazebo (ex. 'aruco_0'), dans le repère du monde : celle du
    visuel plus la position de son modèle (rotation du modèle ignorée : les visuels cherchés sont au sol,
    dans des modèles non tournés)."""
    for modele in ET.parse(sdf).getroot().find('world').findall('model'):
        for v in modele.iter('visual'):
            if v.get('name') == visuel:
                pv = [float(c) for c in v.findtext('pose', '0 0 0').split()[:3]]
                pm = [float(c) for c in modele.findtext('pose', '0 0 0').split()[:3]]
                return tuple(a + b for a, b in zip(pv, pm))
    raise RuntimeError(f'pas de visuel {visuel} dans {sdf}')


def xy(pose):
    """(x, y) d'une geometry_msgs/Pose."""
    return pose.position.x, pose.position.y


def entier(reponse):
    """Valeur entière d'une réponse GetParameters à un seul nom (ex. paramètre de l'autopilote lu par MAVROS,
    /mavros/param/get_parameters), None sans réponse ou si le paramètre n'est pas encore lu."""
    from rcl_interfaces.msg import ParameterType
    if reponse is None or not reponse.values or reponse.values[0].type != ParameterType.PARAMETER_INTEGER:
        return None
    return reponse.values[0].integer_value


def ros2(*args, timeout=30, essais=3):
    """Commande ros2 (CLI) ; vrai si elle a réussi dans le temps imparti. Réessayée : au premier appel dans un
    conteneur, la découverte des nœuds peut ne pas être finie (« Node not found »)."""
    for essai in range(essais):
        try:
            if subprocess.run(['ros2', *args], capture_output=True, text=True, timeout=timeout).returncode == 0:
                return True
        except subprocess.TimeoutExpired:
            pass
        time.sleep(1.0)
    return False


class Radio:
    """Radio du pilote simulée : voies RC envoyées à l'entrée RC du SITL (UDP 5501, 16 voies en µs), qui les
    traite comme un vrai récepteur. Seule la radio passe en GUIDED (FLTMODE_GCSBLOCK de pixhawk.param) : les
    tests qui décollent l'utilisent comme le ferait le pilote. Le SITL garde les dernières valeurs reçues
    jusqu'à son redémarrage (sim/scripts/reset_drone.sh le redémarre : valeurs par défaut du SITL).

    Voies (pixhawk.param, interieur.param) : 3 gaz, 6 modes (FLTMODE_CH), 8 kill motor (RC8_OPTION 31,
    haut = moteurs coupés : laissé bas, relâché).
    """
    ADRESSE = ('127.0.0.1', 5501)
    MODES = {'stabilize': 1000, 'flowhold': 1500, 'guided': 1800}   # positions 1, 4 et 6 de la voie 6
    DEBOUNCE = 0.5   # s : ArduPilot ne change de mode que sur une position stable

    def __init__(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.mode = None   # dernière position envoyée (None : valeurs par défaut du SITL)

    def envoyer(self, mode, gaz=1000):
        voies = [1500, 1500, gaz, 1500, 1800, self.MODES[mode], 1000, 1000] + [1000] * 8
        # Plusieurs envois : UDP sans accusé, et la position doit rester stable le temps du debounce
        for _ in range(5):
            self.socket.sendto(struct.pack('<16H', *voies), self.ADRESSE)
            time.sleep(self.DEBOUNCE / 5)
        self.mode = mode

    def guided(self):
        """Interrupteur de modes sur GUIDED. ArduPilot ne change de mode qu'au changement de position :
        déjà sur GUIDED (après un atterrissage en LAND, par exemple), il est rebasculé comme le ferait le pilote."""
        if self.mode == 'guided':
            self.envoyer('stabilize')
        self.envoyer('guided')

    def fermer(self):
        self.socket.close()


def set_uwb(actif):
    """Capteur UWB simulé en marche ou en panne (paramètre actif du service uwb)."""
    return ros2('param', 'set', '/uwb_sim', 'actif', 'true' if actif else 'false')


def set_vitesse(v):
    """Vitesse maximale des missions deplacement et releve (vitesse_max de main_node, modifiable en vol)."""
    return ros2('param', 'set', '/main_node', 'vitesse_max', str(float(v)))


class ClientMqtt:
    """Le drone vu du dashboard : télémétrie, logs, position, carte ; commandes sur drone/cmd.

    Attributs mis à jour par MQTT : telemetry (dict), position (drone/position), carte (drone/carte),
    logs et heures_logs (time.monotonic() de réception). Les logs et les changements d'état de mission
    sont affichés.
    """

    def __init__(self, nom, afficher_logs=True, afficher_mission=True):
        self.telemetry = None
        self.position = None
        self.carte = None
        self.logs = []
        self.heures_logs = []
        self.afficher_logs = afficher_logs
        self.afficher_mission = afficher_mission   # faux pour un test ROS qui suit déjà /mission/state
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, nom)
        self.client.on_message = self._on_message
        self.client.connect(*BROKER)
        self.client.subscribe([('drone/telemetry', 0), ('drone/logs', 0), ('drone/position', 0), ('drone/carte', 0)])
        self.client.loop_start()

    def _on_message(self, client, userdata, msg):
        if msg.topic == 'drone/logs':
            texte = msg.payload.decode()
            self.logs.append(texte)
            self.heures_logs.append(time.monotonic())
            if self.afficher_logs:
                print(f'    log : {texte}', flush=True)
            return
        try:
            data = json.loads(msg.payload)
        except ValueError:
            return
        if msg.topic == 'drone/telemetry':
            if self.afficher_mission and self.telemetry and data.get('mission') != self.telemetry.get('mission'):
                print(f"    état mission : {data.get('mission')}", flush=True)
            self.telemetry = data
        elif msg.topic == 'drone/position':
            self.position = data
        else:
            self.carte = data

    # --- Commandes (comme le dashboard) ---
    def commande(self, action, **data):
        self.client.publish('drone/cmd', json.dumps({'action': action, **data}))

    def set_mode(self, mode):
        self.commande('set_mode', mode=mode)

    def set_point(self, x, y):
        self.commande('set_point', x=float(x), y=float(y))

    # --- Lecture ---
    @property
    def mission(self):
        return (self.telemetry or {}).get('mission')

    @property
    def armed(self):
        return bool((self.telemetry or {}).get('armed'))

    def en_vol(self, tolerance=0.2):
        t = self.telemetry or {}
        return t.get('armed') and t.get('height') is not None and abs(t['height'] - TAKEOFF_ALT) < tolerance

    def point(self, genre, nom):
        """Point de la carte (drone/carte) ou None."""
        return next((p for p in self.carte or [] if p.get('type') == genre and p.get('nom') == str(nom)), None)

    def ecart_ekf_verite(self, decalage=(0.0, 0.0)):
        """Distance horizontale EKF - vérité Gazebo (m), EKF décalé de decalage (départ sans UWB) ; None sans les deux."""
        p = self.position or {}
        if not p.get('ekf') or not p.get('verite'):
            return None
        return math.dist((p['ekf']['x'] + decalage[0], p['ekf']['y'] + decalage[1]),
                         (p['verite']['x'], p['verite']['y']))

    def heure_log(self, motif, depuis=0):
        """time.monotonic() de réception du premier log contenant motif (à partir de l'indice depuis), ou None."""
        return next((h for h, log in zip(self.heures_logs[depuis:], self.logs[depuis:]) if motif in log), None)

    def log_recu(self, motif, depuis=0):
        return self.heure_log(motif, depuis) is not None

    def attendre(self, condition, timeout):
        return attendre(condition, timeout)

    # --- Fin ---
    def atterrir_si_arme(self, timeout=60):
        """Test interrompu en vol : ne jamais laisser le drone voler."""
        if self.armed:
            print('    test interrompu drone armé : atterrissage', flush=True)
            self.set_mode('land')
            attendre(lambda: not self.armed, timeout)

    def fermer(self):
        self.client.loop_stop()
