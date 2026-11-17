"""Code partagé des tests de simulation (tests/simu/) : étapes OK/ÉCHEC, client MQTT du dashboard, outils.

Lancés dans un conteneur de la simu (companion ou drone), par tests/run.py ou à la main :
    docker compose exec companion bash -lc 'python3 tests/simu/<test>.py'
Le dossier du script est dans sys.path : `from commun import ...` suffit. Écrire un test : docs/tests.md.
"""
import json
import math
import os
import subprocess
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import paho.mqtt.client as mqtt

REPO = Path(__file__).resolve().parents[2]
SDF = REPO / 'sim' / 'worlds' / 'voliere.sdf'
BROKER = ('127.0.0.1', 1883)
TAKEOFF_ALT = 1.0   # = takeoff_altitude de drone.yaml
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
