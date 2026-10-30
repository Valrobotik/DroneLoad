"""Liaison MQTT avec le dashboard du PC (ground/) et le robot sol ESP32.

Même protocole que le code 2026, donc rien ne change côté ground/ :
- drone/cmd (JSON {"action": ...}) : set_mode, set_class, set_hook, arm, disarm ;
- drone/system : reboot, restart_script, shutdown, reboot_fcu (Pixhawk) (refusés drone armé) ;
- robot/ping : heartbeat de l'ESP32 (déconnecté après 4 s sans ping) ;
- publiés : drone/logs (texte) et drone/telemetry (JSON, toutes les secondes).

Les logs des nœuds du drone (/rosout) sont aussi renvoyés sur drone/logs.

Bilan de démarrage sur drone/logs BILAN_DELAY après le lancement (Pixhawk, lidar, batterie, caméra,
vision, crochet, install/check.py sur la Pi) : un coup d'œil au dashboard dit si tout est reparti.
"""
import json
import math
import os
import subprocess
import threading
import time

from pathlib import Path

import paho.mqtt.client as mqtt
import psutil
import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandLong
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, CameraInfo, Range
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray

ESP_TIMEOUT = 4.0     # s sans ping avant de considérer l'ESP32 déconnecté
HEIGHT_MAX_AGE = 1.0  # s : au-delà, la hauteur lidar n'est plus affichée
HOOK_MAX_AGE = 2.5    # s sans /hook/ready (publié chaque seconde) : nœud servo arrêté
SYSTEM_DELAY = 0.5    # s laissées au message de partir vers le PC avant reboot, restart_script, shutdown
FORWARDED_NODES = {'main_node', 'vision', 'aruco', 'gstream', 'camera', 'servo'}   # logs renvoyés au PC
HOOK_STATES = {'hook_open': True, 'hook_close': False}
BILAN_DELAY = 20.0    # s après le lancement : Pixhawk connectée, caméra ouverte, modèle chargé
MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 246
REPO = Path(__file__).resolve().parents[3]   # --symlink-install : drone/<paquet>/<paquet>/ -> racine du repo


def cpu_temperature():
    """Température du CPU (°C), None si inconnue (hors Linux, conteneur sans capteur)."""
    sensors = getattr(psutil, 'sensors_temperatures', lambda: {})()
    for name in ('cpu_thermal', 'coretemp', 'k10temp'):   # Pi, puis PC Intel / AMD
        if sensors.get(name):
            return round(sensors[name][0].current, 1)
    return None


def check_install():
    """Résultat de install/check.py en quelques mots (Pi seulement : ailleurs, il signalerait l'environnement)."""
    if not Path('/boot/firmware/config.txt').exists():
        return 'install non vérifié'
    try:
        resultat = subprocess.run(['python3', str(REPO / 'install' / 'check.py')], capture_output=True, timeout=60)
    except (OSError, subprocess.TimeoutExpired):
        return 'INSTALL ?'
    return 'install OK' if resultat.returncode == 0 else 'ÉCARTS INSTALL (install/check.py)'


class MqttServer(Node):
    def __init__(self):
        super().__init__('mqtt_server')
        self.declare_parameter('broker_ip', '127.0.0.1')
        self.declare_parameter('broker_port', 1883)

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.mode_pub = self.create_publisher(String, '/mission/mode', latched)
        self.class_pub = self.create_publisher(String, '/mission/target_class', latched)
        self.hook_pub = self.create_publisher(Bool, '/hook/cmd', 10)
        self.arming = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.command = self.create_client(CommandLong, '/mavros/cmd/command')
        self.create_subscription(BatteryState, '/mavros/battery', self._on_battery, qos_profile_sensor_data)
        self.create_subscription(State, '/mavros/state', self._on_state, 10)
        self.create_subscription(Range, '/mavros/rangefinder/rangefinder', self._on_range,
                                 qos_profile_sensor_data)
        self.create_subscription(String, '/mission/state', self._on_mission_state, latched)
        self.create_subscription(Detection2DArray, '/detections', self._on_detections, qos_profile_sensor_data)
        # camera_info accompagne chaque image (camera sur la Pi, bridge en simu) : compte les images sans les recevoir
        self.create_subscription(CameraInfo, '/camera/camera_info', self._on_camera_info, qos_profile_sensor_data)
        self.create_subscription(Bool, '/hook/ready', self._on_hook_ready, 10)
        self.create_subscription(Log, '/rosout', self._on_rosout, 100)

        self.current_mode = 'attente'
        self.mission_state = None     # état réel publié par main_node (<mode>:<état>)
        self.fcu = State()            # connected, armed, mode de l'autopilote (MAVROS)
        self.height = None
        self.height_time = 0.0
        self.flight_start = None      # time.monotonic() de l'armement, None drone désarmé
        self.flight_time = 0.0        # durée du dernier vol terminé (s)
        self.frames = 0               # images traitées par la vision depuis la dernière télémétrie
        self.camera_frames = 0        # images de la caméra depuis la dernière télémétrie
        self.hook_ready = None
        self.hook_ready_time = 0.0
        self.batt_v = -1
        self.batt_pct = -1
        self.last_esp_ping = 0.0
        self.esp_connected = False

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, 'Drone_Main')
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        broker = self.get_parameter('broker_ip').value
        port = self.get_parameter('broker_port').value
        self.get_logger().info(f'Connexion MQTT à {broker}:{port}...')
        self.client.connect_async(broker, port, 60)   # reconnexion automatique si le broker tombe
        self.client.loop_start()

        self.telemetry = {}           # dernière télémétrie envoyée (reprise par le bilan de démarrage)
        self.create_timer(1.0, self._telemetry)
        # Temps réel, pas un timer ROS : en simu, l'horloge simulée saute au 1er /clock et le déclencherait aussitôt
        self.bilan_at = time.monotonic() + BILAN_DELAY

    def log_to_pc(self, message):
        """Journal local et envoi vers le dashboard."""
        self.get_logger().info(message)
        self.client.publish('drone/logs', message)

    # --- MQTT (thread de paho) ---
    def _on_connect(self, client, userdata, flags, reason_code, properties):
        client.subscribe('drone/cmd')
        client.subscribe('drone/system')
        client.subscribe('robot/ping')
        self.log_to_pc('Drone connecté au broker MQTT (prêt)')

    def _on_message(self, client, userdata, msg):
        # Une exception ici arrêterait la boucle de paho : plus aucune commande ne serait reçue
        try:
            self._handle_message(msg)
        except Exception as e:
            self.log_to_pc(f'Erreur sur le message MQTT {msg.topic} : {e!r}')

    def _handle_message(self, msg):
        if msg.topic == 'robot/ping':
            self.last_esp_ping = time.time()
            if not self.esp_connected:
                self.esp_connected = True
                self.log_to_pc('ESP32 (robot sol) CONNECTÉ')
            return

        payload = msg.payload.decode('utf-8', errors='replace')
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            data = None
        if not isinstance(data, dict):
            self.log_to_pc(f'Erreur : message MQTT non JSON reçu : {payload}')
            return

        action = data.get('action')
        if msg.topic == 'drone/cmd':
            self._on_cmd(action, data)
        elif msg.topic == 'drone/system':
            self._on_system(action)

    def _on_cmd(self, action, data):
        if action == 'set_mode':
            mode = data.get('mode')
            if not isinstance(mode, str) or not mode:
                self.log_to_pc(f'Erreur : set_mode sans mode valide : {data}')
                return
            self.current_mode = mode
            self.mode_pub.publish(String(data=mode))
            self.log_to_pc(f'Mode changé : {mode}')
        elif action == 'set_class':
            class_name = data.get('class_name')
            if not isinstance(class_name, str) or not class_name:
                self.log_to_pc(f'Erreur : set_class sans class_name valide : {data}')
                return
            self.class_pub.publish(String(data=class_name))
            self.log_to_pc(f'Cible IA (YOLO) changée : {class_name}')
        elif action == 'set_hook':
            state = data.get('state')
            if state not in HOOK_STATES:
                self.log_to_pc(f'Erreur : set_hook avec un état inconnu : {state}')
                return
            self.hook_pub.publish(Bool(data=HOOK_STATES[state]))
            self.log_to_pc(f'Action crochet demandée : {state}')
        elif action in ('arm', 'disarm'):
            self.log_to_pc(f'Commande de {"" if action == "arm" else "dés"}armement reçue !')
            if self.arming.service_is_ready():
                self.arming.call_async(CommandBool.Request(value=action == 'arm'))
            else:
                self.log_to_pc('MAVROS indisponible : commande ignorée')
        else:
            self.log_to_pc(f'Erreur : commande inconnue : {action}')

    def _on_system(self, action):
        commands = {
            'reboot': ('!!! REDÉMARRAGE DU RASPBERRY PI !!!', lambda: os.system('sudo reboot')),
            # Le launch s'arrête avec ce nœud ; le service (systemd ou Docker) le relance
            'restart_script': ('Redémarrage du code du drone...', lambda: os._exit(1)),
            'shutdown': ('!!! EXTINCTION DU PI 5 !!!', lambda: os.system('sudo poweroff')),
            # La Pixhawk a sa propre alimentation : redémarrer la Pi ne la redémarre pas
            'reboot_fcu': ('Redémarrage de la Pixhawk...', self._reboot_fcu),
        }
        if action not in commands:
            self.log_to_pc(f'Erreur : commande système inconnue : {action}')
            return
        # En vol, ces commandes couperaient tout le code du drone (MAVROS compris)
        if self.fcu.armed:
            self.log_to_pc(f'Refusé : {action} impossible drone armé (atterrir d\'abord)')
            return
        message, run = commands[action]
        self.log_to_pc(message)
        # Hors du thread de paho, qui doit rester libre pour envoyer le message vers le PC
        threading.Timer(SYSTEM_DELAY, run).start()

    def _reboot_fcu(self):
        if not self.command.service_is_ready():
            self.log_to_pc('MAVROS indisponible : redémarrage de la Pixhawk impossible')
            return
        # param1 = 1 : redémarrer l'autopilote ; MAVROS se reconnecte seul quand elle revient
        future = self.command.call_async(CommandLong.Request(command=MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, param1=1.0))
        future.add_done_callback(self._on_reboot_fcu_done)

    def _on_reboot_fcu_done(self, future):
        response = future.result() if future.exception() is None else None
        if response is None:
            self.log_to_pc(f'Pixhawk : pas de réponse au redémarrage ({future.exception()!r})')
        elif not response.success:
            self.log_to_pc(f'Pixhawk : redémarrage refusé (résultat MAVLink {response.result})')
        else:
            self.log_to_pc('Pixhawk : redémarrage accepté, reconnexion de MAVROS dans quelques secondes')

    def _bilan(self):
        """Résumé de l'état du drone une fois le démarrage fini (2 lignes, largeur du dashboard)."""
        t = self.telemetry
        fcu = (f"FCU OK {self.fcu.mode or '?'}" if self.fcu.connected else 'FCU DÉCONNECTÉE')
        hauteur = f"lidar {t['height']:.2f} m" if t.get('height') is not None else 'LIDAR MUET'
        batterie = f"{t['batt_v']} V" if t.get('batt_v', -1) != -1 else 'BATTERIE ?'
        self.log_to_pc(f'Démarrage : {fcu}, {hauteur}, {batterie}')
        camera = f"caméra {t.get('cam_fps', 0)}/s" if t.get('cam_fps') else 'CAMÉRA MUETTE'
        vision = f"IA {t.get('fps', 0)}/s" if t.get('fps') else 'IA MUETTE'
        crochet = 'crochet OK' if t.get('hook_ready') else 'CROCHET HS'
        # Hors du thread de ROS : check.py prend quelques secondes
        threading.Thread(target=lambda: self.log_to_pc(f'Démarrage : {camera}, {vision}, {crochet}, {check_install()}'),
                         daemon=True).start()

    # --- ROS ---
    def _on_battery(self, msg):
        self.batt_v = round(msg.voltage, 2)
        self.batt_pct = -1 if math.isnan(msg.percentage) else round(100 * msg.percentage)

    def _on_state(self, msg):
        # Temps de vol = temps armé : remis à zéro à chaque armement, figé au désarmement
        now = time.monotonic()
        if msg.armed and self.flight_start is None:
            self.flight_start = now
        elif not msg.armed and self.flight_start is not None:
            self.flight_time = now - self.flight_start
            self.flight_start = None
            minutes, seconds = divmod(round(self.flight_time), 60)
            self.log_to_pc(f'Vol terminé : {minutes} min {seconds:02d} s')
        self.fcu = msg

    def _on_range(self, msg):
        if math.isfinite(msg.range):
            self.height = msg.range
            self.height_time = time.monotonic()

    def _on_mission_state(self, msg):
        self.mission_state = msg.data

    def _on_detections(self, msg):
        self.frames += 1

    def _on_camera_info(self, msg):
        self.camera_frames += 1

    def _on_hook_ready(self, msg):
        self.hook_ready = msg.data
        self.hook_ready_time = time.monotonic()

    def _on_rosout(self, msg):
        if msg.name in FORWARDED_NODES and msg.level >= Log.INFO:
            self.client.publish('drone/logs', f'[{msg.name}] {msg.msg}')

    def flight_seconds(self):
        if self.flight_start is not None:
            return time.monotonic() - self.flight_start
        return self.flight_time

    def _telemetry(self):
        now = time.time()
        if self.esp_connected and now - self.last_esp_ping > ESP_TIMEOUT:
            self.esp_connected = False
            self.log_to_pc('ESP32 (robot sol) DÉCONNECTÉ (timeout)')

        fresh = time.monotonic() - self.height_time < HEIGHT_MAX_AGE
        hook_alive = time.monotonic() - self.hook_ready_time < HOOK_MAX_AGE
        telemetry = {
            'fps': self.frames,
            'cam_fps': self.camera_frames,
            'hook_ready': self.hook_ready and hook_alive,   # false : GPIO indisponible ou nœud servo arrêté
            'cpu_percent': psutil.cpu_percent(),
            'cpu_temp': cpu_temperature(),
            'ram_percent': psutil.virtual_memory().percent,
            'flight_time_sec': int(self.flight_seconds()),   # vol en cours, sinon dernier vol
            'mode': self.current_mode,          # dernier mode demandé
            'mission': self.mission_state,      # état réel de main_node, ex. takeoff:MONTEE
            'fcu_connected': self.fcu.connected,
            'fcu_mode': self.fcu.mode,
            'armed': self.fcu.armed,
            'height': round(self.height, 2) if fresh and self.height is not None else None,
            'batt_v': self.batt_v,
            'batt_pct': self.batt_pct,
            'esp_connected': self.esp_connected,
        }
        self.telemetry = telemetry
        self.frames = 0
        self.camera_frames = 0
        self.client.publish('drone/telemetry', json.dumps(telemetry))
        if self.bilan_at is not None and time.monotonic() >= self.bilan_at:
            self.bilan_at = None
            self._bilan()

    def destroy_node(self):
        self.client.loop_stop()
        self.client.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = MqttServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
