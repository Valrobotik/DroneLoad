"""Liaison MQTT avec le dashboard du PC (ground/) et le robot sol ESP32.

Même protocole que le code 2026, donc rien ne change côté ground/ :
- drone/cmd (JSON {"action": ...}) : set_mode, set_class, set_hook, arm, disarm ;
- drone/system : reboot, restart_script, shutdown ;
- robot/ping : heartbeat de l'ESP32 (déconnecté après 4 s sans ping) ;
- publiés : drone/logs (texte) et drone/telemetry (JSON, toutes les secondes).

Les logs des nœuds du drone (/rosout) sont aussi renvoyés sur drone/logs.
"""
import json
import math
import os
import time

import paho.mqtt.client as mqtt
import psutil
import rclpy
from mavros_msgs.srv import CommandBool
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray

ESP_TIMEOUT = 4.0     # s sans ping avant de considérer l'ESP32 déconnecté
FORWARDED_NODES = {'main_node', 'vision', 'gstream', 'camera', 'servo'}   # logs renvoyés au PC


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
        self.create_subscription(BatteryState, '/mavros/battery', self._on_battery, qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, '/detections', self._on_detections, qos_profile_sensor_data)
        self.create_subscription(Log, '/rosout', self._on_rosout, 100)

        self.current_mode = 'attente'
        self.start_time = time.time()
        self.frames = 0               # images traitées par la vision depuis la dernière télémétrie
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

        self.create_timer(1.0, self._telemetry)

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
        payload = msg.payload.decode('utf-8')
        if msg.topic == 'robot/ping':
            self.last_esp_ping = time.time()
            if not self.esp_connected:
                self.esp_connected = True
                self.log_to_pc('ESP32 (robot sol) CONNECTÉ')
            return

        try:
            data = json.loads(payload)
            action = data.get('action')
        except (json.JSONDecodeError, AttributeError):
            self.log_to_pc(f'Erreur : message MQTT non JSON reçu : {payload}')
            return

        if msg.topic == 'drone/cmd':
            self._on_cmd(action, data)
        elif msg.topic == 'drone/system':
            self._on_system(action)

    def _on_cmd(self, action, data):
        if action == 'set_mode':
            self.current_mode = data.get('mode')
            self.mode_pub.publish(String(data=self.current_mode))
            self.log_to_pc(f'Mode changé : {self.current_mode}')
        elif action == 'set_class':
            self.class_pub.publish(String(data=data.get('class_name')))
            self.log_to_pc(f'Cible IA (YOLO) changée : {data.get("class_name")}')
        elif action == 'set_hook':
            state = data.get('state')
            self.hook_pub.publish(Bool(data=state == 'hook_open'))
            self.log_to_pc(f'Action crochet demandée : {state}')
        elif action in ('arm', 'disarm'):
            self.log_to_pc(f'Commande de {"" if action == "arm" else "dés"}armement reçue !')
            if self.arming.service_is_ready():
                self.arming.call_async(CommandBool.Request(value=action == 'arm'))
            else:
                self.log_to_pc('MAVROS indisponible : commande ignorée')

    def _on_system(self, action):
        if action == 'reboot':
            self.log_to_pc('!!! REDÉMARRAGE DU RASPBERRY PI !!!')
            os.system('sudo reboot')
        elif action == 'restart_script':
            self.log_to_pc('Redémarrage du code du drone...')
            # Le launch s'arrête avec ce nœud ; le service (systemd ou Docker) le relance
            time.sleep(0.5)
            os._exit(1)
        elif action == 'shutdown':
            self.log_to_pc('!!! EXTINCTION DU PI 5 !!!')
            time.sleep(0.5)   # laisse le temps au message de partir vers le PC
            os.system('sudo poweroff')

    # --- ROS ---
    def _on_battery(self, msg):
        self.batt_v = round(msg.voltage, 2)
        self.batt_pct = -1 if math.isnan(msg.percentage) else round(100 * msg.percentage)

    def _on_detections(self, msg):
        self.frames += 1

    def _on_rosout(self, msg):
        if msg.name in FORWARDED_NODES and msg.level >= Log.INFO:
            self.client.publish('drone/logs', f'[{msg.name}] {msg.msg}')

    def _telemetry(self):
        now = time.time()
        if self.esp_connected and now - self.last_esp_ping > ESP_TIMEOUT:
            self.esp_connected = False
            self.log_to_pc('ESP32 (robot sol) DÉCONNECTÉ (timeout)')

        telemetry = {
            'fps': self.frames,
            'cpu_percent': psutil.cpu_percent(),
            'ram_percent': psutil.virtual_memory().percent,
            'flight_time_sec': int(now - self.start_time),
            'mode': self.current_mode,
            'batt_v': self.batt_v,
            'batt_pct': self.batt_pct,
            'esp_connected': self.esp_connected,
        }
        self.frames = 0
        self.client.publish('drone/telemetry', json.dumps(telemetry))

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
