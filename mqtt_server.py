import paho.mqtt.client as mqtt
import json
import psutil
import time
import os


class MqttManager:
    def __init__(self, hw_manager, drone_ctrl):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "Drone_Main")
        self.hw = hw_manager
        self.drone = drone_ctrl  # On a besoin d'accéder à MAVLink pour la batterie et l'armement

        # Variables d'état
        self.current_mode = "attente"
        self.hook_action = None
        self.target_class = None
        self.black_bg = False

        # Pour les statistiques
        self.start_time = time.time()
        self.fps_counter = 0
        self.last_fps_time = time.time()
        self.current_fps = 0

        # Suivi de l'ESP32
        self.last_esp_ping = 0.0  # Timestamp du dernier signe de vie de l'ESP32
        self.esp_connected = False

        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def start(self, broker_ip="127.0.0.1", port=1883):
        print("Connexion MQTT...")
        self.client.connect(broker_ip, port, 60)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, reason_code, properties):
        self.log_to_pc("Drone connecté au broker MQTT (Prêt)")
        client.subscribe("drone/cmd")
        client.subscribe("drone/system")

        #ping esp32
        client.subscribe("robot/ping")
        self.log_to_pc("Drone Pi 5 en ligne et prêt.")

    def log_to_pc(self, message):
        """ Envoie un texte (façon 'print') vers le PC """
        print(f"[LOG] {message}")  # Affiche aussi dans le terminal du Pi
        self.client.publish("drone/logs", message)

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode("utf-8")

        # ping ESP32 
        if topic == "robot/ping":
            self.last_esp_ping = time.time()
            if not self.esp_connected:
                self.esp_connected = True
                self.log_to_pc("ESP32 (Robot Sol) CONNECTÉ")
            return
        
    
        try:
            data = json.loads(payload)
        except json.JSONDecodeError:
            self.log_to_pc(f"Erreur: Message MQTT non-JSON reçu: {payload}")
            return

        action = data.get("action")

        if topic == "drone/cmd":
            if action == "set_mode":
                self.current_mode = data.get("mode")
                self.log_to_pc(f"Mode changé : {self.current_mode}")
            elif action == "set_hook":
                self.hook_action = data.get("state")
                self.log_to_pc(f"Action crochet demandée : {self.hook_action}")
            elif action == "video_bg":
                self.black_bg = data.get("black", False)
                self.log_to_pc(f"Vidéo fond noir : {self.black_bg}")
            elif action == "hook":
                self.hw.set_hook(data.get("open", True))
            elif action == "set_class": # <--- NOUVELLE ACTION
                self.target_class = data.get("class_name")
                self.log_to_pc(f"Cible IA (YOLO) changée : {self.target_class}")
            elif action == "arm":
                self.log_to_pc("Commande d'armement reçue !")
                # Remarque : Idéalement self.drone.arm()
                self.drone.master.arducopter_arm()
            elif action == "disarm":
                self.log_to_pc("Commande de désarmement reçue !")
                self.drone.master.arducopter_disarm()

        elif topic == "drone/system":
            if action == "reboot":
                self.log_to_pc("!!! REDÉMARRAGE DU RASPBERRY PI !!!")
                os.system("sudo reboot")
            elif action == "restart_script":
                self.log_to_pc("Redémarrage du script Python...")
                # Quitter proprement permet à un service Systemd de le relancer
                os._exit(1)
            elif action == "shutdown":
                self.log_to_pc("!!! EXTINCTION DU PI 5 !!!")
                time.sleep(0.5) # Laisse le temps au message de partir vers le PC
                os.system("sudo poweroff")

    def update_telemetry(self):
        """ Appelé à chaque tour de la boucle principale """
        self.fps_counter += 1
        now = time.time()

        # Si pas de ping esp32 depuis plus de 4 secondes = déconnecté
        if now - self.last_esp_ping > 4.0 and self.esp_connected:
            self.esp_connected = False
            self.log_to_pc("ESP32 (Robot Sol) DÉCONNECTÉ (Timeout)")

        # Envoi de la télémétrie toutes les secondes
        if now - self.last_fps_time >= 1.0:
            self.current_fps = self.fps_counter
            self.fps_counter = 0
            self.last_fps_time = now

            flight_time = int(now - self.start_time)

            # --- Lecture Batterie (Simulée ou via MAVLink) ---
            battery_voltage = -1
            battery_remaining = -1
            msg = self.drone.master.recv_match(type='SYS_STATUS', blocking=False)
            if msg:
                battery_voltage = msg.voltage_battery / 1000.0  # en Volts
                battery_remaining = msg.battery_remaining  # en %

            telemetry_data = {
                "fps": self.current_fps,
                "cpu_percent": psutil.cpu_percent(),
                "ram_percent": psutil.virtual_memory().percent,
                "flight_time_sec": flight_time,
                "mode": self.current_mode,
                "batt_v": battery_voltage,
                "batt_pct": battery_remaining,
                "esp_connected": self.esp_connected
            }

            self.client.publish("drone/telemetry", json.dumps(telemetry_data))