import paho.mqtt.client as mqtt
import json
import time
import os
import sys

BROKER_IP = "192.168.31.113"

# Variables globales
telemetry_data = {
    "mode": "attente", "esp_connected": False, "fps": 0, 
    "cpu_percent": 0, "ram_percent": 0, "flight_time_sec": 0, 
    "batt_v": -1, "batt_pct": -1
}
logs_buffer = []
black_bg_state = False
is_first_connect = True  

def print_at(line, col, text):
    """ 
    CORRECTION MAC : Utilisation de \0337 (Save) et \0338 (Restore) au lieu de [s et [u.
    Cela fige le curseur de saisie de l'utilisateur sans jamais perturber l'input.
    """
    print(f"\0337\033[{line};{col}H{text}\033[K\0338", end="", flush=True)

def init_static_interface():
    """ Dessine la structure fixe de l'interface au tout début """
    os.system('clear')
    print("=======================================================================")
    print("                       PANNEAU DE CONTRÔLE DRONE                       ")
    print("=======================================================================")
    print(" [1] Mode ATTENTE      [2] Mode EPREUVE 1      [3] Mode EPREUVE 2")
    print(" [4] DÉCOLLAGE         [5] ATTERRISSAGE")
    print(" [O] OUVRIR CROCHET    [C] FERMER CROCHET")
    print(" [B] Toggle Fond Noir  [S] Restart Script      [R] Reboot Raspberry Pi")
    print(" [X] Éteindre le Pi    [Q] Quitter le Dashboard")
    print("=======================================================================")
    print(" MODE: ATTENTE    | ESP32: DISCNN | IA: 0  FPS") # Ligne 10 (décalée en raison des nouvelles lignes)
    print(" CPU: 0% | RAM: 0% | Chrono: 0s | Batterie: N/C") # Ligne 11
    print("==========================================[ DERNIERS LOGS DRONE ]======") # Ligne 12
    print(" -> En attente de logs...") # Ligne 13
    print("") 
    print("") 
    print("") 
    print("") 
    print("") 
    print("") 
    print("=======================================================================") # Ligne 20
    print("> ", end="", flush=True)

def update_telemetry_ui():
    """ Met à jour uniquement les lignes 8 et 9 (Télémétrie) """
    esp_status = "OK" if telemetry_data["esp_connected"] else "DISCNN"
    batt_str = f"{telemetry_data['batt_v']}V ({telemetry_data['batt_pct']}/%)" if telemetry_data['batt_v'] != -1 else "N/C"
    
    line10 = f" MODE: {telemetry_data['mode'].upper():<10} | ESP32: {esp_status:<6} | IA: {telemetry_data['fps']:<2} FPS"
    line11 = f" CPU: {telemetry_data['cpu_percent']}% | RAM: {telemetry_data['ram_percent']}% | Chrono: {telemetry_data['flight_time_sec']}s | Batterie: {batt_str}"
    
    print_at(10, 2, line10)
    print_at(11, 2, line11)

def update_logs_ui():
    """ Met à jour uniquement la zone des logs (Lignes 11 à 16) """
    current_logs = logs_buffer[-6:]
    for i, log in enumerate(current_logs):
        print_at(13 + i, 5, f"{log:<62}")

def log_local(message):        
    timestamp = time.strftime("%H:%M:%S")
    logs_buffer.append(f"[{timestamp}] {message}")
    update_logs_ui()

# --- CONFIGURATION MQTT ---
def on_connect(client, userdata, flags, reason_code, properties):
    global is_first_connect
    if is_first_connect:
        init_static_interface()
        log_local(f"Connecté au Broker MQTT ({BROKER_IP})")
        is_first_connect = False
        
    client.subscribe("drone/logs")
    client.subscribe("drone/telemetry")

def on_message(client, userdata, msg):
    global telemetry_data
    topic = msg.topic
    payload = msg.payload.decode("utf-8")
    
    if topic == "drone/logs":
        log_local(payload)
        
    elif topic == "drone/telemetry":
        try:
            telemetry_data = json.loads(payload)
            update_telemetry_ui()
        except Exception:
            pass

# Connexion initiale
client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "PC_Dashboard")
client.on_connect = on_connect
client.on_message = on_message
client.connect(BROKER_IP, 1883, 60)
client.loop_start()

time.sleep(0.2)

# --- BOUCLE PRINCIPALE DES COMMANDES ---
while True:
    try:
        # CORRECTION : On supprime le repositionnement de force qui cassait la frappe en boucle
        cmd = input().strip().upper()
    except (KeyboardInterrupt, EOFError):
        break
    
    # Nettoie la ligne de saisie de l'écran après chaque validation
    print("\033[21;3H\033[K", end="", flush=True)
    
    if cmd == '1':
        client.publish("drone/cmd", json.dumps({"action": "set_mode", "mode": "attente"}))
    elif cmd == '2':
        client.publish("drone/cmd", json.dumps({"action": "set_mode", "mode": "epreuve1"}))
    elif cmd == '3':
        client.publish("drone/cmd", json.dumps({"action": "set_mode", "mode": "epreuve2"}))
    elif cmd == '4':
        client.publish("drone/cmd", json.dumps({"action": "set_mode", "mode": "takeoff"}))
    elif cmd == '5':
        client.publish("drone/cmd", json.dumps({"action": "set_mode", "mode": "land"}))
    elif cmd == 'O':
        client.publish("drone/cmd", json.dumps({"action": "set_hook", "state": "hook_open"}))
    elif cmd == 'C':
        client.publish("drone/cmd", json.dumps({"action": "set_hook", "state": "hook_close"}))
    elif cmd == 'B':
        black_bg_state = not black_bg_state
        client.publish("drone/cmd", json.dumps({"action": "video_bg", "black": black_bg_state}))
        log_local(f"Commande Fond Noir : {black_bg_state}")
    elif cmd == 'S':
        client.publish("drone/system", json.dumps({"action": "restart_script"}))
        log_local("Demande de redémarrage du script Pi 5...")
    elif cmd == 'R':
        print("\033[21;1H\033[K> Confirmer REBOOT du Pi (O/N) ? ", end="", flush=True)
        if input().strip().upper() == 'O':
            client.publish("drone/system", json.dumps({"action": "reboot"}))
        print("\033[21;1H\033[K> ", end="", flush=True) 
    elif cmd == 'X':
        print("\033[21;1H\033[K> !!! CONFIRMER EXTINCTION PI 5 (O/N) ? ", end="", flush=True)
        if input().strip().upper() == 'O':
            client.publish("drone/system", json.dumps({"action": "shutdown"}))
            time.sleep(1)
            break
        print("\033[21;1H\033[K> ", end="", flush=True)
    elif cmd == 'Q':
        break

client.loop_stop()
print("\033[22;1H\nDashboard fermé.\n")