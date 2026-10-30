import paho.mqtt.client as mqtt
import json
import time
import os
import argparse

parser = argparse.ArgumentParser(description="Dashboard MQTT du drone")
parser.add_argument("--broker", default="192.168.2.3", help="IP du Raspberry Pi (broker MQTT)")
BROKER_IP = parser.parse_args().broker

# Variables globales
telemetry_data = {
    "mode": "attente", "esp_connected": False, "fps": 0, 
    "cpu_percent": 0, "ram_percent": 0, "flight_time_sec": 0, 
    "batt_v": -1, "batt_pct": -1
}
logs_buffer = []
is_first_connect = True  

def print_at(line, col, text):
    """ 
    CORRECTION MAC : Utilisation de \0337 (Save) et \0338 (Restore) au lieu de [s et [u.
    Cela fige le curseur de saisie de l'utilisateur sans jamais perturber l'input.
    """
    print(f"\0337\033[{line};{col}H{text}\033[K\0338", end="", flush=True)

def init_static_interface():
    os.system('cls' if os.name == 'nt' else 'clear')
    print("=======================================================================")
    print("                       PANNEAU DE CONTRÔLE DRONE                       ")
    print("=======================================================================")
    print(" [1] Mode ATTENTE      [2] Mode EPREUVE 1      [3] Mode EPREUVE 2")
    print(" [4] DÉCOLLAGE         [5] ATTERRISSAGE")
    print(" [O] OUVRIR CROCHET    [C] FERMER CROCHET")
    print(" CIBLES IA : [F] Fleur  [V] Vache (cow)  [D] Chien (dog)  [M] Mouton")
    print(" [S] Restart Script    [R] Reboot Pi           [P] Reboot Pixhawk")
    print(" [X] Éteindre le Pi    [Q] Quitter le Dashboard")
    print("=======================================================================")
    print(" MISSION: ?               | FCU: ?                 | H: ?   | CROCHET: ?") # Ligne 11
    print(" ESP32: OFF | CAM/IA: 0/0 FPS | CPU: 0% | RAM: 0% | Batt: N/C") # Ligne 12
    print("==========================================[ DERNIERS LOGS DRONE ]======") # Ligne 13
    print(" -> En attente de logs...") # Ligne 14
    print("") 
    print("") 
    print("") 
    print("") 
    print("") 
    print("=======================================================================") # Ligne 20
    print("> ", end="", flush=True) # Ligne 21

def update_telemetry_ui():
    """ Met à jour uniquement les lignes 11 et 12 (Télémétrie) """
    t = telemetry_data   # .get : un drone plus ancien n'envoie pas tous les champs
    esp_status = "OK" if t.get("esp_connected") else "OFF"
    batt_str = "N/C" if t.get("batt_v", -1) == -1 else f"{t['batt_v']}V" + (f" ({t['batt_pct']}%)" if t.get("batt_pct", -1) != -1 else "")
    # État réel de la mission (main_node), sinon le dernier mode demandé
    mission = (t.get("mission") or t.get("mode") or "?").upper()
    if "fcu_connected" not in t:
        fcu = "?"
    elif not t["fcu_connected"]:
        fcu = "DÉCONNECTÉ"
    else:
        fcu = f"{t.get('fcu_mode') or '?'} {'ARMÉ' if t.get('armed') else 'désarmé'}"
        if t.get('armed') and 'flight_time_sec' in t:   # chrono du vol en cours
            minutes, seconds = divmod(t['flight_time_sec'], 60)
            fcu += f" {minutes}:{seconds:02d}"
    height = f"{t['height']:.2f} m" if t.get("height") is not None else "?"
    # hook_ready : null = jamais reçu (drone plus ancien ou nœud servo pas encore lancé)
    crochet = {True: "OK", False: "HS"}.get(t.get("hook_ready"), "?")
    cam = t.get("cam_fps", "?")
    cpu = f"{t.get('cpu_percent', 0)}%" + (f" {t['cpu_temp']:.0f}°C" if t.get("cpu_temp") is not None else "")

    line11 = f" MISSION: {mission:<16} | FCU: {fcu:<16} | H: {height:<6} | CROCHET: {crochet}"
    line12 = f" ESP32: {esp_status:<3} | CAM/IA: {cam}/{t.get('fps', 0)} FPS | CPU: {cpu} | RAM: {t.get('ram_percent', 0)}% | Batt: {batt_str}"
    
    print_at(11, 2, line11)
    print_at(12, 2, line12)

def update_logs_ui():
    """ Met à jour uniquement la zone des logs (Lignes 11 à 16) """
    current_logs = logs_buffer[-6:]
    for i, log in enumerate(current_logs):
        print_at(14 + i, 5, f"{log:<62}")

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
        
    # --- Classes cibles IA ---
    elif cmd == 'F':
        client.publish("drone/cmd", json.dumps({"action": "set_class", "class_name": "flower"}))
    elif cmd == 'V':
        client.publish("drone/cmd", json.dumps({"action": "set_class", "class_name": "cow"}))
    elif cmd == 'D':
        client.publish("drone/cmd", json.dumps({"action": "set_class", "class_name": "dog"}))
    elif cmd == 'M':
        client.publish("drone/cmd", json.dumps({"action": "set_class", "class_name": "sheep"}))
    # ------------------------------------------

    elif cmd == 'S':
        print("\033[21;1H\033[K> Confirmer RESTART du code du drone (O/N) ? ", end="", flush=True)
        if input().strip().upper() == 'O':
            client.publish("drone/system", json.dumps({"action": "restart_script"}))
            log_local("Demande de redémarrage du script Pi 5...")
        print("\033[21;1H\033[K> ", end="", flush=True)
    elif cmd == 'R':
        print("\033[21;1H\033[K> Confirmer REBOOT du Pi (O/N) ? ", end="", flush=True)
        if input().strip().upper() == 'O':
            client.publish("drone/system", json.dumps({"action": "reboot"}))
        print("\033[21;1H\033[K> ", end="", flush=True) 
    elif cmd == 'P':
        print("\033[21;1H\033[K> Confirmer REBOOT de la Pixhawk (O/N) ? ", end="", flush=True)
        if input().strip().upper() == 'O':
            client.publish("drone/system", json.dumps({"action": "reboot_fcu"}))
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
