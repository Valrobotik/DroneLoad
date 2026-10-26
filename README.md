# DroneLoad

Code embarqué du drone de l'équipe Valrobotik pour le concours **DroneLoad** (INSA). Il couvre le vol autonome en intérieur, la reconnaissance d'objets par IA (YOLO), le dépôt d'objet avec un crochet et la collaboration avec un robot au sol (ESP32).

## Matériel

- Châssis F450, Pixhawk 6C mini sous ArduPilot (Copter), sans GPS.
- Raspberry Pi 5 sous Ubuntu, avec une caméra Pi (libcamera) montée sur un servo.
- MTF-01 (flux optique + lidar), lu via MAVLink comme rangefinder.
- Servos : GPIO 17 pour la caméra, GPIO 13 pour le crochet.

## Arborescence

```
droneload/           code embarqué (Raspberry Pi)
  main.py            boucle principale cadencée par la caméra
  config.py          IP, ports, MAVLink, caméra, chemin du modèle
  mavlink.py         DroneController (pymavlink) + DummyDroneController
  hardware.py        servos (gpiozero/lgpio)
  video.py           capture libcamera + renvoi RTP/JPEG vers le PC
  mqtt_server.py     commandes et télémétrie MQTT
  tasks/             épreuves (machines à états, non bloquantes)
  vision/            ArUco (pas encore branché)
ground/              PC de pilotage : dashboard MQTT + lanceur macOS
tools/               scripts autonomes (calibration caméra)
models/              modèle YOLO entraîné (best.pt + export NCNN)
```

## Installation

Sur le Raspberry Pi, il faut un broker MQTT local (mosquitto) sur 127.0.0.1:1883 et une version d'OpenCV compilée avec GStreamer :
```bash
python3 -m venv .venv --system-site-packages   # pour récupérer l'OpenCV système avec GStreamer
.venv/bin/pip install -r requirements.txt
```

Sur le PC de pilotage :
```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements-ground.txt
```

## Lancement

Sur le Pi, depuis la racine du repo :
```bash
python -m droneload                 # vol réel : MAVLink sur /dev/ttyAMA0 @ 921600
python -m droneload --sim           # SITL : udp:127.0.0.1:14550
python -m droneload --no-pixhawk    # vision seule, aucune commande MAVLink
python -m droneload --pc-ip 192.168.2.9   # change la destination du flux vidéo
```

Sur le PC (macOS) :
```bash
./ground/dashboard.command --broker <IP du Pi>
```
Le script ouvre un terminal avec le récepteur vidéo GStreamer (UDP 5000) et un autre avec le dashboard.

Les valeurs par défaut (IP, ports, résolution, modèle) sont dans `droneload/config.py`.

## Protocole MQTT

| Topic | Sens | Contenu |
|---|---|---|
| `drone/cmd` | PC → Pi | `{"action": "set_mode", "mode": "attente\|takeoff\|land\|epreuve1\|epreuve2"}`, `{"action": "set_class", "class_name": "cow"}`, `{"action": "set_hook", "state": "hook_open\|hook_close"}`, `{"action": "arm"}`, `{"action": "disarm"}` |
| `drone/system` | PC → Pi | `{"action": "reboot\|restart_script\|shutdown"}` |
| `drone/logs` | Pi → PC | texte |
| `drone/telemetry` | Pi → PC | JSON envoyé chaque seconde (fps, cpu, ram, mode, batterie, état de l'ESP32) |
| `robot/ping` | ESP32 → Pi | heartbeat ; l'ESP32 est considéré déconnecté après 4 s sans ping |

## Sécurité

- La commande `arm` **arme réellement les moteurs**.
- Toute modification de la logique de vol doit être validée en `--sim` avant un essai réel.
