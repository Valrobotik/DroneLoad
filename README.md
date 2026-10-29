# DroneLoad

Code embarqué du drone de l'équipe Valrobotik pour le concours **DroneLoad** (INSA). Il couvre le vol autonome en intérieur, la reconnaissance d'objets par IA (YOLO), le dépôt d'objet avec un crochet et la collaboration avec un robot au sol (ESP32).

## Matériel

- Châssis F450, Pixhawk 6C mini sous ArduPilot (Copter), sans GPS.
- Raspberry Pi 5 sous Ubuntu, avec une caméra Pi (libcamera) montée sur un servo.
- MTF-01 (flux optique + lidar), lu via MAVLink comme rangefinder.
- Servos : GPIO 17 pour la caméra, GPIO 13 pour le crochet.

## Arborescence

```
drone/               code embarqué (Raspberry Pi)
  main.py            boucle principale cadencée par la caméra
  config.py          IP, ports, MAVLink, caméra, chemin du modèle
  mavlink.py         DroneController (pymavlink) + DummyDroneController
  hardware.py        servos (gpiozero/lgpio)
  video.py           capture libcamera + renvoi RTP/JPEG vers le PC
  mqtt_server.py     commandes et télémétrie MQTT
  tasks/             épreuves (machines à états, non bloquantes)
  vision/            ArUco (pas encore branché)
tools/               scripts autonomes (calibration caméra)
ground/              PC de pilotage : dashboard MQTT + lanceur (macOS, Linux, Windows)
  requirements.txt
sim/                 simulation (Docker Compose) : monde Gazebo, modèle du drone, bridge, test de fumée
  docker/            Dockerfiles
config/              configuration des équipements
  ardupilot/         paramètres ArduPilot (communs réel/SITL + spécifiques SITL)
                     (à venir : elrs/, radio/)
install/             installation Ansible des ordinateurs (Pi, images Docker, PC Linux) : voir install/README.md
models/              modèle YOLO entraîné (best.pt + export NCNN)
docs/                documentation (simulation)
.github/             règles de contribution : contrôle des PR, hook et modèle de commit
```

## Installation

### Raspberry Pi

Sur une Pi sous Ubuntu Server 24.04 (arm64), tout s'installe avec Ansible :
```bash
git clone https://github.com/Valrobotik/DroneLoad.git    # ou en SSH : git@github.com:Valrobotik/DroneLoad.git
cd DroneLoad
./install/apply.sh
```
Cette commande installe ROS 2 et MAVROS, le broker MQTT, GStreamer et libcamera, ainsi que les paquets Python dans le venv `/opt/droneload_venv` (avec l'OpenCV système compilé avec GStreamer). On peut la relancer autant de fois qu'on veut : elle ne refait que ce qui manque.

### Ajouter un paquet

**On n'installe jamais rien à la main sur la Pi** (pas de `sudo apt install` ni de `pip install`) : ce serait perdu à la prochaine réinstallation de la carte SD. Tous les paquets sont listés dans un seul fichier, [install/packages.yml](install/packages.yml), rangés par machine : `pi`, `commun` (Pi + simu + PC Linux) et `sim`.

1. Ajouter une ligne sous la bonne machine, dans `apt` (paquet système) ou `pip` (paquet Python, version figée) :
   ```yaml
   pi:
     apt:
       - i2c-tools
     pip:
       - smbus2==0.5.1
   ```
2. Installer : sur la Pi, `./install/apply.sh` ; pour la simu, `cd sim && docker compose build`.
3. Committer `install/packages.yml` dans la même PR que le code qui utilise le paquet.

`python3 install/check.py` liste ce qui a été installé sans passer par ce fichier. Sur la Pi, il tourne toutes les heures et affiche les écarts dans le dashboard. Détails : [install/README.md](install/README.md).

### PC de pilotage

```bash
python3 -m venv .venv
.venv/bin/pip install -r ground/requirements.txt     # Windows : .venv\Scripts\pip
```

Le flux vidéo nécessite GStreamer (`gst-launch-1.0`) :

| OS | Installation |
|---|---|
| macOS | `brew install gstreamer` |
| Ubuntu / Debian | `sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-good` |
| Windows | installeur « runtime » MSVC 64 bits sur [gstreamer.freedesktop.org](https://gstreamer.freedesktop.org/download/), puis ajouter `C:\Program Files\gstreamer\1.0\msvc_x86_64\bin` au PATH |

Vérification : `gst-inspect-1.0 rtpjpegdepay` doit décrire l'élément.

## Lancement

Sur le Pi, depuis la racine du repo :
```bash
python -m drone                 # vol réel : MAVLink sur /dev/ttyAMA0 @ 921600
python -m drone --sim           # SITL : udp:127.0.0.1:14550
python -m drone --no-pixhawk    # vision seule, aucune commande MAVLink
python -m drone --pc-ip 192.168.2.9   # change la destination du flux vidéo
```

Sur le PC, depuis la racine du repo :
```bash
.venv/bin/python ground/launch.py --broker <IP du Pi>    # Windows : .venv\Scripts\python
```
Le script ouvre la fenêtre vidéo GStreamer (UDP 5000) et lance le dashboard dans le terminal. Sur macOS, `ground/dashboard.command` fait la même chose en double-clic.

Les valeurs par défaut (IP, ports, résolution, modèle) sont dans `drone/config.py`.

## Simulation

ArduPilot SITL + Gazebo + ROS 2 dans Docker, identique sur Mac et Linux. Voir [docs/simulation.md](docs/simulation.md).
```bash
cd sim
docker compose up -d
docker compose exec companion bash -lc 'python3 sim/scripts/smoke_test.py'
```

## Protocole MQTT

| Topic | Sens | Contenu |
|---|---|---|
| `drone/cmd` | PC → Pi | `{"action": "set_mode", "mode": "attente\|takeoff\|land\|epreuve1\|epreuve2"}`, `{"action": "set_class", "class_name": "cow"}`, `{"action": "set_hook", "state": "hook_open\|hook_close"}`, `{"action": "arm"}`, `{"action": "disarm"}` |
| `drone/system` | PC → Pi | `{"action": "reboot\|restart_script\|shutdown"}` |
| `drone/logs` | Pi → PC | texte |
| `drone/telemetry` | Pi → PC | JSON envoyé chaque seconde (fps, cpu, ram, mode, batterie, état de l'ESP32) |
| `robot/ping` | ESP32 → Pi | heartbeat ; l'ESP32 est considéré déconnecté après 4 s sans ping |

## Contribuer

Le travail est suivi dans les issues et le Project GitHub. Règles :

- **Une branche par issue**, nommée `<numéro>-titre`. Le plus simple est de la créer depuis l'issue (bouton « Create a branch ») ou avec :
  ```bash
  gh issue develop <numéro> --checkout
  ```
- **Pas de push direct sur `main`.** Tout passe par une pull request depuis la branche de l'issue, avec `Closes #<numéro>` dans la description. La PR doit être **validée par un autre membre** avant d'être fusionnée ; la fusion ferme l'issue.
- **Chaque commit dit ce qui fonctionne, ce qui ne fonctionne pas et comment il a été testé** :
  ```
  Décollage depuis le topic /vol/cmd

  Fonctionne : décollage et vol stationnaire à 1 m
  Ne fonctionne pas : l'atterrissage dérive de 20 cm
  Testé : simu
  ```
  `Testé` vaut `simu`, `réel`, `simu+réel` ou `non`. Si tout marche, écrire `Ne fonctionne pas : rien`.

À faire une fois après le clone, pour avoir le modèle de commit et la vérification avant chaque commit :
```bash
git config core.hooksPath .github/hooks
git config commit.template .github/gitmessage
```
**Dépendances** : un paquet (apt ou pip) nécessaire au drone ou à la simu s'ajoute dans `install/packages.yml`, dans la même PR que le code qui en a besoin. Jamais d'installation à la main sur la Pi (voir [install/README.md](install/README.md)).

Les mêmes règles sont vérifiées automatiquement sur chaque PR ; une PR qui ne les respecte pas ne peut pas être fusionnée.

Cycle d'une issue dans le Project : **Todo** → **In progress** (on commence) → PR ouverte → validation → **Done** (fermée par la fusion).

Pour les issues sans code (mécanique, rapport, tests, réunions) : on note le résultat dans l'issue et on passe son Status à **À valider**. C'est **un autre membre** qui vérifie, remplit « Validé par » et ferme l'issue.

## Sécurité

- La commande `arm` **arme réellement les moteurs**.
- Toute modification de la logique de vol doit être validée en simulation avant un essai réel.
