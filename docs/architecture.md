# Architecture ROS 2 du drone

Le code embarqué est découpé en **nœuds ROS 2** (Jazzy) qui échangent des messages sur des topics. Le lien avec la Pixhawk passe par **MAVROS**, et celui avec le PC de pilotage et le robot sol (ESP32) par **MQTT**, qui garde le protocole de 2026. Le même code tourne sur la Pi et dans la simulation : seules les sources de quelques topics changent (voir [simulation.md](simulation.md)).

## Paquets

Tout est dans `drone/`, qui est un workspace colcon avec un paquet par rôle :

```
drone/
├── droneload_bringup/            # lancement et paramètres (sans code)
│   ├── launch/drone.launch.py    # lance tout
│   ├── launch/mavros.launch.py   # MAVROS, seule configuration (drone et simu)
│   └── config/drone.yaml         # paramètres de chaque nœud
├── droneload_vol/
│   └── droneload_vol/
│       ├── main_node.py          # nœud principal : modes, appel des missions
│       ├── mavros.py             # MavrosDrone : commandes et état de l'autopilote, sans bloquer
│       └── missions/             # décollage, atterrissage, épreuve 1, épreuve 2
├── droneload_vision/             # traitement d'image seulement (aucun matériel, testable partout)
│   └── droneload_vision/
│       ├── vision.py             # détection YOLO
│       └── aruco.py              # ArUco : identifiant et pose 3D (calibration de /camera/camera_info)
└── droneload_hardware/           # entrées/sorties du drone : capteurs, actionneurs, liaisons au sol
    └── droneload_hardware/
        ├── camera.py             # caméra Module 3 (Pi seulement)
        ├── servo.py              # servo du crochet (GPIO 13)
        ├── gstream.py            # flux vidéo vers le PC
        └── mqtt_server.py        # dashboard et ESP32 (MQTT)
```

Chaque paquet contient aussi `package.xml` (dépendances ROS), `setup.py` (liste des nœuds exécutables) et `setup.cfg`. Le code se trouve dans `<paquet>/<paquet>/`, car ament_python impose ce double niveau.

## Nœuds et topics

```mermaid
flowchart LR
  subgraph PC[PC de pilotage]
    dash[dashboard]
    vid[récepteur vidéo]
  end
  esp[ESP32 robot sol]
  broker((broker MQTT))
  subgraph Pi[Raspberry Pi 5]
    mqtt[mqtt_server]
    main[main_node]
    cam[camera]
    vis[vision]
    aru[aruco]
    gst[gstream]
    servo[servo]
    mavros[MAVROS]
  end
  px[Pixhawk]

  dash <--> broker
  esp <--> broker
  broker <--> mqtt
  mqtt -- /mission/mode<br>/mission/target_class --> main
  mqtt -- /hook/cmd --> servo
  main -- /hook/cmd --> servo
  cam -- /camera/image_raw --> vis
  cam -- /camera/image_raw<br>/camera/camera_info --> aru
  vis -- /detections --> main
  vis -- /camera/annotated --> gst
  gst -- RTP/JPEG UDP 5000 --> vid
  main <-- /mavros/... --> mavros
  mavros <-- série 921600 --> px
```

| Topic | Type | Publié par | Lu par |
|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | `camera` (Pi), `bridge` (simu) | `vision`, `aruco` |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` : calibration (`config/camera/module3_640x480.yaml` sur la Pi, caméra Gazebo de même focale en simu) | `camera` (Pi), `bridge` (simu) | `aruco` |
| `/detections` | `vision_msgs/Detection2DArray` (`class_id` = nom de la classe) | `vision` | `main_node`, `mqtt_server` (FPS) |
| `/camera/annotated` | `sensor_msgs/Image` | `vision` | `gstream` |
| `/aruco/detections` | `vision_msgs/Detection3DArray` : un `Detection3D` par marqueur, `class_id` = identifiant, `results[0].pose.pose` = pose du marqueur en m dans le repère optique de la caméra (x à droite, y vers le bas, z dans l'axe : la distance au sol quand le drone est à plat) | `aruco` | (missions à venir, #15) |
| `/aruco/annotated` | `sensor_msgs/Image` : marqueurs, axes et position en cm (débogage, Foxglove) | `aruco` | — |
| `/mission/mode` | `std_msgs/String` : `attente`, `takeoff`, `land`, `epreuve1`, `epreuve2` | `mqtt_server` | `main_node` |
| `/mission/target_class` | `std_msgs/String` | `mqtt_server` | `main_node`, `vision` |
| `/mission/state` | `std_msgs/String` : `<mode>:<état>`, par ex. `takeoff:MONTEE` | `main_node` | (journal, Foxglove) |
| `/hook/cmd` | `std_msgs/Bool` (true = ouvert) | `mqtt_server`, `main_node` | `servo` |
| `/hook/ready` | `std_msgs/Bool`, chaque seconde : true si le crochet est utilisable (GPIO ouvert, ou mock) | `servo` | `mqtt_server` |
| `/mavros/state`, `/mavros/rangefinder/rangefinder` | MAVROS | MAVROS | `main_node` |
| `/mavros/battery` | `sensor_msgs/BatteryState` | MAVROS | `mqtt_server` |

`/mission/mode`, `/mission/target_class` et `/mission/state` sont en QoS *transient local* : un nœud qui (re)démarre reçoit la dernière valeur. Les images et les détections sont publiées en QoS capteur (*best effort*, file de 5). `vision` et `aruco` s'abonnent à `/camera/image_raw` avec une file d'une seule image (`LATEST_IMAGE`) : quand le traitement est plus lent que la caméra, ils reçoivent la plus récente et non la plus ancienne des images en attente (mesuré en simu avec les limites de la Pi : détections vieilles de 1,5 s avec la file de 5, 0,45 s avec la file de 1, soit le temps d'inférence).

Commandes de l'autopilote, toutes envoyées par `MavrosDrone` sans attendre la réponse :

| Action | MAVROS |
|---|---|
| mode GUIDED / LAND | service `/mavros/set_mode` |
| armement, désarmement | service `/mavros/cmd/arming` |
| décollage | service `/mavros/cmd/takeoff` |
| vitesse dans le repère du drone | topic `/mavros/setpoint_raw/local`, `FRAME_BODY_NED` |
| origine EKF (sans GPS, requise par GUIDED) | topic `/mavros/global_position/set_gp_origin` |
| flux lidar (RANGEFINDER) et batterie (SYS_STATUS), pas envoyés par défaut par ArduPilot ; redemandés si le lidar se tait 1 s (autopilote redémarré) | service `/mavros/set_message_interval` |

L'altitude utilisée par les missions est la distance lidar brute du MTF-01 (`/mavros/rangefinder/rangefinder`). `get_current_alt_brute()` renvoie `None` si la dernière mesure a plus de 0,5 s ou sort de la plage du lidar (non finie, hors de 0,05 à 8 m, les bornes `RNGFND1_MIN/MAX` de `drone.parm`) : une mission ne doit jamais deviner une hauteur, elle attend ou s'arrête. (La Pixhawk, elle, lit le lidar directement et tient sa hauteur même si la Pi ne reçoit plus le flux.)

## Nœud principal et missions

`main_node` tourne à 20 Hz. À chaque tick, il fait avancer d'un pas la mission du mode courant :

| Mode | Mission | États |
|---|---|---|
| `attente` | vol stationnaire (vitesse nulle, seulement si le drone est armé en GUIDED) | `ATTENTE` |
| `takeoff` | `Decollage` : GUIDED, armement, décollage à 1 m (ordre renvoyé toutes les 5 s tant que le drone est sous 0,3 m), stationnaire ; `ECHEC` si GUIDED/armement refusés 60 s ou hauteur non atteinte en 30 s (le drone reste en stationnaire, l'opérateur fait `land`) | `GUIDED` → `ARMEMENT` → `MONTEE` → `STATIONNAIRE` (ou `ECHEC`) |
| `land` | `Atterrissage` : mode LAND jusqu'au désarmement | `LAND` → `DESCENTE` → `POSE` |
| `epreuve1` | `Epreuve1` (drone en stationnaire) : cible validée si seule la classe demandée est détectée sur ≥ 80 % des images en 2 s, puis le crochet s'ouvre ; `SANS_VISION` (signalé) si aucune image analysée sur une fenêtre de 2 s, retour en `RECHERCHE` quand les images reviennent | `RECHERCHE` → `TROUVEE` (ou `SANS_VISION`) |
| `epreuve2` | `run_epreuve2` : se tourne vers la vache et avance tant qu'elle est centrée | `SUIVI` |

Une mission ne bloque jamais : elle envoie une demande, puis vérifie le résultat aux ticks suivants. Elle réessaie toutes les secondes si la demande est refusée. À chaque demande de mode, même identique au mode en cours (relancer un `takeoff` en `ECHEC`), les missions sont remises à zéro et, sauf en `takeoff`, une vitesse nulle est envoyée tout de suite : sinon ArduPilot garderait la consigne de vitesse du mode précédent jusqu'à `GUID_TIMEOUT`. Une erreur dans une mission est journalisée et met le drone en stationnaire, sans arrêter le nœud.

## Protocole MQTT

Il est inchangé depuis 2026 : `ground/dashboard.py` et l'ESP32 n'ont rien à modifier.

| Topic | Sens | Contenu |
|---|---|---|
| `drone/cmd` | PC → drone | JSON `{"action": ...}` : `set_mode` (`mode`), `set_class` (`class_name`), `set_hook` (`state` : `hook_open` / `hook_close`), `arm`, `disarm` |
| `drone/system` | PC → drone | `reboot`, `restart_script`, `shutdown`, `reboot_fcu` (redémarre la Pixhawk : `MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN` par `/mavros/cmd/command`) ; refusés si le drone est armé |
| `drone/logs` | drone → PC | texte : messages de `mqtt_server` et logs des autres nœuds |
| `drone/telemetry` | drone → PC | JSON chaque seconde : `fps` (images analysées par la vision), `cam_fps` (images de la caméra, comptées sur `/camera/camera_info`), `hook_ready` (crochet utilisable ; `false` aussi si `servo` ne publie plus depuis 2,5 s, `null` si jamais reçu), `cpu_percent`, `cpu_temp` (°C ou `null`), `ram_percent`, `flight_time_sec` (temps armé du vol en cours, sinon du dernier vol ; « Vol terminé » sur `drone/logs` au désarmement), `mode` (dernier mode demandé), `mission` (état réel de `main_node`, ex. `takeoff:MONTEE`), `fcu_connected`, `fcu_mode`, `armed` (MAVROS), `height` (lidar en m, `null` sans mesure depuis 1 s), `batt_v`, `batt_pct`, `esp_connected` |
| `robot/ping` | ESP32 → drone | heartbeat (déconnecté après 4 s sans ping) |

**La commande `arm` arme réellement les moteurs.**

Un message mal formé (non JSON, champ manquant ou invalide) est signalé sur `drone/logs` et ignoré.

`restart_script` arrête tout le code du drone, qui est relancé par le service qui l'a démarré : Docker en simu (`restart: unless-stopped`), service systemd `droneload` sur la Pi (installé par `install/`, rôle `pi`, démarré au boot, `Restart=always` ; `journalctl -u droneload` pour ses logs). Les autres nœuds sont relancés seuls par le launch s'ils s'arrêtent (`respawn`, 2 s) ; seul l'arrêt de `mqtt_server` arrête tout.

Bilan de démarrage : 20 s après son lancement, `mqtt_server` envoie deux lignes sur `drone/logs` (Pixhawk et son mode, lidar, batterie ; caméra, vision, crochet, résultat d'`install/check.py` sur la Pi), ce qui manque en majuscules. La Pixhawk ne redémarre pas avec la Pi (alimentation séparée) : `reboot_fcu` la redémarre, MAVROS se reconnecte seul.

Pannes du matériel, toutes visibles sur le dashboard (concours 2026 : la Pi ne voyait plus la Pixhawk et rien ne le signalait) :
- **double lancement** : `drone.launch.py` prend un verrou (`/tmp/droneload-drone.lock`) et refuse de démarrer si le code tourne déjà (service `droneload` sur la Pi) ;
- **port série de la Pixhawk** absent, inaccessible ou déjà ouvert : MAVROS n'est pas lancé, raison répétée toutes les 10 s sur `drone/logs` (`mavros.launch.py`) ;
- **caméra** impossible à ouvrir (nappe, caméra prise) : `camera` reste lancé, le signale et réessaie toutes les 5 s ; plus d'image pendant 3 s : le nœud s'arrête et le launch le relance (la lecture GStreamer peut bloquer sans fin) ;
- **crochet** : la puce GPIO est cherchée par son nom (`pinctrl-rp1`), pas par son numéro, qui change selon le noyau (une autre puce accepterait la broche 13 sans erreur). GPIO indisponible : `servo` le signale, réessaie toutes les 5 s, publie `/hook/ready` à false et refuse les commandes (pas d'ouverture différée, le drone aurait bougé).

## Construire et lancer

On construit depuis `drone/` : les dossiers `build/`, `install/` et `log/` sont créés dans `drone/` et ignorés par git. `--symlink-install` est obligatoire, car le launch retrouve le repo (et `models/`) à partir de son propre fichier.

```bash
cd drone
colcon build --symlink-install
source install/setup.bash          # automatique dans les nouveaux shells (install/ Ansible)

ros2 launch droneload_bringup drone.launch.py                     # drone réel
ros2 launch droneload_bringup drone.launch.py vision_only:=true   # sans Pixhawk
ros2 launch droneload_bringup drone.launch.py sim:=true           # simulation (fait par le service drone)
```

| Argument | Défaut | Rôle |
|---|---|---|
| `sim` | `false` | `true` : MAVROS et images fournis par la simu, crochet simulé, horloge de la simu |
| `vision_only` | `false` | `true` : pas de MAVROS (test de la vision sans Pixhawk) |
| `pc_ip` | PC de pilotage (réel), `host.docker.internal` (simu) | destination du flux vidéo |
| `fcu_url` | `/dev/ttyAMA0:921600` | liaison MAVROS → Pixhawk (argument de `mavros.launch.py`) ; sur un port série, MAVROS n'est pas lancé si le port est absent, inaccessible ou déjà ouvert par un autre programme (raison répétée toutes les 10 s sur `drone/logs`, corriger puis `restart_script`) |
| `model` | `models/best_ncnn_model` | modèle YOLO |

Les autres réglages (hauteur de décollage, résolution, broker...) sont dans `drone/droneload_bringup/config/drone.yaml`. La calibration de la caméra est dans `config/camera/module3_640x480.yaml` : la refaire avec `tools/calibration.py` sur la Pi (photos d'un damier), qui réécrit ce fichier. Si la résolution change, il faut une nouvelle calibration (#68).

Ce que lance chaque mode :

| Nœud | Réel | `vision_only` | `sim` |
|---|---|---|---|
| MAVROS (`mavros.launch.py`) | lancé | — | même fichier, lancé par le service `mavros` de la simu |
| `camera` | lancé | lancé | — (le `bridge` publie les images) |
| `servo` | GPIO | GPIO | mock (journal seulement) |
| `main_node`, `vision`, `aruco`, `gstream`, `mqtt_server` | lancés | lancés | lancés |

## Ajouter un nœud

1. Ajouter le fichier dans le paquet du bon rôle, avec une fonction `main()` : ce qui touche au matériel ou à une liaison (capteur, actionneur, réseau) va dans `droneload_hardware`, le traitement d'image dans `droneload_vision`, la logique de vol dans `droneload_vol`.
2. Le déclarer dans `entry_points` du `setup.py` du paquet, et ses dépendances dans `package.xml` (et dans `install/packages.yml` si c'est un paquet apt ou pip).
3. L'ajouter dans `drone.launch.py`, et ses paramètres dans `drone.yaml`.
4. Documenter ses topics dans le tableau ci-dessus.

Pour un message, utiliser d'abord un type standard (`std_msgs`, `sensor_msgs`, `geometry_msgs`, `vision_msgs`). Ne créer un paquet de messages que si aucun type standard ne convient.

## À venir

| Brique | Issue | Point d'entrée |
|---|---|---|
| Calibration de la caméra basse (distorsion) | #68 | `tools/calibration.py` sur la Pi, réécrit `config/camera/module3_640x480.yaml` |
| ArUco : utilisation de la pose dans les missions (passage des fenêtres) | #15 | le nœud `aruco` publie déjà identifiant et pose 3D ; reste à s'en servir dans `droneload_vol`, et à mesurer la taille des vrais marqueurs (`marker_size`) |
| Compensation de l'inclinaison de la caméra | #69 | positions des détections corrigées avec l'attitude (`/mavros/imu/data`), repère horizontal (tf2) |
| AI HAT (Hailo) | #27 | backend `hailo` de `vision.py` (paramètre `backend`) |
| Caméra de profondeur | #28, #34 | `realsense2_camera` sur `/stereo/...`, puis remplacer `DISTANCE_AVANT` de l'épreuve 2 |
| UWB | #17, #24 | nœud qui publie `/uwb/pose` et l'envoie à `/mavros/vision_pose/pose` |
| Étapes avec validation de l'opérateur | #29 | remplace les missions de `droneload_vol` |

## Migration depuis le code 2026

Le code 2026 (une seule boucle Python cadencée par la caméra, pymavlink) a été entièrement remplacé. Il reste disponible dans l'historique git (dernier commit avant la migration : `3c65685`).

| Ancien fichier | Décision | Devenu |
|---|---|---|
| `main.py` (boucle unique) | réécrit | `main_node.py` : timer à 20 Hz, vision dans son propre nœud |
| `mavlink.py` (`DroneController`, pymavlink) | réécrit | `mavros.py` (`MavrosDrone`) : même rôle, via MAVROS, sans aucune attente bloquante |
| `mavlink.py` (`DummyDroneController`) | jeté | inutile : sans MAVROS (`vision_only`), les commandes sont seulement refusées et journalisées |
| `tasks/epreuve1.py` (`TakeOff`, `Land`) | réécrit | `missions/decollage.py`, `missions/atterrissage.py` : nouvel essai en cas de refus, remise à zéro |
| `tasks/epreuve1.py` (`Epreuve1Task`) | gardé (règle des 80 % sur 2 s) | `missions/epreuve1.py` (décision) + `vision.py` (inférence et dessin) |
| `tasks/epreuve2.py` | gardé | `missions/epreuve2.py`, qui reçoit maintenant les vraies détections |
| `video.py` | découpé | `camera.py` (entrée libcamera) + `gstream.py` (sortie vers le PC), dans `droneload_hardware` |
| `hardware.py` | gardé pour le crochet | `servo.py` ; le servo caméra disparaît (caméra fixe vers le sol) |
| `mqtt_server.py` | gardé (même protocole) | `mqtt_server.py` du paquet `droneload_hardware` ; arm/disarm et batterie via MAVROS |
| `config.py` | remplacé | `drone.yaml` + arguments de `drone.launch.py` |
| `vision/aruco.py` | gardé et complété | nœud `aruco` : même détection (DICT_4X4_50, centre en pixels), plus la pose 3D par `cv2.solvePnP` (remplace `estimatePoseSingleMarkers` de 2025, obsolète) avec la calibration ; compatible OpenCV 4.6 (Ubuntu 24.04) et ≥ 4.7 |

Bugs de 2026 corrigés au passage :
- `TakeOff` et `Land` n'étaient jamais réinitialisées, donc une 2e demande ne faisait rien. Les missions sont désormais remises à zéro à chaque changement de mode.
- La fenêtre de détection de l'épreuve 1 n'était jamais réinitialisée. Elle l'est maintenant au changement de mode et au changement de cible.
- L'épreuve 2 ne recevait aucune détection.
- `arm_and_takeoff_guided2` bloquait la boucle pendant le changement de mode et l'armement.
