# Simulation

La simulation fait tourner le vrai autopilote (ArduPilot SITL), un monde Gazebo de la volière et les mêmes topics ROS 2 que sur le drone. Le code applicatif ne voit pas la différence : seules les sources des topics changent (bridge Gazebo en simu, nœuds drivers sur le drone).

| Élément | Version |
|---|---|
| OS des conteneurs | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Simulateur | Gazebo Harmonic (fourni par `ros-jazzy-ros-gz`) |
| Autopilote | ArduCopter SITL, tag `ARDUPILOT_TAG` dans `.env` |
| Liaison ROS 2 ↔ autopilote | MAVROS |

## Prérequis

- **Docker** : Docker Desktop (Mac, Windows) ou Docker Engine + plugin compose (Linux). Prévoir **4 Go de RAM** pour Docker et environ **20 Go de disque**.
- Sur Mac Apple Silicon, les images sont construites en arm64 natif. Vérifier avec `docker image inspect droneload-sim --format '{{.Architecture}}'` → `arm64` (une image amd64 émulée est extrêmement lente).
- Facultatif : [QGroundControl](https://qgroundcontrol.com), [Foxglove](https://foxglove.dev), GStreamer (voir [README](../README.md#pc-de-pilotage)).

## Lancer

**Toutes les commandes se lancent à la racine du repo.** Le repo est monté dans les conteneurs (`/workspace`) : une modification de code est visible sans reconstruire.

```bash
docker compose build                 # la première fois : 20-40 min (compilation d'ArduPilot)
docker compose up -d                 # Mac / Windows
docker compose -f compose.yml -f compose.linux.yml up -d                     # Linux
docker compose -f compose.yml -f compose.linux.yml -f compose.gpu.yml up -d  # Linux + GPU NVIDIA
docker compose logs -f sitl          # suivre un service
docker compose down                  # tout arrêter
```

Vérifier que tout marche avec le test de fumée (décollage à 1 m mesuré au lidar, 5 s de vol stationnaire, atterrissage) :
```bash
docker compose exec companion bash -lc 'python3 sim/scripts/smoke_test.py'
```

Ouvrir un shell ROS 2 dans la simu (l'environnement ROS 2 est chargé par le shell ; pour une commande isolée, passer par `bash -lc '...'`) :
```bash
docker compose exec companion bash
ros2 topic list
ros2 topic hz /camera/image_raw
```

## Services

| Service | Rôle |
|---|---|
| `gz-server` | Gazebo headless, monde `sim/worlds/voliere.sdf` |
| `sitl` | ArduCopter ; paramètres `autopilot/params/*.parm`, rechargés à chaque démarrage |
| `mavros` | MAVLink ↔ ROS 2 (`/mavros/...`) |
| `bridge` | capteurs Gazebo → ROS 2 (`sim/bridge.yaml`) |
| `companion` | conteneur de développement (le futur code ROS 2 tourne ici) |
| `mqtt` | broker MQTT, même rôle que le mosquitto de la Pi |
| `foxglove` | profil `foxglove` : `docker compose --profile foxglove up -d` |
| `gz-gui` | profil `gui` : `docker compose --profile gui up -d gz-gui` |

Tous les conteneurs partagent le même réseau et se parlent sur `127.0.0.1`. Les ports ne sont ouverts que sur la machine locale.

## Topics

| Topic | Type | Source en simu | Source sur le drone |
|---|---|---|---|
| `/camera/image_raw`, `/camera/camera_info` | Image, CameraInfo | Gazebo (640×480, 15 Hz), vers le sol | caméra Module 3 vers le sol (`libcamerasrc`) |
| `/stereo/color/image_raw`, `/stereo/depth/image_raw`, `/stereo/camera_info` | Image, CameraInfo | Gazebo `rgbd_camera` (type RealSense D435), vers l'avant | caméra de profondeur USB vers l'avant |
| `/mavros/...` | état, pose, commandes | SITL via MAVROS | Pixhawk via MAVROS (`/dev/ttyAMA0:921600`) |
| `/clock` | Clock | Gazebo | — |

Le MTF-01 (flux optique + lidar) est branché sur la Pixhawk : en simu il est transmis directement à l'autopilote, pas à ROS (flux optique calculé par le SITL, lidar Gazebo envoyé au SITL par le plugin ArduPilot). L'altitude estimée se lit dans `/mavros/local_position/pose`, la distance lidar dans `/mavros/optical_flow/ground_distance` (ou `/mavros/rangefinder/rangefinder` après avoir demandé le message RANGEFINDER à ArduPilot, `/mavros/set_message_interval` id 173).

## Outils du PC de pilotage

Ils tournent toujours **sur le PC, hors Docker**, en simu comme en réel. Seule l'adresse change :

| Outil | Réel | Simu |
|---|---|---|
| Dashboard : `python ground/launch.py` | `--broker <IP du Pi>` | `--broker localhost` |
| Flux vidéo GStreamer (UDP 5000, lancé par `launch.py`) | envoyé par la Pi vers l'IP du PC | envoyé par le conteneur vers `host.docker.internal` (Mac/Windows) ou `127.0.0.1` (Linux) |
| Foxglove | `ws://<IP du Pi>:8765` | `ws://localhost:8765` |
| QGroundControl | télémétrie ELRS | TCP `localhost:5760` |

Le PC n'a pas besoin de ROS 2.

## Vue 3D

- **Mac / Windows** : `docker compose --profile gui up -d gz-gui` puis http://localhost:6901. Le rendu est logiciel (5-10 FPS), suffisant pour vérifier une scène. Fermer la GUI pendant les tests de vision : elle consomme du CPU.
- **Linux** : GUI native, fluide avec GPU. Installer Gazebo une fois avec le playbook (`cd provision && sudo ansible-playbook -c local -i localhost, site.yml -e with_sim=true -e droneload_repo=$PWD/..`), puis `gz sim -g` pendant que la simu tourne (même `GZ_PARTITION` que dans `.env`).

## Performances

Mesuré sur un Mac Apple Silicon (rendu logiciel) : temps réel ≈ 0,87 (0,65 à 0,8 avec la GUI ouverte), caméras ≈ 12 Hz au lieu des 15 Hz réglés. Les performances de vision se valident sur le vrai drone, pas en simu. Mesures : `ros2 topic hz /camera/image_raw` (dans `companion`), `gz topic -f -t /camera/image_raw` (dans `gz-server`).

## Modifier la simu

- **Monde** : `sim/worlds/voliere.sdf`. Les ArUco (DICT_4X4_50) sont des textures de `sim/models/aruco_markers/`, régénérables avec `python sim/tools/generate_aruco.py`.
- **Drone** : `sim/models/droneload_drone/model.sdf`, dérivé de `iris_with_ardupilot` (ardupilot_gazebo) avec en plus les caméras et le lidar du MTF-01. Géométrie, masse et poussée sont encore celles de l'iris (vrai drone : F450, moteurs 2216 920 KV, hélices 1045) : à recaler quand le drone sera pesé.
- **Paramètres autopilote** : `autopilot/params/drone.parm` (communs avec le vrai drone) et `sitl.parm` (spécifiques SITL). Les paramètres changés depuis QGC sont perdus au redémarrage du SITL : reportez-les dans les fichiers.
- **Version d'ArduPilot** : `ARDUPILOT_TAG` dans `.env`, puis `docker compose build sitl`. Elle doit correspondre au firmware de la Pixhawk.
- **Dépendances** : tout s'installe via `provision/` (Ansible), utilisé par les Dockerfiles et, plus tard, par la Pi. Ajouter une dépendance Python dans `provision/requirements.txt` (version figée), puis `docker compose build`.

## Plusieurs personnes sur la même machine

Chacun choisit un `ROS_DOMAIN_ID` et un `GZ_PARTITION` différents, et un nom de projet différent :
```bash
ROS_DOMAIN_ID=7 GZ_PARTITION=alice docker compose -p sim-alice up -d
```
Sur Mac, les ports publiés sont les mêmes : une seule simu à la fois par machine, sauf à changer les ports dans un override.

## Dépannage

- **Remettre le drone au départ** : le monde n'est pas réinitialisé entre deux vols. Drone posé et désarmé, `./sim/scripts/reset_drone.sh` le replace sur l'ArUco 0 et redémarre le SITL (prêt à armer ~30 s plus tard). Pour tout remettre à zéro : `docker compose down && docker compose up -d`.

- **`docker compose build` échoue sur le SITL** : vérifier que `ARDUPILOT_TAG` existe (`git ls-remote --tags https://github.com/ArduPilot/ardupilot.git | grep Copter-`).
- **Le test de fumée reste bloqué à l'armement** : `docker compose logs sitl` et, dans QGC, les messages de pré-armement.
- **Pas d'image caméra** : `docker compose logs gz-server bridge` ; `docker compose exec gz-server bash -lc 'gz topic -l'` doit lister `/camera/image_raw`.
- **Simu lente (RTF < 1)** : `docker compose exec gz-server bash -lc 'gz topic -e -t /stats -n 1'` ; fermer la GUI, augmenter le CPU alloué à Docker.
