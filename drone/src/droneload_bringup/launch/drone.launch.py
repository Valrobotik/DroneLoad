"""Lance tout le code du drone.

  Drone réel  : ros2 launch droneload_bringup drone.launch.py
  Simulation  : ros2 launch droneload_bringup drone.launch.py sim:=true
  Vision seule (sans Pixhawk) : ... vision_only:=true

Arguments :
  sim          true : MAVROS et images viennent de la simu (services mavros et bridge), crochet simulé
  vision_only  true : pas de MAVROS (remplace l'ancien --no-pixhawk)
  pc_ip        destination du flux vidéo (défaut : PC de pilotage en réel, host.docker.internal en simu)
  fcu_url      liaison MAVROS -> Pixhawk (défaut : celui de mavros.launch.py, port série de la Pi)
  model        modèle YOLO (défaut : models/best_ncnn_model du repo)

Construire avec `colcon build --symlink-install` : le repo est retrouvé depuis ce fichier.

Un seul lancement à la fois sur la machine (verrou) : deux main_node piloteraient le même drone et deux
mqtt_server (même identifiant MQTT) se déconnecteraient l'un l'autre. Sur la Pi, le service systemd
droneload tourne déjà : sudo systemctl stop droneload avant un lancement à la main.
"""
import fcntl
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

PC_IP = '192.168.31.181'        # PC de pilotage sur le réseau du drone
SIM_PC_IP = 'host.docker.internal'
RESPAWN_DELAY = 2.0             # s avant de relancer un nœud arrêté
REPO = Path(__file__).resolve().parents[3]   # drone/droneload_bringup/launch/ -> racine du repo
MODEL = REPO / 'models' / 'best_ncnn_model'
CALIBRATION = REPO / 'config' / 'camera' / 'module3_640x480.yaml'   # calibration de la caméra (camera_info)
LOCK = Path('/tmp/droneload-drone.lock')
_lock_fd = None   # gardé ouvert tant que le launch tourne (libéré par le système à sa sortie)


def prendre_verrou():
    """Refuse un second lancement du code du drone sur cette machine."""
    global _lock_fd
    try:
        # Sans O_CREAT si le fichier existe : /tmp refuse O_CREAT sur un fichier d'un autre utilisateur
        fd = os.open(LOCK, os.O_RDONLY) if LOCK.exists() else os.open(LOCK, os.O_RDONLY | os.O_CREAT, 0o644)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('Code du drone déjà lancé sur cette machine (service droneload ? '
                           'sudo systemctl stop droneload avant un lancement à la main ; '
                           'pgrep -af drone.launch.py pour le trouver)') from None
    _lock_fd = fd   # non hérité par les nœuds (O_CLOEXEC par défaut en Python)


def nodes(context):
    prendre_verrou()
    sim = context.launch_configurations['sim'] == 'true'
    vision_only = context.launch_configurations['vision_only'] == 'true'
    pc_ip = context.launch_configurations['pc_ip'] or (SIM_PC_IP if sim else PC_IP)
    model = context.launch_configurations['model'] or str(MODEL)
    if not Path(model).exists():
        raise RuntimeError(f'Modèle introuvable : {model} (construire avec --symlink-install ou passer model:=)')

    params = str(Path(get_package_share_directory('droneload_bringup')) / 'config' / 'drone.yaml')
    common = [params, {'use_sim_time': sim}]

    def node(package, executable, respawn=True, **extra):
        # Un nœud qui plante est relancé (main_node retrouve le dernier mode demandé, conservé)
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=common + [extra], respawn=respawn, respawn_delay=RESPAWN_DELAY)

    mqtt_server = node('droneload_hardware', 'mqtt_server', respawn=False)
    actions = [
        node('droneload_vol', 'main_node'),
        node('droneload_vision', 'vision', model_path=model),
        node('droneload_vision', 'aruco'),
        node('droneload_hardware', 'gstream', pc_ip=pc_ip),
        node('droneload_hardware', 'servo', mock=sim),
        mqtt_server,
        # restart_script (dashboard) arrête mqtt_server : tout le code s'arrête et le service le relance
        RegisterEventHandler(OnProcessExit(target_action=mqtt_server,
                                           on_exit=[EmitEvent(event=Shutdown(reason='mqtt_server arrêté'))])),
    ]
    if not sim:
        actions.append(node('droneload_hardware', 'camera', calibration_file=str(CALIBRATION)))
    if not sim and not vision_only:
        # En simu, la même configuration MAVROS est lancée par le service mavros de sim/compose.yml
        # fcu_url:= passé en ligne de commande arrive directement à mavros.launch.py
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(__file__).resolve().parent / 'mavros.launch.py'))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('sim', default_value='false'),
        DeclareLaunchArgument('vision_only', default_value='false'),
        DeclareLaunchArgument('pc_ip', default_value=''),
        DeclareLaunchArgument('model', default_value=''),
        OpaqueFunction(function=nodes),
    ])
