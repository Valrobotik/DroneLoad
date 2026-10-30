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
"""
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
REPO = Path(__file__).resolve().parents[3]   # drone/droneload_bringup/launch/ -> racine du repo
MODEL = REPO / 'models' / 'best_ncnn_model'
CALIBRATION = REPO / 'config' / 'camera' / 'module3_640x480.yaml'   # calibration de la caméra (camera_info)


def nodes(context):
    sim = context.launch_configurations['sim'] == 'true'
    vision_only = context.launch_configurations['vision_only'] == 'true'
    pc_ip = context.launch_configurations['pc_ip'] or (SIM_PC_IP if sim else PC_IP)
    model = context.launch_configurations['model'] or str(MODEL)
    if not Path(model).exists():
        raise RuntimeError(f'Modèle introuvable : {model} (construire avec --symlink-install ou passer model:=)')

    params = str(Path(get_package_share_directory('droneload_bringup')) / 'config' / 'drone.yaml')
    common = [params, {'use_sim_time': sim}]

    def node(package, executable, **extra):
        return Node(package=package, executable=executable, name=executable, output='screen',
                    parameters=common + [extra])

    mqtt_server = node('droneload_hardware', 'mqtt_server')
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
