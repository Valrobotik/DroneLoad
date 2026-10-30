"""MAVROS (liaison ROS 2 <-> ArduPilot) : seule configuration, utilisée sur le drone et en simulation.

  Drone réel : inclus par drone.launch.py (fcu_url par défaut : port série de la Pi)
  Simulation : service mavros de sim/compose.yml, lancé par chemin (sans colcon build) :
               ros2 launch drone/droneload_bringup/launch/mavros.launch.py fcu_url:=udp://:14550@

Toute modification de MAVROS (plugins, paramètres) se fait ici, pour être testée en simu.

Sur un port série, MAVROS n'est pas lancé si le port est absent, inaccessible ou déjà ouvert par un
autre programme (deux lecteurs se partagent les octets et aucun ne reçoit de MAVLink valide : panne du
concours 2026). La raison est répétée sur drone/logs (dashboard) ; corriger puis restart_script.
"""
import os
import shutil
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource

FCU_URL = '/dev/ttyAMA0:921600'   # UART de la Pi 5 vers la Pixhawk
ALERTE_PERIODE = 10.0             # s entre deux rappels du problème sur drone/logs


def probleme_port(fcu_url):
    """Raison pour laquelle MAVROS ne peut pas utiliser le port série, None si tout va bien (ou pas de port série)."""
    chemin = fcu_url.removeprefix('serial://').split(':')[0]
    if not chemin.startswith('/dev/'):
        return None   # UDP/TCP (simu)
    if not os.path.exists(chemin):
        return f'{chemin} absent (UART désactivé ? relancer ./install/apply.sh puis redémarrer la Pi)'
    if not os.access(chemin, os.R_OK | os.W_OK):
        return f'{chemin} inaccessible (utilisateur hors du groupe dialout ? relancer ./install/apply.sh)'
    port = os.path.realpath(chemin)
    # Seuls les processus du même utilisateur sont visibles (ceux lancés avec sudo ne le sont pas)
    for fd in Path('/proc').glob('[0-9]*/fd/*'):
        try:
            if os.path.realpath(fd) == port:
                pid = fd.parts[2]
                commande = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode().strip()
                return (f'{chemin} déjà ouvert par le processus {pid} ({commande[:80]}) : '
                        'code du drone lancé deux fois ? (sudo systemctl stop droneload avant un lancement à la main)')
        except OSError:
            continue   # processus terminé ou d'un autre utilisateur
    return None


def mavros(context):
    fcu_url = context.launch_configurations['fcu_url']
    probleme = probleme_port(fcu_url)
    if probleme:
        message = f'[mavros] MAVROS NON LANCÉ : {probleme}'
        actions = [LogInfo(msg=message)]
        if shutil.which('mosquitto_pub'):   # mosquitto-clients (install/, rôle pi) ; sans lui, le launch planterait
            # mosquitto_pub se termine aussitôt : relancé pour que le dashboard voie le message même ouvert plus tard
            actions.append(ExecuteProcess(cmd=['mosquitto_pub', '-t', 'drone/logs', '-m', message],
                                          respawn=True, respawn_delay=ALERTE_PERIODE))
        return actions
    apm = Path(get_package_share_directory('mavros')) / 'launch' / 'apm.launch'
    return [IncludeLaunchDescription(AnyLaunchDescriptionSource(str(apm)),
                                     launch_arguments={'fcu_url': fcu_url}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value=FCU_URL),
        OpaqueFunction(function=mavros),
    ])
