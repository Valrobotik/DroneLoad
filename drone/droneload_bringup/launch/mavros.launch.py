"""MAVROS (liaison ROS 2 <-> ArduPilot) : seule configuration, utilisée sur le drone et en simulation.

  Drone réel : inclus par drone.launch.py (fcu_url par défaut : port série de la Pi)
  Simulation : service mavros de sim/compose.yml, lancé par chemin (sans colcon build) :
               ros2 launch drone/droneload_bringup/launch/mavros.launch.py fcu_url:=udp://:14550@

Toute modification de MAVROS (plugins, paramètres) se fait ici, pour être testée en simu.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

FCU_URL = '/dev/ttyAMA0:921600'   # UART de la Pi 5 vers la Pixhawk


def generate_launch_description():
    apm = Path(get_package_share_directory('mavros')) / 'launch' / 'apm.launch'
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url', default_value=FCU_URL),
        IncludeLaunchDescription(AnyLaunchDescriptionSource(str(apm)),
                                 launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items()),
    ])
