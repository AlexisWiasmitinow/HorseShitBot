"""
Launch file Nav2 pour HorseShitBot.
Inclut bringup_launch.py officiel (AMCL + navigation complète).

Prérequis :
  sudo apt install ros-$ROS_DISTRO-nav2-bringup

Usage :
  ros2 launch horseshitbot nav2_bringup_launch.py map:=/chemin/vers/carte.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    pkg_dir = get_package_share_directory("horseshitbot")

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Chemin complet vers le fichier YAML de la carte",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Utiliser le simulated clock (bag playback)",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_dir, "config", "nav2_params.yaml"),
            description="Fichier de paramètres Nav2",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "bringup_launch.py",
                ])
            ),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": "True",
            }.items(),
        ),
    ])
