"""
Autonomy Launch — Robot base + Perception YOLO + Navigation Nav2.

Lance en une seule commande l'ensemble de la stack autonome.

Usage :
  ros2 launch horseshitbot autonomy_launch.py map:=/home/user/maps/salle.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_camera = LaunchConfiguration("enable_camera")
    enable_mks = LaunchConfiguration("enable_mks")
    enable_lidar = LaunchConfiguration("enable_lidar")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")

    pkg_dir = get_package_share_directory("horseshitbot")

    # Base robot (camera, lidar, wheels, gamepad, dashboard, screen)
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "robot_launch.py")
        ),
        launch_arguments={
            "enable_camera": enable_camera,
            "enable_mks": enable_mks,
            "enable_lidar": enable_lidar,
            "params_file": params_file,
        }.items(),
    )

    # Perception YOLO
    yolo_node = Node(
        package="horseshitbot",
        executable="yolo_detector_node",
        name="yolo_detector_node",
        parameters=[
            {
                "model_path": os.path.join(
                    os.path.expanduser("~"), "models", "best.pt"
                ),
                "confidence_threshold": 0.5,
                "publish_pointcloud": True,
                "depth_max_m": 3.0,
                "target_classes": [],
            }
        ],
        output="screen",
    )

    # Nav2 stack (AMCL, planner, controller, etc.)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, "launch", "nav2_bringup_launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": os.path.join(pkg_dir, "config", "nav2_params.yaml"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("enable_mks", default_value="true"),
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="Chemin vers la carte YAML (.yaml) pour AMCL",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Clock simulée (rosbag) ou temps reel",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value="",
            description="Override du params.yaml du robot",
        ),
        robot,
        yolo_node,
        nav2,
    ])
