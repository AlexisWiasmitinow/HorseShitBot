"""
Isolated Visualization Launch File for October 1, 2026 LiDAR & Map Playback.

Starts:
  1. The isolated offline LiDAR & OccupancyGrid playback node (`validate_lidar_and_map.py --play-ros2`)
     which publishes `/clock`, `/tf_static` (`base_link -> laser` with `roll=pi, yaw=0`),
     `/tf` (`map -> odom -> base_link`), `/scan`, `/odom_corrected`, `/trajectory_path`, and `/map`.
  2. `rviz2` pre-configured with `config/lidar_mapping_validation.rviz` (`use_sim_time:=true`).

Safety:
  - Forces `ROS_LOCALHOST_ONLY=1` and `ROS_DOMAIN_ID=42` by default so replayed topics
    are completely isolated from the physical robot stack and never publish `/cmd_vel`.

Usage:
  ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1 ros2 launch horseshitbot visualize_oct1_mapping_launch.py \
    bag:=/home/hsb/rosbags/mapping_2026-10-01_14-36-31
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def _launch_setup(context):
    try:
        pkg_dir = get_package_share_directory("horseshitbot")
    except Exception:
        pkg_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    script_path = os.path.join(repo_root, "scripts", "validate_lidar_and_map.py")
    if not os.path.isfile(script_path):
        script_path = "/home/hsb/HorseShitBot/scripts/validate_lidar_and_map.py"

    rviz_config = os.path.join(pkg_dir, "config", "lidar_mapping_validation.rviz")
    if not os.path.isfile(rviz_config):
        rviz_config = os.path.join(
            repo_root, "src", "horseshitbot", "config", "lidar_mapping_validation.rviz"
        )

    bag_path = LaunchConfiguration("bag").perform(context)
    rate = LaunchConfiguration("rate").perform(context)
    enable_rviz = LaunchConfiguration("enable_rviz").perform(context).lower() == "true"
    output_dir = LaunchConfiguration("output_dir").perform(context)

    actions = [
        ExecuteProcess(
            cmd=[
                "python3",
                script_path,
                "--bag",
                bag_path,
                "--output-dir",
                output_dir,
                "--play-ros2",
                "--play-rate",
                rate,
                "--play-loop",
            ],
            output="screen",
        )
    ]

    if enable_rviz:
        actions.append(
            ExecuteProcess(
                cmd=[
                    "rviz2",
                    "-d",
                    rviz_config,
                    "--ros-args",
                    "-p",
                    "use_sim_time:=true",
                ],
                output="screen",
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        DeclareLaunchArgument(
            "bag",
            default_value="/home/hsb/rosbags/mapping_2026-10-01_14-36-31",
            description="Path to the October 1 mapping rosbag directory",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="/home/hsb/hsb_analysis/lidar_validation",
            description="Directory for generated maps and validation reports",
        ),
        DeclareLaunchArgument(
            "rate",
            default_value="2.0",
            description="Playback speed multiplier for RViz2 visualization",
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="true",
            description="Launch rviz2 GUI alongside the isolated bag/map publisher",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
