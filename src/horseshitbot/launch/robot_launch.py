"""
Launch file for the complete HorseShitBot ROS 2 system.

Brings up all nodes with parameters from config/params.yaml:
  - mks_bus_node        (Modbus RTU serial owner)
  - wheel_driver_node   (switchable MKS / ODrive backend)
  - lift_node           (dual-motor lift actuator)
  - brush_node          (single-motor brush actuator)
  - bin_door_node       (single-motor bin door actuator)
  - gamepad_teleop_node (Data Frog BT controller)
  - web_dashboard_node  (FastAPI web UI)
  - status_screen_node  (ILI9341 SPI TFT status display)
  - realsense2_camera   (Intel RealSense D415 colour + depth)
  - bag_recorder_node   (rosbag2 recording for ML data collection)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("horseshitbot")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")

    rs_launch_dir = os.path.join(
        get_package_share_directory("realsense2_camera"), "launch"
    )

    return LaunchDescription([
        Node(
            package="horseshitbot",
            executable="mks_bus_node",
            name="mks_bus_node",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="horseshitbot",
            executable="wheel_driver_node",
            name="wheel_driver_node",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="horseshitbot",
            executable="lift_node",
            name="lift_node",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="horseshitbot",
            executable="brush_node",
            name="brush_node",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="horseshitbot",
            executable="bin_door_node",
            name="bin_door_node",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="horseshitbot",
            executable="gamepad_teleop_node",
            name="gamepad_teleop_node",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="horseshitbot",
            executable="web_dashboard_node",
            name="web_dashboard_node",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="horseshitbot",
            executable="status_screen_node",
            name="status_screen_node",
            parameters=[params_file],
            output="screen",
        ),

        # Intel RealSense D415 via the official wrapper
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rs_launch_dir, "rs_launch.py")
            ),
            launch_arguments={
                "camera_name": "camera",
                "device_type": "d415",
                "enable_color": "true",
                "enable_depth": "true",
                "enable_pointcloud": "false",
                "align_depth.enable": "true",
                "rgb_camera.color_profile": "640x480x30",
                "depth_module.depth_profile": "640x480x30",
            }.items(),
        ),

        # Rosbag recorder for ML data collection
        Node(
            package="horseshitbot",
            executable="bag_recorder_node",
            name="bag_recorder_node",
            parameters=[params_file],
            output="screen",
        ),
    ])
