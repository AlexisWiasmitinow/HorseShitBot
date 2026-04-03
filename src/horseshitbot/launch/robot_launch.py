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
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("horseshitbot")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")

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
    ])
