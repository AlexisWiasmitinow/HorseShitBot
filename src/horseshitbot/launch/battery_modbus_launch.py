"""Launch the serial-free battery monitor.

The mks_bus_node bus owner must already be running. This launch file never
opens /dev/mksbus itself.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("horseshitbot")
    config = os.path.join(package_share, "config", "battery_modbus.yaml")
    return LaunchDescription(
        [
            Node(
                package="horseshitbot",
                executable="battery_modbus_node",
                name="battery_modbus_node",
                parameters=[config],
                output="screen",
            )
        ]
    )
