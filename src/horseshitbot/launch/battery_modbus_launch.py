import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("horseshitbot")

    return LaunchDescription(
        [
            Node(
                package="horseshitbot",
                executable="battery_modbus_node",
                name="battery_modbus_node",
                output="screen",
                parameters=[
                    os.path.join(pkg_share, "config", "battery_modbus.yaml")
                ],
            )
        ]
    )