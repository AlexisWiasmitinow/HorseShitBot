"""
Minimal launch for collecting a mapping rosbag.

Starts only: wheel_driver, lidar, gamepad teleop, static TF, and bag
recorder (with mapping topics pre-selected).

Drive the robot around with the gamepad, then stop recording via
the dashboard or:
    ros2 service call /mapping_recorder/stop_recording std_srvs/srv/Trigger

The resulting bag in ~/rosbags/ can be replayed into slam_toolbox:
    ros2 bag play <bag_path>
    ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    pkg_dir = get_package_share_directory("horseshitbot")
    params_override = LaunchConfiguration("params_file").perform(context)
    if params_override and os.path.isfile(params_override):
        params_file = params_override
    else:
        params_file = os.path.join(pkg_dir, "config", "params.yaml")

    enable_mks = LaunchConfiguration("enable_mks").perform(context).lower() == "true"

    nodes = []

    if enable_mks:
        nodes.append(Node(
            package="horseshitbot",
            executable="mks_bus_node",
            name="mks_bus_node",
            parameters=[params_file],
            output="screen",
        ))

    nodes += [
        Node(
            package="horseshitbot",
            executable="wheel_driver_node",
            name="wheel_driver_node",
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
            executable="lidar_node",
            name="lidar_node",
            parameters=[params_file, {"auto_start": True}],
            output="screen",
        ),
        # static TF: base_link -> laser (adjust for your mount)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            arguments=["--x", "0.0", "--y", "0.0", "--z", "0.15",
                        "--roll", "0.0", "--pitch", "0.0", "--yaw", "3.14159",
                        "--frame-id", "base_link", "--child-frame-id", "laser"],
        ),
        Node(
            package="horseshitbot",
            executable="bag_recorder_node",
            name="mapping_recorder",
            parameters=[params_file],
            output="screen",
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_mks", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
