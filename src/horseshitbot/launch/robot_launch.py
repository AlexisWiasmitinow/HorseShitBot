"""
Launch file for the complete HorseShitBot ROS 2 system.

Arguments:
  enable_camera:=true/false   — enable/disable RealSense + bag recorder (default true)
  enable_mks:=true/false      — enable/disable MKS bus node (default true)
  enable_lidar:=true/false    — enable/disable lidar node (default true)

Example:
  ros2 launch horseshitbot robot_launch.py enable_camera:=false enable_mks:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    pkg_dir = get_package_share_directory("horseshitbot")
    params_override = LaunchConfiguration("params_file").perform(context)
    if params_override and os.path.isfile(params_override):
        params_file = params_override
    else:
        params_file = os.path.join(pkg_dir, "config", "params.yaml")

    enable_camera = LaunchConfiguration("enable_camera").perform(context).lower() == "true"
    enable_mks = LaunchConfiguration("enable_mks").perform(context).lower() == "true"
    enable_lidar = LaunchConfiguration("enable_lidar").perform(context).lower() == "true"

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
    ]

    if enable_lidar:
        nodes.append(Node(
            package="horseshitbot",
            executable="lidar_node",
            name="lidar_node",
            parameters=[params_file],
            output="screen",
        ))
        # Static TF: base_link -> laser (adjust xyz for your lidar mount)
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser_tf",
            arguments=["--x", "0.0", "--y", "0.0", "--z", "0.15",
                        "--roll", "0.0", "--pitch", "0.0", "--yaw", "3.14159",
                        "--frame-id", "base_link", "--child-frame-id", "laser"],
        ))

    # Bag recorders (always launched — topics are selectable via dashboard)
    nodes += [
        Node(
            package="horseshitbot",
            executable="bag_recorder_node",
            name="perception_recorder",
            parameters=[params_file],
            output="screen",
        ),
        Node(
            package="horseshitbot",
            executable="bag_recorder_node",
            name="mapping_recorder",
            parameters=[params_file],
            output="screen",
        ),
    ]

    if enable_camera:
        from launch.actions import IncludeLaunchDescription
        rs_launch_dir = os.path.join(
            get_package_share_directory("realsense2_camera"), "launch"
        )
        camera_config = os.path.join(pkg_dir, "config", "camera_config.yaml")
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rs_launch_dir, "rs_launch.py")
            ),
            launch_arguments={
                "camera_name": "camera",
                "camera_namespace": "",
                "device_type": "d415",
                "config_file": camera_config,
                "enable_color": "true",
                "enable_depth": "true",
                "align_depth.enable": "true",
                "rgb_camera.color_profile": "640x480x15",
                "depth_module.depth_profile": "640x480x15",
            }.items(),
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("enable_mks", default_value="true"),
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument("params_file", default_value=""),
        OpaqueFunction(function=_launch_setup),
    ])
