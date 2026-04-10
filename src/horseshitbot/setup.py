from setuptools import setup, find_packages

package_name = "horseshitbot"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={
        "horseshitbot": ["web/static/*"],
    },
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/robot_launch.py"]),
        ("share/" + package_name + "/config", ["config/params.yaml"]),
    ],
    install_requires=[
        "setuptools",
        "pymodbus",
        "pyserial",
        "Pillow",
        "luma.lcd",
    ],
    zip_safe=True,
    maintainer="horseshitbot",
    maintainer_email="dev@horseshitbot.local",
    description="HorseShitBot ROS 2 robot controller",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mks_bus_node = horseshitbot.nodes.mks_bus_node:main",
            "wheel_driver_node = horseshitbot.nodes.wheel_driver_node:main",
            "lift_node = horseshitbot.nodes.lift_node:main",
            "brush_node = horseshitbot.nodes.brush_node:main",
            "bin_door_node = horseshitbot.nodes.bin_door_node:main",
            "gamepad_teleop_node = horseshitbot.nodes.gamepad_teleop_node:main",
            "web_dashboard_node = horseshitbot.nodes.web_dashboard_node:main",
            "status_screen_node = horseshitbot.nodes.status_screen_node:main",
            "bag_recorder_node = horseshitbot.nodes.bag_recorder_node:main",
        ],
    },
)
