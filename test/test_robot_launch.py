"""Hardware-independent regression tests for the complete robot launch."""

import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


LAUNCH_FILE = (
    Path(__file__).parents[1]
    / "src"
    / "horseshitbot"
    / "launch"
    / "robot_launch.py"
)
PACKAGE_SHARE = "/opt/test/share/horseshitbot"
DEFAULT_PARAMS = f"{PACKAGE_SHARE}/config/params.yaml"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("robot_launch_under_test", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _NodeDefinition:
    def __init__(self, **kwargs):
        self.config = kwargs


class TestRobotLaunch(unittest.TestCase):
    def setUp(self):
        self.launch = _load_launch_module()

    def _node_configs(self, *, enable_camera, params_file=""):
        values = {
            "params_file": params_file,
            "enable_camera": str(enable_camera).lower(),
            "enable_mks": "true",
            "enable_battery": "false",
            "enable_lidar": "true",
            "enable_imu": "true",
        }

        def perform(configuration, _context):
            return values[configuration.variable_name[0].text]

        with (
            patch.object(self.launch.LaunchConfiguration, "perform", perform),
            patch.object(
                self.launch,
                "get_package_share_directory",
                return_value=PACKAGE_SHARE,
            ),
            patch.object(self.launch, "Node", _NodeDefinition),
        ):
            actions = self.launch._launch_setup(object())

        return [
            action.config
            for action in actions
            if isinstance(action, _NodeDefinition)
        ]

    @staticmethod
    def _recorders(nodes):
        return [
            node
            for node in nodes
            if node["executable"] == "bag_recorder_node"
        ]

    def test_launches_one_recorder_with_each_existing_name(self):
        recorders = self._recorders(self._node_configs(enable_camera=True))

        self.assertEqual(len(recorders), 2)
        self.assertCountEqual(
            [recorder["name"] for recorder in recorders],
            ["perception_recorder", "mapping_recorder"],
        )

    def test_recorders_retain_package_executable_parameters_and_output(self):
        recorders = self._recorders(self._node_configs(enable_camera=True))

        for recorder in recorders:
            with self.subTest(name=recorder["name"]):
                self.assertEqual(recorder["package"], "horseshitbot")
                self.assertEqual(recorder["executable"], "bag_recorder_node")
                self.assertEqual(recorder["parameters"], [DEFAULT_PARAMS])
                self.assertEqual(recorder["output"], "screen")

    def test_recorders_retain_params_file_override(self):
        with tempfile.TemporaryDirectory() as directory:
            params_file = str(Path(directory) / "robot-override.yaml")
            Path(params_file).touch()
            recorders = self._recorders(
                self._node_configs(enable_camera=False, params_file=params_file)
            )

        self.assertTrue(recorders)
        for recorder in recorders:
            self.assertEqual(recorder["parameters"], [params_file])

    def test_disabling_camera_does_not_disable_recorders(self):
        enabled = self._recorders(self._node_configs(enable_camera=True))
        disabled = self._recorders(self._node_configs(enable_camera=False))

        self.assertEqual(
            [(node["name"], node["executable"]) for node in disabled],
            [(node["name"], node["executable"]) for node in enabled],
        )

    def test_required_non_camera_nodes_remain_present(self):
        nodes = self._node_configs(enable_camera=False)

        self.assertTrue(
            {
                "mks_bus_node",
                "wheel_driver_node",
                "lift_node",
                "brush_node",
                "bin_door_node",
                "gamepad_teleop_node",
                "web_dashboard_node",
                "status_screen_node",
                "lidar_node",
                "base_to_laser_tf",
                "imu_node",
                "perception_recorder",
                "mapping_recorder",
            }.issubset({node["name"] for node in nodes})
        )


if __name__ == "__main__":
    unittest.main()
