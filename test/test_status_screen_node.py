"""Status-screen ROS wiring tests without opening SPI or GPIO."""

import threading
import time
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "horseshitbot"))

try:
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    from horseshitbot.nodes import status_screen_node as screen
except ImportError as exc:  # pragma: no cover
    screen = None
    IMPORT_ERROR = str(exc)
else:
    IMPORT_ERROR = ""


class FakeDisplay:
    width = 320
    height = 240

    def __init__(self):
        self.frames = []

    def draw_frame(self, image):
        self.frames.append(image)


@unittest.skipIf(screen is None, f"ROS dependencies unavailable: {IMPORT_ERROR}")
class TestStatusScreenNode(unittest.TestCase):
    def node(self):
        node = screen.StatusScreenNode.__new__(screen.StatusScreenNode)
        node._disp = FakeDisplay()
        node._battery_stale = 5.0
        node._wheel, node._wheel_ts = {}, None
        node._mks_bus = {}
        node._actuators = {"lift": {}, "brush": {}, "bin_door": {}}
        node._actuator_ts = {}
        node._recorders, node._recorder_ts = {}, {}
        node._gamepad, node._gamepad_ts = {}, None
        node._battery, node._battery_ts = None, None
        node._show_controls = False
        node._net_ifaces, node._net_lock = None, threading.Lock()
        node._thermals, node._thermal_lock = None, threading.Lock()
        return node

    def battery(self, percentage=0.5, voltage=25.9):
        message = BatteryState()
        message.percentage = percentage
        message.voltage = voltage
        message.present = True
        message.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_GOOD
        return message

    def test_missing_battery_is_not_reported_as_ok(self):
        self.assertEqual(self.node().snapshot().battery.status, "missing")

    def test_battery_state_values_pass_through(self):
        node = self.node()
        node._cb_battery(self.battery())
        result = node.snapshot().battery
        self.assertEqual(result.percent_text, "50%")
        self.assertEqual(result.voltage_text, "25.9 V")

    def test_stale_battery_is_hidden(self):
        node = self.node()
        node._cb_battery(self.battery())
        node._battery_ts = time.monotonic() - 6.0
        self.assertEqual(node.snapshot().battery.status, "stale")

    def test_existing_status_sources_remain_supported(self):
        node = self.node()
        node._cb_wheel(
            String(data='{"backend":"mks","left_rpm":0,"right_rpm":0}')
        )
        node._cb_gamepad(String(data='{"connected":true,"name":"Xbox"}'))
        node._cb_recorder(
            "mapping_recorder",
            String(data='{"recording":true,"duration_sec":5}'),
        )
        state = node.snapshot()
        self.assertEqual(state.wheel["backend"], "mks")
        self.assertTrue(state.gamepad["connected"])
        self.assertTrue(state.recorders["mapping_recorder"]["recording"])

    def test_recorder_topics_cover_normal_launch_nodes(self):
        self.assertIn(
            "/mapping_recorder/status", screen.RECORDER_TOPICS.values()
        )
        self.assertIn(
            "/perception_recorder/status", screen.RECORDER_TOPICS.values()
        )

    def test_render_does_not_require_hardware(self):
        node = self.node()
        node._render()
        node._cb_gamepad_btn(String(data="home"))
        node._render()
        self.assertEqual(len(node._disp.frames), 2)
        self.assertTrue(
            all(frame.size == (320, 240) for frame in node._disp.frames)
        )


if __name__ == "__main__":
    unittest.main()
