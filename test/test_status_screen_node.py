"""status_screen_node wiring: message freshness and BatteryState handling.

Builds the node object without Node.__init__, so no ROS context, SPI or GPIO
is touched. Skipped when ROS 2 and horseshitbot_interfaces are not sourced.
"""

import threading
import time
import unittest
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "horseshitbot"
sys.path.insert(0, str(PACKAGE_ROOT))

try:
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    from horseshitbot.nodes import status_screen_node as ssn
except ImportError as exc:  # pragma: no cover - depends on the environment
    ssn = None
    _IMPORT_ERROR = str(exc)
else:
    _IMPORT_ERROR = ""


class _FakeDisplay:
    width = 320
    height = 240

    def __init__(self):
        self.frames = []

    def draw_frame(self, image):
        self.frames.append(image)
        return image


@unittest.skipIf(ssn is None, f"ROS 2 not sourced: {_IMPORT_ERROR}")
class TestStatusScreenNode(unittest.TestCase):
    def make_node(self):
        node = ssn.StatusScreenNode.__new__(ssn.StatusScreenNode)
        node._disp = _FakeDisplay()
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

    @staticmethod
    def battery_msg(percentage, voltage, health=None):
        msg = BatteryState()
        msg.percentage = percentage
        msg.voltage = voltage
        msg.present = True
        msg.power_supply_health = (
            BatteryState.POWER_SUPPLY_HEALTH_GOOD if health is None else health
        )
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        return msg

    def test_nothing_received_is_missing_not_ok(self):
        snap = self.make_node().snapshot()
        self.assertIsNone(snap.wheel)
        self.assertIsNone(snap.gamepad)
        self.assertIsNone(snap.recorders)
        self.assertEqual(snap.actuators, {})
        self.assertEqual(snap.battery.status, "missing")

    def test_battery_message_used_verbatim(self):
        node = self.make_node()
        node._cb_battery(self.battery_msg(0.5, 25.9))
        bat = node.snapshot().battery
        self.assertEqual((bat.percent_text, bat.voltage_text), ("50%", "25.9 V"))

    def test_battery_goes_stale(self):
        node = self.make_node()
        node._cb_battery(self.battery_msg(1.0, 28.0))
        node._battery_ts = time.monotonic() - 6.0
        self.assertEqual(node.snapshot().battery.status, "stale")

    def test_critical_health_propagates(self):
        node = self.make_node()
        node._cb_battery(self.battery_msg(
            0.4, 19.9, BatteryState.POWER_SUPPLY_HEALTH_DEAD))
        self.assertEqual(node.snapshot().battery.level, "critical")

    def test_stale_wheel_and_gamepad_dropped(self):
        node = self.make_node()
        node._cb_wheel(String(data='{"estopped": true, "left_rpm": 0, "right_rpm": 0}'))
        node._cb_gamepad(String(data='{"connected": true, "name": "pad"}'))
        snap = node.snapshot()
        self.assertTrue(snap.wheel["estopped"])
        self.assertTrue(snap.gamepad["connected"])
        node._wheel_ts -= 10
        node._gamepad_ts -= 10
        snap = node.snapshot()
        self.assertIsNone(snap.wheel)
        self.assertIsNone(snap.gamepad)

    def test_launch_recorder_topics(self):
        node = self.make_node()
        self.assertIn("/mapping_recorder/status", ssn.RECORDER_TOPICS.values())
        self.assertIn("/perception_recorder/status", ssn.RECORDER_TOPICS.values())
        node._cb_recorder("mapping_recorder",
                          String(data='{"recording": true, "duration_sec": 5}'))
        self.assertTrue(node.snapshot().recorders["mapping_recorder"]["recording"])

    def test_resume_button_from_map(self):
        node = self.make_node()
        self.assertEqual(node._resume_button(), "B")
        node._gamepad = {"button_map": {"A": "e_stop", "Y": "stop_wheels"}}
        self.assertEqual(node._resume_button(), "Y")

    def test_render_and_home_toggle(self):
        node = self.make_node()
        node._render()
        node._cb_gamepad_btn(String(data="home"))
        self.assertTrue(node._show_controls)
        node._render()
        node._cb_gamepad_btn(String(data="home"))
        self.assertFalse(node._show_controls)
        frames = node._disp.frames
        self.assertEqual(len(frames), 2)
        self.assertTrue(all(f.size == (320, 240) for f in frames))
        self.assertNotEqual(frames[0].tobytes(), frames[1].tobytes())


if __name__ == "__main__":
    unittest.main()
