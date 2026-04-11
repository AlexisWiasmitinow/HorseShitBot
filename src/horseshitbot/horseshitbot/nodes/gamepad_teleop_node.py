"""
Gamepad Teleop Node — reads a Data Frog (or compatible) Bluetooth controller
via evdev and publishes /cmd_vel + actuator commands.

Button-to-action mapping is configurable via the web dashboard and persisted
to ~/.config/horseshitbot/controller_config.json.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from horseshitbot_interfaces.srv import SwitchBackend

try:
    import evdev
    from evdev import InputDevice, ecodes, list_devices

    _HAS_EVDEV = True
except ImportError:
    _HAS_EVDEV = False

CONTROLLER_NAMES = [
    "data frog",
    "datafrog",
    "wireless controller",
    "gamepad",
    "game controller",
    "bluetooth gamepad",
    "xbox",
    "gamesir",
]

_CONFIG_DIR = Path.home() / ".config" / "horseshitbot"
_CONFIG_FILE = _CONFIG_DIR / "controller_config.json"


def _find_repo_config() -> Path | None:
    p = Path(__file__).resolve()
    for parent in p.parents:
        candidate = parent / "src" / "horseshitbot" / "config" / "controller_config.json"
        if (parent / ".git").is_dir() and (parent / "src" / "horseshitbot").is_dir():
            return candidate
    return None


_REPO_CONFIG_FILE = _find_repo_config()

# Trigger analog → digital threshold (normalized: -1 = rest, +1 = full)
_TRIGGER_THRESHOLD = 0.0

BUTTON_NAMES = [
    "A", "B", "X", "Y",
    "LB", "RB", "LT", "RT",
    "L3", "R3",
    "D-Pad Up", "D-Pad Down", "D-Pad Left", "D-Pad Right",
    "Options", "Menu", "Logo",
]

DEFAULT_BUTTON_MAP: dict[str, str] = {
    "A": "e_stop",
    "B": "stop_wheels",
    "X": "stop_actuators",
    "Y": "stop_wheels",
    "LB": "bin_door",
    "RB": "brush",
    "LT": "none",
    "RT": "none",
    "L3": "e_stop",
    "R3": "e_stop",
    "D-Pad Up": "lift_up",
    "D-Pad Down": "lift_down",
    "D-Pad Left": "none",
    "D-Pad Right": "none",
    "Options": "camera_bag_recording",
    "Menu": "reference_all",
    "Logo": "toggle_controls",
}

AVAILABLE_ACTIONS: dict[str, str] = {
    "e_stop": "E-STOP (fast stop)",
    "stop_wheels": "Stop wheels (gentle)",
    "stop_actuators": "Stop all actuators",
    "brush": "Brush deploy (hold)",
    "bin_door": "Bin door open (hold)",
    "lift_up": "Lift up (hold)",
    "lift_down": "Lift down (hold)",
    "reference_all": "Reference all actuators",
    "camera_bag_recording": "Camera bag recording",
    "toggle_controls": "Toggle TFT controls",
    "none": "Unassigned",
}

AXIS_INFO = {
    "L Stick": "Drive (fwd/back + turn)",
    "R Stick": "Not assigned",
}

_EVDEV_TO_BTN: dict[int, str] = {}
if _HAS_EVDEV:
    _EVDEV_TO_BTN = {
        ecodes.BTN_A: "A",
        ecodes.BTN_B: "B",
        ecodes.BTN_X: "X",
        ecodes.BTN_Y: "Y",
        ecodes.BTN_TL: "LB",
        ecodes.BTN_TR: "RB",
        ecodes.BTN_TL2: "LT",
        ecodes.BTN_TR2: "RT",
        ecodes.BTN_THUMBL: "L3",
        ecodes.BTN_THUMBR: "R3",
        ecodes.BTN_START: "Menu",
        ecodes.BTN_SELECT: "Options",
        ecodes.KEY_BACK: "Options",
        ecodes.KEY_HOMEPAGE: "Logo",
        ecodes.BTN_MODE: "Logo",
    }


def _find_controller(device_path: str, keywords: list[str]) -> InputDevice | None:
    if not _HAS_EVDEV:
        return None
    if device_path:
        try:
            return InputDevice(device_path)
        except Exception:
            return None

    devices = [InputDevice(p) for p in list_devices()]
    for dev in devices:
        name_lower = dev.name.lower()
        if any(kw in name_lower for kw in keywords):
            return dev

    for dev in devices:
        caps = dev.capabilities(verbose=False)
        has_abs = ecodes.EV_ABS in caps
        has_key = ecodes.EV_KEY in caps
        if has_abs and has_key:
            abs_codes = [c[0] if isinstance(c, tuple) else c for c in caps[ecodes.EV_ABS]]
            if ecodes.ABS_X in abs_codes and ecodes.ABS_Y in abs_codes:
                return dev
    return None


def _normalize_axis(value: int, info) -> float:
    mid = (info.max + info.min) / 2.0
    half = (info.max - info.min) / 2.0
    if half == 0:
        return 0.0
    return (value - mid) / half


def _load_config_file() -> dict[str, str]:
    paths = [_CONFIG_FILE]
    if _REPO_CONFIG_FILE is not None:
        paths.append(_REPO_CONFIG_FILE)
    for path in paths:
        if path.exists():
            try:
                with open(path) as f:
                    cfg = json.load(f)
                return cfg.get("buttons", dict(DEFAULT_BUTTON_MAP))
            except Exception:
                continue
    return dict(DEFAULT_BUTTON_MAP)


class GamepadTeleopNode(Node):
    def __init__(self):
        super().__init__("gamepad_teleop_node")

        self.declare_parameter("device_path", "")
        self.declare_parameter("deadzone", 0.08)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("max_linear_speed", 1.0)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("invert_y", True)

        self._deadzone = self.get_parameter("deadzone").get_parameter_value().double_value
        self._rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self._max_lin = self.get_parameter("max_linear_speed").get_parameter_value().double_value
        self._max_ang = self.get_parameter("max_angular_speed").get_parameter_value().double_value
        self._invert_y = self.get_parameter("invert_y").get_parameter_value().bool_value
        self._device_path = self.get_parameter("device_path").get_parameter_value().string_value

        # Publishers
        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._gamepad_status_pub = self.create_publisher(String, "/gamepad/status", 10)
        self._gamepad_btn_pub = self.create_publisher(String, "/gamepad/button", 10)
        self._lift_pub = self.create_publisher(String, "/lift/command", 10)
        self._brush_pub = self.create_publisher(String, "/brush/command", 10)
        self._bin_door_pub = self.create_publisher(String, "/bin_door/command", 10)

        # Service clients
        self._stop_fast_cli = self.create_client(Trigger, "/wheel_driver_node/stop_fast")
        self._stop_cli = self.create_client(Trigger, "/wheel_driver_node/stop")
        self._switch_cli = self.create_client(SwitchBackend, "/wheel_driver_node/switch_backend")
        self._rec_start_cli = self.create_client(Trigger, "/bag_recorder_node/start_recording")
        self._rec_stop_cli = self.create_client(Trigger, "/bag_recorder_node/stop_recording")

        # Internal state
        self._device: InputDevice | None = None
        self._axis_x = 0.0
        self._axis_y = 0.0
        self._raxis_x = 0.0
        self._raxis_y = 0.0
        self._connected = False
        self._is_recording = False
        self._active_inputs: set[str] = set()
        self._button_map: dict[str, str] = _load_config_file()
        self._lock = threading.Lock()

        # Axis-as-button state tracking
        self._dpad_y: str | None = None
        self._dpad_x: str | None = None
        self._lt_pressed = False
        self._rt_pressed = False

        self._axis_info: dict = {}

        self.create_subscription(String, "/gamepad/config", self._cb_config, 10)

        if not _HAS_EVDEV:
            self.get_logger().error("evdev not installed — gamepad disabled")
        else:
            self._try_connect()

        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        period = 1.0 / max(1.0, self._rate)
        self.create_timer(period, self._publish_cmd_vel)

        self.get_logger().info(
            f"Gamepad teleop node started (config: "
            f"{'loaded' if _CONFIG_FILE.exists() else 'defaults'})"
        )

    # ── config ────────────────────────────────────────────────────────

    def _cb_config(self, msg: String):
        try:
            cfg = json.loads(msg.data)
            with self._lock:
                self._button_map = cfg.get("buttons", self._button_map)
            self.get_logger().info("Controller config updated via topic")
        except Exception as e:
            self.get_logger().warn(f"Invalid config message: {e}")

    # ── connection ────────────────────────────────────────────────────

    def _try_connect(self):
        self._device = _find_controller(self._device_path, CONTROLLER_NAMES)
        if self._device:
            self._connected = True
            self.get_logger().info(f"Controller found: {self._device.name} ({self._device.path})")
            caps = self._device.capabilities(verbose=False)
            if ecodes.EV_ABS in caps:
                for item in caps[ecodes.EV_ABS]:
                    code = item[0] if isinstance(item, tuple) else item
                    info = item[1] if isinstance(item, tuple) else None
                    self._axis_info[code] = info
        else:
            self._connected = False
            self.get_logger().warn("No controller found, will retry...")

    def _reader_loop(self):
        while rclpy.ok():
            if not self._connected or self._device is None:
                time.sleep(2.0)
                if _HAS_EVDEV:
                    self._try_connect()
                continue

            try:
                for event in self._device.read_loop():
                    if not rclpy.ok():
                        return
                    self._handle_event(event)
            except OSError:
                self.get_logger().warn("Controller disconnected, zeroing velocity")
                with self._lock:
                    self._axis_x = 0.0
                    self._axis_y = 0.0
                    self._raxis_x = 0.0
                    self._raxis_y = 0.0
                    self._active_inputs.clear()
                self._dpad_y = None
                self._dpad_x = None
                self._lt_pressed = False
                self._rt_pressed = False
                self._connected = False
                self._device = None

    # ── event handling ────────────────────────────────────────────────

    def _handle_event(self, event):
        if event.type == ecodes.EV_ABS:
            info = self._axis_info.get(event.code)
            if info is None:
                return
            val = _normalize_axis(event.value, info)

            if event.code == ecodes.ABS_X:
                with self._lock:
                    self._axis_x = self._apply_deadzone(val)
                    if self._axis_x != 0.0 or self._axis_y != 0.0:
                        self._active_inputs.add("L Stick")
                    else:
                        self._active_inputs.discard("L Stick")

            elif event.code == ecodes.ABS_Y:
                with self._lock:
                    raw = self._apply_deadzone(val)
                    self._axis_y = -raw if self._invert_y else raw
                    if self._axis_x != 0.0 or self._axis_y != 0.0:
                        self._active_inputs.add("L Stick")
                    else:
                        self._active_inputs.discard("L Stick")

            elif event.code == ecodes.ABS_RX:
                with self._lock:
                    self._raxis_x = self._apply_deadzone(val)
                    if self._raxis_x != 0.0 or self._raxis_y != 0.0:
                        self._active_inputs.add("R Stick")
                    else:
                        self._active_inputs.discard("R Stick")

            elif event.code == ecodes.ABS_RY:
                with self._lock:
                    self._raxis_y = self._apply_deadzone(val)
                    if self._raxis_x != 0.0 or self._raxis_y != 0.0:
                        self._active_inputs.add("R Stick")
                    else:
                        self._active_inputs.discard("R Stick")

            elif event.code == ecodes.ABS_HAT0Y:
                if event.value == -1:
                    new = "D-Pad Up"
                elif event.value == 1:
                    new = "D-Pad Down"
                else:
                    new = None
                self._axis_button(new, "_dpad_y")

            elif event.code == ecodes.ABS_HAT0X:
                if event.value == -1:
                    new = "D-Pad Left"
                elif event.value == 1:
                    new = "D-Pad Right"
                else:
                    new = None
                self._axis_button(new, "_dpad_x")

            elif event.code == ecodes.ABS_Z:
                pressed = val > _TRIGGER_THRESHOLD
                if pressed and not self._lt_pressed:
                    self._lt_pressed = True
                    self._btn_press("LT")
                elif not pressed and self._lt_pressed:
                    self._lt_pressed = False
                    self._btn_release("LT")

            elif event.code == ecodes.ABS_RZ:
                pressed = val > _TRIGGER_THRESHOLD
                if pressed and not self._rt_pressed:
                    self._rt_pressed = True
                    self._btn_press("RT")
                elif not pressed and self._rt_pressed:
                    self._rt_pressed = False
                    self._btn_release("RT")

        elif event.type == ecodes.EV_KEY:
            btn = _EVDEV_TO_BTN.get(event.code)
            if not btn:
                return
            if event.value == 1:
                self._btn_press(btn)
            elif event.value == 0:
                self._btn_release(btn)

    # ── button press / release (unified for real and virtual buttons) ─

    def _btn_press(self, btn: str):
        with self._lock:
            self._active_inputs.add(btn)
            action = self._button_map.get(btn, "none")
        self._execute_press(action)

    def _btn_release(self, btn: str):
        with self._lock:
            self._active_inputs.discard(btn)
            action = self._button_map.get(btn, "none")
        self._execute_release(action)

    def _axis_button(self, new_btn: str | None, attr: str):
        """Convert an axis value into button press/release events."""
        prev_btn = getattr(self, attr, None)
        setattr(self, attr, new_btn)
        if prev_btn and prev_btn != new_btn:
            self._btn_release(prev_btn)
        if new_btn and new_btn != prev_btn:
            self._btn_press(new_btn)

    # ── action dispatch ───────────────────────────────────────────────

    def _execute_press(self, action: str):
        if action == "e_stop":
            self._call_trigger(self._stop_fast_cli)
        elif action == "stop_wheels":
            self._call_trigger(self._stop_cli)
        elif action == "stop_actuators":
            for pub in (self._lift_pub, self._brush_pub, self._bin_door_pub):
                msg = String()
                msg.data = "stop"
                pub.publish(msg)
        elif action == "brush":
            msg = String()
            msg.data = "open"
            self._brush_pub.publish(msg)
        elif action == "bin_door":
            msg = String()
            msg.data = "open"
            self._bin_door_pub.publish(msg)
        elif action == "lift_up":
            msg = String()
            msg.data = "open"
            self._lift_pub.publish(msg)
        elif action == "lift_down":
            msg = String()
            msg.data = "close"
            self._lift_pub.publish(msg)
        elif action == "reference_all":
            for pub in (self._lift_pub, self._brush_pub, self._bin_door_pub):
                msg = String()
                msg.data = "reference"
                pub.publish(msg)
        elif action == "camera_bag_recording":
            self._toggle_recording()
        elif action == "toggle_controls":
            msg = String()
            msg.data = "home"
            self._gamepad_btn_pub.publish(msg)

    def _execute_release(self, action: str):
        if action == "brush":
            msg = String()
            msg.data = "stop"
            self._brush_pub.publish(msg)
        elif action == "bin_door":
            msg = String()
            msg.data = "stop"
            self._bin_door_pub.publish(msg)
        elif action in ("lift_up", "lift_down"):
            msg = String()
            msg.data = "stop"
            self._lift_pub.publish(msg)

    # ── helpers ───────────────────────────────────────────────────────

    def _apply_deadzone(self, val: float) -> float:
        if abs(val) < self._deadzone:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - self._deadzone) / (1.0 - self._deadzone)

    def _publish_cmd_vel(self):
        with self._lock:
            x = self._axis_x
            y = self._axis_y
            active = sorted(self._active_inputs)
            btn_map = dict(self._button_map)

        msg = Twist()
        msg.linear.x = y * self._max_lin
        msg.angular.z = x * self._max_ang
        self._cmd_vel_pub.publish(msg)

        status = String()
        status.data = json.dumps({
            "connected": self._connected,
            "name": self._device.name if self._device else "",
            "active_inputs": active,
            "button_map": btn_map,
        })
        self._gamepad_status_pub.publish(status)

    def _call_trigger(self, client):
        if not client.service_is_ready():
            self.get_logger().warn(f"Service {client.srv_name} not ready, skipping")
            return
        self.get_logger().info(f"Calling service {client.srv_name}")
        req = Trigger.Request()
        client.call_async(req)

    def _switch_backend(self):
        if not self._switch_cli.service_is_ready():
            return
        req = SwitchBackend.Request()
        req.backend = "toggle"
        self._switch_cli.call_async(req)

    def _toggle_recording(self):
        if self._is_recording:
            self._call_trigger(self._rec_stop_cli)
        else:
            self._call_trigger(self._rec_start_cli)
        self._is_recording = not self._is_recording
        self.get_logger().info(
            f"Recording {'started' if self._is_recording else 'stopped'} (gamepad)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = GamepadTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
