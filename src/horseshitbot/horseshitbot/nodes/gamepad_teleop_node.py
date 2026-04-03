"""
Gamepad Teleop Node — reads a Data Frog (or compatible) Bluetooth controller
via evdev and publishes /cmd_vel + actuator commands.
"""

from __future__ import annotations

import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from horseshitbot.srv import SwitchBackend

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
    """Normalise a raw axis value to [-1.0, 1.0] using the reported range."""
    mid = (info.max + info.min) / 2.0
    half = (info.max - info.min) / 2.0
    if half == 0:
        return 0.0
    return (value - mid) / half


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
        self._lift_pub = self.create_publisher(String, "/lift/command", 10)
        self._brush_pub = self.create_publisher(String, "/brush/command", 10)
        self._bin_door_pub = self.create_publisher(String, "/bin_door/command", 10)

        # Service clients
        self._stop_fast_cli = self.create_client(Trigger, "/wheel_driver_node/stop_fast")
        self._stop_cli = self.create_client(Trigger, "/wheel_driver_node/stop")
        self._switch_cli = self.create_client(SwitchBackend, "/wheel_driver_node/switch_backend")

        # Internal state
        self._device: InputDevice | None = None
        self._axis_x = 0.0
        self._axis_y = 0.0
        self._connected = False
        self._lock = threading.Lock()

        # Axis info cache (min/max/mid for normalization)
        self._axis_info: dict = {}

        if not _HAS_EVDEV:
            self.get_logger().error("evdev not installed — gamepad disabled")
        else:
            self._try_connect()

        # Reader thread
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # Publish timer
        period = 1.0 / max(1.0, self._rate)
        self.create_timer(period, self._publish_cmd_vel)

        self.get_logger().info("Gamepad teleop node started")

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
                self._connected = False
                self._device = None

    def _handle_event(self, event):
        if event.type == ecodes.EV_ABS:
            info = self._axis_info.get(event.code)
            if info is None:
                return
            val = _normalize_axis(event.value, info)

            if event.code == ecodes.ABS_X:
                with self._lock:
                    self._axis_x = self._apply_deadzone(val)
            elif event.code == ecodes.ABS_Y:
                with self._lock:
                    raw = self._apply_deadzone(val)
                    self._axis_y = -raw if self._invert_y else raw

            elif event.code == ecodes.ABS_HAT0Y:
                msg = String()
                if event.value == -1:
                    msg.data = "open"
                elif event.value == 1:
                    msg.data = "close"
                else:
                    msg.data = "stop"
                self._lift_pub.publish(msg)

        elif event.type == ecodes.EV_KEY:
            if event.value == 1:  # press
                self._handle_button_press(event.code)
            elif event.value == 0:  # release
                self._handle_button_release(event.code)

    def _handle_button_press(self, code):
        if code == ecodes.BTN_A:
            self._call_trigger(self._stop_fast_cli)
        elif code == ecodes.BTN_B:
            self._call_trigger(self._stop_cli)
        elif code == ecodes.BTN_Y:
            self._switch_backend()
        elif code == ecodes.BTN_X:
            for pub in (self._lift_pub, self._brush_pub, self._bin_door_pub):
                msg = String()
                msg.data = "stop"
                pub.publish(msg)
        elif code == ecodes.BTN_TR:
            msg = String()
            msg.data = "open"
            self._brush_pub.publish(msg)
        elif code == ecodes.BTN_TL:
            msg = String()
            msg.data = "open"
            self._bin_door_pub.publish(msg)
        elif code == ecodes.BTN_START:
            for pub in (self._lift_pub, self._brush_pub, self._bin_door_pub):
                msg = String()
                msg.data = "reference"
                pub.publish(msg)

    def _handle_button_release(self, code):
        if code == ecodes.BTN_TR:
            msg = String()
            msg.data = "stop"
            self._brush_pub.publish(msg)
        elif code == ecodes.BTN_TL:
            msg = String()
            msg.data = "stop"
            self._bin_door_pub.publish(msg)

    def _apply_deadzone(self, val: float) -> float:
        if abs(val) < self._deadzone:
            return 0.0
        sign = 1.0 if val > 0 else -1.0
        return sign * (abs(val) - self._deadzone) / (1.0 - self._deadzone)

    def _publish_cmd_vel(self):
        with self._lock:
            x = self._axis_x
            y = self._axis_y

        msg = Twist()
        msg.linear.x = y * self._max_lin
        msg.angular.z = x * self._max_ang
        self._cmd_vel_pub.publish(msg)

    def _call_trigger(self, client):
        if not client.service_is_ready():
            return
        req = Trigger.Request()
        client.call_async(req)

    def _switch_backend(self):
        if not self._switch_cli.service_is_ready():
            return
        req = SwitchBackend.Request()
        req.backend = "toggle"
        self._switch_cli.call_async(req)


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
