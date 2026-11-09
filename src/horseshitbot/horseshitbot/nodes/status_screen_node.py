"""
Status Screen Node — subscribes to wheel, actuator, battery, gamepad and
recorder status and renders a live dashboard on the on-robot 320x240 ILI9341
SPI TFT at ~5 Hz.

Press the Xbox/Home button on the gamepad to toggle the controls overlay.
The controls screen is built dynamically from the gamepad's button_map config.

Drawing lives in drivers/status_screen_render.py so previews and tests use the
exact same renderer without hardware.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

from horseshitbot_interfaces.msg import ActuatorState as ActuatorStateMsg
from ..drivers.battery_readout import battery_readout
from ..drivers.ili9341_display import ILI9341Display, default_display_pins
from ..drivers.network_manager import get_ip_for_display
from ..drivers.status_screen_render import (
    SCREEN_H,
    SCREEN_W,
    ScreenSnapshot,
    render_controls,
    render_dashboard,
)

from PIL import Image

# Status publishers run at 2-20 Hz; older data is shown as missing, not as OK.
WHEEL_STALE_SEC = 3.0
ACTUATOR_STALE_SEC = 3.0
GAMEPAD_STALE_SEC = 3.0
RECORDER_STALE_SEC = 3.0

RECORDER_TOPICS = {
    "perception_recorder": "/perception_recorder/status",
    "mapping_recorder": "/mapping_recorder/status",
    # Standalone `ros2 run horseshitbot bag_recorder_node`.
    "bag_recorder_node": "/bag_recorder_node/status",
}


class StatusScreenNode(Node):
    def __init__(self):
        super().__init__("status_screen_node")

        _dp = default_display_pins()
        self.declare_parameter("spi_port", 0)
        self.declare_parameter("spi_device", 0)
        self.declare_parameter("dc_pin", _dp["dc"])
        self.declare_parameter("rst_pin", _dp["rst"])
        self.declare_parameter("led_pin", _dp["led"])
        self.declare_parameter("rotation", 0)
        self.declare_parameter("refresh_hz", 5.0)
        self.declare_parameter("backlight_brightness", 1.0)
        self.declare_parameter("battery_stale_sec", 5.0)

        spi_port = self.get_parameter("spi_port").get_parameter_value().integer_value
        spi_dev = self.get_parameter("spi_device").get_parameter_value().integer_value
        dc = self.get_parameter("dc_pin").get_parameter_value().integer_value
        rst = self.get_parameter("rst_pin").get_parameter_value().integer_value
        led = self.get_parameter("led_pin").get_parameter_value().integer_value
        rot = self.get_parameter("rotation").get_parameter_value().integer_value
        hz = self.get_parameter("refresh_hz").get_parameter_value().double_value
        self._battery_stale = max(
            1.0, float(self.get_parameter("battery_stale_sec").value)
        )

        try:
            self._disp = ILI9341Display(
                spi_port=spi_port,
                spi_device=spi_dev,
                dc_pin=dc,
                rst_pin=rst,
                led_pin=led,
                rotation=rot,
            )
            self.get_logger().info("ILI9341 display initialised (luma.lcd)")
        except Exception as e:
            self.get_logger().error(f"Display init failed: {e}")
            self._disp = None

        if self._disp is not None:
            size = (self._disp.width, self._disp.height)
            if size != (SCREEN_W, SCREEN_H):
                self.get_logger().warning(
                    f"Display reports {size[0]}x{size[1]}; the layout is designed "
                    f"for {SCREEN_W}x{SCREEN_H} landscape (rotation 0 or 180)"
                )

        self._wheel: dict = {}
        self._wheel_ts: float | None = None
        self._mks_bus: dict = {}
        self._actuators: dict[str, dict] = {"lift": {}, "brush": {}, "bin_door": {}}
        self._actuator_ts: dict[str, float] = {}
        self._recorders: dict[str, dict] = {}
        self._recorder_ts: dict[str, float] = {}
        self._gamepad: dict = {}
        self._gamepad_ts: float | None = None
        self._battery: dict | None = None
        self._battery_ts: float | None = None
        self._show_controls = False
        self._net_ifaces: list[dict] | None = None
        self._net_lock = threading.Lock()
        self._poll_network()
        self.create_timer(10.0, self._poll_network)

        self._thermals: list[dict] | None = None
        self._thermal_lock = threading.Lock()
        self._poll_thermals()
        self.create_timer(5.0, self._poll_thermals)

        self.create_subscription(String, "/wheel_status", self._cb_wheel, 10)
        self.create_subscription(String, "/mks_bus/status", self._cb_mks_bus, 10)
        self.create_subscription(ActuatorStateMsg, "/lift/state", lambda m: self._cb_act("lift", m), 10)
        self.create_subscription(ActuatorStateMsg, "/brush/state", lambda m: self._cb_act("brush", m), 10)
        self.create_subscription(ActuatorStateMsg, "/bin_door/state", lambda m: self._cb_act("bin_door", m), 10)
        for name, topic in RECORDER_TOPICS.items():
            self.create_subscription(
                String, topic, lambda m, n=name: self._cb_recorder(n, m), 10
            )
        self.create_subscription(BatteryState, "/battery/status", self._cb_battery, 10)
        self.create_subscription(String, "/gamepad/status", self._cb_gamepad, 10)
        self.create_subscription(String, "/gamepad/button", self._cb_gamepad_btn, 10)

        period = 1.0 / max(0.5, hz)
        self.create_timer(period, self._render)

        self.get_logger().info(f"Status screen running at {hz} Hz")

    def _cb_wheel(self, msg: String):
        try:
            self._wheel = json.loads(msg.data)
            self._wheel_ts = time.monotonic()
        except Exception:
            pass

    def _cb_mks_bus(self, msg: String):
        try:
            self._mks_bus = json.loads(msg.data)
        except Exception:
            pass

    def _cb_act(self, name: str, msg: ActuatorStateMsg):
        self._actuators[name] = {
            "state": int(msg.state),
            "is_referenced": msg.is_referenced,
            "direction": msg.direction,
            "speed_rpm": msg.speed_rpm,
            "error_message": msg.error_message,
        }
        self._actuator_ts[name] = time.monotonic()

    def _cb_recorder(self, name: str, msg: String):
        try:
            self._recorders[name] = json.loads(msg.data)
            self._recorder_ts[name] = time.monotonic()
        except Exception:
            pass

    def _cb_battery(self, msg: BatteryState):
        # Stored verbatim: battery_modbus_node owns calibration, the SoC curve
        # and the shutdown decision. power_supply_status is deliberately not
        # read; the node hardcodes DISCHARGING, which is not a measurement.
        self._battery = {
            "percentage": msg.percentage,
            "voltage": msg.voltage,
            "health": msg.power_supply_health,
            "present": msg.present,
        }
        self._battery_ts = time.monotonic()

    def _cb_gamepad(self, msg: String):
        try:
            self._gamepad = json.loads(msg.data)
            self._gamepad_ts = time.monotonic()
        except Exception:
            pass

    def _cb_gamepad_btn(self, msg: String):
        if msg.data == "home":
            self._show_controls = not self._show_controls

    def _poll_network(self):
        def _bg():
            try:
                ifaces = get_ip_for_display()
            except Exception:
                ifaces = []
            with self._net_lock:
                self._net_ifaces = ifaces
        threading.Thread(target=_bg, daemon=True).start()

    def _poll_thermals(self):
        def _bg():
            zones = []
            thermal_base = Path("/sys/class/thermal")
            if thermal_base.is_dir():
                for zdir in sorted(thermal_base.iterdir()):
                    if not zdir.name.startswith("thermal_zone"):
                        continue
                    try:
                        zone_type = (zdir / "type").read_text().strip()
                        temp_mc = int((zdir / "temp").read_text().strip())
                        zones.append({"type": zone_type, "temp_c": round(temp_mc / 1000.0, 1)})
                    except Exception:
                        continue
            with self._thermal_lock:
                self._thermals = zones
        threading.Thread(target=_bg, daemon=True).start()

    @staticmethod
    def _fresh(ts: float | None, now: float, max_age: float) -> bool:
        return ts is not None and (now - ts) <= max_age

    def snapshot(self) -> ScreenSnapshot:
        now = time.monotonic()
        wheel = dict(self._wheel) if self._fresh(self._wheel_ts, now, WHEEL_STALE_SEC) else None
        actuators = {
            name: dict(state)
            for name, state in self._actuators.items()
            if state and self._fresh(self._actuator_ts.get(name), now, ACTUATOR_STALE_SEC)
        }
        recorders = {
            name: dict(st)
            for name, st in self._recorders.items()
            if self._fresh(self._recorder_ts.get(name), now, RECORDER_STALE_SEC)
        }
        gamepad = (
            dict(self._gamepad)
            if self._fresh(self._gamepad_ts, now, GAMEPAD_STALE_SEC) else None
        )

        motor_temps = {}
        diag = (wheel or {}).get("diag") or {}
        for side, label in (("left", "ODL"), ("right", "ODR")):
            t = diag.get(f"fet_temp_{side}")
            if t is not None:
                motor_temps[label] = t

        with self._net_lock:
            net = None if self._net_ifaces is None else list(self._net_ifaces)
        with self._thermal_lock:
            thermals = None if self._thermals is None else list(self._thermals)

        return ScreenSnapshot(
            clock=time.strftime("%H:%M"),
            wheel=wheel,
            mks_bus=dict(self._mks_bus) or None,
            actuators=actuators,
            battery=battery_readout(
                self._battery, self._battery_ts, now, self._battery_stale
            ),
            thermals=thermals,
            motor_temps=motor_temps,
            network=net,
            gamepad=gamepad,
            recorders=recorders or None,
            resume_button=self._resume_button(),
        )

    def _resume_button(self) -> str:
        for btn, action in (self._gamepad.get("button_map") or {}).items():
            if action == "stop_wheels":
                return btn
        return "B"

    def _render(self):
        if self._disp is None:
            return
        if self._show_controls:
            result = render_controls(self._gamepad)
        else:
            result = render_dashboard(self.snapshot())
        img = result.image
        if (self._disp.width, self._disp.height) == (SCREEN_H, SCREEN_W):
            img = img.transpose(Image.ROTATE_90)
        self._disp.draw_frame(img)


def main(args=None):
    rclpy.init(args=args)
    node = StatusScreenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._disp is not None:
            node._disp.set_backlight(False)
        node.destroy_node()
        rclpy.shutdown()
