"""
Status Screen Node — subscribes to wheel_status and actuator states,
renders a live dashboard on the on-robot ILI9341 SPI TFT at ~5 Hz.

Press the Xbox/Home button on the gamepad to toggle the controls overlay.
The controls screen is built dynamically from the gamepad's button_map config.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from horseshitbot_interfaces.msg import ActuatorState as ActuatorStateMsg
from ..drivers.ili9341_display import ILI9341Display, default_display_pins
from ..drivers.network_manager import get_ip_for_display

from PIL import Image, ImageDraw

STATE_NAMES = ["IDLE", "REFERENCING", "REFERENCED", "MOVING_OPEN", "MOVING_CLOSE", "ERROR"]
STATE_COLOURS = [
    (138, 138, 154),  # IDLE — grey
    (240, 201, 41),   # REFERENCING — yellow
    (78, 204, 163),   # REFERENCED — green
    (240, 201, 41),   # MOVING_OPEN — yellow
    (240, 201, 41),   # MOVING_CLOSE — yellow
    (233, 69, 96),    # ERROR — red
]

COL_BG = (10, 10, 30)
COL_HEADER = (15, 30, 60)
COL_SECTION = (15, 50, 96)
COL_ROW = (15, 35, 55)
COL_FOOTER = (15, 30, 60)
COL_OK = (78, 204, 163)
COL_ACCENT = (233, 69, 96)
COL_TEXT = (220, 220, 220)
COL_MUTED = (138, 138, 154)
COL_BAR_BG = (50, 50, 70)
COL_WARN = (240, 201, 41)
COL_HIGHLIGHT = (40, 70, 120)

THERMAL_WARN_C = 65.0
THERMAL_CRIT_C = 85.0

MAX_RPM = 500.0

ACTION_LABELS = {
    "e_stop": "E-STOP (fast)",
    "stop_wheels": "Stop wheels",
    "stop_actuators": "Stop actuators",
    "brush": "Brush (hold)",
    "bin_door": "Bin door (hold)",
    "lift_up": "Lift up (hold)",
    "lift_down": "Lift down (hold)",
    "reference_all": "Reference all",
    "camera_bag_recording": "Camera rec",
    "toggle_controls": "Controls",
    "none": "--",
}

FIXED_AXES = [
    ("L Stick", "Drive"),
    ("R Stick", "--"),
]


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

        spi_port = self.get_parameter("spi_port").get_parameter_value().integer_value
        spi_dev = self.get_parameter("spi_device").get_parameter_value().integer_value
        dc = self.get_parameter("dc_pin").get_parameter_value().integer_value
        rst = self.get_parameter("rst_pin").get_parameter_value().integer_value
        led = self.get_parameter("led_pin").get_parameter_value().integer_value
        rot = self.get_parameter("rotation").get_parameter_value().integer_value
        hz = self.get_parameter("refresh_hz").get_parameter_value().double_value

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

        self._wheel = {}
        self._mks_bus: dict = {}
        self._actuators = {"lift": {}, "brush": {}, "bin_door": {}}
        self._bag_recorder: dict = {}
        self._gamepad: dict = {}
        self._show_controls = False
        self._net_ifaces: list[dict] = []
        self._net_lock = threading.Lock()
        self._poll_network()
        self.create_timer(10.0, self._poll_network)

        self._thermals: list[dict] = []
        self._thermal_lock = threading.Lock()
        self._poll_thermals()
        self.create_timer(5.0, self._poll_thermals)

        self.create_subscription(String, "/wheel_status", self._cb_wheel, 10)
        self.create_subscription(String, "/mks_bus/status", self._cb_mks_bus, 10)
        self.create_subscription(ActuatorStateMsg, "/lift/state", lambda m: self._cb_act("lift", m), 10)
        self.create_subscription(ActuatorStateMsg, "/brush/state", lambda m: self._cb_act("brush", m), 10)
        self.create_subscription(ActuatorStateMsg, "/bin_door/state", lambda m: self._cb_act("bin_door", m), 10)
        self.create_subscription(String, "/bag_recorder_node/status", self._cb_bag_recorder, 10)
        self.create_subscription(String, "/gamepad/status", self._cb_gamepad, 10)
        self.create_subscription(String, "/gamepad/button", self._cb_gamepad_btn, 10)

        period = 1.0 / max(0.5, hz)
        self.create_timer(period, self._render)
        self._start_time = time.monotonic()

        self.get_logger().info(f"Status screen running at {hz} Hz")

    def _cb_wheel(self, msg: String):
        try:
            self._wheel = json.loads(msg.data)
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

    def _cb_bag_recorder(self, msg: String):
        try:
            self._bag_recorder = json.loads(msg.data)
        except Exception:
            pass

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

    def _render(self):
        if self._disp is None:
            return

        if self._show_controls:
            self._render_controls()
        else:
            self._render_dashboard()

    # ── Controls screen ───────────────────────────────────────────

    def _render_controls(self):
        W = self._disp.width
        H = self._disp.height
        img = self._disp.new_image(COL_BG)
        draw = ImageDraw.Draw(img)
        font_sm = self._disp.font_sm()
        font_lg = self._disp.font_lg()

        active = set(self._gamepad.get("active_inputs", []))
        button_map = self._gamepad.get("button_map", {})

        rows: list[tuple[str, str]] = list(FIXED_AXES)
        for btn, action in button_map.items():
            desc = ACTION_LABELS.get(action, action)
            rows.append((btn, desc))

        # Header
        draw.rectangle([0, 0, W, 22], fill=COL_HEADER)
        draw.text((6, 2), "CONTROLS", fill=COL_ACCENT, font=font_lg)

        footer_h = 16
        usable = H - 28 - footer_h
        row_h = max(14, min(19, usable // max(len(rows), 1)))

        y = 28
        for i, (btn, desc) in enumerate(rows):
            if y + row_h > H - footer_h:
                break
            is_active = btn in active
            if is_active:
                row_bg = COL_HIGHLIGHT
                btn_col = COL_OK
                act_col = COL_OK
            else:
                row_bg = COL_ROW if (i % 2 == 0) else COL_BG
                btn_col = COL_WARN
                act_col = COL_TEXT

            draw.rectangle([0, y, W, y + row_h - 1], fill=row_bg)
            if is_active:
                draw.rectangle([0, y, 3, y + row_h - 1], fill=COL_OK)
            draw.text((6, y + 2), btn, fill=btn_col, font=font_sm)
            draw.text((90, y + 2), desc, fill=act_col, font=font_sm)
            y += row_h

        draw.rectangle([0, H - footer_h, W, H], fill=COL_FOOTER)
        draw.text((6, H - footer_h + 2), "[Home] back to status", fill=COL_MUTED, font=font_sm)

        self._disp.draw_frame(img)

    # ── Dashboard screen ──────────────────────────────────────────

    def _render_dashboard(self):
        W = self._disp.width
        H = self._disp.height
        img = self._disp.new_image(COL_BG)
        draw = ImageDraw.Draw(img)
        font = self._disp.font()
        font_sm = self._disp.font_sm()
        font_lg = self._disp.font_lg()

        uptime = int(time.monotonic() - self._start_time)
        mins, secs = divmod(uptime, 60)

        # Header — 22px
        draw.rectangle([0, 0, W, 22], fill=COL_HEADER)
        draw.text((6, 2), "HORSESHITBOT", fill=COL_ACCENT, font=font_lg)
        draw.text((W - 48, 4), f"{mins:02d}:{secs:02d}", fill=COL_MUTED, font=font_sm)

        # Wheels — single line (e-stop gets two lines)
        y = 24
        w = self._wheel
        backend = w.get("backend", "--")
        left = w.get("left_rpm", 0)
        right = w.get("right_rpm", 0)
        estopped = w.get("estopped", False)

        if estopped:
            draw.rectangle([0, y, W, y + 28], fill=(100, 10, 10))
            draw.text((6, y + 1), "E-STOP ACTIVE", fill=COL_ACCENT, font=font_lg)
            draw.text((6, y + 16), "Press B to resume", fill=COL_TEXT, font=font_sm)
            y += 30
        else:
            draw.rectangle([0, y, W, y + 14], fill=COL_SECTION)
            draw.text((6, y + 1), f"WHEELS [{backend}]", fill=COL_TEXT, font=font_sm)
            draw.text((W // 2 - 40, y + 1), f"L:{left:+.0f}", fill=COL_OK, font=font_sm)
            draw.text((W // 2 + 30, y + 1), f"R:{right:+.0f}", fill=COL_OK, font=font_sm)
            y += 16

        # Actuators — compact 16px rows
        for name, label in [("lift", "LIFT"), ("brush", "BRUSH"), ("bin_door", "BIN DOOR")]:
            a = self._actuators.get(name, {})
            st_idx = a.get("state", 0)
            st_name = STATE_NAMES[st_idx] if st_idx < len(STATE_NAMES) else "?"
            st_col = STATE_COLOURS[st_idx] if st_idx < len(STATE_COLOURS) else COL_MUTED
            direction = a.get("direction", "---")

            draw.rectangle([0, y, W, y + 14], fill=COL_ROW)
            draw.text((6, y + 1), label, fill=COL_TEXT, font=font_sm)
            draw.text((90, y + 1), f"[{st_name}]", fill=st_col, font=font_sm)
            draw.text((W - 46, y + 1), direction.upper()[:5], fill=COL_TEXT, font=font_sm)
            y += 16

        # Thermals — single line (Jetson zones + ODrive FET temps)
        with self._thermal_lock:
            thermals = list(self._thermals)
        diag = w.get("diag", {})
        for side, label in [("left", "ODL"), ("right", "ODR")]:
            t = diag.get(f"fet_temp_{side}")
            if t is not None:
                thermals.append({"type": label, "temp_c": t})
        if thermals:
            y += 2
            draw.rectangle([0, y, W, y + 14], fill=COL_SECTION)
            draw.text((6, y + 1), "TEMP", fill=COL_TEXT, font=font_sm)
            tx = 40
            for tz in thermals:
                try:
                    temp = tz.get("temp_c", 0.0)
                    if temp >= THERMAL_CRIT_C:
                        tc = COL_ACCENT
                    elif temp >= THERMAL_WARN_C:
                        tc = COL_WARN
                    else:
                        tc = COL_OK
                    name = tz.get("type", "?").replace("-thermal", "")
                    tag = name[:3].upper()
                    draw.text((tx, y + 1), f"{tag}:{temp:.0f}", fill=tc, font=font_sm)
                    tx += 46
                except Exception:
                    pass
            y += 16

        # Footer
        footer_rows = 5
        row_h = 14
        footer_h = footer_rows * row_h + 6
        y = H - footer_h
        draw.rectangle([0, y, W, H], fill=COL_FOOTER)
        fy = y + 2

        # Network status
        with self._net_lock:
            net = list(self._net_ifaces)
        if net:
            parts = []
            for iface in net:
                tag = "W" if iface["type"] == "wifi" else "E"
                ip = iface.get("ip", "")
                if iface["connected"] and ip:
                    label = iface.get("ssid") or iface["name"]
                    if len(label) > 10:
                        label = label[:9] + "."
                    parts.append((f"{tag}:{label}", ip, COL_OK))
                else:
                    parts.append((f"{tag}:{iface['name']}", "--", COL_MUTED))
            for tag_str, ip_str, col in parts[:2]:
                draw.ellipse([6, fy + 3, 12, fy + 9], fill=col)
                draw.text((16, fy), tag_str, fill=col, font=font_sm)
                draw.text((110, fy), ip_str, fill=col, font=font_sm)
                fy += row_h
        else:
            draw.text((6, fy), "NET: scanning...", fill=COL_MUTED, font=font_sm)
            fy += row_h

        # Gamepad status
        gp_conn = self._gamepad.get("connected", False)
        gp_age = time.monotonic() - getattr(self, "_gamepad_ts", 0)
        gp_alive = gp_conn and gp_age < 3.0
        if gp_alive:
            gp_name = self._gamepad.get("name", "Gamepad")
            if len(gp_name) > 22:
                gp_name = gp_name[:20] + ".."
            draw.ellipse([6, fy + 3, 12, fy + 9], fill=COL_OK)
            draw.text((16, fy), gp_name, fill=COL_OK, font=font_sm)
        else:
            draw.ellipse([6, fy + 3, 12, fy + 9], fill=COL_WARN)
            draw.text((16, fy), "GAMEPAD: --", fill=COL_WARN, font=font_sm)
        fy += row_h

        # Recording status
        is_recording = self._bag_recorder.get("recording", False)
        if is_recording:
            dur = self._bag_recorder.get("duration_sec", 0)
            m, s = divmod(int(dur), 60)
            frames = self._bag_recorder.get("frame_count", 0)
            draw.ellipse([6, fy + 3, 12, fy + 9], fill=COL_ACCENT)
            draw.text((16, fy), f"REC {m}:{s:02d}  {frames}f", fill=COL_ACCENT, font=font_sm)
        else:
            draw.ellipse([6, fy + 3, 12, fy + 9], fill=COL_MUTED)
            draw.text((16, fy), "REC: idle", fill=COL_MUTED, font=font_sm)
        fy += row_h

        # Error line or Home hint
        err_parts: list[str] = []
        vbus = diag.get("vbus", 0)
        if 0 < vbus < 10:
            err_parts.append(f"ODrive no pwr ({vbus:.0f}V)")
        wheel_err = self._wheel.get("error", "")
        if wheel_err:
            err_parts.append(wheel_err)
        motors = self._mks_bus.get("motors", {})
        offline = [mid for mid, ok in motors.items() if not ok]
        if offline:
            err_parts.append(f"MKS {','.join(offline)} offline")
        if not self._mks_bus.get("bus_connected", True) and motors:
            err_parts.append("MKS bus down")

        if err_parts:
            err_text = " | ".join(err_parts)
            if len(err_text) > 35:
                err_text = err_text[:33] + ".."
            draw.text((6, fy), err_text, fill=COL_ACCENT, font=font_sm)
        else:
            draw.text((6, fy), "[Home] controls", fill=COL_MUTED, font=font_sm)

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
