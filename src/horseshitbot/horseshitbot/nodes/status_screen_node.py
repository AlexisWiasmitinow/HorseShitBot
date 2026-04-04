"""
Status Screen Node — subscribes to wheel_status and actuator states,
renders a live dashboard on the on-robot ILI9341 SPI TFT at ~5 Hz.
"""

from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from horseshitbot.msg import ActuatorState as ActuatorStateMsg
from ..drivers.ili9341_display import ILI9341Display

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

MAX_RPM = 500.0


class StatusScreenNode(Node):
    def __init__(self):
        super().__init__("status_screen_node")

        self.declare_parameter("spi_port", 0)
        self.declare_parameter("spi_cs", 0)
        self.declare_parameter("dc_pin", 24)
        self.declare_parameter("rst_pin", 25)
        self.declare_parameter("led_pin", 18)
        self.declare_parameter("rotation", 0)
        self.declare_parameter("refresh_hz", 5.0)
        self.declare_parameter("backlight_brightness", 1.0)

        cs = self.get_parameter("spi_cs").get_parameter_value().integer_value
        dc = self.get_parameter("dc_pin").get_parameter_value().integer_value
        rst = self.get_parameter("rst_pin").get_parameter_value().integer_value
        led = self.get_parameter("led_pin").get_parameter_value().integer_value
        rot = self.get_parameter("rotation").get_parameter_value().integer_value
        hz = self.get_parameter("refresh_hz").get_parameter_value().double_value

        # Map spi_cs 0 → GPIO 8 (CE0), 1 → GPIO 7 (CE1)
        cs_gpio = 8 if cs == 0 else 7

        try:
            self._disp = ILI9341Display(
                cs_pin=cs_gpio, dc_pin=dc, rst_pin=rst, led_pin=led, rotation=rot
            )
            self.get_logger().info("ILI9341 display initialised")
        except Exception as e:
            self.get_logger().error(f"Display init failed: {e}")
            self._disp = None

        self._wheel = {}
        self._actuators = {"lift": {}, "brush": {}, "bin_door": {}}
        self._bag_recorder: dict = {}

        self.create_subscription(String, "/wheel_status", self._cb_wheel, 10)
        self.create_subscription(ActuatorStateMsg, "/lift/state", lambda m: self._cb_act("lift", m), 10)
        self.create_subscription(ActuatorStateMsg, "/brush/state", lambda m: self._cb_act("brush", m), 10)
        self.create_subscription(ActuatorStateMsg, "/bin_door/state", lambda m: self._cb_act("bin_door", m), 10)
        self.create_subscription(String, "/bag_recorder_node/status", self._cb_bag_recorder, 10)

        period = 1.0 / max(0.5, hz)
        self.create_timer(period, self._render)
        self._start_time = time.monotonic()

        self.get_logger().info(f"Status screen running at {hz} Hz")

    def _cb_wheel(self, msg: String):
        try:
            self._wheel = json.loads(msg.data)
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

    def _render(self):
        if self._disp is None:
            return

        W = self._disp.width
        H = self._disp.height
        img = self._disp.new_image(COL_BG)
        draw = ImageDraw.Draw(img)
        font = self._disp.font()
        font_sm = self._disp.font_sm()
        font_lg = self._disp.font_lg()

        uptime = int(time.monotonic() - self._start_time)
        mins, secs = divmod(uptime, 60)

        # Header
        draw.rectangle([0, 0, W, 28], fill=COL_HEADER)
        draw.text((8, 5), "HORSESHITBOT", fill=COL_ACCENT, font=font_lg)
        draw.text((W - 50, 8), f"{mins:02d}:{secs:02d}", fill=COL_MUTED, font=font_sm)

        # Wheels
        y = 34
        w = self._wheel
        backend = w.get("backend", "--")
        left = w.get("left_rpm", 0)
        right = w.get("right_rpm", 0)

        draw.rectangle([0, y, W, y + 50], fill=COL_SECTION)
        draw.text((8, y + 4), f"WHEELS [{backend}]", fill=COL_TEXT, font=font)
        draw.text((8, y + 22), f"L: {left:+.0f}", fill=COL_OK, font=font)
        draw.text((W // 2, y + 22), f"R: {right:+.0f}", fill=COL_OK, font=font)

        bar_y = y + 40
        bar_w = W // 2 - 20
        for i, rpm in enumerate([abs(left), abs(right)]):
            bx = 8 + i * (W // 2)
            pct = min(1.0, rpm / MAX_RPM) if MAX_RPM > 0 else 0
            draw.rectangle([bx, bar_y, bx + bar_w, bar_y + 6], fill=COL_BAR_BG)
            if pct > 0:
                draw.rectangle([bx, bar_y, bx + int(bar_w * pct), bar_y + 6], fill=COL_OK)

        # Actuators
        y = 92
        for name, label in [("lift", "LIFT"), ("brush", "BRUSH"), ("bin_door", "BIN DOOR")]:
            a = self._actuators.get(name, {})
            st_idx = a.get("state", 0)
            st_name = STATE_NAMES[st_idx] if st_idx < len(STATE_NAMES) else "?"
            st_col = STATE_COLOURS[st_idx] if st_idx < len(STATE_COLOURS) else COL_MUTED
            direction = a.get("direction", "---")

            draw.rectangle([0, y, W, y + 28], fill=COL_ROW)
            draw.text((8, y + 6), label, fill=COL_TEXT, font=font)
            draw.text((110, y + 6), f"[{st_name}]", fill=st_col, font=font_sm)
            draw.text((W - 50, y + 6), direction.upper()[:5], fill=COL_TEXT, font=font)
            y += 30

        # Footer
        y = H - 50
        draw.rectangle([0, y, W, H], fill=COL_FOOTER)

        is_recording = self._bag_recorder.get("recording", False)
        if is_recording:
            dur = self._bag_recorder.get("duration_sec", 0)
            m, s = divmod(int(dur), 60)
            frames = self._bag_recorder.get("frame_count", 0)
            draw.ellipse([8, y + 7, 18, y + 17], fill=COL_ACCENT)
            draw.text((22, y + 6), f"REC {m}:{s:02d}  {frames}f", fill=COL_ACCENT, font=font_sm)
        else:
            draw.ellipse([8, y + 7, 18, y + 17], fill=COL_MUTED)
            draw.text((22, y + 6), "REC: idle", fill=COL_MUTED, font=font_sm)

        draw.text((8, y + 22), "ERRORS: None", fill=COL_OK, font=font_sm)

        self._disp.draw_frame(img)


def main(args=None):
    rclpy.init(args=args)
    node = StatusScreenNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
