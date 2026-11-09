"""Hardware-independent renderer for the 320x240 TFT status screen."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import lru_cache

from PIL import Image, ImageDraw, ImageFont

from .battery_readout import (
    CHARGE_TEXT,
    LEVEL_CRITICAL,
    LEVEL_LOW,
    STATUS_OK,
    BatteryReadout,
    battery_readout,
)


SCREEN_W = 320
SCREEN_H = 240

# Clean, high-contrast light palette for the physical TFT.
COL_BG = (255, 255, 255)       # #FFFFFF
COL_CARD = (245, 247, 250)     # #F5F7FA
COL_BORDER = (226, 232, 240)   # #E2E8F0
COL_TRACK = COL_BORDER
COL_BLUE = (37, 99, 235)
COL_BLUE_LIGHT = (29, 78, 216)
COL_TEXT = (17, 24, 39)        # #111827
COL_TEXT_2 = (100, 116, 139)   # #64748B
COL_MUTED = COL_TEXT_2
COL_GREEN = (22, 163, 74)      # #16A34A
COL_AMBER = (245, 158, 11)     # #F59E0B
COL_RED = (220, 38, 38)        # #DC2626
COL_RED_SOFT = (254, 226, 226)
COL_HIGHLIGHT = (219, 234, 254)

THERMAL_WARN_C = 65.0
THERMAL_CRIT_C = 85.0
_FONT_DIR = "/usr/share/fonts/truetype/dejavu/"


@dataclass(frozen=True)
class Box:
    x0: int
    y0: int
    x1: int
    y1: int

    @property
    def w(self):
        return self.x1 - self.x0

    @property
    def h(self):
        return self.y1 - self.y0

    @property
    def cy(self):
        return (self.y0 + self.y1) / 2

    def inset(self, amount):
        return Box(
            self.x0 + amount,
            self.y0 + amount,
            self.x1 - amount,
            self.y1 - amount,
        )

    def pil(self):
        return (self.x0, self.y0, self.x1 - 1, self.y1 - 1)


REGION_HEADER = Box(0, 0, 320, 22)
REGION_ROBOT = Box(4, 24, 316, 76)
REGION_BATTERY = Box(4, 79, 316, 173)
REGION_TEMPS = Box(4, 176, 316, 197)
REGION_FOOTER = Box(0, 200, 320, 240)


@lru_cache(maxsize=None)
def font(size: int, bold: bool = False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(_FONT_DIR + name, size)
    except OSError:
        return ImageFont.load_default()


@dataclass
class ScreenSnapshot:
    clock: str = "--:--"
    wheel: dict | None = None
    mks_bus: dict | None = None
    actuators: dict = field(default_factory=dict)
    battery: BatteryReadout = field(default_factory=lambda: battery_readout(None))
    thermals: list | None = None
    motor_temps: dict = field(default_factory=dict)
    network: list | None = None
    gamepad: dict | None = None
    recorders: dict | None = None
    resume_button: str = "B"


@dataclass
class RenderResult:
    image: Image.Image
    mode: str


def _text(draw, xy, text, size=10, colour=COL_TEXT, bold=False, anchor="la"):
    draw.text(xy, str(text), font=font(size, bold), fill=colour, anchor=anchor)


def _num(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _level_colour(level):
    if level == LEVEL_CRITICAL:
        return COL_RED
    if level == LEVEL_LOW:
        return COL_AMBER
    return COL_GREEN if level == "ok" else COL_MUTED


def _tile(draw, box, label, state, colour, detail=""):
    draw.rounded_rectangle(box.pil(), 4, fill=COL_CARD, outline=COL_BORDER)
    _text(draw, (box.x0 + 5, box.y0 + 4), label, 9, COL_TEXT_2, True)
    draw.ellipse(
        (box.x0 + 5, box.y0 + 22, box.x0 + 11, box.y0 + 28), fill=colour
    )
    _text(draw, (box.x0 + 15, box.y0 + 25), state, 10, colour, True, "lm")
    if detail:
        _text(draw, (box.x0 + 5, box.y1 - 6), detail[:12], 8, COL_MUTED, False, "ls")


def _wheel_status(wheel):
    if wheel is None:
        return "NO DATA", COL_MUTED, ""
    if wheel.get("estopped"):
        return "E-STOP", COL_RED, ""
    if wheel.get("error"):
        return "ERROR", COL_RED, str(wheel["error"])
    left, right = _num(wheel.get("left_rpm")), _num(wheel.get("right_rpm"))
    detail = str(wheel.get("backend") or "").upper()
    if left is not None and right is not None:
        detail = f"L{left:+.0f} R{right:+.0f}"
        if max(abs(left), abs(right)) >= 0.5:
            return "DRIVING", COL_BLUE_LIGHT, detail
    return "READY", COL_GREEN, detail


def _actuator_status(state):
    if not state:
        return "NO DATA", COL_MUTED, ""
    code = state.get("state")
    if code == 5:
        return "ERROR", COL_RED, str(state.get("error_message") or "fault")
    if code == 1:
        return "HOMING", COL_AMBER, str(state.get("direction") or "")
    if code in (3, 4):
        return "MOVING", COL_BLUE_LIGHT, str(state.get("direction") or "")
    if code == 2:
        return "READY", COL_GREEN, str(state.get("direction") or "")
    return "IDLE", COL_TEXT_2, "homed" if state.get("is_referenced") else "not homed"


def _draw_header(draw, snapshot, fault):
    draw.line((0, 21, 319, 21), fill=COL_BORDER)
    draw.rounded_rectangle((6, 6, 15, 15), 2, fill=COL_BLUE)
    _text(draw, (20, 11), "HORSESHITBOT", 13, COL_TEXT, True, "lm")
    _text(draw, (314, 11), snapshot.clock, 13, COL_TEXT_2, True, "rm")
    if snapshot.wheel and snapshot.wheel.get("estopped"):
        draw.rounded_rectangle((191, 4, 243, 18), 3, fill=COL_RED)
        _text(draw, (217, 11), "E-STOP", 9, COL_TEXT, True, "mm")
    elif fault:
        draw.rounded_rectangle((195, 4, 243, 18), 3, fill=COL_RED)
        _text(draw, (219, 11), "FAULT", 9, COL_TEXT, True, "mm")


def _draw_robot(draw, snapshot):
    estop = bool(snapshot.wheel and snapshot.wheel.get("estopped"))
    offline = [
        str(mid)
        for mid, online in ((snapshot.mks_bus or {}).get("motors") or {}).items()
        if not online
    ]
    faults = bool(offline or (snapshot.wheel or {}).get("error"))
    _draw_header(draw, snapshot, faults)

    items = [("WHEELS", *_wheel_status(snapshot.wheel))]
    for name, label in (("lift", "LIFT"), ("brush", "BRUSH"), ("bin_door", "BIN DOOR")):
        items.append((label, *_actuator_status(snapshot.actuators.get(name))))

    top = REGION_ROBOT.y0
    if estop:
        draw.rounded_rectangle((4, top, 315, top + 19), 4, fill=COL_RED)
        _text(draw, (12, top + 10), "E-STOP ACTIVE", 12, COL_TEXT, True, "lm")
        _text(
            draw,
            (307, top + 10),
            f"Press {snapshot.resume_button} to resume",
            9,
            COL_TEXT,
            False,
            "rm",
        )
        top += 22
    gap = 4
    width = (REGION_ROBOT.w - gap * 3) // 4
    for index, (label, state, colour, detail) in enumerate(items):
        x0 = REGION_ROBOT.x0 + index * (width + gap)
        x1 = REGION_ROBOT.x1 if index == 3 else x0 + width
        if estop:
            draw.rounded_rectangle((x0, top, x1 - 1, 75), 3, fill=COL_CARD)
            _text(draw, (x0 + 4, (top + 76) / 2), label[:4], 8, COL_TEXT_2, True, "lm")
            _text(draw, (x1 - 4, (top + 76) / 2), state, 8, colour, True, "rm")
        else:
            _tile(draw, Box(x0, top, x1, REGION_ROBOT.y1), label, state, colour, detail)
    return "estop" if estop else "dashboard"


def _draw_battery(draw, battery):
    region = REGION_BATTERY
    colour = _level_colour(battery.level)
    live = battery.status == STATUS_OK
    draw.rounded_rectangle(region.pil(), 6, fill=COL_CARD, outline=COL_BORDER)
    _text(draw, (12, 92), "BATTERY", 10, COL_TEXT_2, True)
    if battery.badge:
        badge_width = max(43, font(9, True).getlength(battery.badge) + 10)
        draw.rounded_rectangle((71, 85, 71 + badge_width, 99), 3, outline=colour)
        _text(draw, (71 + badge_width / 2, 92), battery.badge, 9, colour, True, "mm")
    _text(draw, (308, 92), CHARGE_TEXT, 10, COL_MUTED, False, "ra")

    # Pack icon.
    draw.rounded_rectangle((12, 113, 44, 137), 3, outline=colour, width=2)
    draw.rectangle((45, 120, 48, 130), fill=colour)
    fill_width = int(24 * battery.bar_fraction)
    if fill_width:
        draw.rectangle((16, 117, 16 + fill_width, 133), fill=colour)

    _text(draw, (60, 141), battery.percent_text, 38, colour, True, "ls")
    _text(
        draw,
        (308, 139),
        battery.voltage_text,
        20,
        COL_TEXT if live else COL_MUTED,
        True,
        "rs",
    )
    _text(draw, (308, 113), "VOLTAGE", 9, COL_MUTED, True, "rs")

    draw.rounded_rectangle((12, 149, 307, 162), 5, fill=COL_TRACK)
    bar_width = int(296 * battery.bar_fraction)
    if bar_width:
        draw.rounded_rectangle((12, 149, 12 + max(10, bar_width), 162), 5, fill=colour)


def _draw_temperatures(draw, snapshot):
    region = REGION_TEMPS
    draw.rounded_rectangle(region.pil(), 4, fill=COL_CARD, outline=COL_BORDER)
    values = []
    for item in snapshot.thermals or []:
        value = _num(item.get("temp_c"))
        if value is not None and -40 < value < 150:
            values.append((str(item.get("type", "?")), value))
    for label, raw in snapshot.motor_temps.items():
        value = _num(raw)
        if value is not None and -40 < value < 150:
            values.append((label, value))
    if not values:
        _text(draw, (12, region.cy), "TEMP", 10, COL_TEXT_2, True, "lm")
        _text(draw, (50, region.cy), "no data", 10, COL_MUTED, False, "lm")
        return
    selected = values[:2]
    if len(values) > 2:
        selected.append(("MAX", max(value for _, value in values)))
    for index, (name, value) in enumerate(selected):
        colour = COL_RED if value >= THERMAL_CRIT_C else COL_AMBER if value >= THERMAL_WARN_C else COL_TEXT
        _text(draw, (12 + index * 100, region.cy), f"{name[:3].upper()} {value:.0f}°", 11, colour, True, "lm")


def _draw_footer(draw, snapshot):
    region = REGION_FOOTER
    draw.rectangle(region.pil(), fill=COL_CARD)
    draw.line((0, 200, 319, 200), fill=COL_BORDER)
    connected = [
        interface
        for interface in snapshot.network or []
        if interface.get("connected") and interface.get("ip")
    ]
    if connected:
        interface = connected[0]
        tag = "ETH" if interface.get("type") == "ethernet" else "WIFI"
        _text(draw, (12, 212), f"{tag} {interface['ip']}", 10, COL_GREEN, False, "lm")
    else:
        _text(draw, (12, 212), "NET --", 10, COL_MUTED, False, "lm")

    gamepad = snapshot.gamepad
    pad_text = str(gamepad.get("name") or "connected") if gamepad and gamepad.get("connected") else "--"
    pad_colour = COL_GREEN if gamepad and gamepad.get("connected") else COL_MUTED
    _text(draw, (12, 230), f"PAD {pad_text[:18]}", 9, pad_colour, False, "lm")

    active = []
    for name, status in sorted((snapshot.recorders or {}).items()):
        if status.get("recording"):
            minutes, seconds = divmod(int(_num(status.get("duration_sec")) or 0), 60)
            active.append(f"{name[:3].upper()} {minutes}:{seconds:02d}")
    _text(
        draw,
        (307, 230),
        "REC " + (" ".join(active) if active else "idle"),
        9,
        COL_RED if active else COL_MUTED,
        False,
        "rm",
    )
    _text(draw, (307, 212), "Home: controls", 9, COL_MUTED, False, "rm")


def render_dashboard(snapshot: ScreenSnapshot) -> RenderResult:
    image = Image.new("RGB", (SCREEN_W, SCREEN_H), COL_BG)
    draw = ImageDraw.Draw(image)
    mode = _draw_robot(draw, snapshot)
    _draw_battery(draw, snapshot.battery)
    _draw_temperatures(draw, snapshot)
    _draw_footer(draw, snapshot)
    return RenderResult(image, mode)


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
    "perception_bag_recording": "Camera rec",
    "mapping_bag_recording": "Mapping rec",
    "toggle_controls": "Controls",
    "none": "--",
}


def render_controls(gamepad: dict | None) -> RenderResult:
    gamepad = gamepad or {}
    image = Image.new("RGB", (SCREEN_W, SCREEN_H), COL_BG)
    draw = ImageDraw.Draw(image)
    draw.line((0, 21, 319, 21), fill=COL_BORDER)
    _text(draw, (12, 11), "CONTROLS", 13, COL_TEXT, True, "lm")
    _text(draw, (308, 11), "[Home] back", 9, COL_MUTED, False, "rm")
    rows = [("L Stick", "Drive"), ("R Stick", "--")]
    rows.extend(
        (button, ACTION_LABELS.get(action, action))
        for button, action in (gamepad.get("button_map") or {}).items()
    )
    active = set(gamepad.get("active_inputs") or [])
    per_column = max(1, math.ceil(len(rows) / 2))
    row_height = min(19, 212 // per_column)
    for index, (button, action) in enumerate(rows):
        column, row = divmod(index, per_column)
        x0 = 4 + column * 158
        y0 = 25 + row * row_height
        box = (x0, y0, x0 + 153, y0 + row_height - 1)
        draw.rounded_rectangle(
            box, 3, fill=COL_HIGHLIGHT if button in active else COL_CARD
        )
        _text(draw, (x0 + 6, y0 + row_height / 2), button.replace("D-Pad ", "D-"), 9, COL_BLUE_LIGHT, True, "lm")
        _text(draw, (x0 + 57, y0 + row_height / 2), action, 9, COL_TEXT, False, "lm")
    if not rows[2:]:
        _text(draw, (12, 230), "No gamepad button map yet", 10, COL_MUTED, False, "lm")
    return RenderResult(image, "controls")
