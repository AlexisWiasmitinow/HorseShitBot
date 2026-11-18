"""Renderer for the 320x240 landscape ILI9341 status screen.

Pure Pillow: no ROS, GPIO or SPI access. status_screen_node gathers telemetry
into a ScreenSnapshot and blits the returned image; the preview script and the
tests call the same functions, so previews are pixel-identical to the panel.

The clean light palette comes from the physically reviewed TFT design in
commit 51a4d41. Every section owns a fixed Box and nothing is drawn outside it;
each drawn element is recorded so geometry can be tested without hardware.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import lru_cache

from PIL import Image, ImageDraw, ImageFont

from .battery_readout import (
    CHARGE_TEXT,
    LEVEL_CRITICAL,
    LEVEL_LOW,
    LEVEL_OK,
    STATUS_OK,
    BatteryReadout,
    battery_readout,
)

SCREEN_W = 320
SCREEN_H = 240

# ── Palette (established clean white TFT design, commit 51a4d41) ─────
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
COL_RED_TEXT = COL_RED
COL_RED_SOFT = (254, 226, 226)
COL_HIGHLIGHT = (219, 234, 254)

THERMAL_WARN_C = 65.0
THERMAL_CRIT_C = 85.0


@dataclass(frozen=True)
class Box:
    """Half-open pixel rectangle: x0 <= x < x1, y0 <= y < y1."""

    x0: int
    y0: int
    x1: int
    y1: int

    @property
    def w(self) -> int:
        return self.x1 - self.x0

    @property
    def h(self) -> int:
        return self.y1 - self.y0

    @property
    def cy(self) -> float:
        return (self.y0 + self.y1) / 2

    def inset(self, dx: int, dy: int | None = None) -> "Box":
        dy = dx if dy is None else dy
        return Box(self.x0 + dx, self.y0 + dy, self.x1 - dx, self.y1 - dy)

    def contains(self, bbox) -> bool:
        x0, y0, x1, y1 = bbox
        return self.x0 <= x0 and self.y0 <= y0 and x1 <= self.x1 and y1 <= self.y1

    def intersects(self, other: "Box") -> bool:
        return (
            self.x0 < other.x1 and other.x0 < self.x1
            and self.y0 < other.y1 and other.y0 < self.y1
        )

    def pil(self):
        """Pillow's inclusive rectangle coordinates."""
        return [self.x0, self.y0, self.x1 - 1, self.y1 - 1]


SCREEN = Box(0, 0, SCREEN_W, SCREEN_H)

# Fixed layout. Heights: 22 + 52 + 94 + 21 + 40 plus 2-3 px gutters = 240.
REGION_HEADER = Box(0, 0, 320, 22)
REGION_ROBOT = Box(4, 24, 316, 76)
REGION_BATTERY = Box(4, 79, 316, 173)
REGION_TEMPS = Box(4, 176, 316, 197)
REGION_FOOTER = Box(0, 200, 320, 240)

DASHBOARD_REGIONS = {
    "header": REGION_HEADER,
    "robot": REGION_ROBOT,
    "battery": REGION_BATTERY,
    "temps": REGION_TEMPS,
    "footer": REGION_FOOTER,
}

TILE_GAP = 4
ROW_GAP = 2

# ── Fonts ─────────────────────────────────────────────────────────────
_FONT_DIR = "/usr/share/fonts/truetype/dejavu/"


@lru_cache(maxsize=None)
def font(size: int, bold: bool = False):
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    try:
        return ImageFont.truetype(_FONT_DIR + name, size)
    except OSError:
        try:
            return ImageFont.load_default(size)
        except TypeError:
            return ImageFont.load_default()


def fonts_available() -> bool:
    return isinstance(font(12), ImageFont.FreeTypeFont)


def text_width(text: str, fnt) -> float:
    return fnt.getlength(text)


def ellipsize(text: str, fnt, max_w: float) -> str:
    """Trim ``text`` with a trailing ellipsis so it renders within max_w px."""
    if max_w <= 0:
        return ""
    if text_width(text, fnt) <= max_w:
        return text
    ell = "\u2026"
    for n in range(len(text) - 1, 0, -1):
        candidate = text[:n].rstrip() + ell
        if text_width(candidate, fnt) <= max_w:
            return candidate
    return ell if text_width(ell, fnt) <= max_w else ""


def fit_font(text: str, sizes, max_w: float, bold: bool = True):
    """Largest font from ``sizes`` that fits; the smallest one otherwise."""
    for size in sizes:
        fnt = font(size, bold)
        if text_width(text, fnt) <= max_w:
            return fnt
    return font(sizes[-1], bold)


# ── Drawing with a geometry log ───────────────────────────────────────


@dataclass(frozen=True)
class DrawnItem:
    region: str
    kind: str       # "text" | "shape"
    bbox: tuple     # half-open (x0, y0, x1, y1)
    limit: Box
    text: str = ""
    source: str = ""  # text before any ellipsis

    @property
    def truncated(self) -> bool:
        return self.kind == "text" and self.text != self.source


@dataclass
class RenderResult:
    image: Image.Image
    items: list = field(default_factory=list)
    mode: str = ""

    def texts(self, region: str | None = None) -> list:
        return [
            it for it in self.items
            if it.kind == "text" and (region is None or it.region == region)
        ]

    def all_text(self) -> str:
        return " | ".join(it.text for it in self.texts())


class _Canvas:
    def __init__(self, bg=COL_BG):
        self.image = Image.new("RGB", (SCREEN_W, SCREEN_H), bg)
        self.draw = ImageDraw.Draw(self.image)
        self.items: list[DrawnItem] = []

    def rect(self, region: str, box: Box, fill=None, outline=None, radius=0, width=1):
        if box.w <= 0 or box.h <= 0:
            return
        if radius:
            self.draw.rounded_rectangle(
                box.pil(), radius=radius, fill=fill, outline=outline, width=width
            )
        else:
            self.draw.rectangle(box.pil(), fill=fill, outline=outline, width=width)
        self.items.append(DrawnItem(region, "shape", (box.x0, box.y0, box.x1, box.y1), box))

    def dot(self, region: str, cx: float, cy: float, r: float, fill):
        box = Box(int(round(cx - r)), int(round(cy - r)),
                  int(round(cx + r)) + 1, int(round(cy + r)) + 1)
        self.draw.ellipse(box.pil(), fill=fill)
        self.items.append(DrawnItem(region, "shape", (box.x0, box.y0, box.x1, box.y1), box))

    def text(self, region: str, limit: Box, xy, text: str, fnt, fill, anchor="lm"):
        """Draw text anchored at ``xy``, ellipsized to stay inside ``limit``."""
        x, _ = xy
        h = anchor[0]
        if h == "l":
            max_w = limit.x1 - x
        elif h == "r":
            max_w = x - limit.x0
        else:
            max_w = 2 * min(x - limit.x0, limit.x1 - x)
        shown = ellipsize(text, fnt, max_w)
        while shown:
            l, t, r, b = self.draw.textbbox(xy, shown, font=fnt, anchor=anchor)
            # Glyph side bearings can overhang the anchor by a pixel; nudge
            # the text back inside rather than let it touch a neighbour.
            dx = min(0, limit.x1 - r) or max(0, limit.x0 - l)
            if limit.x0 <= l + dx and r + dx <= limit.x1:
                break
            shown = ellipsize(text, fnt, text_width(shown, fnt) - 1)
        if not shown:
            return Box(int(x), int(xy[1]), int(x), int(xy[1]))
        xy = (xy[0] + dx, xy[1])
        self.draw.text(xy, shown, font=fnt, fill=fill, anchor=anchor)
        l, t, r, b = self.draw.textbbox(xy, shown, font=fnt, anchor=anchor)
        self.items.append(DrawnItem(region, "text", (l, t, r, b), limit, shown, text))
        return Box(l, t, r, b)


# ── Telemetry snapshot ────────────────────────────────────────────────

ACTUATORS = (
    ("lift", "LIFT", "LIFT"),
    ("brush", "BRUSH", "BRSH"),
    ("bin_door", "BIN DOOR", "DOOR"),
)

RECORDER_SHORT = {"perception_recorder": "CAM", "mapping_recorder": "MAP"}


@dataclass
class ScreenSnapshot:
    """Fresh telemetry only. None means "no fresh message", never "fine"."""

    clock: str = "--:--"
    wheel: dict | None = None
    mks_bus: dict | None = None
    actuators: dict = field(default_factory=dict)
    battery: BatteryReadout = field(default_factory=lambda: battery_readout(None))
    thermals: list | None = None        # None until the first sysfs poll
    motor_temps: dict = field(default_factory=dict)
    network: list | None = None         # None until the first poll
    gamepad: dict | None = None
    recorders: dict | None = None       # recorder name -> status dict
    resume_button: str = "B"


@dataclass(frozen=True)
class TileInfo:
    label: str
    short: str
    status: str
    colour: tuple
    detail: str = ""
    detail_colour: tuple = COL_MUTED
    detail_short: str = ""  # used when ``detail`` does not fit


def wheel_tile(wheel: dict | None) -> TileInfo:
    label = "WHEELS"
    if wheel is None:
        return TileInfo(label, "WHL", "NO DATA", COL_MUTED)
    backend = str(wheel.get("backend") or "").upper()
    backend = "" if backend == "--" else backend
    left = _num(wheel.get("left_rpm"))
    right = _num(wheel.get("right_rpm"))
    rpm = f"L{left:+.0f} R{right:+.0f}" if left is not None and right is not None else ""
    detail = f"{backend} {rpm}".strip()
    vbus = _num((wheel.get("diag") or {}).get("vbus"))
    release_error = wheel.get("estop_release_error")
    if release_error:
        return TileInfo(
            label, "WHL", "REL FAIL", COL_RED, str(release_error), COL_RED_TEXT
        )
    if wheel.get("estop_state") == "releasing":
        return TileInfo(label, "WHL", "RELEASING", COL_AMBER, detail, detail_short=rpm)
    if wheel.get("estopped"):
        return TileInfo(label, "WHL", "E-STOP", COL_RED, detail, detail_short=rpm)
    if wheel.get("error"):
        return TileInfo(label, "WHL", "ERROR", COL_RED, str(wheel["error"]), COL_RED_TEXT)
    if vbus is not None and 0 < vbus < 10:
        return TileInfo(label, "WHL", "NO PWR", COL_RED, f"bus {vbus:.0f} V", COL_RED_TEXT)
    if wheel.get("stopping"):
        return TileInfo(label, "WHL", "STOPPING", COL_AMBER, detail, detail_short=rpm)
    if left is not None and right is not None and max(abs(left), abs(right)) >= 0.5:
        return TileInfo(label, "WHL", "DRIVING", COL_BLUE_LIGHT, detail, COL_TEXT_2,
                        detail_short=rpm)
    return TileInfo(label, "WHL", "READY", COL_GREEN, detail, detail_short=rpm)


def actuator_tile(label: str, short: str, act: dict | None) -> TileInfo:
    if not act:
        return TileInfo(label, short, "NO DATA", COL_MUTED)
    state = act.get("state")
    direction = str(act.get("direction") or "").strip()
    if direction in ("---", "--", "-", "none", "stopped"):
        direction = ""
    referenced = bool(act.get("is_referenced"))
    if state == 5:
        msg = str(act.get("error_message") or "fault")
        return TileInfo(label, short, "ERROR", COL_RED, msg, COL_RED_TEXT)
    if state == 1:
        return TileInfo(label, short, "HOMING", COL_AMBER, direction)
    if state in (3, 4):
        return TileInfo(label, short, "MOVING", COL_BLUE_LIGHT,
                        direction or ("open" if state == 3 else "close"), COL_TEXT_2)
    if state == 2:
        return TileInfo(label, short, "READY", COL_GREEN, direction)
    if state == 0:
        if referenced:
            return TileInfo(label, short, "IDLE", COL_TEXT_2, "homed")
        return TileInfo(label, short, "IDLE", COL_TEXT_2, "not homed", COL_AMBER)
    return TileInfo(label, short, "UNKNOWN", COL_MUTED, f"state {state}")


def system_errors(snap: ScreenSnapshot) -> list[str]:
    """Faults not owned by a single tile: backend, bus and power problems."""
    errors: list[str] = []
    wheel = snap.wheel or {}
    if wheel.get("error"):
        errors.append(f"Wheels: {wheel['error']}")
    if wheel.get("estop_release_error"):
        errors.append(f"E-stop release: {wheel['estop_release_error']}")
    vbus = _num((wheel.get("diag") or {}).get("vbus"))
    if vbus is not None and 0 < vbus < 10:
        errors.append(f"ODrive no power ({vbus:.0f} V)")
    mks = snap.mks_bus or {}
    motors = mks.get("motors") or {}
    offline = [str(mid) for mid, ok in motors.items() if not ok]
    if offline:
        errors.append(f"MKS {','.join(offline)} offline")
    if motors and not mks.get("bus_connected", True):
        errors.append("MKS bus down")
    return errors


def actuator_errors(snap: ScreenSnapshot) -> list[str]:
    out = []
    for name, label, _ in ACTUATORS:
        act = snap.actuators.get(name)
        if act and act.get("state") == 5:
            out.append(f"{label.title()}: {act.get('error_message') or 'fault'}")
    return out


def temperature_summary(thermals, motor_temps=None) -> list[tuple[str, float]]:
    """CPU / GPU / MAX from genuinely readable sensors only.

    MAX covers every readable Jetson zone plus motor-driver FETs and is only
    shown when it adds a sensor not already on screen.
    """
    values: dict[str, float] = {}
    for zone in thermals or []:
        t = _valid_temp(zone.get("temp_c"))
        if t is not None:
            values[str(zone.get("type", "?"))] = t
    for label, temp in (motor_temps or {}).items():
        t = _valid_temp(temp)
        if t is not None:
            values[str(label)] = t

    out: list[tuple[str, float]] = []
    shown: set[str] = set()
    for label, key in (("CPU", "cpu"), ("GPU", "gpu")):
        match = next((k for k in values if k.lower().startswith(key)), None)
        if match is not None:
            out.append((label, values[match]))
            shown.add(match)
    if values and len(values) > len(shown):
        out.append(("MAX", max(values.values())))
    return out


def temperature_colour(temp: float):
    if temp >= THERMAL_CRIT_C:
        return COL_RED
    if temp >= THERMAL_WARN_C:
        return COL_AMBER
    return COL_TEXT


def battery_colour(level: str):
    return {
        LEVEL_OK: COL_GREEN,
        LEVEL_LOW: COL_AMBER,
        LEVEL_CRITICAL: COL_RED,
    }.get(level, COL_MUTED)


def _num(value):
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _valid_temp(value):
    v = _num(value)
    if v is None or not (-40.0 < v < 150.0):
        return None
    return v


# ── Dashboard ─────────────────────────────────────────────────────────


def render_dashboard(snap: ScreenSnapshot) -> RenderResult:
    c = _Canvas()
    errors = system_errors(snap)
    act_errors = actuator_errors(snap)
    estop = bool(snap.wheel and snap.wheel.get("estopped"))

    _draw_header(c, snap, estop, bool(errors or act_errors))
    mode = _draw_robot(c, snap, estop, errors, act_errors)
    _draw_battery(c, snap.battery)
    _draw_temps(c, snap)
    _draw_footer(c, snap)
    return RenderResult(c.image, c.items, mode)


def _draw_header(c: _Canvas, snap: ScreenSnapshot, estop: bool, fault: bool):
    R = REGION_HEADER
    c.rect("header", Box(R.x0, R.y1 - 1, R.x1, R.y1), fill=COL_BORDER)
    c.rect("header", Box(6, 6, 16, 16), fill=COL_BLUE, radius=2)
    title = c.text("header", Box(20, R.y0, 200, R.y1 - 1), (20, 11),
                   "HORSESHITBOT", font(13, True), COL_TEXT)
    clock = c.text("header", Box(240, R.y0, R.x1 - 5, R.y1 - 1), (R.x1 - 6, 11),
                   snap.clock, font(13, True), COL_TEXT_2, anchor="rm")

    pill = "E-STOP" if estop else ("FAULT" if fault else None)
    if pill:
        pf = font(9, True)
        pw = int(math.ceil(text_width(pill, pf))) + 10
        px1 = clock.x0 - 8
        box = Box(max(title.x1 + 8, px1 - pw), 4, px1, 18)
        c.rect("header", box, fill=COL_RED, radius=3)
        c.text("header", box, ((box.x0 + box.x1) / 2, 11), pill, pf, COL_TEXT, anchor="mm")


def _draw_robot(c, snap, estop, errors, act_errors) -> str:
    R = REGION_ROBOT
    tiles = [wheel_tile(snap.wheel)] + [
        actuator_tile(label, short, snap.actuators.get(name))
        for name, label, short in ACTUATORS
    ]

    y = R.y0
    if not estop:
        if errors:
            mode, tile_h, strip = "compact", 36, errors + act_errors
        else:
            mode, tile_h, strip = "full", R.h, []
    else:
        strip = errors + act_errors
        banner_h = 20 if strip else 22
        _draw_estop_banner(c, Box(R.x0, y, R.x1, y + banner_h), snap.resume_button)
        y += banner_h + ROW_GAP
        mode, tile_h = ("inline", 14) if strip else ("compact", 28)

    tile_w = (R.w - 3 * TILE_GAP) // 4
    for i, tile in enumerate(tiles):
        x0 = R.x0 + i * (tile_w + TILE_GAP)
        x1 = R.x1 if i == 3 else x0 + tile_w
        _draw_tile(c, Box(x0, y, x1, y + tile_h), tile, mode)
    y += tile_h

    if strip:
        y += ROW_GAP
        _draw_error_strip(c, Box(R.x0, y, R.x1, R.y1), strip)
    return f"{'estop-' if estop else ''}{mode}"


def _draw_estop_banner(c, box: Box, resume_button: str):
    c.rect("robot", box, fill=COL_RED, radius=4)
    inner = box.inset(8, 0)
    title = c.text("robot", inner, (inner.x0, box.cy), "E-STOP ACTIVE",
                   font(13, True), COL_TEXT)
    c.text("robot", Box(title.x1 + 10, box.y0, inner.x1, box.y1), (inner.x1, box.cy),
           f"Press {resume_button} to resume", font(10), COL_TEXT, anchor="rm")


def _draw_tile(c, box: Box, tile: TileInfo, mode: str):
    c.rect("robot", box, fill=COL_CARD, outline=COL_BORDER, radius=4)
    inner = box.inset(4 if mode == "inline" else 5, 0)
    if mode == "inline":
        lbl = c.text("robot", inner, (inner.x0, box.cy), tile.short,
                     font(9, True), COL_TEXT_2)
        status_box = Box(lbl.x1 + 4, box.y0, inner.x1, box.y1)
        sf = fit_font(tile.status, (9, 8), status_box.w)
        c.text("robot", status_box, (inner.x1, box.cy), tile.status, sf,
               tile.colour, anchor="rm")
        return

    c.text("robot", inner, (inner.x0, box.y0 + 4), tile.label,
           font(9, True), COL_TEXT_2, anchor="la")
    status_cy = box.y0 + 22 if mode == "full" else box.y1 - 9
    c.dot("robot", inner.x0 + 3, status_cy, 3, tile.colour)
    status_box = Box(inner.x0 + 10, box.y0 + 14, inner.x1, box.y1)
    sf = fit_font(tile.status, (12, 11, 10), status_box.w)
    c.text("robot", status_box, (status_box.x0, status_cy), tile.status, sf, tile.colour)
    if mode == "full" and tile.detail:
        detail = tile.detail
        if tile.detail_short and text_width(detail, font(9)) > inner.w:
            detail = tile.detail_short
        c.text("robot", Box(inner.x0, box.y0 + 32, inner.x1, box.y1 - 2),
               (inner.x0, box.y1 - 11), detail, font(9), tile.detail_colour)


def _draw_error_strip(c, box: Box, errors: list[str]):
    c.rect("robot", box, fill=COL_RED_SOFT, outline=COL_RED, radius=3)
    inner = box.inset(6, 0)
    c.text("robot", inner, (inner.x0, box.cy), "  |  ".join(errors), font(10), COL_RED_TEXT)


def _draw_battery(c, bat: BatteryReadout):
    R = REGION_BATTERY
    colour = battery_colour(bat.level)
    live = bat.status == STATUS_OK
    c.rect("battery", R, fill=COL_CARD, outline=COL_BORDER, radius=6)
    inner = R.inset(8, 0)

    row_y = R.y0 + 13
    title = c.text("battery", inner, (inner.x0, row_y), "BATTERY", font(10, True), COL_TEXT_2)
    left = title.x1
    if bat.badge:
        bf = font(9, True)
        bw = int(math.ceil(text_width(bat.badge, bf))) + 10
        badge = Box(title.x1 + 6, row_y - 7, min(title.x1 + 6 + bw, inner.x1), row_y + 7)
        if bat.level in (LEVEL_LOW, LEVEL_CRITICAL):
            c.rect("battery", badge, fill=colour, radius=3)
            fg = COL_BG if bat.level == LEVEL_LOW else COL_TEXT
        else:
            c.rect("battery", badge, outline=COL_MUTED, radius=3)
            fg = COL_TEXT_2
        c.text("battery", badge, ((badge.x0 + badge.x1) / 2, row_y), bat.badge, bf, fg,
               anchor="mm")
        left = badge.x1
    c.text("battery", Box(left + 8, R.y0, inner.x1, R.y0 + 24), (inner.x1, row_y),
           CHARGE_TEXT, font(10), COL_MUTED, anchor="rm")

    # Big percentage with a pack icon, voltage right-aligned on the same baseline.
    baseline = R.y0 + 60
    icon = Box(inner.x0, R.y0 + 34, inner.x0 + 38, R.y0 + 57)
    _draw_battery_icon(c, icon, bat.bar_fraction, colour)

    volt_font = font(20, True)
    volt = c.text("battery", Box(inner.x1 - 110, R.y0 + 24, inner.x1, R.y0 + 64),
                  (inner.x1, baseline), bat.voltage_text, volt_font,
                  COL_TEXT if live else COL_MUTED, anchor="rs")
    c.text("battery", Box(volt.x0 - 20, R.y0 + 24, inner.x1, volt.y0 - 1),
           (inner.x1, volt.y0 - 3), "VOLTAGE", font(9, True), COL_MUTED, anchor="rs")
    pct_box = Box(icon.x1 + 12, R.y0 + 22, volt.x0 - 8, R.y0 + 64)
    pf = fit_font(bat.percent_text, (40, 36, 32), pct_box.w)
    c.text("battery", pct_box, (pct_box.x0, baseline), bat.percent_text, pf,
           colour if bat.percent is not None else COL_MUTED, anchor="ls")

    bar = Box(inner.x0, R.y0 + 70, inner.x1, R.y0 + 84)
    c.rect("battery", bar, fill=COL_TRACK, radius=5)
    fill_w = int(round(bar.w * bat.bar_fraction))
    if fill_w > 0:
        c.rect("battery", Box(bar.x0, bar.y0, bar.x0 + max(fill_w, 10), bar.y1),
               fill=colour, radius=5)


def _draw_battery_icon(c, box: Box, fraction: float, colour):
    body = Box(box.x0, box.y0, box.x1 - 4, box.y1)
    c.rect("battery", body, outline=colour, radius=3, width=2)
    c.rect("battery", Box(body.x1, int(box.cy) - 4, box.x1, int(box.cy) + 5),
           fill=colour, radius=1)
    cell = body.inset(4)
    fill_w = int(round(cell.w * max(0.0, min(1.0, fraction))))
    if fill_w > 0:
        c.rect("battery", Box(cell.x0, cell.y0, cell.x0 + max(fill_w, 2), cell.y1),
               fill=colour, radius=1)


def _draw_temps(c, snap: ScreenSnapshot):
    R = REGION_TEMPS
    c.rect("temps", R, fill=COL_CARD, outline=COL_BORDER, radius=4)
    items = temperature_summary(snap.thermals, snap.motor_temps)
    lf, vf = font(10, True), font(12, True)
    if not items:
        inner = R.inset(8, 0)
        t = c.text("temps", inner, (inner.x0, R.cy), "TEMP", lf, COL_TEXT_2)
        msg = "reading\u2026" if snap.thermals is None else "no sensors"
        c.text("temps", Box(t.x1 + 6, R.y0, inner.x1, R.y1), (t.x1 + 8, R.cy),
               msg, font(10), COL_MUTED)
        return
    col_w = R.w / len(items)
    for i, (label, temp) in enumerate(items):
        col = Box(int(R.x0 + i * col_w), R.y0, int(R.x0 + (i + 1) * col_w), R.y1)
        value = f"{temp:.0f}\u00b0"
        total = text_width(label, lf) + 5 + text_width(value, vf)
        x = (col.x0 + col.x1) / 2 - total / 2
        lbl = c.text("temps", col, (x, R.cy), label, lf, COL_TEXT_2)
        c.text("temps", col, (lbl.x1 + 5, R.cy), value, vf, temperature_colour(temp))


def _draw_footer(c, snap: ScreenSnapshot):
    R = REGION_FOOTER
    c.rect("footer", R, fill=COL_CARD)
    c.rect("footer", Box(R.x0, R.y0, R.x1, R.y0 + 1), fill=COL_BORDER)
    inner = Box(R.x0 + 6, R.y0 + 1, R.x1 - 6, R.y1)
    row1 = Box(inner.x0, R.y0 + 2, inner.x1, R.y0 + 20)
    row2 = Box(inner.x0, R.y0 + 21, inner.x1, R.y1)

    nets = _network_entries(snap.network)
    seg_w = row1.w / len(nets)
    for i, (colour, tag, value, extra) in enumerate(nets):
        seg = Box(int(row1.x0 + i * seg_w), row1.y0, int(row1.x0 + (i + 1) * seg_w) - 6, row1.y1)
        _footer_item(c, seg, colour, tag, value, extra)

    hint = c.text("footer", Box(row2.x1 - 80, row2.y0, row2.x1, row2.y1),
                  (row2.x1, row2.cy), "Home: controls", font(9), COL_MUTED, anchor="rm")
    rec_colour, rec_value = _recording_entry(snap.recorders)
    pad_colour, pad_value = _gamepad_entry(snap.gamepad)
    # REC is sized to its content (capped) so the gamepad name gets the rest.
    rec_w = 10 + text_width("REC", font(9, True)) + 5 + text_width(rec_value, font(10)) + 2
    rec_w = int(math.ceil(min(rec_w, 124)))
    rec_seg = Box(hint.x0 - 8 - rec_w, row2.y0, hint.x0 - 8, row2.y1)
    pad_seg = Box(row2.x0, row2.y0, rec_seg.x0 - 10, row2.y1)
    _footer_item(c, pad_seg, pad_colour, "PAD", pad_value)
    _footer_item(c, rec_seg, rec_colour, "REC", rec_value)


def _footer_item(c, seg: Box, colour, tag: str, value: str, extra: str = ""):
    c.dot("footer", seg.x0 + 3, seg.cy, 3, colour)
    t = c.text("footer", seg, (seg.x0 + 10, seg.cy), tag, font(9, True), COL_TEXT_2)
    value_col = COL_TEXT if colour not in (COL_MUTED,) else COL_MUTED
    v = c.text("footer", seg, (t.x1 + 5, seg.cy), value, font(10), value_col)
    if extra and seg.x1 - (v.x1 + 6) > 24:
        c.text("footer", seg, (v.x1 + 6, seg.cy), extra, font(9), COL_MUTED)


def _network_entries(network):
    if network is None:
        return [(COL_MUTED, "NET", "scanning\u2026", "")]
    connected = [i for i in network if i.get("connected") and i.get("ip")]
    if not connected:
        return [(COL_AMBER, "NET", "no connection", "")]
    connected.sort(key=lambda i: 0 if i.get("type") == "ethernet" else 1)
    out = []
    for iface in connected[:2]:
        tag = "ETH" if iface.get("type") == "ethernet" else "WIFI"
        extra = str(iface.get("ssid") or "") if len(connected) == 1 else ""
        out.append((COL_GREEN, tag, str(iface["ip"]), extra))
    return out


def _gamepad_entry(gamepad):
    if gamepad is None:
        return COL_MUTED, "--"
    if gamepad.get("connected"):
        return COL_GREEN, str(gamepad.get("name") or "connected")
    return COL_AMBER, "not connected"


def _recording_entry(recorders):
    if not recorders:
        return COL_MUTED, "--"
    active = []
    for name, st in sorted(recorders.items()):
        if st.get("recording"):
            m, s = divmod(int(_num(st.get("duration_sec")) or 0), 60)
            active.append(f"{RECORDER_SHORT.get(name, 'BAG')} {m}:{s:02d}")
    if active:
        return COL_RED, "  ".join(active)
    return COL_TEXT_2, "idle"


# ── Controls overlay (gamepad Home button) ────────────────────────────

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

FIXED_AXES = [
    ("L Stick", "Drive"),
    ("R Stick", "--"),
]


def render_controls(gamepad: dict | None) -> RenderResult:
    c = _Canvas()
    gamepad = gamepad or {}
    active = set(gamepad.get("active_inputs", []))
    rows = list(FIXED_AXES) + [
        (btn, ACTION_LABELS.get(action, action))
        for btn, action in (gamepad.get("button_map") or {}).items()
    ]

    R = REGION_HEADER
    c.rect("header", Box(R.x0, R.y1 - 1, R.x1, R.y1), fill=COL_BORDER)
    c.rect("header", Box(6, 6, 16, 16), fill=COL_BLUE, radius=2)
    c.text("header", Box(20, 0, 160, 21), (20, 11), "CONTROLS", font(13, True), COL_TEXT)
    c.text("header", Box(170, 0, 314, 21), (314, 11), "[Home] back to status",
           font(9), COL_MUTED, anchor="rm")

    body = Box(4, 25, 316, 238)
    if not gamepad.get("button_map"):
        c.text("body", body, (body.x0 + 4, body.y1 - 10), "No gamepad button map yet",
               font(10), COL_MUTED)
        body = Box(body.x0, body.y0, body.x1, body.y1 - 22)

    per_col = max(1, math.ceil(len(rows) / 2))
    row_h = max(12, min(19, body.h // per_col))
    per_col = min(per_col, body.h // row_h)
    col_w = (body.w - TILE_GAP) // 2
    for i, (btn, desc) in enumerate(rows[: per_col * 2]):
        col, row = divmod(i, per_col)
        x0 = body.x0 + col * (col_w + TILE_GAP)
        box = Box(x0, body.y0 + row * row_h, x0 + col_w, body.y0 + (row + 1) * row_h - 1)
        is_active = btn in active
        c.rect("body", box, fill=COL_HIGHLIGHT if is_active else COL_CARD, radius=3)
        if is_active:
            c.rect("body", Box(box.x0, box.y0, box.x0 + 3, box.y1), fill=COL_GREEN)
        inner = box.inset(6, 0)
        c.text("body", Box(inner.x0, box.y0, inner.x0 + 50, box.y1), (inner.x0, box.cy),
               btn.replace("D-Pad ", "D-"), font(9, True),
               COL_GREEN if is_active else COL_BLUE_LIGHT)
        c.text("body", Box(inner.x0 + 52, box.y0, inner.x1, box.y1),
               (inner.x0 + 52, box.cy), desc, font(10),
               COL_GREEN if is_active else (COL_MUTED if desc == "--" else COL_TEXT))
    return RenderResult(c.image, c.items, "controls")
