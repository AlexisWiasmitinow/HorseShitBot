#!/usr/bin/env python3
"""Render PNG previews of the TFT status screen without any hardware.

Uses the exact renderer status_screen_node blits to the ILI9341, fed with
synthetic telemetry. Output (gitignored):

    previews/status_screen/<scenario>.png   native 320x240
    previews/status_screen/contact_sheet.png

Usage:
    python3 scripts/render_status_screen_previews.py [--out DIR] [--scale N]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "horseshitbot"))

from PIL import Image, ImageDraw  # noqa: E402

from horseshitbot.drivers.battery_readout import (  # noqa: E402
    HEALTH_DEAD,
    HEALTH_GOOD,
    battery_readout,
)
from horseshitbot.drivers.status_screen_render import (  # noqa: E402
    COL_TEXT,
    COL_TEXT_2,
    ScreenSnapshot,
    font,
    render_controls,
    render_dashboard,
)

NOW = 1000.0

THERMALS = [
    {"type": "cpu-thermal", "temp_c": 55.4},
    {"type": "gpu-thermal", "temp_c": 55.6},
    {"type": "soc0-thermal", "temp_c": 54.2},
    {"type": "soc1-thermal", "temp_c": 56.2},
    {"type": "soc2-thermal", "temp_c": 54.3},
    {"type": "tj-thermal", "temp_c": 56.3},
]

NETWORK = [
    {"name": "wlan0", "type": "wifi", "connected": True,
     "ip": "192.168.178.57", "ssid": "HSB-Stable"},
    {"name": "eth0", "type": "ethernet", "connected": False, "ip": "", "ssid": ""},
]

GAMEPAD = {
    "connected": True,
    "name": "Xbox Wireless Controller",
    "active_inputs": [],
    "button_map": {
        "A": "e_stop", "B": "stop_wheels", "X": "stop_actuators",
        "Y": "stop_wheels", "LB": "bin_door", "RB": "brush", "LT": "none",
        "RT": "none", "L3": "e_stop", "R3": "e_stop", "D-Pad Up": "lift_up",
        "D-Pad Down": "lift_down", "D-Pad Left": "mapping_bag_recording",
        "D-Pad Right": "none", "Options": "perception_bag_recording",
        "Menu": "reference_all", "Logo": "toggle_controls",
    },
}

READY = {"state": 2, "is_referenced": True, "direction": "", "speed_rpm": 0.0,
         "error_message": ""}


def _battery(percentage, voltage, health=HEALTH_GOOD, age=0.4):
    sample = {"percentage": percentage, "voltage": voltage, "health": health,
              "present": True}
    return battery_readout(sample, NOW - age, NOW)


def _base(**overrides) -> ScreenSnapshot:
    snap = ScreenSnapshot(
        clock="14:20",
        wheel={"backend": "mks", "left_rpm": 0.0, "right_rpm": 0.0,
               "stopping": False, "estopped": False, "error": ""},
        mks_bus={"bus_connected": True, "motors": {"1": True, "2": True}},
        actuators={"lift": dict(READY), "brush": dict(READY),
                   "bin_door": dict(READY)},
        battery=_battery(1.0, 28.0),
        thermals=THERMALS,
        network=NETWORK,
        gamepad=GAMEPAD,
        recorders={"perception_recorder": {"recording": False},
                   "mapping_recorder": {"recording": False}},
    )
    for key, value in overrides.items():
        setattr(snap, key, value)
    return snap


def scenarios():
    """(file stem, caption, RenderResult). The first six are the required set."""
    yield "01_battery_100", "Normal: 100% / 28.0 V", render_dashboard(_base())
    yield "02_battery_50", "Half: 50% / 25.9 V", render_dashboard(
        _base(battery=_battery(0.50, 25.9),
              wheel={"backend": "mks", "left_rpm": 42.0, "right_rpm": 40.0,
                     "estopped": False, "error": ""},
              recorders={"perception_recorder": {"recording": False},
                         "mapping_recorder": {"recording": True,
                                              "duration_sec": 83.0}}))
    yield "03_battery_10", "Low: 10% / 25.3 V", render_dashboard(
        _base(battery=_battery(0.10, 25.3)))
    yield "04_battery_critical", "Critical: 0% / 19.8 V (node: DEAD)", render_dashboard(
        _base(battery=_battery(0.0, 19.8, HEALTH_DEAD)))
    yield "05_battery_stale", "Stale telemetry (last msg 12 s ago)", render_dashboard(
        _base(battery=_battery(0.5, 25.9, age=12.0), gamepad=None))
    yield "06_estop", "E-STOP active", render_dashboard(
        _base(wheel={"backend": "mks", "left_rpm": 0.0, "right_rpm": 0.0,
                     "estopped": True, "error": ""},
              battery=_battery(0.72, 26.2)))

    # Extra states worth eyeballing; not part of the required contact sheet.
    yield "07_battery_missing", "Missing: no /battery/status yet", render_dashboard(
        _base(battery=battery_readout(None), network=None, recorders=None,
              gamepad=None, thermals=None, wheel=None, mks_bus=None,
              actuators={}))
    yield "08_estop_with_faults", "E-STOP + bus and actuator faults", render_dashboard(
        _base(wheel={"backend": "mks", "left_rpm": 0.0, "right_rpm": 0.0,
                     "estopped": True, "error": ""},
              mks_bus={"bus_connected": True, "motors": {"1": True, "2": False}},
              actuators={"lift": {"state": 5, "is_referenced": False,
                                  "error_message": "stall on reference"},
                         "brush": dict(READY), "bin_door": dict(READY)},
              battery=_battery(0.10, 25.3)))
    yield "09_faults", "Faults without E-STOP", render_dashboard(
        _base(wheel={"backend": "mks", "left_rpm": 0.0, "right_rpm": 0.0,
                     "estopped": False, "error": "motor 2 timeout"},
              actuators={"lift": {"state": 1, "is_referenced": False},
                         "brush": {"state": 0, "is_referenced": False},
                         "bin_door": {"state": 5, "error_message": "overcurrent"}}))
    pad = dict(GAMEPAD, active_inputs=["L Stick", "RB"])
    yield "10_controls_overlay", "Home button: controls overlay", render_controls(pad)


def contact_sheet(entries, scale: int, cols: int = 3) -> Image.Image:
    pad, caption_h = 12, 22
    cell_w, cell_h = 320 * scale, 240 * scale
    rows = (len(entries) + cols - 1) // cols
    sheet = Image.new(
        "RGB",
        (pad + cols * (cell_w + pad), pad + rows * (cell_h + caption_h + pad)),
        (4, 10, 20),
    )
    draw = ImageDraw.Draw(sheet)
    for i, (stem, caption, image) in enumerate(entries):
        r, c = divmod(i, cols)
        x = pad + c * (cell_w + pad)
        y = pad + r * (cell_h + caption_h + pad)
        draw.text((x, y + 2), f"{stem}  -  {caption}", font=font(14, True), fill=COL_TEXT)
        framed = image.resize((cell_w, cell_h), Image.NEAREST)
        sheet.paste(framed, (x, y + caption_h))
        draw.rectangle([x - 1, y + caption_h - 1, x + cell_w, y + caption_h + cell_h],
                       outline=COL_TEXT_2)
    return sheet


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", type=Path, default=REPO_ROOT / "previews" / "status_screen")
    parser.add_argument("--scale", type=int, default=2, help="contact sheet zoom")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    entries = []
    for stem, caption, result in scenarios():
        assert result.image.size == (320, 240), result.image.size
        path = args.out / f"{stem}.png"
        result.image.save(path)
        entries.append((stem, caption, result.image))
        print(f"{path}  [{result.mode}]")

    sheet_path = args.out / "contact_sheet.png"
    contact_sheet(entries[:6], args.scale).save(sheet_path)
    print(sheet_path)
    extras_path = args.out / "contact_sheet_extras.png"
    contact_sheet(entries[6:], args.scale, cols=2).save(extras_path)
    print(extras_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
