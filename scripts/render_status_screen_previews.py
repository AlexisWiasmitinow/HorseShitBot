#!/usr/bin/env python3
"""Render the production TFT renderer to PNG without touching SPI or GPIO."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "horseshitbot"))

from horseshitbot.drivers.battery_readout import (  # noqa: E402
    HEALTH_DEAD,
    HEALTH_GOOD,
    battery_readout,
)
from horseshitbot.drivers.status_screen_render import (  # noqa: E402
    ScreenSnapshot,
    font,
    render_dashboard,
)


NOW = 1000.0
READY = {"state": 2, "is_referenced": True, "direction": ""}


def battery(percentage, voltage, health=HEALTH_GOOD, age=0.4):
    return battery_readout(
        {
            "percentage": percentage,
            "voltage": voltage,
            "health": health,
            "present": True,
        },
        NOW - age,
        NOW,
    )


def snapshot(readout, **overrides):
    state = ScreenSnapshot(
        clock="14:20",
        wheel={
            "backend": "mks",
            "left_rpm": 0,
            "right_rpm": 0,
            "estopped": False,
            "error": "",
        },
        mks_bus={"bus_connected": True, "motors": {"1": True, "2": True}},
        actuators={
            "lift": dict(READY),
            "brush": dict(READY),
            "bin_door": dict(READY),
        },
        battery=readout,
        thermals=[
            {"type": "cpu", "temp_c": 55.4},
            {"type": "gpu", "temp_c": 55.6},
            {"type": "tj", "temp_c": 56.3},
        ],
        network=[
            {
                "type": "wifi",
                "connected": True,
                "ip": "192.168.178.57",
            }
        ],
        gamepad={"connected": True, "name": "Xbox Controller"},
        recorders={"mapping_recorder": {"recording": False}},
    )
    for key, value in overrides.items():
        setattr(state, key, value)
    return state


def scenarios():
    yield "battery_100", "100% / 28.0 V", snapshot(battery(1.0, 28.0))
    yield "battery_50", "50% / 25.9 V", snapshot(battery(0.5, 25.9))
    yield "battery_10", "LOW 10% / 25.3 V", snapshot(battery(0.1, 25.3))
    yield "battery_critical", "CRITICAL 0% / 19.8 V", snapshot(
        battery(0.0, 19.8, HEALTH_DEAD)
    )
    yield "battery_stale", "stale data", snapshot(
        battery(0.5, 25.9, age=12.0)
    )
    yield "battery_missing", "no battery data", snapshot(
        battery_readout(None)
    )
    yield "recording_active", "mapping recording active", snapshot(
        battery(0.72, 26.2),
        recorders={
            "mapping_recorder": {
                "recording": True,
                "duration_sec": 83,
            }
        },
    )
    yield "emergency_stop", "E-STOP indication", snapshot(
        battery(0.72, 26.2),
        wheel={
            "backend": "mks",
            "left_rpm": 0,
            "right_rpm": 0,
            "estopped": True,
            "error": "",
        },
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        type=Path,
        default=ROOT / "previews" / "status_screen",
    )
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    entries = []
    for name, caption, state in scenarios():
        image = render_dashboard(state).image
        image.save(args.out / f"{name}.png")
        entries.append((name, caption, image))

    sheet = Image.new("RGB", (660, 1092), "white")
    draw = ImageDraw.Draw(sheet)
    for index, (name, caption, image) in enumerate(entries):
        row, column = divmod(index, 2)
        x, y = 10 + column * 325, 10 + row * 270
        draw.text(
            (x, y),
            f"{name}: {caption}",
            font=font(12, True),
            fill=(17, 24, 39),
        )
        sheet.paste(image, (x, y + 22))
    sheet.save(args.out / "contact_sheet.png")
    print(args.out / "contact_sheet.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
