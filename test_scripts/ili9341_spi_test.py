#!/usr/bin/env python3
"""
Standalone test script for the 2.8" ILI9341 SPI display (240x320).
No ROS dependency — run directly on the Pi to verify wiring, colour fills,
text rendering, and a mock status screen layout.

Usage:
    python3 ili9341_spi_test.py                 # defaults
    python3 ili9341_spi_test.py --dc 24 --rst 25 --cs 8
    python3 ili9341_spi_test.py --mock-only     # skip colour fills, just show status mockup
    python3 ili9341_spi_test.py --fps            # measure full-screen refresh rate
"""

import argparse
import sys
import time

from PIL import Image, ImageDraw, ImageFont

# ── try to import the shared driver (works if run from repo root) ──
try:
    sys.path.insert(0, "src/horseshitbot")
    from horseshitbot.drivers.ili9341_display import ILI9341Display
    _DRIVER_OK = True
except ImportError:
    _DRIVER_OK = False


# ── fallback fonts ───────────────────────────────────────────────
try:
    FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 14)
    FONT_SM = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.thf", 11)
    FONT_LG = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 18)
except Exception:
    FONT = ImageFont.load_default()
    FONT_SM = FONT
    FONT_LG = FONT


# ── colour fills ────────────────────────────────────────────────
COLOURS = [
    ("Red",    (255, 0, 0)),
    ("Green",  (0, 255, 0)),
    ("Blue",   (0, 0, 255)),
    ("White",  (255, 255, 255)),
    ("Black",  (0, 0, 0)),
]


def test_colour_fills(disp):
    print("=== Colour fill test ===")
    for name, colour in COLOURS:
        print(f"  {name}...", end=" ", flush=True)
        disp.clear(colour)
        time.sleep(0.8)
        print("ok")


def test_text(disp):
    print("=== Text rendering test ===")
    img = disp.new_image((0, 0, 40))
    draw = ImageDraw.Draw(img)

    y = 10
    for label, font in [("Large font", FONT_LG), ("Normal font", FONT), ("Small font", FONT_SM)]:
        draw.text((10, y), label, fill=(255, 255, 255), font=font)
        y += 30

    draw.text((10, y), "0123456789 ABCDEF", fill=(0, 255, 200), font=FONT)
    y += 25
    draw.text((10, y), "!@#$%^&*()_+-=[]", fill=(255, 200, 0), font=FONT)

    disp.draw_frame(img)
    time.sleep(2)
    print("  done")


def test_mock_status(disp):
    print("=== Mock status screen ===")
    W, H = disp.width, disp.height
    img = disp.new_image((10, 10, 30))
    draw = ImageDraw.Draw(img)

    # Header
    draw.rectangle([0, 0, W, 28], fill=(15, 30, 60))
    draw.text((8, 5), "HORSESHITBOT", fill=(233, 69, 96), font=FONT_LG)
    draw.text((W - 50, 8), "00:00", fill=(200, 200, 200), font=FONT_SM)

    # Wheels section
    y = 34
    draw.rectangle([0, y, W, y + 50], fill=(15, 50, 96))
    draw.text((8, y + 4), "WHEELS [odrive]", fill=(255, 255, 255), font=FONT)
    draw.text((8, y + 22), "L: >>> 123", fill=(78, 204, 163), font=FONT)
    draw.text((W // 2, y + 22), "R: >>> 117", fill=(78, 204, 163), font=FONT)
    # Speed bars
    bar_y = y + 40
    draw.rectangle([8, bar_y, 8 + 100, bar_y + 6], fill=(50, 50, 70))
    draw.rectangle([8, bar_y, 8 + 60, bar_y + 6], fill=(78, 204, 163))
    draw.rectangle([W // 2, bar_y, W // 2 + 100, bar_y + 6], fill=(50, 50, 70))
    draw.rectangle([W // 2, bar_y, W // 2 + 57, bar_y + 6], fill=(78, 204, 163))

    # Actuators
    y = 92
    actuators = [
        ("LIFT",     "REFERENCED", "UP",     (78, 204, 163)),
        ("BRUSH",    "IDLE",       "---",    (138, 138, 154)),
        ("BIN DOOR", "MOVING",     "OPEN",   (240, 201, 41)),
    ]
    for name, state, direction, colour in actuators:
        draw.rectangle([0, y, W, y + 28], fill=(15, 35, 55))
        draw.text((8, y + 6), name, fill=(200, 200, 200), font=FONT)
        draw.text((110, y + 6), f"[{state}]", fill=colour, font=FONT_SM)
        draw.text((W - 45, y + 6), direction, fill=(255, 255, 255), font=FONT)
        y += 30

    # Footer
    y = H - 50
    draw.rectangle([0, y, W, H], fill=(15, 30, 60))
    draw.text((8, y + 6), "GAMEPAD: Connected", fill=(78, 204, 163), font=FONT_SM)
    draw.text((8, y + 22), "ERRORS: None", fill=(78, 204, 163), font=FONT_SM)

    disp.draw_frame(img)
    time.sleep(4)
    print("  done")


def test_fps(disp, frames=50):
    print(f"=== FPS test ({frames} frames) ===")
    img = disp.new_image((0, 0, 0))
    draw = ImageDraw.Draw(img)

    t0 = time.perf_counter()
    for i in range(frames):
        draw.rectangle([0, 0, disp.width, disp.height], fill=(i * 5 % 256, 0, 0))
        draw.text((10, 10), f"Frame {i+1}/{frames}", fill=(255, 255, 255), font=FONT)
        disp.draw_frame(img)
    elapsed = time.perf_counter() - t0
    fps = frames / elapsed
    print(f"  {frames} frames in {elapsed:.2f}s = {fps:.1f} FPS")


def main():
    parser = argparse.ArgumentParser(description="ILI9341 SPI display test")
    parser.add_argument("--cs", type=int, default=8, help="GPIO pin for CS (default 8 = CE0)")
    parser.add_argument("--dc", type=int, default=24, help="GPIO pin for D/C (default 24)")
    parser.add_argument("--rst", type=int, default=25, help="GPIO pin for RESET (default 25)")
    parser.add_argument("--led", type=int, default=18, help="GPIO pin for LED backlight (default 18)")
    parser.add_argument("--rotation", type=int, default=0, choices=[0, 90, 180, 270])
    parser.add_argument("--mock-only", action="store_true", help="Skip colour fills, show status mockup only")
    parser.add_argument("--fps", action="store_true", help="Run FPS benchmark")
    parser.add_argument("--no-hw", action="store_true", help="Run without hardware (renders to in-memory images)")
    args = parser.parse_args()

    if args.no_hw or not _DRIVER_OK:
        print("Running in no-hardware mode (images rendered in memory only)")

        class FakeDisplay:
            width, height = 240, 320
            def clear(self, c): pass
            def draw_frame(self, img): return img
            def new_image(self, bg=(0,0,0)): return Image.new("RGB", (240, 320), bg)

        disp = FakeDisplay()
    else:
        print(f"Initialising ILI9341: CS=GPIO{args.cs}, DC=GPIO{args.dc}, "
              f"RST=GPIO{args.rst}, LED=GPIO{args.led}, rotation={args.rotation}")
        disp = ILI9341Display(
            cs_pin=args.cs,
            dc_pin=args.dc,
            rst_pin=args.rst,
            led_pin=args.led,
            rotation=args.rotation,
        )

    if not args.mock_only:
        test_colour_fills(disp)
        test_text(disp)

    test_mock_status(disp)

    if args.fps:
        test_fps(disp)

    print("\nAll tests complete.")


if __name__ == "__main__":
    main()
