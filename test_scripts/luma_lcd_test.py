#!/usr/bin/env python3
"""
Standalone test script for the 2.8" ILI9341 SPI display (240x320) using luma.lcd.
No ROS dependency — run on the robot SBC to verify wiring, colour fills,
text rendering, and a mock status screen layout.

Uses luma.lcd instead of the Adafruit stack. luma.lcd talks to spidev directly
(no Blinka layer) and uses RPi.GPIO or Jetson.GPIO for DC/RST/backlight.

Pin numbers on the CLI follow the same convention as ili9341_spi_test.py:
  Jetson Nano: physical 40-pin header pins (converted to BCM internally).
  Raspberry Pi: BCM GPIO (SPI CE0 = BCM 8).
luma.lcd requires BCM mode, so on Jetson the script auto-converts physical→BCM.

Dependencies:
    pip install luma.lcd Pillow
    # Jetson also needs:  pip install Jetson.GPIO   (usually pre-installed)
    # Pi also needs:      pip install RPi.GPIO      (usually pre-installed)
    # For --spi-loopback: sudo apt install python3-spidev

Usage:
    python3 luma_lcd_test.py                          # platform defaults
    python3 luma_lcd_test.py --mock-only              # skip fills, show status mockup
    python3 luma_lcd_test.py --fps                    # measure refresh rate
    python3 luma_lcd_test.py --chip st7789            # try ST7789 driver instead
    python3 luma_lcd_test.py --no-hw                  # in-memory only (no hardware)
    python3 luma_lcd_test.py --spi-loopback --spi-bus 1 --spi-device 0
"""

import argparse
import glob
import sys
import time

from PIL import Image, ImageDraw, ImageFont


# ── platform detection ──────────────────────────────────────────

def _is_jetson_soc() -> bool:
    try:
        with open("/proc/device-tree/model", "rb") as f:
            return b"Jetson" in f.read()
    except OSError:
        return False


def _pin_defaults():
    """Platform-specific default wiring (physical pins on Jetson, BCM on Pi)."""
    if _is_jetson_soc():
        return {"cs": 24, "dc": 22, "rst": 18, "led": 12, "spi_port": 0, "spi_device": 0}
    return {"cs": 8, "dc": 24, "rst": 25, "led": 18, "spi_port": 0, "spi_device": 0}


# Jetson Nano 40-pin header: physical pin → BCM-equivalent GPIO number.
# Jetson.GPIO's BCM mode uses the same numbering as RPi for compatibility.
_JETSON_PHYS_TO_BCM = {
    7: 4, 11: 17, 12: 18, 13: 27, 15: 22, 16: 23, 18: 24,
    19: 10, 21: 9, 22: 25, 23: 11, 24: 8, 26: 7, 29: 5,
    31: 6, 32: 12, 33: 13, 35: 19, 36: 16, 37: 26, 38: 20, 40: 21,
}


def _to_bcm(pin: int) -> int:
    """Convert a physical header pin to BCM on Jetson; pass-through on Pi (already BCM)."""
    if not _is_jetson_soc():
        return pin
    bcm = _JETSON_PHYS_TO_BCM.get(pin)
    if bcm is None:
        raise ValueError(
            f"Jetson physical pin {pin} has no BCM mapping (not a GPIO-capable pin). "
            f"Valid physical pins: {sorted(_JETSON_PHYS_TO_BCM)}"
        )
    return bcm


def _load_gpio():
    """
    Import the right GPIO library and set BCM mode.

    luma.core internally assumes BCM numbering.  On Jetson, Jetson.GPIO
    installs an RPi.GPIO compatibility shim that shares global state, so
    every caller must agree on the same mode — BCM everywhere avoids
    the "A different mode has already been set!" error.
    """
    if _is_jetson_soc():
        import Jetson.GPIO as GPIO
    else:
        import RPi.GPIO as GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    return GPIO


# ── fonts (same as the Adafruit test script) ────────────────────

try:
    FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 14)
    FONT_SM = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 11)
    FONT_LG = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 18)
except Exception:
    FONT = ImageFont.load_default()
    FONT_SM = FONT
    FONT_LG = FONT


# ── allowed bus speeds (luma.core asserts on these) ─────────────

_LUMA_VALID_HZ = sorted(mhz * 1_000_000 for mhz in
    [0.5, 1, 2, 4, 8, 16, 20, 24, 28, 32, 36, 40, 44, 48, 50, 52])

def _nearest_valid_hz(target: int) -> int:
    return min(_LUMA_VALID_HZ, key=lambda h: abs(h - target))


# ── test functions ──────────────────────────────────────────────

COLOURS = [
    ("Red",    (255, 0, 0)),
    ("Green",  (0, 255, 0)),
    ("Blue",   (0, 0, 255)),
    ("White",  (255, 255, 255)),
    ("Black",  (0, 0, 0)),
]


def test_colour_fills(device):
    from luma.core.render import canvas
    print("=== Colour fill test ===")
    for name, colour in COLOURS:
        print(f"  {name}...", end=" ", flush=True)
        with canvas(device) as draw:
            draw.rectangle(device.bounding_box, fill=colour)
        time.sleep(0.8)
        print("ok")


def test_text(device):
    from luma.core.render import canvas
    print("=== Text rendering test ===")
    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, fill=(0, 0, 40))
        y = 10
        for label, font in [("Large font", FONT_LG), ("Normal font", FONT), ("Small font", FONT_SM)]:
            draw.text((10, y), label, fill=(255, 255, 255), font=font)
            y += 30
        draw.text((10, y), "0123456789 ABCDEF", fill=(0, 255, 200), font=FONT)
        y += 25
        draw.text((10, y), "!@#$%^&*()_+-=[]", fill=(255, 200, 0), font=FONT)
    time.sleep(2)
    print("  done")


def test_mock_status(device):
    from luma.core.render import canvas
    print("=== Mock status screen ===")
    W, H = device.width, device.height

    with canvas(device) as draw:
        draw.rectangle(device.bounding_box, fill=(10, 10, 30))

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
        bar_y = y + 40
        draw.rectangle([8, bar_y, 108, bar_y + 6], fill=(50, 50, 70))
        draw.rectangle([8, bar_y, 68, bar_y + 6], fill=(78, 204, 163))
        draw.rectangle([W // 2, bar_y, W // 2 + 100, bar_y + 6], fill=(50, 50, 70))
        draw.rectangle([W // 2, bar_y, W // 2 + 57, bar_y + 6], fill=(78, 204, 163))

        # Actuators
        y = 92
        actuators = [
            ("LIFT",     "REFERENCED", "UP",   (78, 204, 163)),
            ("BRUSH",    "IDLE",       "---",  (138, 138, 154)),
            ("BIN DOOR", "MOVING",     "OPEN", (240, 201, 41)),
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

    time.sleep(4)
    print("  done")


def test_fps(device, frames=50):
    print(f"=== FPS test ({frames} frames) ===")

    t0 = time.perf_counter()
    for i in range(frames):
        img = Image.new("RGB", device.size, (i * 5 % 256, 0, 0))
        draw = ImageDraw.Draw(img)
        draw.text((10, 10), f"Frame {i + 1}/{frames}", fill=(255, 255, 255), font=FONT)
        device.display(img)
    elapsed = time.perf_counter() - t0
    fps = frames / elapsed
    print(f"  {frames} frames in {elapsed:.2f}s = {fps:.1f} FPS")


def test_spi_loopback(args):
    """
    Pure spidev loopback — no display library needed.
    Jumper MOSI to MISO on the same SPI header (disconnect TFT from those pins).
    """
    try:
        import spidev
    except ImportError:
        print("ERROR: install spidev (e.g. sudo apt install python3-spidev)")
        return

    devpath = f"/dev/spidev{args.spi_bus}.{args.spi_device}"
    baud = min(int(args.spi_baud), 2_000_000)
    print("=== SPI loopback (MOSI jumpered to MISO) ===")
    print(f"  Device: {devpath}  mode={args.spi_mode}  max_hz={baud}")
    print("  Disconnect the display from MOSI/MISO; bridge those two pins on the header only.")

    spi = spidev.SpiDev()
    try:
        spi.open(args.spi_bus, args.spi_device)
    except OSError as e:
        print(f"  ERROR: open {devpath}: {e}")
        return
    spi.mode = int(args.spi_mode) & 3
    spi.max_speed_hz = baud
    spi.bits_per_word = 8

    patterns = (
        bytes([0x00, 0xFF, 0x5A, 0xA5]),
        bytes([i & 0xFF for i in range(32)]),
        bytes([0xAA, 0x55] * 64),
    )
    ok_all = True
    for p in patterns:
        rx = bytes(spi.xfer2(list(p)))
        if rx == p:
            print(f"  OK   len {len(p):3d}  tx==rx")
        else:
            ok_all = False
            print(f"  FAIL len {len(p):3d}  tx {p.hex()}  rx {rx.hex()}")
    spi.close()
    if ok_all:
        print("  Loopback OK — this /dev/spidev matches the pins you jumpered.")
    else:
        print("  Loopback failed — wrong bus/device, no jumper, bad mode, or a device still on the bus.")


# ── main ────────────────────────────────────────────────────────

def main():
    dp = _pin_defaults()

    parser = argparse.ArgumentParser(
        description="ILI9341 / ST7789 SPI display test using luma.lcd",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
examples:
  python3 luma_lcd_test.py                           # platform defaults
  python3 luma_lcd_test.py --chip st7789             # many 2.8" boards are ST7789
  python3 luma_lcd_test.py --mock-only               # show status mockup only
  python3 luma_lcd_test.py --fps                     # refresh rate benchmark
  python3 luma_lcd_test.py --no-hw                   # no hardware (in-memory)
  python3 luma_lcd_test.py --spi-loopback --spi-bus 1 --spi-device 0
""",
    )

    # Pin / bus arguments
    parser.add_argument("--dc", type=int, default=dp["dc"],
        help=f"D/C GPIO pin (default {dp['dc']}; Jetson=physical, Pi=BCM)")
    parser.add_argument("--rst", type=int, default=dp["rst"],
        help=f"RESET GPIO pin (default {dp['rst']})")
    parser.add_argument("--led", type=int, default=dp["led"],
        help=f"Backlight GPIO pin (default {dp['led']})")
    parser.add_argument("--spi-port", type=int, default=dp["spi_port"],
        help=f"SPI port / bus (default {dp['spi_port']})")
    parser.add_argument("--spi-device", type=int, default=dp["spi_device"],
        help=f"SPI chip-select device index (default {dp['spi_device']})")
    parser.add_argument("--spi-baud", type=int, default=8_000_000,
        help="SPI bus speed in Hz (snapped to nearest luma-valid value; default 8000000)")
    parser.add_argument("--spi-mode", type=int, choices=(0, 1, 2, 3), default=None,
        help="SPI mode (CPOL/CPHA); try 0 first, then 3 if panel is silent")

    # Display options
    parser.add_argument("--chip", choices=("ili9341", "st7789"), default="ili9341",
        help="LCD controller chip (default ili9341)")
    parser.add_argument("--rotation", type=int, default=0, choices=[0, 90, 180, 270],
        help="Display rotation in degrees")

    # Test selection
    parser.add_argument("--mock-only", action="store_true",
        help="Skip colour fills and text, show mock status screen only")
    parser.add_argument("--fps", action="store_true",
        help="Run FPS benchmark after other tests")
    parser.add_argument("--no-hw", action="store_true",
        help="Run without hardware (luma dummy device, in-memory rendering)")

    # SPI loopback (pure spidev, no display library)
    parser.add_argument("--spi-loopback", action="store_true",
        help="MOSI jumpered to MISO on header SPI; requires --spi-bus")
    parser.add_argument("--spi-bus", type=int, default=None, metavar="N",
        help="Linux SPI bus for --spi-loopback (/dev/spidevN.M)")

    args = parser.parse_args()

    # ── loopback branch (no luma needed) ──
    if args.spi_loopback:
        if args.spi_bus is None:
            print("ERROR: --spi-loopback requires --spi-bus (and --spi-device if not 0)")
            sys.exit(1)
        test_spi_loopback(args)
        return

    # ── resolve rotation to luma's 0-3 ──
    rotate_map = {0: 0, 90: 1, 180: 2, 270: 3}
    rotate = rotate_map[args.rotation]

    # ── build device ──
    if args.no_hw:
        print("Running in no-hardware mode (luma dummy device)")
        from luma.core.device import dummy
        if rotate in (1, 3):
            device = dummy(width=320, height=240, rotate=0, mode="RGB")
        else:
            device = dummy(width=240, height=320, rotate=0, mode="RGB")
    else:
        try:
            from luma.core.interface.serial import spi as luma_spi
            from luma.lcd.device import ili9341 as luma_ili9341, st7789 as luma_st7789
        except ImportError as e:
            print(f"ERROR: {e}")
            print("\nInstall:  pip install luma.lcd Pillow")
            sys.exit(1)

        gpio = _load_gpio()

        # Convert CLI pin numbers (physical on Jetson, BCM on Pi) to BCM
        # so everything agrees with the BCM mode we set above.
        try:
            dc_bcm = _to_bcm(args.dc)
            rst_bcm = _to_bcm(args.rst)
            led_bcm = _to_bcm(args.led)
        except ValueError as e:
            print(f"  ERROR: {e}")
            sys.exit(1)

        baud = _nearest_valid_hz(args.spi_baud)
        if baud != args.spi_baud:
            print(f"  Note: SPI baud {args.spi_baud} snapped to nearest valid value {baud}")

        spidevs = sorted(glob.glob("/dev/spidev*"))
        print(f"  SPI devices: {spidevs if spidevs else '(none — enable SPI in jetson-io / raspi-config)'}")
        print(f"  Chip: {args.chip}   SPI: port={args.spi_port} device={args.spi_device} baud={baud}")
        if _is_jetson_soc():
            print(f"  GPIO (physical→BCM):  DC={args.dc}→{dc_bcm}  RST={args.rst}→{rst_bcm}"
                  f"  LED={args.led}→{led_bcm}  rotation={args.rotation}°")
        else:
            print(f"  GPIO (BCM):  DC={dc_bcm}  RST={rst_bcm}  LED={led_bcm}  rotation={args.rotation}°")

        spi_kwargs = dict(
            gpio=gpio,
            port=args.spi_port,
            device=args.spi_device,
            bus_speed_hz=baud,
            gpio_DC=dc_bcm,
            gpio_RST=rst_bcm,
        )
        if args.spi_mode is not None:
            spi_kwargs["spi_mode"] = args.spi_mode

        try:
            serial = luma_spi(**spi_kwargs)
        except Exception as e:
            print(f"  ERROR initialising SPI: {e}")
            print(f"  Check that /dev/spidev{args.spi_port}.{args.spi_device} exists and SPI is enabled.")
            if _is_jetson_soc():
                print("  Jetson: sudo /opt/nvidia/jetson-io/jetson-io.py → enable SPI1, reboot.")
                print("  Try --spi-port 0 vs --spi-port 1 to match your device tree.")
            sys.exit(1)

        driver_cls = luma_st7789 if args.chip == "st7789" else luma_ili9341

        try:
            device = driver_cls(serial, rotate=rotate)
        except Exception as e:
            print(f"  ERROR creating {args.chip} device: {e}")
            sys.exit(1)

        try:
            gpio.setup(led_bcm, gpio.OUT)
            gpio.output(led_bcm, gpio.HIGH)
            print(f"  Backlight: BCM {led_bcm} HIGH")
        except Exception as e:
            print(f"  WARNING: could not drive backlight on BCM pin {led_bcm}: {e}")
            print("           Tie TFT LED to 3.3V if the panel stays dark.")

        print(f"  Display: {device.width}x{device.height}  (chip={args.chip}, rotate={rotate})")

    # ── run tests ──
    if not args.mock_only:
        test_colour_fills(device)
        test_text(device)

    test_mock_status(device)

    if args.fps:
        test_fps(device)

    # luma clears the display on exit by default; keep it showing the last frame
    # for a moment so the user can see the result before cleanup runs.
    if not args.no_hw:
        print("\nAll tests complete. Display will stay on for 3 seconds...")
        time.sleep(3)
        try:
            gpio.output(led_bcm, gpio.LOW)
        except Exception:
            pass
    else:
        print("\nAll tests complete (no-hw mode).")


if __name__ == "__main__":
    main()
