#!/usr/bin/env python3
"""
Standalone test script for the 2.8" ILI9341 SPI display (240x320).
No ROS dependency — run on the robot SBC to verify wiring, colour fills,
text rendering, and a mock status screen layout.

Pin numbers follow ``ili9341_display.default_display_pins()``:
  Jetson Nano: physical 40-pin header (after ``jetson-io``: SPI1 + GPIO for DC/RST/LED).
  Raspberry Pi: BCM GPIO (legacy).

Usage:
    python3 ili9341_spi_test.py                 # platform defaults
    python3 ili9341_spi_test.py --mock-only     # skip colour fills, just show status mockup
    python3 ili9341_spi_test.py --fps           # measure full-screen refresh rate
    python3 ili9341_spi_test.py --raw-test --chip st7789 --raw-baud 500000
    python3 ili9341_spi_test.py --raw-test --spi-bus 0 --raw-invert-cmd-test
    python3 ili9341_spi_test.py --spi-loopback --spi-bus 1 --spi-device 0
"""

import argparse
import glob
import os
import struct
import sys
import time

from PIL import Image, ImageDraw, ImageFont

# Same register payload order as adafruit_rgb_display.ili9341.ILI9341._INIT
_ILI9341_INIT_SEQUENCE = (
    (0xEF, b"\x03\x80\x02"),
    (0xCF, b"\x00\xc1\x30"),
    (0xED, b"\x64\x03\x12\x81"),
    (0xE8, b"\x85\x00\x78"),
    (0xCB, b"\x39\x2c\x00\x34\x02"),
    (0xF7, b"\x20"),
    (0xEA, b"\x00\x00"),
    (0xC0, b"\x23"),
    (0xC1, b"\x10"),
    (0xC5, b"\x3e\x28"),
    (0xC7, b"\x86"),
    (0x36, b"\x48"),
    (0x3A, b"\x55"),
    (0xB1, b"\x00\x18"),
    (0xB6, b"\x08\x82\x27"),
    (0xF2, b"\x00"),
    (0x26, b"\x01"),
    (
        0xE0,
        b"\x0f\x31\x2b\x0c\x0e\x08\x4e\xf1\x37\x07\x10\x03\x0e\x09\x00",
    ),
    (
        0xE1,
        b"\x00\x0e\x14\x03\x11\x07\x31\xc1\x48\x08\x0f\x0c\x31\x36\x0f",
    ),
    (0x11, None),
    (0x29, None),
)

# py-spidev on Jetson often rejects xfer2/writebytes buffers larger than this.
_SPIDEV_XFER_MAX = 4096

# ── try to import the shared driver ──
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_DIR = os.path.dirname(_SCRIPT_DIR)
sys.path.insert(0, os.path.join(_REPO_DIR, "src", "horseshitbot"))

_DRIVER_OK = False
_DRIVER_ERR = ""
try:
    from horseshitbot.drivers.ili9341_display import ILI9341Display, default_display_pins
    _DRIVER_OK = True
except ImportError as e:
    _DRIVER_ERR = str(e)


def _cli_pin_defaults():
    if _DRIVER_OK:
        return default_display_pins()
    try:
        with open("/proc/device-tree/model", "rb") as f:
            if b"Jetson" in f.read():
                return {"cs": 24, "dc": 22, "rst": 18, "led": 12}
    except OSError:
        pass
    return {"cs": 8, "dc": 24, "rst": 25, "led": 18}


def _is_jetson_soc() -> bool:
    try:
        with open("/proc/device-tree/model", "rb") as f:
            return b"Jetson" in f.read()
    except OSError:
        return False


def _effective_spi_baud(args) -> int:
    """--raw-slow caps rate for long jumpers / weak edges; else use --raw-baud."""
    if getattr(args, "raw_slow", False):
        return 50_000
    return int(args.raw_baud)


def _spidev_set_mode_no_cs_ioctl(spi) -> None:
    """SPI_NO_CS via ioctl (Tegra/Jetson often returns EINVAL — use hardware CS instead)."""
    import array
    import fcntl

    SPI_IOC_WR_MODE = 0x40016B01
    buf = array.array("B", [0x40])  # mode 0 | SPI_NO_CS
    fcntl.ioctl(spi.fileno(), SPI_IOC_WR_MODE, buf)


# ── fallback fonts ───────────────────────────────────────────────
try:
    FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 14)
    FONT_SM = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 11)
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


def test_spi_loopback(args):
    """
    With MOSI and MISO shorted on the same SPI header, xfer2 should read back what was sent.
    Disconnect the TFT (or anything else) from MOSI/MISO first so the loop is only host pins.
    """
    try:
        import spidev
    except ImportError:
        print("ERROR: install spidev (e.g. sudo apt install python3-spidev)")
        return

    devpath = f"/dev/spidev{args.spi_bus}.{args.spi_device}"
    print("=== SPI loopback (MOSI jumpered to MISO) ===")
    hz = min(_effective_spi_baud(args), 2_000_000)
    print(f"  Device: {devpath}  mode={args.spi_mode}  max_hz={hz}")
    print("  Disconnect the display from MOSI/MISO; bridge those two pins on the Jetson header only.")

    spi = spidev.SpiDev()
    try:
        spi.open(args.spi_bus, args.spi_device)
    except OSError as e:
        print(f"  ERROR: open {devpath}: {e}")
        return
    spi.mode = int(args.spi_mode) & 3
    spi.max_speed_hz = min(_effective_spi_baud(args), 2_000_000)
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


def test_raw_spi(args):
    """Drive the panel with library-equivalent init (not the previous minimal ILI9341 stub)."""
    print("=== Raw SPI test (bypassing Adafruit driver) ===")
    try:
        import board
        import busio
        import digitalio
    except ImportError as e:
        print(f"  ERROR: {e}")
        print("  Install: pip install adafruit-blinka")
        return

    spidevs = sorted(glob.glob("/dev/spidev*"))
    print(f"  SPI devices: {spidevs if spidevs else '(none — enable SPI in jetson-io / raspi-config)'}")
    baud = _effective_spi_baud(args)
    print(f"  Controller: {args.chip}  SPI baud={baud}" + (" (--raw-slow)" if args.raw_slow else ""))

    # Blinka maps board.SCK/MOSI to spiPorts[0] → typically /dev/spidev0.0. On Jetson the 40-pin
    # SPI you enabled may be spidev1.0 instead, so MOSI/SCK never reach the TFT. Use --spi-bus
    # to pick the Linux bus explicitly (GPIO CS/DC/RST stay on --cs/--dc/--rst).

    spidev_dev = None
    use_spidev = args.spi_bus is not None
    # True: kernel drives CS for spidev B.D (wire TFT CS to that CS pin; jetson-io mux).
    # False: GPIO --cs + SPI_NO_CS (often EINVAL on Tegra — use --spi-gpio-cs only if ioctl works).
    use_hw_cs_for_spidev = use_spidev and not args.spi_gpio_cs

    if use_spidev:
        try:
            import spidev
        except ImportError:
            print("  ERROR: spidev not installed. On Debian/Ubuntu: sudo apt install python3-spidev")
            return
        spidev_dev = spidev.SpiDev()
        devpath = f"/dev/spidev{args.spi_bus}.{args.spi_device}"
        try:
            spidev_dev.open(args.spi_bus, args.spi_device)
        except OSError as e:
            print(f"  ERROR: could not open {devpath}: {e}")
            return
        if use_hw_cs_for_spidev:
            spidev_dev.mode = int(args.spi_mode) & 3
            spidev_dev.max_speed_hz = baud
            spidev_dev.bits_per_word = 8
            print(
                f"  Using {devpath} with kernel CS (device index {args.spi_device}). "
                f"Wire TFT CS to that hardware CS; ignore GPIO --cs for SPI."
            )
        else:
            try:
                _spidev_set_mode_no_cs_ioctl(spidev_dev)
            except OSError as e:
                print(f"  ERROR: SPI_NO_CS ioctl failed on {devpath}: {e}")
                print("  On Jetson, omit --spi-gpio-cs so the kernel drives CS (default).")
                spidev_dev.close()
                return
            spidev_dev.max_speed_hz = baud
            spidev_dev.bits_per_word = 8
            print(f"  Using {devpath} with SPI_NO_CS + GPIO CS on pin {args.cs}")

        def spi_write(buf: bytes) -> None:
            if not buf:
                return
            # xfer2 keeps the same CS/SPI semantics as most kernel drivers; writebytes is fine too,
            # but xfer2 is what many display examples use on Linux.
            spidev_dev.xfer2(list(buf))

        def spi_done() -> None:
            spidev_dev.close()

    else:
        print("  SPI transport: Blinka busio.SPI(board.SCK, MOSI=board.MOSI) → first matching spidev")
        if _is_jetson_soc():
            print("  Jetson hint: if the screen stays black, retry with --spi-bus 0 or --spi-bus 1")

        spi = busio.SPI(board.SCK, MOSI=board.MOSI)
        while not spi.try_lock():
            pass
        spi.configure(baudrate=baud)

        def spi_write(buf: bytes) -> None:
            if not buf:
                return
            spi.write(buf)

        def spi_done() -> None:
            spi.unlock()

    rst = digitalio.DigitalInOut(getattr(board, f"D{args.rst}"))
    rst.direction = digitalio.Direction.OUTPUT
    dc = digitalio.DigitalInOut(getattr(board, f"D{args.dc}"))
    dc.direction = digitalio.Direction.OUTPUT
    # Standard ILI9341 SPI: DC low = command, DC high = data. Some breakouts wire this inverted.
    dc_for_cmd = bool(args.raw_dc_invert)
    dc_for_data = not dc_for_cmd
    dc.value = dc_for_cmd
    if args.raw_dc_invert:
        print("  DC wiring: INVERTED mode (command = GPIO HIGH, data = LOW)")

    if use_hw_cs_for_spidev:
        cs = None
    else:
        cs = digitalio.DigitalInOut(getattr(board, f"D{args.cs}"))
        cs.direction = digitalio.Direction.OUTPUT
        cs.value = True

    if not args.raw_no_backlight:
        try:
            _led = digitalio.DigitalInOut(getattr(board, f"D{args.led}"))
            _led.direction = digitalio.Direction.OUTPUT
            _led.value = True
            print(f"  Backlight: GPIO pin {args.led} HIGH (use --raw-no-backlight to skip)")
        except Exception as e:
            print(f"  WARNING: could not drive backlight on pin {args.led}: {e}")
            print("           Tie TFT LED to 3.3V if the panel stays dark.")

    print("  Hardware reset pulse...")
    rst.value = False
    time.sleep(0.05)
    rst.value = True
    time.sleep(0.05)

    def send_cmd(cmd, data=None):
        if cs is not None:
            cs.value = False
        dc.value = dc_for_cmd
        spi_write(bytes([cmd]))
        if data is not None and len(data) > 0:
            dc.value = dc_for_data
            spi_write(bytes(data))
        if cs is not None:
            cs.value = True

    def set_window_240x320():
        send_cmd(0x2A, [0x00, 0x00, 0x00, 0xEF])
        send_cmd(0x2B, [0x00, 0x00, 0x01, 0x3F])

    print("  Controller init...")
    if args.chip == "ili9341":
        if args.raw_init_minimal:
            print("  Using minimal ILI9341 init (not full Adafruit table)")
            send_cmd(0x01, None)
            time.sleep(0.12)
            send_cmd(0x11, None)
            time.sleep(0.12)
            send_cmd(0x3A, b"\x55")
            send_cmd(0x36, b"\x48")
            send_cmd(0x13, None)
            send_cmd(0x29, None)
            time.sleep(0.05)
        else:
            send_cmd(0x01, None)
            time.sleep(0.15)
            for cmd, payload in _ILI9341_INIT_SEQUENCE:
                send_cmd(cmd, payload)
                if cmd == 0x11:
                    time.sleep(0.15)
    else:
        send_cmd(0x01, None)
        time.sleep(0.15)
        send_cmd(0x11, None)
        time.sleep(0.12)
        send_cmd(0x3A, b"\x55")
        send_cmd(0x36, b"\x08")
        cols = struct.pack(">HH", 0, 239)
        rows = struct.pack(">HH", 0, 319)
        send_cmd(0x2A, cols)
        send_cmd(0x2B, rows)
        send_cmd(0x21, None)
        send_cmd(0x13, None)
        send_cmd(0x29, None)
        send_cmd(0x36, b"\xc0")

    time.sleep(0.05)

    if args.raw_diagnose and use_spidev and spidev_dev is not None and args.chip == "ili9341":
        print("  ILI9341 SPI readback (SDO/MISO must reach the Jetson for this to be meaningful):")
        for reg in (0xDA, 0xDB, 0xDC, 0xD3):
            try:
                if cs is not None:
                    cs.value = False
                dc.value = dc_for_cmd
                spidev_dev.xfer2([reg])
                dc.value = dc_for_data
                raw = spidev_dev.xfer2([0x00, 0x00, 0x00, 0x00])
                if cs is not None:
                    cs.value = True
                print(f"    read 0x{reg:02X}: {bytes(raw).hex()}")
            except OSError as e:
                print(f"    read 0x{reg:02X}: OSError {e}")
        print(
            "    Interpret: all 0xff -> MISO floating or wrong spidev; all 0x00 -> MISO low / SDO not wired to "
            "host / read not usable on this flex; non-trivial bytes -> chip is replying (then check CS/window)."
        )

    if args.chip == "ili9341":
        send_cmd(0x13, None)
        send_cmd(0x29, None)
        time.sleep(0.04)

    if args.chip == "ili9341" and args.raw_madctl is not None:
        mv = args.raw_madctl & 0xFF
        send_cmd(0x36, bytes([mv]))
        print(f"  MADCTL override 0x36 = 0x{mv:02X} (try 0xE8 or 0x28 if image stays white/wrong)")

    def _raw_fill_rgb565(hi: int, lo: int) -> None:
        set_window_240x320()
        if cs is not None:
            cs.value = False
        dc.value = dc_for_cmd
        spi_write(bytes([0x2C]))
        dc.value = dc_for_data
        frame = bytes([hi, lo] * (240 * 320))
        if use_spidev and spidev_dev is not None:
            try:
                for off in range(0, len(frame), _SPIDEV_XFER_MAX):
                    part = frame[off : off + _SPIDEV_XFER_MAX]
                    spidev_dev.xfer2(list(part))
            except (OSError, MemoryError, TypeError, OverflowError):
                chunk = bytes([hi, lo] * 240)
                for _ in range(320):
                    spi_write(chunk)
        else:
            chunk = bytes([hi, lo] * 240)
            for _ in range(320):
                spi_write(chunk)
        if cs is not None:
            cs.value = True

    if args.raw_invert_cmd_test:
        # INVON=0x21, INVOFF=0x20 — same on ILI9341 and ST7789. Commands require D/C = command side.
        gh, gl = 0x07, 0xE0  # green
        print("  Inversion command test: solid green, then INVOFF / INVON (~0.7s each)...")
        print("  If the panel never flips, command bytes may be going out as data (check D/C).")
        _raw_fill_rgb565(gh, gl)
        time.sleep(0.3)
        for n in range(1, 6):
            send_cmd(0x20, None)
            print(f"    [{n}/5] INVOFF (0x20)")
            time.sleep(0.7)
            send_cmd(0x21, None)
            print(f"    [{n}/5] INVON (0x21)")
            time.sleep(0.7)
        send_cmd(0x20, None)
        print("  Final: INVOFF — inversion command test done.")
        spi_done()
        print("  Raw SPI test complete (invert-cmd path only).")
        return

    colours = [
        ("Red",   0xF800),
        ("Green", 0x07E0),
        ("Blue",  0x001F),
        ("White", 0xFFFF),
    ]

    for name, colour in colours:
        hi = (colour >> 8) & 0xFF
        lo = colour & 0xFF
        print(f"  Filling {name}...", end=" ", flush=True)
        _raw_fill_rgb565(hi, lo)
        print("ok")
        time.sleep(1)

    spi_done()
    print("  Raw SPI test complete.")
    if _is_jetson_soc() and args.spi_bus is None:
        print("  Jetson: still blank? Try the other controller: --spi-bus 0  then  --spi-bus 1")
    print("  Use the same --spi-bus that passes --spi-loopback on your board (0 vs 1 depends on device tree).")
    print("  If still no image: try --spi-device 1 (CS1), --spi-mode 3, or run the full test (not --raw-test).")
    print(
        "  LED/backlight: held HIGH for the whole test (not strobed per fill). One hardware RST pulse + one SW "
        "reset (0x01) can each flash the panel once. Red/green/blue/white steps blink by design (~1s apart)."
    )
    print(
        "  A short glitch when the script exits is common: Blinka/Jetson.GPIO runs cleanup and may release pins."
    )
    print(
        "  Still no picture? 1) Confirm D/C/RST/MOSI/SCK wiring  2) If stuck white: --raw-madctl 0xE8 then 0x28 "
        "3) --raw-dc-invert  4) --raw-init-minimal  5) --chip st7789"
    )


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
    dp = _cli_pin_defaults()
    parser = argparse.ArgumentParser(description="ILI9341 SPI display test")
    parser.add_argument(
        "--cs",
        type=int,
        default=dp["cs"],
        help="CS pin: Jetson = physical pin (default 24 = SPI1 CS0); Pi = BCM (default 8 = CE0)",
    )
    parser.add_argument(
        "--dc",
        type=int,
        default=dp["dc"],
        help="D/C pin: Jetson physical (default 22); Pi BCM (default 24)",
    )
    parser.add_argument(
        "--rst",
        type=int,
        default=dp["rst"],
        help="RESET pin: Jetson physical (default 18); Pi BCM (default 25)",
    )
    parser.add_argument(
        "--led",
        type=int,
        default=dp["led"],
        help="Backlight pin: Jetson physical (default 12); Pi BCM (default 18). "
        "With --raw-test, driven HIGH unless --raw-no-backlight.",
    )
    parser.add_argument("--rotation", type=int, default=0, choices=[0, 90, 180, 270])
    parser.add_argument("--mock-only", action="store_true", help="Skip colour fills, show status mockup only")
    parser.add_argument("--fps", action="store_true", help="Run FPS benchmark")
    parser.add_argument("--no-hw", action="store_true", help="Run without hardware (renders to in-memory images)")
    parser.add_argument("--raw-test", action="store_true", help="Bypass Adafruit driver, test with raw SPI commands")
    parser.add_argument(
        "--chip",
        choices=("ili9341", "st7789"),
        default="ili9341",
        help="LCD controller for --raw-test (many 2.8\" modules are ST7789, not ILI9341)",
    )
    parser.add_argument(
        "--raw-baud",
        type=int,
        default=1_000_000,
        help="SPI clock (Hz) for --raw-test / --spi-loopback (try 100000, 50000, or 10000 if edges look weak)",
    )
    parser.add_argument(
        "--raw-slow",
        action="store_true",
        help="Cap SPI at 50 kHz (same as --raw-baud 50000); use with --raw-test or --spi-loopback",
    )
    parser.add_argument(
        "--spi-loopback",
        action="store_true",
        help="MOSI jumpered to MISO on same header SPI; requires --spi-bus (disconnect TFT from MOSI/MISO)",
    )
    parser.add_argument(
        "--spi-bus",
        type=int,
        default=None,
        metavar="N",
        help="For --raw-test: use /dev/spidevN.M (kernel CS by default; try N=0 vs 1 on Jetson).",
    )
    parser.add_argument(
        "--spi-device",
        type=int,
        default=0,
        metavar="M",
        help="spidev chip-select index with --spi-bus (header CS0 → usually 0, CS1 → 1)",
    )
    parser.add_argument(
        "--spi-gpio-cs",
        action="store_true",
        help="With --spi-bus: GPIO --cs + SPI_NO_CS ioctl instead of kernel CS (often EINVAL on Jetson)",
    )
    parser.add_argument(
        "--spi-mode",
        type=int,
        choices=(0, 1, 2, 3),
        default=0,
        help="SPI mode (CPOL/CPHA) for --spi-bus; try 0 first, then 3 if the panel is silent",
    )
    parser.add_argument(
        "--raw-diagnose",
        action="store_true",
        help="With --raw-test + --spi-bus + ili9341: read ID registers over MISO (debug wiring/bus)",
    )
    parser.add_argument(
        "--raw-no-backlight",
        action="store_true",
        help="With --raw-test: do not drive --led (default is GPIO HIGH for backlight)",
    )
    parser.add_argument(
        "--raw-dc-invert",
        action="store_true",
        help="With --raw-test: swap D/C polarity (try if glass stays black but SPI/bus look OK)",
    )
    parser.add_argument(
        "--raw-init-minimal",
        action="store_true",
        help="With --raw-test + ili9341: shorter init instead of full Adafruit sequence",
    )
    parser.add_argument(
        "--raw-madctl",
        type=lambda s: int(s, 0),
        default=None,
        metavar="BYTE",
        help="ILI9341 only: set MADCTL (reg 0x36) after init, e.g. 0xE8 (BGR) or 0x28 (common alt)",
    )
    parser.add_argument(
        "--raw-invert-cmd-test",
        action="store_true",
        help="With --raw-test: fill green then toggle INVOFF/INVON (0x20/0x21); if the image never "
        "inverts, SPI may be treated as all data (D/C wrong). Exits after test (skips RGBW sweep).",
    )
    args = parser.parse_args()

    if args.spi_loopback:
        if args.spi_bus is None:
            print("ERROR: --spi-loopback requires --spi-bus (and --spi-device if not 0)")
            sys.exit(1)
        test_spi_loopback(args)
        return

    if args.raw_test:
        test_raw_spi(args)
        return

    if not args.no_hw and not _DRIVER_OK:
        print(f"ERROR: Could not import ILI9341 display driver: {_DRIVER_ERR}")
        print("")
        print("Install the required libraries:")
        print("  sudo pip install adafruit-circuitpython-rgb-display adafruit-blinka Pillow")
        print("Enable SPI: Raspberry Pi → raspi-config; Jetson Nano → sudo /opt/nvidia/jetson-io/jetson-io.py")
        print("\nTo run without hardware (in-memory only), use --no-hw")
        sys.exit(1)

    if args.no_hw:
        print("Running in no-hardware mode (images rendered in memory only)")

        class FakeDisplay:
            width, height = 240, 320
            def clear(self, c): pass
            def draw_frame(self, img): return img
            def new_image(self, bg=(0,0,0)): return Image.new("RGB", (240, 320), bg)

        disp = FakeDisplay()
    else:
        print(
            f"Initialising ILI9341: CS pin {args.cs}, DC pin {args.dc}, "
            f"RST pin {args.rst}, LED pin {args.led}, rotation={args.rotation}"
        )
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
