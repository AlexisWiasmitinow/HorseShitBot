"""
ILI9341 / ST7789 SPI display driver (luma.lcd).

Wraps luma.lcd + Pillow to provide a simple blit-frame API for the
2.8" 240x320 TFT.  Used by the ROS 2 status_screen_node.

Pin arguments on the CLI / params.yaml follow the same convention as
before: **physical** 40-pin header pins on Jetson, **BCM** GPIO on Pi.
The driver converts to BCM internally (luma.lcd requires BCM mode).
"""

from __future__ import annotations

from typing import Dict, Tuple

from PIL import Image, ImageDraw, ImageFont

_HAS_HW = False
_HW_IMPORT_ERR = ""

try:
    from luma.core.interface.serial import spi as _luma_spi
    from luma.lcd.device import ili9341 as _luma_ili9341, st7789 as _luma_st7789
    _HAS_HW = True
except ImportError as _e:
    _HW_IMPORT_ERR = str(_e)


def _is_jetson_soc() -> bool:
    try:
        with open("/proc/device-tree/model", "rb") as f:
            return b"Jetson" in f.read()
    except OSError:
        return False


# Jetson Nano 40-pin header: physical pin → BCM-equivalent GPIO.
# Jetson.GPIO's BCM mode uses the same numbering as RPi for compatibility.
_JETSON_PHYS_TO_BCM = {
    7: 4, 11: 17, 12: 18, 13: 27, 15: 22, 16: 23, 18: 24,
    19: 10, 21: 9, 22: 25, 23: 11, 24: 8, 26: 7, 29: 5,
    31: 6, 32: 12, 33: 13, 35: 19, 36: 16, 37: 26, 38: 20, 40: 21,
}


def _to_bcm(pin: int) -> int:
    """Convert a physical header pin to BCM on Jetson; pass-through on Pi."""
    if not _is_jetson_soc():
        return pin
    bcm = _JETSON_PHYS_TO_BCM.get(pin)
    if bcm is None:
        raise ValueError(
            f"Jetson physical pin {pin} has no BCM mapping. "
            f"Valid: {sorted(_JETSON_PHYS_TO_BCM)}"
        )
    return bcm


def _load_gpio():
    """Import the platform GPIO library and set BCM mode."""
    if _is_jetson_soc():
        import Jetson.GPIO as GPIO
    else:
        import RPi.GPIO as GPIO
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    return GPIO


def default_display_pins() -> Dict[str, int]:
    """
    Suggested wiring for ILI9341 control lines.

    Jetson Nano (40-pin header): **physical** pin numbers.
    Raspberry Pi: **BCM** GPIO numbers.
    """
    if _is_jetson_soc():
        return {"dc": 22, "rst": 18, "led": 12}
    return {"dc": 24, "rst": 25, "led": 18}


# luma.core asserts bus_speed_hz is one of these values.
_LUMA_VALID_HZ = sorted(
    mhz * 1_000_000
    for mhz in [0.5, 1, 2, 4, 8, 16, 20, 24, 28, 32, 36, 40, 44, 48, 50, 52]
)


def _snap_baud(target: int) -> int:
    return min(_LUMA_VALID_HZ, key=lambda h: abs(h - target))


try:
    _FONT = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 14)
    _FONT_SM = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 11)
    _FONT_LG = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 18)
except Exception:
    _FONT = ImageFont.load_default()
    _FONT_SM = _FONT
    _FONT_LG = _FONT


class ILI9341Display:
    """
    Thin wrapper around luma.lcd (ILI9341 / ST7789).

    Constructor pin arguments use :func:`default_display_pins` when omitted.
    On Jetson those values are **physical** 40-pin header numbers; on Pi
    they are **BCM** GPIO numbers.  Pins are converted to BCM internally.

    If luma.lcd is not installed (e.g. on a dev machine), methods fall back
    to no-ops and ``draw_frame()`` returns the PIL image for inspection.
    """

    WIDTH = 240
    HEIGHT = 320

    def __init__(
        self,
        spi_port: int = 0,
        spi_device: int = 0,
        dc_pin: int | None = None,
        rst_pin: int | None = None,
        led_pin: int | None = None,
        rotation: int = 0,
        baudrate: int = 8_000_000,
        chip: str = "ili9341",
    ):
        dp = default_display_pins()
        dc_pin = dp["dc"] if dc_pin is None else dc_pin
        rst_pin = dp["rst"] if rst_pin is None else rst_pin
        led_pin = dp["led"] if led_pin is None else led_pin

        self._rotation = rotation
        self._device = None
        self._gpio = None
        self._led_bcm: int | None = None

        if _HAS_HW:
            gpio = _load_gpio()
            self._gpio = gpio

            dc_bcm = _to_bcm(dc_pin)
            rst_bcm = _to_bcm(rst_pin)
            self._led_bcm = _to_bcm(led_pin) if led_pin is not None else None

            baud = _snap_baud(baudrate)

            serial = _luma_spi(
                gpio=gpio,
                port=spi_port,
                device=spi_device,
                bus_speed_hz=baud,
                gpio_DC=dc_bcm,
                gpio_RST=rst_bcm,
            )

            rotate_map = {0: 0, 90: 1, 180: 2, 270: 3}
            rotate = rotate_map.get(rotation, 0)

            driver_cls = _luma_st7789 if chip == "st7789" else _luma_ili9341
            self._device = driver_cls(serial, rotate=rotate)

            if self._led_bcm is not None:
                try:
                    gpio.setup(self._led_bcm, gpio.OUT)
                    gpio.output(self._led_bcm, gpio.HIGH)
                except Exception as e:
                    print(f"WARNING: Could not set LED backlight on BCM {self._led_bcm}: {e}")
                    self._led_bcm = None

    @property
    def width(self) -> int:
        if self._device:
            return self._device.width
        if self._rotation in (90, 270):
            return self.HEIGHT
        return self.WIDTH

    @property
    def height(self) -> int:
        if self._device:
            return self._device.height
        if self._rotation in (90, 270):
            return self.WIDTH
        return self.HEIGHT

    def clear(self, color: Tuple[int, int, int] = (0, 0, 0)):
        img = Image.new("RGB", (self.width, self.height), color)
        self.draw_frame(img)

    def draw_frame(self, image: Image.Image):
        """Blit a PIL image to the display. Returns the image for chaining."""
        if self._device:
            self._device.display(image)
        return image

    def set_backlight(self, on: bool):
        if self._gpio and self._led_bcm is not None:
            try:
                self._gpio.output(self._led_bcm, self._gpio.HIGH if on else self._gpio.LOW)
            except Exception:
                pass

    def new_image(self, bg: Tuple[int, int, int] = (0, 0, 0)) -> Image.Image:
        return Image.new("RGB", (self.width, self.height), bg)

    @staticmethod
    def font():
        return _FONT

    @staticmethod
    def font_sm():
        return _FONT_SM

    @staticmethod
    def font_lg():
        return _FONT_LG
