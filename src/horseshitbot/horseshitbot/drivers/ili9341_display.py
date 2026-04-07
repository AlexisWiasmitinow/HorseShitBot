"""
ILI9341 SPI display driver.

Wraps adafruit_rgb_display + Pillow to provide a simple blit-frame API
for the 2.8" 240x320 TFT. Shared by the standalone test script and
the ROS 2 status_screen_node.
"""

from __future__ import annotations

import time
from typing import Tuple

from PIL import Image, ImageDraw, ImageFont

try:
    import board
    import digitalio
    import adafruit_rgb_display.ili9341 as ili9341
    from adafruit_rgb_display.ili9341 import ILI9341

    _HAS_HW = True
except ImportError:
    _HAS_HW = False

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
    Thin wrapper around the Adafruit ILI9341 driver.

    If the hardware libraries are not available (e.g. running on a dev
    machine), methods fall back to no-ops and draw_frame() just returns
    the PIL image for inspection.
    """

    WIDTH = 240
    HEIGHT = 320

    def __init__(
        self,
        spi_port: int = 0,
        cs_pin: int = 8,
        dc_pin: int = 24,
        rst_pin: int = 25,
        led_pin: int = 18,
        rotation: int = 0,
        baudrate: int = 16_000_000,
    ):
        self._rotation = rotation
        self._led_pin = led_pin
        self._disp = None
        self._led_gpio = None

        if _HAS_HW:
            cs = digitalio.DigitalInOut(_board_pin(cs_pin))
            dc = digitalio.DigitalInOut(_board_pin(dc_pin))
            rst = digitalio.DigitalInOut(_board_pin(rst_pin))

            spi = board.SPI()
            self._disp = ILI9341(
                spi,
                cs=cs,
                dc=dc,
                rst=rst,
                baudrate=baudrate,
            )

            if led_pin is not None:
                try:
                    self._led_gpio = digitalio.DigitalInOut(_board_pin(led_pin))
                    self._led_gpio.direction = digitalio.Direction.OUTPUT
                    self._led_gpio.value = True
                except Exception as e:
                    print(f"WARNING: Could not set LED backlight on GPIO {led_pin}: {e}")
                    self._led_gpio = None

    @property
    def width(self) -> int:
        if self._rotation in (90, 270):
            return self.HEIGHT
        return self.WIDTH

    @property
    def height(self) -> int:
        if self._rotation in (90, 270):
            return self.WIDTH
        return self.HEIGHT

    def clear(self, color: Tuple[int, int, int] = (0, 0, 0)):
        img = Image.new("RGB", (self.width, self.height), color)
        self.draw_frame(img)

    def draw_frame(self, image: Image.Image):
        if self._disp:
            self._disp.image(image, rotation=self._rotation)
        return image

    def set_backlight(self, on: bool):
        if self._led_gpio:
            self._led_gpio.value = on

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


def _board_pin(gpio_num: int):
    """Convert a BCM GPIO number to a board pin object."""
    pin_map = {
        4: board.D4, 5: board.D5, 6: board.D6, 7: board.D7,
        8: board.D8, 9: board.D9, 10: board.D10, 11: board.D11,
        12: board.D12, 13: board.D13, 14: board.D14, 15: board.D15,
        16: board.D16, 17: board.D17, 18: board.D18, 19: board.D19,
        20: board.D20, 21: board.D21, 22: board.D22, 23: board.D23,
        24: board.D24, 25: board.D25, 26: board.D26, 27: board.D27,
    }
    return pin_map.get(gpio_num, board.D24)
