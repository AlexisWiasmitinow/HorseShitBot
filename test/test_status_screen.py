"""Hardware-independent battery formatting and TFT renderer tests."""

import math
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "horseshitbot"))

from horseshitbot.drivers.battery_readout import (  # noqa: E402
    HEALTH_DEAD,
    STATUS_INVALID,
    STATUS_MISSING,
    STATUS_OK,
    STATUS_STALE,
    battery_readout,
    display_percent,
)
from horseshitbot.drivers.ili9341_display import ILI9341Display  # noqa: E402
from horseshitbot.drivers.status_screen_render import (  # noqa: E402
    COL_AMBER,
    COL_BG,
    COL_BORDER,
    COL_CARD,
    COL_GREEN,
    COL_RED,
    COL_TEXT,
    COL_TEXT_2,
    SCREEN_H,
    SCREEN_W,
    ScreenSnapshot,
    render_controls,
    render_dashboard,
)


NOW = 100.0


def readout(percentage, voltage, health=1, age=0.1):
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


def snapshot(battery):
    ready = {"state": 2, "is_referenced": True}
    return ScreenSnapshot(
        clock="12:34",
        wheel={
            "backend": "mks",
            "left_rpm": 0,
            "right_rpm": 0,
            "estopped": False,
            "error": "",
        },
        mks_bus={"motors": {"1": True, "2": True}},
        actuators={
            "lift": ready,
            "brush": ready,
            "bin_door": ready,
        },
        battery=battery,
        thermals=[{"type": "cpu", "temp_c": 50.0}],
        network=[{"type": "wifi", "connected": True, "ip": "10.0.0.2"}],
        gamepad={"connected": True, "name": "Xbox"},
        recorders={"mapping_recorder": {"recording": False}},
    )


class TestBatteryFormatting(unittest.TestCase):
    def test_fraction_to_display_percentage(self):
        for source, expected in (
            (0.0, 0),
            (0.1, 10),
            (0.5, 50),
            (1.0, 100),
            (0.255, 26),
        ):
            with self.subTest(source=source):
                self.assertEqual(display_percent(source), expected)

    def test_invalid_percentages_are_unavailable(self):
        for value in (
            None,
            math.nan,
            math.inf,
            -0.01,
            1.01,
            50,
            True,
        ):
            with self.subTest(value=value):
                self.assertIsNone(display_percent(value))

    def test_valid_message(self):
        result = readout(0.5, 25.9)
        self.assertEqual(result.status, STATUS_OK)
        self.assertEqual(result.percent_text, "50%")
        self.assertEqual(result.voltage_text, "25.9 V")

    def test_missing_and_stale_hide_values(self):
        missing = battery_readout(None)
        stale = readout(0.5, 25.9, age=6.0)
        self.assertEqual(missing.status, STATUS_MISSING)
        self.assertEqual(stale.status, STATUS_STALE)
        for result in (missing, stale):
            self.assertEqual(result.percent_text, "--%")
            self.assertEqual(result.voltage_text, "--.- V")
            self.assertEqual(result.bar_fraction, 0.0)

    def test_invalid_field_does_not_fabricate_value(self):
        result = readout(math.nan, 25.9)
        self.assertEqual(result.status, STATUS_INVALID)
        self.assertEqual(result.percent_text, "--%")
        self.assertEqual(result.voltage_text, "25.9 V")

    def test_health_can_escalate_display(self):
        self.assertEqual(readout(0.5, 19.9, HEALTH_DEAD).level, "critical")


class TestRenderer(unittest.TestCase):
    def test_required_light_theme_palette(self):
        self.assertEqual(COL_BG, (255, 255, 255))
        self.assertEqual(COL_CARD, (245, 247, 250))
        self.assertEqual(COL_TEXT, (17, 24, 39))
        self.assertEqual(COL_TEXT_2, (100, 116, 139))
        self.assertEqual(COL_BORDER, (226, 232, 240))
        self.assertEqual(COL_GREEN, (22, 163, 74))
        self.assertEqual(COL_AMBER, (245, 158, 11))
        self.assertEqual(COL_RED, (220, 38, 38))

    def test_default_canvas_is_white_and_cards_are_light(self):
        image = render_dashboard(snapshot(readout(0.5, 25.9))).image
        self.assertEqual(image.getpixel((0, 0)), COL_BG)
        self.assertEqual(image.getpixel((6, 26)), COL_CARD)
        self.assertNotIn(
            (11, 23, 41),
            {colour for _, colour in image.getcolors(1 << 20)},
        )

    def test_geometry_matches_physical_landscape_panel(self):
        self.assertEqual((SCREEN_W, SCREEN_H), (320, 240))
        self.assertEqual(
            (ILI9341Display.WIDTH, ILI9341Display.HEIGHT), (320, 240)
        )

    def test_required_battery_states_render(self):
        for battery in (
            readout(1.0, 28.0),
            readout(0.5, 25.9),
            readout(0.1, 25.3),
            readout(0.0, 19.8, HEALTH_DEAD),
            readout(0.5, 25.9, age=6.0),
            battery_readout(None),
        ):
            image = render_dashboard(snapshot(battery)).image
            self.assertEqual(image.size, (320, 240))
            self.assertEqual(image.mode, "RGB")

    def test_status_indicators_change_pixels(self):
        normal = render_dashboard(snapshot(readout(0.5, 25.9))).image
        low = render_dashboard(snapshot(readout(0.1, 25.3))).image
        missing = render_dashboard(snapshot(battery_readout(None))).image
        self.assertNotEqual(normal.tobytes(), low.tobytes())
        self.assertNotEqual(normal.tobytes(), missing.tobytes())
        self.assertIn(
            COL_AMBER,
            {colour for _, colour in low.getcolors(1 << 20)},
        )

    def test_estop_and_controls_preserve_screen_size(self):
        state = snapshot(readout(0.5, 25.9))
        state.wheel["estopped"] = True
        self.assertEqual(render_dashboard(state).image.size, (320, 240))
        controls = render_controls(
            {"button_map": {"A": "e_stop", "B": "stop_wheels"}}
        )
        self.assertEqual(controls.image.size, (320, 240))


if __name__ == "__main__":
    unittest.main()
