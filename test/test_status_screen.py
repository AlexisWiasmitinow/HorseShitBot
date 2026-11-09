"""Hardware-independent tests for the TFT status screen.

Covers the battery formatting over /battery/status values and the geometry of
the real renderer on the 320x240 landscape panel. Nothing here touches ROS,
SPI, GPIO or the ADC.
"""

import itertools
import math
import unittest
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "horseshitbot"
sys.path.insert(0, str(PACKAGE_ROOT))

from horseshitbot.drivers import battery_readout as br  # noqa: E402
from horseshitbot.drivers.battery_readout import (  # noqa: E402
    CHARGE_TEXT,
    HEALTH_DEAD,
    HEALTH_GOOD,
    HEALTH_UNSPEC_FAILURE,
    LEVEL_CRITICAL,
    LEVEL_LOW,
    LEVEL_OK,
    LEVEL_UNKNOWN,
    STATUS_INVALID,
    STATUS_MISSING,
    STATUS_OK,
    STATUS_STALE,
    battery_readout,
    display_percent,
    format_percent,
    format_voltage,
)
from horseshitbot.drivers.ili9341_display import ILI9341Display  # noqa: E402
from horseshitbot.drivers.status_screen_render import (  # noqa: E402
    COL_AMBER,
    COL_BG,
    COL_GREEN,
    COL_MUTED,
    COL_RED,
    COL_TEXT,
    DASHBOARD_REGIONS,
    REGION_BATTERY,
    REGION_FOOTER,
    REGION_HEADER,
    REGION_ROBOT,
    REGION_TEMPS,
    SCREEN,
    SCREEN_H,
    SCREEN_W,
    Box,
    ScreenSnapshot,
    battery_colour,
    ellipsize,
    font,
    fonts_available,
    render_controls,
    render_dashboard,
    temperature_colour,
    temperature_summary,
    wheel_tile,
)

NOW = 500.0


def sample(percentage, voltage, health=HEALTH_GOOD, present=True):
    return {"percentage": percentage, "voltage": voltage, "health": health,
            "present": present}


def fresh(percentage, voltage, health=HEALTH_GOOD):
    return battery_readout(sample(percentage, voltage, health), NOW - 0.5, NOW)


READY = {"state": 2, "is_referenced": True, "direction": "", "error_message": ""}
THERMALS = [
    {"type": "cpu-thermal", "temp_c": 55.4},
    {"type": "gpu-thermal", "temp_c": 55.6},
    {"type": "tj-thermal", "temp_c": 56.3},
]
WIFI = [{"name": "wlan0", "type": "wifi", "connected": True,
         "ip": "192.168.178.57", "ssid": "HSB-Stable"}]
PAD = {"connected": True, "name": "Xbox Wireless Controller",
       "button_map": {"B": "stop_wheels"}}


class WheelTileTest(unittest.TestCase):
    def test_release_pending_and_failure_are_explicit(self):
        pending = wheel_tile(
            {"estopped": True, "estop_state": "releasing", "backend": "mks"}
        )
        self.assertEqual(pending.status, "RELEASING")
        self.assertEqual(pending.colour, COL_AMBER)

        failed = wheel_tile(
            {
                "estopped": True,
                "estop_state": "active",
                "estop_release_error": "timeout",
            }
        )
        self.assertEqual(failed.status, "REL FAIL")
        self.assertEqual(failed.colour, COL_RED)


def snapshot(**overrides):
    snap = ScreenSnapshot(
        clock="14:20",
        wheel={"backend": "mks", "left_rpm": 0.0, "right_rpm": 0.0,
               "estopped": False, "error": ""},
        mks_bus={"bus_connected": True, "motors": {"1": True, "2": True}},
        actuators={"lift": dict(READY), "brush": dict(READY), "bin_door": dict(READY)},
        battery=fresh(1.0, 28.0),
        thermals=THERMALS,
        network=WIFI,
        gamepad=PAD,
        recorders={"mapping_recorder": {"recording": False}},
    )
    for key, value in overrides.items():
        setattr(snap, key, value)
    return snap


ESTOP_WHEEL = {"backend": "mks", "left_rpm": 0.0, "right_rpm": 0.0,
               "estopped": True, "error": ""}
LONG = "x" * 40 + " very long text that can never fit on a 320 px wide panel " * 4


def scenarios():
    """Every screen state the layout has to survive."""
    return {
        "battery_100": snapshot(),
        "battery_50": snapshot(battery=fresh(0.5, 25.9)),
        "battery_10": snapshot(battery=fresh(0.10, 25.3)),
        "battery_critical": snapshot(battery=fresh(0.0, 19.8, HEALTH_DEAD)),
        "battery_stale": snapshot(battery=battery_readout(
            sample(0.5, 25.9), NOW - 60.0, NOW)),
        "battery_missing": snapshot(battery=battery_readout(None)),
        "battery_invalid": snapshot(battery=fresh(float("nan"), 25.9)),
        "everything_missing": ScreenSnapshot(),
        "estop": snapshot(wheel=ESTOP_WHEEL),
        "estop_faults": snapshot(
            wheel=ESTOP_WHEEL,
            mks_bus={"bus_connected": False, "motors": {"1": False, "2": False}},
            actuators={"lift": {"state": 5, "error_message": LONG},
                       "brush": {"state": 5, "error_message": "jam"},
                       "bin_door": {"state": 5, "error_message": "overcurrent"}}),
        "faults": snapshot(
            wheel={"backend": "mks", "left_rpm": 0, "right_rpm": 0,
                   "estopped": False, "error": LONG},
            actuators={"lift": {"state": 1}, "brush": {"state": 0},
                       "bin_door": {"state": 5, "error_message": LONG}}),
        "long_strings": snapshot(
            gamepad={"connected": True, "name": LONG},
            network=[{"type": "wifi", "connected": True, "ip": "192.168.100.123",
                      "ssid": LONG},
                     {"type": "ethernet", "connected": True, "ip": "10.200.100.123"}],
            recorders={"perception_recorder": {"recording": True, "duration_sec": 3599},
                       "mapping_recorder": {"recording": True, "duration_sec": 61}},
            wheel={"backend": "odrive-with-a-long-name", "left_rpm": -499.9,
                   "right_rpm": 499.9, "estopped": False, "error": ""},
            thermals=THERMALS + [{"type": "soc0-thermal", "temp_c": 99.0}],
            motor_temps={"ODL": 101.0, "ODR": 88.0}),
        "hot": snapshot(thermals=[{"type": "cpu-thermal", "temp_c": 86.0},
                                  {"type": "gpu-thermal", "temp_c": 70.0}]),
        "offline": snapshot(network=[{"type": "wifi", "connected": False, "ip": ""}],
                            gamepad={"connected": False}, recorders=None),
    }


# ── Battery formatting ────────────────────────────────────────────────


class TestPercentFormatting(unittest.TestCase):
    def test_fraction_converted_to_whole_percent(self):
        cases = {1.0: 100, 0.5: 50, 0.10: 10, 0.0: 0, 0.999: 100,
                 0.004: 0, 0.005: 1, 0.2549: 25, 0.255: 26}
        for fraction, expected in cases.items():
            with self.subTest(fraction=fraction):
                self.assertEqual(display_percent(fraction), expected)

    def test_text(self):
        self.assertEqual(format_percent(100), "100%")
        self.assertEqual(format_percent(50), "50%")
        self.assertEqual(format_percent(10), "10%")
        self.assertEqual(format_percent(None), "--%")

    def test_invalid_fractions_are_unknown_not_clamped(self):
        for value in (None, float("nan"), float("inf"), -float("inf"), -0.01,
                      1.01, 50.0, 100.0, "abc", True, object()):
            with self.subTest(value=value):
                self.assertIsNone(display_percent(value))

    def test_voltage_text(self):
        self.assertEqual(format_voltage(28.0), "28.0 V")
        self.assertEqual(format_voltage(25.94), "25.9 V")
        self.assertEqual(format_voltage(25.96), "26.0 V")
        for value in (None, float("nan"), float("inf"), 0.0, -3.0, "x"):
            with self.subTest(value=value):
                self.assertEqual(format_voltage(value), "--.- V")

    def test_uses_published_percentage_not_voltage(self):
        # 28 V would be 100 % on the SoC curve; the screen must show what the
        # battery node published, not recompute it.
        self.assertEqual(fresh(0.5, 28.0).percent_text, "50%")
        self.assertEqual(fresh(0.1, 26.4).percent_text, "10%")

    def test_no_soc_curve_in_screen_code(self):
        source = Path(br.__file__).read_text()
        self.assertNotIn("modbus_adc", source)
        self.assertNotIn("state_of_charge", source)


class TestBatteryTelemetryStates(unittest.TestCase):
    def test_normal(self):
        r = fresh(1.0, 28.0)
        self.assertEqual((r.status, r.percent, r.voltage_text), (STATUS_OK, 100, "28.0 V"))
        self.assertIsNone(r.badge)
        self.assertEqual(r.bar_fraction, 1.0)

    def test_never_received(self):
        r = battery_readout(None)
        self.assertEqual(r.status, STATUS_MISSING)
        self.assertEqual((r.percent_text, r.voltage_text), ("--%", "--.- V"))
        self.assertEqual(r.level, LEVEL_UNKNOWN)
        self.assertEqual(r.badge, "NO DATA")
        self.assertEqual(r.bar_fraction, 0.0)

    def test_not_present(self):
        r = battery_readout(sample(0.9, 27.0, present=False), NOW, NOW)
        self.assertEqual(r.status, STATUS_MISSING)

    def test_stale_boundary(self):
        at_limit = battery_readout(sample(0.5, 25.9), NOW - 5.0, NOW)
        self.assertEqual(at_limit.status, STATUS_OK)
        stale = battery_readout(sample(0.5, 25.9), NOW - 5.1, NOW)
        self.assertEqual(stale.status, STATUS_STALE)
        self.assertEqual(stale.badge, "STALE 5s")

    def test_stale_hides_last_values(self):
        r = battery_readout(sample(0.9, 27.0), NOW - 30.0, NOW, stale_after=5.0)
        self.assertEqual((r.percent_text, r.voltage_text), ("--%", "--.- V"))
        self.assertEqual(r.level, LEVEL_UNKNOWN)
        self.assertEqual(r.bar_fraction, 0.0)
        self.assertEqual(r.age_sec, 30.0)

    def test_stale_age_units(self):
        badge = lambda age: battery_readout(sample(0.5, 25), NOW - age, NOW).badge
        self.assertEqual(badge(12.4), "STALE 12s")
        self.assertEqual(badge(300), "STALE 5m")
        self.assertEqual(badge(3 * 3600), "STALE 3h")

    def test_nan_percentage_keeps_voltage(self):
        r = fresh(float("nan"), 25.9)
        self.assertEqual(r.status, STATUS_INVALID)
        self.assertEqual((r.percent_text, r.voltage_text), ("--%", "25.9 V"))
        self.assertEqual(r.badge, "INVALID")

    def test_invalid_voltage(self):
        r = fresh(0.8, float("nan"))
        self.assertEqual(r.status, STATUS_INVALID)
        self.assertEqual(r.percent_text, "80%")

    def test_charge_is_always_unknown(self):
        self.assertEqual(CHARGE_TEXT, "Charge: unknown")


class TestBatteryColours(unittest.TestCase):
    def test_percent_bands(self):
        cases = {1.0: LEVEL_OK, 0.5: LEVEL_OK, 0.21: LEVEL_OK, 0.20: LEVEL_LOW,
                 0.10: LEVEL_LOW, 0.06: LEVEL_LOW, 0.05: LEVEL_CRITICAL,
                 0.0: LEVEL_CRITICAL}
        for fraction, level in cases.items():
            with self.subTest(fraction=fraction):
                self.assertEqual(fresh(fraction, 25.0).level, level)

    def test_node_health_escalates(self):
        self.assertEqual(fresh(0.5, 21.0, HEALTH_UNSPEC_FAILURE).level, LEVEL_LOW)
        self.assertEqual(fresh(0.5, 19.9, HEALTH_DEAD).level, LEVEL_CRITICAL)
        self.assertEqual(fresh(0.10, 21.0, HEALTH_DEAD).level, LEVEL_CRITICAL)

    def test_badges(self):
        self.assertEqual(fresh(0.1, 25.3).badge, "LOW")
        self.assertEqual(fresh(0.0, 19.8, HEALTH_DEAD).badge, "CRITICAL")

    def test_colour_mapping(self):
        self.assertEqual(battery_colour(LEVEL_OK), COL_GREEN)
        self.assertEqual(battery_colour(LEVEL_LOW), COL_AMBER)
        self.assertEqual(battery_colour(LEVEL_CRITICAL), COL_RED)
        self.assertEqual(battery_colour(LEVEL_UNKNOWN), COL_MUTED)

    @unittest.skipUnless(fonts_available(), "DejaVu fonts not installed")
    def test_rendered_colours(self):
        def has_colour(snap, colour):
            img = render_dashboard(snap).image.crop(
                (REGION_BATTERY.x0, REGION_BATTERY.y0, REGION_BATTERY.x1, REGION_BATTERY.y1))
            return colour in {c for _, c in img.getcolors(1 << 16)}

        self.assertTrue(has_colour(snapshot(battery=fresh(1.0, 28.0)), COL_GREEN))
        self.assertTrue(has_colour(snapshot(battery=fresh(0.1, 25.3)), COL_AMBER))
        self.assertFalse(has_colour(snapshot(battery=fresh(0.1, 25.3)), COL_GREEN))
        crit = snapshot(battery=fresh(0.0, 19.8, HEALTH_DEAD))
        self.assertTrue(has_colour(crit, COL_RED))
        for missing in (battery_readout(None),
                        battery_readout(sample(1.0, 28.0), NOW - 60, NOW)):
            snap = snapshot(battery=missing)
            self.assertFalse(has_colour(snap, COL_GREEN))
            self.assertFalse(has_colour(snap, COL_RED))


# ── Temperatures ──────────────────────────────────────────────────────


class TestTemperatures(unittest.TestCase):
    def test_cpu_gpu_max(self):
        self.assertEqual(temperature_summary(THERMALS),
                         [("CPU", 55.4), ("GPU", 55.6), ("MAX", 56.3)])

    def test_missing_gpu_not_fabricated(self):
        zones = [{"type": "cpu-thermal", "temp_c": 50.0},
                 {"type": "tj-thermal", "temp_c": 52.0}]
        self.assertEqual(temperature_summary(zones), [("CPU", 50.0), ("MAX", 52.0)])

    def test_max_only_when_it_adds_a_sensor(self):
        self.assertEqual(temperature_summary([{"type": "cpu-thermal", "temp_c": 40}]),
                         [("CPU", 40.0)])

    def test_unreadable_values_dropped(self):
        zones = [{"type": "cpu-thermal", "temp_c": float("nan")},
                 {"type": "gpu-thermal", "temp_c": -256.0},
                 {"type": "cv0-thermal", "temp_c": None}]
        self.assertEqual(temperature_summary(zones), [])
        self.assertEqual(temperature_summary(None), [])

    def test_motor_fets_count_towards_max(self):
        out = temperature_summary(THERMALS, {"ODL": 70.0})
        self.assertEqual(out[-1], ("MAX", 70.0))

    def test_threshold_colours(self):
        self.assertEqual(temperature_colour(64.9), COL_TEXT)
        self.assertEqual(temperature_colour(65.0), COL_AMBER)
        self.assertEqual(temperature_colour(84.9), COL_AMBER)
        self.assertEqual(temperature_colour(85.0), COL_RED)


# ── Geometry ──────────────────────────────────────────────────────────


def _intersects(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


@unittest.skipUnless(fonts_available(), "DejaVu fonts not installed")
class TestLayoutGeometry(unittest.TestCase):
    def test_display_geometry_is_landscape(self):
        self.assertEqual((SCREEN_W, SCREEN_H), (320, 240))
        self.assertEqual((ILI9341Display.WIDTH, ILI9341Display.HEIGHT), (320, 240))

    def test_luma_reports_landscape_at_configured_rotation(self):
        try:
            from luma.core.interface.serial import noop
            from luma.lcd.device import ili9341
        except ImportError:
            self.skipTest("luma.lcd not installed")
        # No-op serial and backlight: luma must not claim SPI or GPIO lines
        # that the running status_screen_node owns.
        for rotation, expected in ((0, (320, 240)), (180, (320, 240)),
                                   (90, (240, 320)), (270, (240, 320))):
            device = ili9341(noop(), rotate=rotation // 90, backlight=lambda on: None)
            self.assertEqual((device.width, device.height), expected, rotation)

    def test_regions_inside_screen_and_disjoint(self):
        regions = list(DASHBOARD_REGIONS.values())
        for region in regions:
            self.assertTrue(SCREEN.contains((region.x0, region.y0, region.x1, region.y1)))
        for a, b in itertools.combinations(regions, 2):
            self.assertFalse(a.intersects(b), f"{a} overlaps {b}")

    def test_regions_stacked_top_to_bottom(self):
        order = [REGION_HEADER, REGION_ROBOT, REGION_BATTERY, REGION_TEMPS, REGION_FOOTER]
        for upper, lower in zip(order, order[1:]):
            self.assertLessEqual(upper.y1, lower.y0)
        self.assertEqual(REGION_FOOTER.y1, SCREEN_H)

    def test_battery_is_the_largest_section(self):
        areas = {name: r.w * r.h for name, r in DASHBOARD_REGIONS.items()}
        self.assertEqual(max(areas, key=areas.get), "battery")

    def test_image_size(self):
        for name, snap in scenarios().items():
            with self.subTest(name):
                self.assertEqual(render_dashboard(snap).image.size, (320, 240))

    def test_every_item_stays_in_its_region_and_limit(self):
        for name, snap in scenarios().items():
            result = render_dashboard(snap)
            for item in result.items:
                with self.subTest(name, item=item.text or item.kind, region=item.region):
                    region = DASHBOARD_REGIONS[item.region]
                    self.assertTrue(region.contains(item.bbox), f"{item.bbox} outside {region}")
                    self.assertTrue(item.limit.contains(item.bbox),
                                    f"{item.bbox} outside limit {item.limit}")

    def test_text_never_overlaps_text(self):
        for name, snap in scenarios().items():
            texts = render_dashboard(snap).texts()
            for a, b in itertools.combinations(texts, 2):
                with self.subTest(name, a=a.text, b=b.text):
                    self.assertFalse(_intersects(a.bbox, b.bbox))

    def test_gutters_and_margins_stay_background(self):
        bg = COL_BG
        for name, snap in scenarios().items():
            img = render_dashboard(snap).image
            rows = [22, 23, 76, 77, 78, 173, 174, 175, 197, 198, 199]
            for y in rows:
                with self.subTest(name, row=y):
                    self.assertTrue(all(img.getpixel((x, y)) == bg for x in range(SCREEN_W)))
            for y in range(REGION_ROBOT.y0, REGION_TEMPS.y1):
                for x in (0, 1, 2, 3, 316, 317, 318, 319):
                    self.assertEqual(img.getpixel((x, y)), bg, f"{name} ({x},{y})")

    def test_footer_present_in_every_state(self):
        for name, snap in scenarios().items():
            with self.subTest(name):
                footer = [t.text for t in render_dashboard(snap).texts("footer")]
                self.assertIn("PAD", footer)
                self.assertIn("REC", footer)
                self.assertTrue({"NET", "WIFI", "ETH"} & set(footer), footer)

    def test_key_text_not_truncated(self):
        must_show = {"HORSESHITBOT", "14:20", "BATTERY", "Charge: unknown",
                     "100%", "28.0 V", "CPU", "GPU", "MAX", "192.168.178.57",
                     "READY", "WHEELS", "LIFT", "BRUSH", "BIN DOOR"}
        result = render_dashboard(snapshot())
        shown = {t.text for t in result.texts()}
        self.assertTrue(must_show <= shown, must_show - shown)
        self.assertFalse([t.text for t in result.texts() if t.truncated])

    def test_required_battery_states_render_untruncated(self):
        for frac, volts, pct, vtext in ((1.0, 28.0, "100%", "28.0 V"),
                                        (0.5, 25.9, "50%", "25.9 V"),
                                        (0.1, 25.3, "10%", "25.3 V")):
            with self.subTest(pct):
                texts = render_dashboard(snapshot(battery=fresh(frac, volts))).texts("battery")
                shown = {t.text for t in texts}
                self.assertIn(pct, shown)
                self.assertIn(vtext, shown)
                self.assertFalse([t.text for t in texts if t.truncated])

    def test_percentage_is_the_largest_text(self):
        result = render_dashboard(snapshot())
        tallest = max(result.texts(), key=lambda t: t.bbox[3] - t.bbox[1])
        self.assertEqual(tallest.text, "100%")

    def test_long_strings_are_ellipsized_inside_limits(self):
        result = render_dashboard(scenarios()["long_strings"])
        truncated = [t for t in result.texts() if t.truncated]
        self.assertTrue(truncated)
        for t in truncated:
            self.assertTrue(t.text.endswith("\u2026"), t.text)
            self.assertTrue(t.limit.contains(t.bbox))

    def test_ellipsize(self):
        f = font(10)
        self.assertEqual(ellipsize("short", f, 200), "short")
        clipped = ellipsize(LONG, f, 80)
        self.assertTrue(clipped.endswith("\u2026"))
        self.assertLessEqual(f.getlength(clipped), 80)
        self.assertEqual(ellipsize("abc", f, 0), "")

    def test_charge_state_never_invented(self):
        for name, snap in scenarios().items():
            with self.subTest(name):
                text = render_dashboard(snap).all_text().lower()
                self.assertIn("charge: unknown", text)
                self.assertNotIn("discharg", text)
                self.assertNotIn("charging", text)

    def test_no_green_health_claim_without_telemetry(self):
        result = render_dashboard(ScreenSnapshot())
        img = result.image
        self.assertNotIn(COL_GREEN, {c for _, c in img.getcolors(1 << 16)})
        text = result.all_text()
        for word in ("READY", "OK", "Healthy", "ON"):
            self.assertNotIn(word, text.split(" | "))


@unittest.skipUnless(fonts_available(), "DejaVu fonts not installed")
class TestEstopGeometry(unittest.TestCase):
    def _items(self, snap, region):
        return [(i.kind, i.bbox, i.text) for i in render_dashboard(snap).items
                if i.region == region]

    def test_estop_banner_shown(self):
        result = render_dashboard(snapshot(wheel=ESTOP_WHEEL))
        texts = {t.text for t in result.texts("robot")}
        self.assertIn("E-STOP ACTIVE", texts)
        self.assertIn("Press B to resume", texts)
        self.assertIn("E-STOP", {t.text for t in result.texts("header")})
        self.assertTrue(result.mode.startswith("estop"))

    def test_resume_button_follows_mapping(self):
        result = render_dashboard(snapshot(wheel=ESTOP_WHEEL, resume_button="Y"))
        self.assertIn("Press Y to resume", {t.text for t in result.texts("robot")})

    def test_estop_does_not_move_battery_temps_or_footer(self):
        for estop in ("estop", "estop_faults"):
            snap = scenarios()[estop]
            base = snapshot(battery=snap.battery, network=snap.network,
                            gamepad=snap.gamepad, recorders=snap.recorders,
                            thermals=snap.thermals)
            for region in ("battery", "temps", "footer"):
                with self.subTest(estop, region=region):
                    self.assertEqual(self._items(snap, region), self._items(base, region))

    def test_estop_content_fits_robot_region(self):
        for name in ("estop", "estop_faults"):
            result = render_dashboard(scenarios()[name])
            robot = [i for i in result.items if i.region == "robot"]
            self.assertTrue(robot)
            for item in robot:
                self.assertTrue(REGION_ROBOT.contains(item.bbox), (name, item))
            self.assertLessEqual(max(i.bbox[3] for i in robot), REGION_ROBOT.y1)
            self.assertLess(REGION_ROBOT.y1, REGION_BATTERY.y0)

    def test_estop_with_faults_keeps_error_text(self):
        result = render_dashboard(scenarios()["estop_faults"])
        robot_text = " ".join(t.text for t in result.texts("robot"))
        self.assertIn("MKS", robot_text)
        self.assertEqual(result.mode, "estop-inline")
        banner = [t for t in result.texts("robot") if t.text == "E-STOP ACTIVE"]
        self.assertEqual(len(banner), 1)
        self.assertFalse(banner[0].truncated)


@unittest.skipUnless(fonts_available(), "DejaVu fonts not installed")
class TestControlsOverlay(unittest.TestCase):
    BUTTON_MAP = {
        "A": "e_stop", "B": "stop_wheels", "X": "stop_actuators", "Y": "stop_wheels",
        "LB": "bin_door", "RB": "brush", "LT": "none", "RT": "none", "L3": "e_stop",
        "R3": "e_stop", "D-Pad Up": "lift_up", "D-Pad Down": "lift_down",
        "D-Pad Left": "mapping_bag_recording", "D-Pad Right": "none",
        "Options": "perception_bag_recording", "Menu": "reference_all",
        "Logo": "toggle_controls",
    }

    def test_all_default_mappings_visible_and_on_screen(self):
        result = render_controls({"button_map": self.BUTTON_MAP, "active_inputs": ["RB"]})
        self.assertEqual(result.image.size, (320, 240))
        shown = {t.text for t in result.texts()}
        for btn in list(self.BUTTON_MAP) + ["L Stick", "R Stick"]:
            self.assertIn(btn.replace("D-Pad ", "D-"), shown)
        self.assertIn("[Home] back to status", shown)
        for item in result.items:
            self.assertTrue(SCREEN.contains(item.bbox), item)
            self.assertTrue(item.limit.contains(item.bbox), item)

    def test_empty_gamepad(self):
        result = render_controls(None)
        self.assertIn("No gamepad button map yet", {t.text for t in result.texts()})


class TestBoxHelpers(unittest.TestCase):
    def test_contains_and_intersects(self):
        a = Box(0, 0, 10, 10)
        self.assertTrue(a.contains((0, 0, 10, 10)))
        self.assertFalse(a.contains((0, 0, 11, 10)))
        self.assertFalse(a.intersects(Box(10, 0, 20, 10)))
        self.assertTrue(a.intersects(Box(9, 9, 20, 20)))
        self.assertTrue(math.isclose(a.cy, 5.0))


if __name__ == "__main__":
    unittest.main()
