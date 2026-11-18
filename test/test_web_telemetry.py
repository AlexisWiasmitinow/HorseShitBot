"""Dashboard telemetry helpers: camera freshness and battery scaling."""

import math
import sys
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "src" / "horseshitbot"))

from horseshitbot.web.telemetry import (  # noqa: E402
    BATTERY_STALE_SEC,
    CAMERA_STALE_SEC,
    battery_percent_from_fraction,
    battery_percent_from_json,
    battery_view,
    camera_status,
    imu_orientation_available,
    telemetry_health,
)


class CameraStatusTest(unittest.TestCase):
    def test_no_frames_ever_is_known_unavailable(self):
        status = camera_status(100.0, None, None)
        self.assertIs(status["connected"], False)
        self.assertIs(status["color_available"], False)
        self.assertIs(status["depth_available"], False)
        self.assertIsNone(status["last_frame_age_sec"])

    def test_recent_color_and_depth_frames(self):
        status = camera_status(100.0, 99.97, 99.9)
        self.assertIs(status["connected"], True)
        self.assertIs(status["streaming"], True)
        self.assertIs(status["color_available"], True)
        self.assertIs(status["depth_available"], True)
        self.assertAlmostEqual(status["last_frame_age_sec"], 0.03)
        self.assertEqual(status["stale_after_sec"], CAMERA_STALE_SEC)

    def test_color_only(self):
        status = camera_status(100.0, 99.5, None)
        self.assertIs(status["connected"], True)
        self.assertIs(status["depth_available"], False)
        self.assertIsNone(status["depth_age_sec"])

    def test_frames_stop_and_go_stale(self):
        last = 100.0
        self.assertIs(camera_status(last + CAMERA_STALE_SEC, last, last)["connected"], True)
        stale = camera_status(last + CAMERA_STALE_SEC + 0.01, last, last)
        self.assertIs(stale["connected"], False)
        self.assertIs(stale["color_available"], False)
        self.assertAlmostEqual(stale["last_frame_age_sec"], CAMERA_STALE_SEC + 0.01)

    def test_clock_skew_never_negative(self):
        self.assertEqual(camera_status(10.0, 11.0, None)["color_age_sec"], 0.0)


class BatteryScalingTest(unittest.TestCase):
    def test_fraction_scaled_exactly_once(self):
        self.assertEqual(battery_percent_from_fraction(0.8221), 82.2)
        self.assertEqual(battery_percent_from_fraction(1.0), 100.0)
        self.assertEqual(battery_percent_from_fraction(0.0), 0.0)
        # 1 % must stay 1 %, not be mistaken for an already-scaled value.
        self.assertEqual(battery_percent_from_fraction(0.01), 1.0)

    def test_fraction_out_of_range_or_invalid(self):
        self.assertEqual(battery_percent_from_fraction(1.2), 100.0)
        self.assertIsNone(battery_percent_from_fraction(-1.0))
        self.assertIsNone(battery_percent_from_fraction(math.nan))
        self.assertIsNone(battery_percent_from_fraction(None))
        self.assertIsNone(battery_percent_from_fraction("x"))

    def test_json_percentage_is_fraction(self):
        self.assertEqual(battery_percent_from_json({"percentage": 0.79}), 79.0)
        self.assertEqual(battery_percent_from_json({"percentage": 0.01}), 1.0)

    def test_json_percent_is_already_percent(self):
        self.assertEqual(battery_percent_from_json({"percent": 79}), 79.0)
        self.assertEqual(battery_percent_from_json({"percent": 0.5}), 0.5)

    def test_json_percentage_wins_over_percent(self):
        self.assertEqual(battery_percent_from_json({"percentage": 0.5, "percent": 12}), 50.0)

    def test_json_without_percentage(self):
        self.assertIsNone(battery_percent_from_json({"battery_voltage": 26.2}))


class BatteryViewTest(unittest.TestCase):
    def test_fresh_battery_available(self):
        view = battery_view({"percent": 82.2, "updated_at": 1000.0}, 1001.0)
        self.assertIs(view["available"], True)
        self.assertEqual(view["age_sec"], 1.0)
        self.assertEqual(view["stale_after_sec"], BATTERY_STALE_SEC)
        self.assertEqual(view["percent"], 82.2)

    def test_stale_battery_unavailable(self):
        view = battery_view({"percent": 82.2, "updated_at": 1000.0}, 1000.0 + BATTERY_STALE_SEC + 0.1)
        self.assertIs(view["available"], False)
        self.assertEqual(view["percent"], 82.2)

    def test_node_reported_unavailable(self):
        view = battery_view({"percent": 82.2, "available": False, "updated_at": 1000.0}, 1000.5)
        self.assertIs(view["available"], False)

    def test_no_battery_messages(self):
        view = battery_view({}, 1000.0)
        self.assertIs(view["available"], False)
        self.assertIsNone(view["age_sec"])

    def test_does_not_modify_input(self):
        state = {"percent": 50.0, "updated_at": 1000.0}
        battery_view(state, 1000.5)
        self.assertEqual(state, {"percent": 50.0, "updated_at": 1000.0})


class TelemetryHealthTest(unittest.TestCase):
    def test_distinguishes_no_data_live_and_stale(self):
        self.assertEqual(telemetry_health("imu", {}, 10.0)["state"], "no_data")
        self.assertEqual(
            telemetry_health("imu", {"_received_at": 9.0}, 10.0)["state"],
            "live",
        )
        self.assertEqual(
            telemetry_health("imu", {"_received_at": 6.0}, 10.0)["state"],
            "stale",
        )

    def test_distinguishes_disabled_and_error(self):
        self.assertEqual(
            telemetry_health(
                "lidar",
                {"_received_at": 9.0, "connected": True, "scanning": False},
                10.0,
            )["state"],
            "disabled",
        )
        self.assertEqual(
            telemetry_health(
                "lidar",
                {"_received_at": 9.0, "connected": False},
                10.0,
            )["state"],
            "error",
        )
        self.assertEqual(
            telemetry_health(
                "wheel_status",
                {"_received_at": 9.0, "estop_release_error": "timeout"},
                10.0,
            )["state"],
            "error",
        )


class ImuOrientationAvailabilityTest(unittest.TestCase):
    def test_ros_unknown_orientation_sentinel(self):
        self.assertFalse(imu_orientation_available([-1.0] + [0.0] * 8))

    def test_numpy_style_array_does_not_use_ambiguous_truth_value(self):
        import numpy as np

        self.assertTrue(imu_orientation_available(np.zeros(9)))

    def test_empty_invalid_and_nonfinite_covariance_are_unavailable(self):
        self.assertFalse(imu_orientation_available([]))
        self.assertFalse(imu_orientation_available(None))
        self.assertFalse(imu_orientation_available([math.nan]))


if __name__ == "__main__":
    unittest.main()
