"""Tests for the bounded, subscription-only navigation health report."""

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "check_navigation_sensor_health.py"
SPEC = importlib.util.spec_from_file_location("check_navigation_sensor_health", SCRIPT)
HEALTH = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = HEALTH
SPEC.loader.exec_module(HEALTH)


class TestNavigationSensorHealth(unittest.TestCase):
    def test_required_topic_without_publisher_fails(self):
        spec = HEALTH.TopicSpec(object, "odom", 40.0)
        result = HEALTH.assess(spec, HEALTH.Observation(), publisher_count=0)
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("no publisher", result["reasons"])

    def test_optional_cmd_vel_without_publisher_warns(self):
        spec = HEALTH.TopicSpec(object, None, None, required=False)
        result = HEALTH.assess(spec, HEALTH.Observation(), publisher_count=0)
        self.assertEqual(result["status"], "WARN")

    def test_rate_frame_and_monotonic_timestamps_pass(self):
        spec = HEALTH.TopicSpec(object, "laser", 4.0)
        observation = HEALTH.Observation(
            received_monotonic=[0.0, 0.2, 0.4],
            source_stamps_ns=[1, 2, 3],
            frames={"laser"},
        )
        result = HEALTH.assess(spec, observation, publisher_count=1)
        self.assertEqual(result["status"], "PASS")
        self.assertAlmostEqual(result["frequency_hz"], 5.0)

    def test_bad_sample_and_timestamp_are_reported(self):
        spec = HEALTH.TopicSpec(object, "imu_link", 40.0)
        observation = HEALTH.Observation(
            received_monotonic=[0.0, 0.02],
            source_stamps_ns=[2, 1],
            frames={"wrong_frame"},
            invalid_samples=1,
        )
        result = HEALTH.assess(spec, observation, publisher_count=1)
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["timestamp_regressions"], 1)
        self.assertTrue(any("unexpected frame" in item for item in result["reasons"]))


if __name__ == "__main__":
    unittest.main()
