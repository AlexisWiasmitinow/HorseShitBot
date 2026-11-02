"""Tests for deterministic October 1 scan correction and replay metadata."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest

from sensor_msgs.msg import LaserScan


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = REPO_ROOT / "scripts" / "oct1_bag_player.py"
SPEC = importlib.util.spec_from_file_location("oct1_bag_player", SCRIPT)
PLAYER = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = PLAYER
SPEC.loader.exec_module(PLAYER)


class TestOct1BagPlayer(unittest.TestCase):
    def test_reflection_preserves_zero_angle_and_reverses_lateral_rays(self):
        message = LaserScan()
        message.ranges = [0.0, 1.0, 2.0, 3.0]
        message.intensities = [10.0, 11.0, 12.0, 13.0]
        corrected = PLAYER.correct_scan_handedness(message)
        self.assertEqual(list(corrected.ranges), [0.0, 3.0, 2.0, 1.0])
        self.assertEqual(list(corrected.intensities), [10.0, 13.0, 12.0, 11.0])
        self.assertEqual(list(message.ranges), [0.0, 1.0, 2.0, 3.0])

    def test_reflection_is_its_own_inverse(self):
        message = LaserScan()
        message.ranges = [float(index) for index in range(720)]
        twice = PLAYER.correct_scan_handedness(
            PLAYER.correct_scan_handedness(message)
        )
        self.assertEqual(list(twice.ranges), list(message.ranges))

    def test_recording_date_static_transform_is_explicit(self):
        transform = PLAYER.configured_static_transform()
        self.assertEqual(transform.header.frame_id, "base_link")
        self.assertEqual(transform.child_frame_id, "laser")
        self.assertAlmostEqual(transform.transform.translation.x, 0.0)
        self.assertAlmostEqual(transform.transform.translation.y, 0.0)
        self.assertAlmostEqual(transform.transform.translation.z, 0.15)
        self.assertAlmostEqual(transform.transform.rotation.z, 1.0, places=8)
        self.assertAlmostEqual(transform.transform.rotation.w, 0.0, places=5)


if __name__ == "__main__":
    unittest.main()
