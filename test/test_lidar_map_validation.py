"""Focused synthetic-geometry tests for scripts/validate_lidar_and_map.py."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "validate_lidar_and_map.py"
SPEC = importlib.util.spec_from_file_location("validate_lidar_and_map", SCRIPT_PATH)
VALIDATOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = VALIDATOR
SPEC.loader.exec_module(VALIDATOR)


class TestLidarMapValidation(unittest.TestCase):
    def test_clockwise_scan_is_converted_to_right_handed_points(self):
        ranges = np.ones(4)
        scan = VALIDATOR.ScanRecord(
            stamp_ns=1,
            bag_stamp_ns=1,
            ranges=ranges,
            intensities=np.zeros(4),
            angle_min=0.0,
            angle_increment=math.pi / 2.0,
            range_min=0.02,
            range_max=12.0,
            frame_id="laser",
        )
        points = VALIDATOR.scan_points(scan)
        expected = np.array(
            [
                [1.0, 0.0],   # phi=0: front
                [0.0, -1.0],  # phi=+90 CW: robot right
                [-1.0, 0.0],  # phi=180: rear
                [0.0, 1.0],   # phi=270 CW: robot left
            ]
        )
        np.testing.assert_allclose(points, expected, atol=1e-8)

    def test_pair_registration_recovers_known_se2_transform(self):
        rng = np.random.default_rng(42)
        previous = rng.normal(size=(500, 2))
        # Per-scan motion is intentionally small (5 Hz LiDAR); nearest-neighbor
        # ICP is not expected to solve arbitrary global registration.
        theta = math.radians(1.0)
        rotation = np.array(
            [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
        )
        translation = np.array([0.03, -0.015])
        # Construct current so applying the known transform aligns it to previous.
        current = (previous - translation) @ rotation
        dx, dy, dtheta, inlier, rmse = VALIDATOR.register_pair(previous, current)
        self.assertAlmostEqual(dx, translation[0], places=3)
        self.assertAlmostEqual(dy, translation[1], places=3)
        self.assertAlmostEqual(dtheta, theta, places=3)
        self.assertGreater(inlier, 0.95)
        self.assertLess(rmse, 0.01)

    def test_metric_occupancy_map_preserves_resolution_and_origin(self):
        points = np.array([[-1.0, -2.0], [0.0, 0.0], [2.0, 3.0]])
        with tempfile.TemporaryDirectory() as directory:
            metadata = VALIDATOR.save_occupancy_map(
                points, Path(directory), "synthetic", resolution=0.05
            )
            self.assertEqual(metadata["resolution_m_per_pixel"], 0.05)
            self.assertLessEqual(metadata["origin"][0], -1.0)
            self.assertLessEqual(metadata["origin"][1], -2.0)
            self.assertTrue(Path(metadata["pgm"]).is_file())
            self.assertTrue(Path(metadata["yaml"]).is_file())
            occupied, loaded = VALIDATOR.load_map_geometry(Path(metadata["yaml"]))
            self.assertEqual(loaded["resolution_m_per_pixel"], 0.05)
            self.assertGreater(len(occupied), 0)
            self.assertGreaterEqual(float(np.min(occupied[:, 0])), metadata["origin"][0])
            self.assertGreaterEqual(float(np.min(occupied[:, 1])), metadata["origin"][1])

    def test_measurement_error_statistics(self):
        measurements = [
            {"id": "a", "reference_key": "width", "measured_m": 4.0},
            {"id": "b", "reference_key": "length", "measured_m": 6.0},
            {"id": "missing", "reference_key": "unknown", "measured_m": 3.0},
        ]
        results, statistics = VALIDATOR.compare_measurements(
            measurements,
            {"width": 4.1, "length": 5.8},
            {},
        )
        self.assertEqual(statistics["comparison_count"], 2)
        self.assertAlmostEqual(statistics["mean_absolute_error_m"], 0.15)
        self.assertAlmostEqual(statistics["maximum_absolute_error_m"], 0.2)
        self.assertAlmostEqual(statistics["rmse_m"], round(math.sqrt(0.025), 4))
        self.assertEqual(statistics["lidar_comparison_count"], 2)
        self.assertEqual(statistics["map_comparison_count"], 0)
        self.assertAlmostEqual(results[0]["lidar_absolute_error_m"], 0.1)
        self.assertIsNone(results[0]["map_absolute_error_m"])
        self.assertIsNone(results[2]["absolute_error_m"])

    def test_coordinate_peaks_recover_parallel_walls(self):
        rng = np.random.default_rng(7)
        values = np.concatenate(
            [
                rng.normal(-1.25, 0.015, size=5000),
                rng.normal(2.75, 0.015, size=4800),
                rng.uniform(-1.25, 2.75, size=200),
            ]
        )
        peaks = VALIDATOR.coordinate_peaks(values)
        span = VALIDATOR.closest_wall_pair(peaks)
        self.assertIsNotNone(span)
        self.assertAlmostEqual(span["span_m"], 4.0, delta=0.08)


if __name__ == "__main__":
    unittest.main()
