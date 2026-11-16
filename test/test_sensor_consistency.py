"""Focused tests for scripts/analyze_sensor_consistency.py."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "analyze_sensor_consistency.py"
SPEC = importlib.util.spec_from_file_location("analyze_sensor_consistency", SCRIPT)
ANALYSIS = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = ANALYSIS
SPEC.loader.exec_module(ANALYSIS)


class TestSensorConsistency(unittest.TestCase):
    def test_trapezoid_has_correct_units_and_no_endpoint_inflation(self):
        time = np.array([0.0, 0.5, 1.0])
        velocity = np.array([2.0, 2.0, 2.0])
        self.assertAlmostEqual(ANALYSIS.trapezoid(time, velocity), 2.0)

    def test_window_integration_interpolates_exact_boundaries(self):
        time = np.array([0.0, 1.0, 2.0])
        velocity = np.array([0.0, 1.0, 2.0])
        self.assertAlmostEqual(
            ANALYSIS.integrate_window(time, velocity, 0.5, 1.5),
            1.0,
        )

    def test_clockwise_scan_reflection_preserves_ranges(self):
        ranges = np.ones(4)
        points = ANALYSIS.corrected_scan_points(
            ranges,
            angle_min=0.0,
            angle_increment=math.pi / 2.0,
            range_min=0.02,
            range_max=12.0,
            stride=1,
        )
        expected = np.array(
            [
                [1.0, 0.0],
                [0.0, -1.0],
                [-1.0, 0.0],
                [0.0, 1.0],
            ]
        )
        np.testing.assert_allclose(points, expected, atol=1e-8)
        np.testing.assert_allclose(np.linalg.norm(points, axis=1), ranges)

    def test_point_to_plane_icp_recovers_small_se2_motion(self):
        x = np.linspace(-2.0, 2.0, 120)
        target = np.vstack(
            [
                np.column_stack((x, np.full_like(x, -1.2))),
                np.column_stack((x, np.full_like(x, 1.3))),
                np.column_stack((np.full_like(x, -2.0), x)),
            ]
        )
        yaw = math.radians(1.2)
        rotation = np.array(
            [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        )
        translation = np.array([0.025, -0.012])
        source = (target - translation) @ rotation
        dx, dy, dyaw, rmse, inliers = ANALYSIS.point_to_plane_icp(
            source, target, initial_yaw=yaw
        )
        self.assertAlmostEqual(dx, translation[0], delta=0.004)
        self.assertAlmostEqual(dy, translation[1], delta=0.004)
        self.assertAlmostEqual(dyaw, yaw, delta=math.radians(0.2))
        self.assertLess(rmse, 0.01)
        self.assertGreater(inliers, 300)

    def test_pose_composition_uses_current_heading(self):
        pose = np.array([1.0, 2.0, math.pi / 2.0])
        result = ANALYSIS.compose_pose(pose, (1.0, 0.0, math.pi / 4.0))
        np.testing.assert_allclose(result[:2], [1.0, 3.0], atol=1e-8)
        self.assertAlmostEqual(result[2], 3.0 * math.pi / 4.0)

    def test_robust_bias_ignores_turning_outlier(self):
        values = np.concatenate((np.full(100, -0.0015), np.array([2.0])))
        self.assertAlmostEqual(ANALYSIS.robust_bias(values), -0.0015)

    def test_zero_seed_does_not_use_imu(self):
        imu = np.array(
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0],
                [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0],
            ]
        )
        self.assertEqual(
            ANALYSIS.scan_seed_yaw(imu, 0.0, 0.1, 0.0, "zero"),
            0.0,
        )
        self.assertAlmostEqual(
            ANALYSIS.scan_seed_yaw(imu, 0.0, 0.1, 0.0, "imu"),
            0.2,
        )

    def test_unknown_seed_mode_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "unsupported ICP seed mode"):
            ANALYSIS.scan_seed_yaw(np.zeros((0, 7)), 0.0, 0.1, 0.0, "odom")

    def test_nearest_timestamp_offset_preserves_sign(self):
        offsets = ANALYSIS._nearest_timestamp_offsets(
            np.array([0.09, 0.31]),
            np.array([0.0, 0.1, 0.2, 0.3]),
        )
        np.testing.assert_allclose(offsets, [0.01, -0.01], atol=1e-12)

    def test_distribution_reports_signed_and_absolute_timing_bounds(self):
        result = ANALYSIS._distribution(
            np.array([-0.2, -0.1, 0.05, 0.1])
        )
        self.assertEqual(result["minimum"], -0.2)
        self.assertEqual(result["maximum"], 0.1)
        self.assertEqual(result["absolute_maximum"], 0.2)
        self.assertGreaterEqual(result["absolute_p95"], 0.1)

    def test_imu_lidar_lag_recovers_known_offset(self):
        imu_t = np.arange(0.0, 5.0, 0.01)
        imu = np.zeros((len(imu_t), 7))
        imu[:, 0] = imu_t
        imu[:, 6] = np.sin(imu_t)
        motion = []
        for t0 in np.arange(0.5, 4.5, 0.2):
            t1 = t0 + 0.2
            midpoint = (t0 + t1) / 2.0
            motion.append(
                {
                    "t0": t0,
                    "t1": t1,
                    "dt": 0.2,
                    "dyaw": math.sin(midpoint + 0.05) * 0.2,
                    "rmse": 0.01,
                    "inliers": 100,
                }
            )
        result = ANALYSIS.estimate_imu_lidar_lag(
            imu,
            motion,
            gyro_bias=0.0,
            rmse_max=0.25,
            min_inliers=20,
        )
        self.assertAlmostEqual(result["best_imu_sample_offset_s"], 0.05, places=3)

    def test_missing_topic_error_names_incomplete_streams(self):
        with self.assertRaisesRegex(
            ValueError, r"/imu/data_raw, /scan"
        ):
            ANALYSIS.require_sensor_streams(10, 0, 0, "incomplete_bag")


if __name__ == "__main__":
    unittest.main()
