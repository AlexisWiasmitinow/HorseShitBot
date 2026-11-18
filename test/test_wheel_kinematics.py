"""Hardware-independent tests for ROS Twist and wheel-speed conversion."""

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "horseshitbot"))

from horseshitbot.drivers.wheel_kinematics import (  # noqa: E402
    twist_to_wheel_speeds,
    wheel_rpm_to_twist,
)


class TestWheelKinematics(unittest.TestCase):
    METERS_PER_TURN = 0.084
    SEPARATION = 0.40
    MAX_SPEED = 1.5

    def convert(self, linear: float, angular: float):
        return twist_to_wheel_speeds(
            linear,
            angular,
            self.SEPARATION,
            self.METERS_PER_TURN,
            self.MAX_SPEED,
        )

    def test_forward_command_is_metres_per_second_not_normalized(self):
        speeds = self.convert(1.2, 0.0)
        self.assertAlmostEqual(speeds.left_m_s, 1.2)
        self.assertAlmostEqual(speeds.right_m_s, 1.2)
        self.assertAlmostEqual(speeds.left_rpm, 857.142857, places=5)

    def test_one_radian_per_second_uses_track_separation(self):
        speeds = self.convert(0.0, 1.0)
        self.assertAlmostEqual(speeds.left_m_s, -0.2)
        self.assertAlmostEqual(speeds.right_m_s, 0.2)
        self.assertAlmostEqual(speeds.left_rpm, -142.857143, places=5)
        self.assertAlmostEqual(speeds.right_rpm, 142.857143, places=5)

        linear, angular = wheel_rpm_to_twist(
            speeds.left_rpm,
            speeds.right_rpm,
            self.SEPARATION,
            self.METERS_PER_TURN,
        )
        self.assertAlmostEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 1.0)

    def test_saturation_preserves_curvature(self):
        speeds = self.convert(1.5, 1.0)
        self.assertAlmostEqual(speeds.right_m_s, 1.5)
        self.assertAlmostEqual(speeds.left_m_s, 1.5 * 1.3 / 1.7)
        self.assertAlmostEqual(speeds.saturation_scale, 1.5 / 1.7)
        linear, angular = wheel_rpm_to_twist(
            speeds.left_rpm,
            speeds.right_rpm,
            self.SEPARATION,
            self.METERS_PER_TURN,
        )
        self.assertAlmostEqual(linear / angular, 1.5)

    def test_reverse_and_turn_round_trip(self):
        speeds = self.convert(-0.7, -0.4)
        linear, angular = wheel_rpm_to_twist(
            speeds.left_rpm,
            speeds.right_rpm,
            self.SEPARATION,
            self.METERS_PER_TURN,
        )
        self.assertAlmostEqual(linear, -0.7)
        self.assertAlmostEqual(angular, -0.4)

    def test_current_16_52_drivetrain_produces_nonzero_motor_commands(self):
        meters_per_motor_turn = (16.0 / 52.0) * 0.420
        speeds = twist_to_wheel_speeds(
            1.0,
            0.0,
            self.SEPARATION,
            meters_per_motor_turn,
            self.MAX_SPEED,
        )
        self.assertAlmostEqual(speeds.left_rpm, 464.285714, places=5)
        self.assertAlmostEqual(speeds.right_rpm, 464.285714, places=5)

    def test_invalid_geometry_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "wheel_separation_m"):
            twist_to_wheel_speeds(0.0, 0.0, 0.0, 0.084, 1.5)
        with self.assertRaisesRegex(ValueError, "meters_per_motor_turn"):
            wheel_rpm_to_twist(0.0, 0.0, 0.4, -0.084)


class TestRobotLaunchStructure(unittest.TestCase):
    def test_each_bag_recorder_is_declared_once(self):
        source = (
            ROOT
            / "src"
            / "horseshitbot"
            / "launch"
            / "robot_launch.py"
        ).read_text()
        self.assertEqual(source.count('name="perception_recorder"'), 1)
        self.assertEqual(source.count('name="mapping_recorder"'), 1)


if __name__ == "__main__":
    unittest.main()
