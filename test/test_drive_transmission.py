import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "horseshitbot"
sys.path.insert(0, str(PACKAGE_ROOT))

from horseshitbot.drivers.drive_transmission import DriveTransmission  # noqa: E402


class DriveTransmissionTest(unittest.TestCase):
    def setUp(self):
        self.transmission = DriveTransmission(
            motor_pulley_teeth=16,
            driven_pulley_teeth=52,
            meters_per_wheel_turn=0.420,
        )

    def test_325_motor_rpm_produces_100_wheel_rpm(self):
        self.assertAlmostEqual(
            self.transmission.motor_to_wheel_rpm(325.0), 100.0
        )

    def test_3_25_motor_revolutions_produce_one_wheel_revolution(self):
        self.assertAlmostEqual(
            self.transmission.motor_to_wheel_revolutions(3.25), 1.0
        )

    def test_motor_and_wheel_conversions_are_inverses(self):
        for wheel_value in (-123.4, 0.0, 100.0):
            motor_value = self.transmission.wheel_to_motor_rpm(wheel_value)
            self.assertAlmostEqual(
                self.transmission.motor_to_wheel_rpm(motor_value), wheel_value
            )

    def test_wheel_travel_uses_existing_sprocket_circumference(self):
        self.assertAlmostEqual(
            self.transmission.motor_revolutions_to_distance(3.25), 0.420
        )
        self.assertAlmostEqual(
            self.transmission.meters_per_motor_turn, 0.12923076923076923
        )

    def test_odometry_twist_uses_transmission_ratio(self):
        linear, angular = self.transmission.motor_rpms_to_twist(
            left_motor_rpm=325.0,
            right_motor_rpm=325.0,
            wheel_separation=0.40,
        )
        self.assertAlmostEqual(linear, 0.7)
        self.assertAlmostEqual(angular, 0.0)

        linear, angular = self.transmission.motor_rpms_to_twist(
            left_motor_rpm=-325.0,
            right_motor_rpm=325.0,
            wheel_separation=0.40,
        )
        self.assertAlmostEqual(linear, 0.0)
        self.assertAlmostEqual(angular, 3.5)

    def test_linear_velocity_command_converts_to_motor_rpm(self):
        motor_rpm = self.transmission.linear_velocity_to_motor_rpm(0.7)
        self.assertAlmostEqual(motor_rpm, 325.0)
        self.assertAlmostEqual(
            self.transmission.motor_rpm_to_linear_velocity(motor_rpm), 0.7
        )


if __name__ == "__main__":
    unittest.main()
