"""Hardware-independent battery voltage and SoC tests."""

import math
import unittest

from horseshitbot.drivers.modbus_adc import (
    DEFAULT_SOC_PERCENT_POINTS,
    DEFAULT_SOC_VOLTAGE_POINTS,
    AdcRegisterMap,
    MedianVoltageFilter,
    state_of_charge_percent,
    validate_soc_curve,
)

try:
    from horseshitbot.nodes.battery_modbus_node import battery_state_message
except ImportError:  # pragma: no cover - ROS environment not sourced
    battery_state_message = None


CALIBRATED = AdcRegisterMap(
    raw_scale=49.25,
    voltage_divider_factor=1.0,
    voltage_offset=15.2 / 49.25,
)


class TestVoltageCalibration(unittest.TestCase):
    def test_measured_calibration_points(self):
        for raw, expected in (
            (970, 20.0),
            (1068, 22.0),
            (1167, 24.0),
            (1265, 26.0),
            (1364, 28.0),
        ):
            with self.subTest(raw=raw):
                voltage = CALIBRATED.convert(raw, 1)["battery_voltage"]
                self.assertAlmostEqual(voltage, expected, delta=0.01)


class TestStateOfCharge(unittest.TestCase):
    def test_curve_points_and_range(self):
        for voltage, expected in zip(
            DEFAULT_SOC_VOLTAGE_POINTS, DEFAULT_SOC_PERCENT_POINTS
        ):
            self.assertAlmostEqual(
                state_of_charge_percent(voltage), expected
            )
        for voltage in (18.0, 20.0, 23.4, 26.4, 30.0):
            result = state_of_charge_percent(voltage)
            self.assertGreaterEqual(result, 0.0)
            self.assertLessEqual(result, 100.0)

    def test_interpolation(self):
        self.assertAlmostEqual(
            state_of_charge_percent(26.125),
            (70.8 + 52.1) / 2,
        )

    def test_invalid_curve_rejected(self):
        with self.assertRaises(ValueError):
            validate_soc_curve([26.4, 20.0], [0.0, 100.0])
        with self.assertRaises(ValueError):
            validate_soc_curve([26.4], [100.0])
        with self.assertRaises(ValueError):
            state_of_charge_percent(math.nan)

    def test_ascending_curve_is_normalized(self):
        self.assertEqual(
            validate_soc_curve([20.0, 26.4], [0.0, 100.0]),
            ((26.4, 20.0), (100.0, 0.0)),
        )


class TestMedianFilter(unittest.TestCase):
    def test_rejects_outlier(self):
        filter_ = MedianVoltageFilter(5)
        for value in (26.0, 26.0, 26.0, 26.0, 10.0):
            result = filter_.update(value)
        self.assertEqual(result, 26.0)

    def test_invalid_samples_rejected(self):
        with self.assertRaises(ValueError):
            MedianVoltageFilter(0)
        with self.assertRaises(ValueError):
            MedianVoltageFilter(3).update(math.nan)

    def test_safety_can_use_instantaneous_voltage(self):
        filter_ = MedianVoltageFilter(30)
        for _ in range(30):
            filter_.update(26.0)
        instantaneous = 20.5
        self.assertGreater(filter_.update(instantaneous), 25.0)
        self.assertLessEqual(instantaneous, 21.0)


@unittest.skipIf(
    battery_state_message is None, "ROS 2 Python packages are not sourced"
)
class TestBatteryStatePublishing(unittest.TestCase):
    def test_percentage_uses_zero_to_one_units(self):
        message = battery_state_message(25.9, 0.5, False, False)
        self.assertEqual(message.percentage, 0.5)
        self.assertEqual(message.voltage, 25.9)
        self.assertTrue(message.present)

    def test_invalid_percentage_is_rejected(self):
        for value in (-0.01, 1.01, 50.0):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    battery_state_message(25.9, value, False, False)


if __name__ == "__main__":
    unittest.main()
