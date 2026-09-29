"""Hardware-independent tests for the battery state-of-charge helpers.

The measured ADC calibration itself is covered by test_shared_modbus.py; this
module only covers the SoC curve, its smoothing, and the rule that smoothing
must never reach the shutdown monitor.
"""

import unittest
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPO_ROOT / "src" / "horseshitbot"
sys.path.insert(0, str(PACKAGE_ROOT))

from horseshitbot.drivers.modbus_adc import (  # noqa: E402
    CRITICAL,
    DEFAULT_SOC_PERCENT_POINTS,
    DEFAULT_SOC_VOLTAGE_POINTS,
    LowVoltageMonitor,
    MedianVoltageFilter,
    state_of_charge_percent,
    validate_soc_curve,
)


class TestStateOfCharge(unittest.TestCase):
    def test_above_top_point_is_full(self):
        for voltage in (26.41, 27.0, 28.0, 30.0):
            with self.subTest(voltage=voltage):
                self.assertEqual(state_of_charge_percent(voltage), 100.0)

    def test_exactly_top_point_is_full(self):
        self.assertEqual(state_of_charge_percent(26.40), 100.0)

    def test_exactly_bottom_point_is_empty(self):
        self.assertEqual(state_of_charge_percent(20.00), 0.0)

    def test_below_bottom_point_is_empty(self):
        for voltage in (19.99, 19.0, 0.0, -5.0):
            with self.subTest(voltage=voltage):
                self.assertEqual(state_of_charge_percent(voltage), 0.0)

    def test_every_calibration_point_returns_its_own_percentage(self):
        pairs = zip(DEFAULT_SOC_VOLTAGE_POINTS, DEFAULT_SOC_PERCENT_POINTS)
        for voltage, percent in pairs:
            with self.subTest(voltage=voltage):
                self.assertAlmostEqual(
                    state_of_charge_percent(voltage), percent, places=6
                )

    def test_interpolates_midway_between_points(self):
        # Midway between (26.20, 70.8) and (26.05, 52.1).
        self.assertAlmostEqual(
            state_of_charge_percent(26.125), (70.8 + 52.1) / 2, places=6
        )
        # Midway between (24.00, 3.0) and (23.00, 1.5).
        self.assertAlmostEqual(
            state_of_charge_percent(23.5), (3.0 + 1.5) / 2, places=6
        )

    def test_interpolated_value_stays_between_its_neighbours(self):
        # 25.90 V lies between the (25.95, 39.8) and (25.85, 32.2) points.
        percent = state_of_charge_percent(25.90)
        self.assertLess(percent, 39.8)
        self.assertGreater(percent, 32.2)

    def test_monotonic_and_bounded_across_whole_range(self):
        previous = None
        voltage = 30.0
        while voltage >= 18.0:
            percent = state_of_charge_percent(voltage)
            self.assertGreaterEqual(percent, 0.0)
            self.assertLessEqual(percent, 100.0)
            if previous is not None:
                self.assertLessEqual(percent, previous + 1e-9)
            previous = percent
            voltage -= 0.01

    def test_fraction_form_is_always_within_zero_and_one(self):
        # This is what BatteryState.percentage is set from.
        voltage = 40.0
        while voltage >= -5.0:
            fraction = state_of_charge_percent(voltage) / 100.0
            self.assertGreaterEqual(fraction, 0.0)
            self.assertLessEqual(fraction, 1.0)
            voltage -= 0.05

    def test_accepts_a_custom_curve(self):
        volts, pcts = (30.0, 20.0), (100.0, 0.0)
        self.assertAlmostEqual(state_of_charge_percent(25.0, volts, pcts), 50.0)

    def test_rejects_non_finite_voltage(self):
        with self.assertRaises(ValueError):
            state_of_charge_percent(float("nan"))


class TestValidateSocCurve(unittest.TestCase):
    def test_accepts_the_defaults_unchanged(self):
        volts, pcts = validate_soc_curve(
            DEFAULT_SOC_VOLTAGE_POINTS, DEFAULT_SOC_PERCENT_POINTS
        )
        self.assertEqual(volts, DEFAULT_SOC_VOLTAGE_POINTS)
        self.assertEqual(pcts, DEFAULT_SOC_PERCENT_POINTS)

    def test_normalises_ascending_input_to_descending(self):
        volts, pcts = validate_soc_curve([20.0, 26.4], [0.0, 100.0])
        self.assertEqual(volts, (26.4, 20.0))
        self.assertEqual(pcts, (100.0, 0.0))

    def test_normalised_ascending_curve_interpolates_correctly(self):
        volts, pcts = validate_soc_curve([20.0, 30.0], [0.0, 100.0])
        self.assertAlmostEqual(state_of_charge_percent(25.0, volts, pcts), 50.0)

    def test_rejects_length_mismatch(self):
        with self.assertRaises(ValueError):
            validate_soc_curve([26.4, 25.0, 20.0], [100.0, 0.0])

    def test_rejects_fewer_than_two_points(self):
        with self.assertRaises(ValueError):
            validate_soc_curve([26.4], [100.0])

    def test_rejects_out_of_range_percentages(self):
        with self.assertRaises(ValueError):
            validate_soc_curve([26.4, 20.0], [101.0, 0.0])
        with self.assertRaises(ValueError):
            validate_soc_curve([26.4, 20.0], [100.0, -1.0])

    def test_rejects_non_monotonic_voltages(self):
        with self.assertRaises(ValueError):
            validate_soc_curve([26.4, 22.0, 24.0], [100.0, 50.0, 0.0])

    def test_rejects_percentages_rising_as_voltage_falls(self):
        with self.assertRaises(ValueError):
            validate_soc_curve([26.4, 24.0, 20.0], [100.0, 10.0, 50.0])

    def test_rejects_non_finite_entries(self):
        with self.assertRaises(ValueError):
            validate_soc_curve([26.4, float("inf")], [100.0, 0.0])

    def test_rejects_empty_curve(self):
        with self.assertRaises(ValueError):
            validate_soc_curve([], [])


class TestMedianVoltageFilter(unittest.TestCase):
    def test_reports_from_the_very_first_sample(self):
        f = MedianVoltageFilter(30)
        self.assertEqual(f.update(26.0), 26.0)
        self.assertEqual(f.count, 1)

    def test_rejects_a_single_outlier(self):
        f = MedianVoltageFilter(5)
        for _ in range(4):
            f.update(26.0)
        self.assertEqual(f.update(10.0), 26.0)

    def test_window_bounds_the_history(self):
        f = MedianVoltageFilter(3)
        for value in (1.0, 2.0, 3.0, 100.0, 100.0, 100.0):
            f.update(value)
        self.assertEqual(f.count, 3)
        self.assertEqual(f.window, 3)
        self.assertEqual(f.update(100.0), 100.0)

    def test_even_window_averages_the_middle_pair(self):
        f = MedianVoltageFilter(4)
        for value in (1.0, 2.0, 3.0, 4.0):
            f.update(value)
        self.assertAlmostEqual(f.update(4.0), 3.5)

    def test_tracks_a_steady_decline(self):
        f = MedianVoltageFilter(30)
        last = None
        for step in range(200):
            last = f.update(26.4 - step * 0.01)
        self.assertLess(last, 26.4)
        self.assertGreater(last, 24.3)

    def test_window_of_one_is_passthrough(self):
        f = MedianVoltageFilter(1)
        self.assertEqual(f.update(26.0), 26.0)
        self.assertEqual(f.update(21.0), 21.0)

    def test_rejects_invalid_window(self):
        with self.assertRaises(ValueError):
            MedianVoltageFilter(0)

    def test_rejects_non_finite_sample(self):
        with self.assertRaises(ValueError):
            MedianVoltageFilter(3).update(float("nan"))

    def test_reset_clears_history(self):
        f = MedianVoltageFilter(3)
        f.update(26.0)
        f.reset()
        self.assertEqual(f.count, 0)
        self.assertEqual(f.update(21.0), 21.0)


class TestSmoothingIsIsolatedFromShutdown(unittest.TestCase):
    """The rule the node depends on: safety reads the instantaneous voltage.

    These mirror battery_modbus_node._read_done, which calls
    _monitor.update(voltage) with the raw calibrated sample and only then
    feeds the same sample to the median filter.
    """

    def test_filtered_value_still_looks_healthy_when_pack_is_critical(self):
        f = MedianVoltageFilter(30)
        for _ in range(30):
            f.update(26.0)
        instant = 19.5
        self.assertGreater(f.update(instant), 25.0)
        self.assertLess(instant, 20.0)

    def test_monitor_fed_instantaneous_voltage_shuts_down_on_time(self):
        monitor = LowVoltageMonitor(
            low_voltage=22.0, critical_voltage=20.0, critical_hold_sec=5.0
        )
        soc_filter = MedianVoltageFilter(30)
        for _ in range(30):
            monitor.update(26.0, 0.0)
            soc_filter.update(26.0)

        status = None
        smoothed = None
        for second in range(1, 8):
            # Exactly the node's ordering: monitor first, on the raw sample.
            status = monitor.update(19.5, 30.0 + second)
            smoothed = soc_filter.update(19.5)
            if status.shutdown_due:
                break

        self.assertIsNotNone(status)
        self.assertEqual(status.state, CRITICAL)
        self.assertTrue(status.shutdown_due)
        # The display voltage is still far above critical at that moment,
        # which is exactly why it must not drive the decision.
        self.assertGreater(smoothed, 20.0)
        self.assertGreater(
            state_of_charge_percent(smoothed), 0.0
        )

    def test_monitor_fed_filtered_voltage_would_miss_the_collapse(self):
        """Demonstrates the bug the separation avoids."""
        monitor = LowVoltageMonitor(
            low_voltage=22.0, critical_voltage=20.0, critical_hold_sec=5.0
        )
        soc_filter = MedianVoltageFilter(30)
        for _ in range(30):
            soc_filter.update(26.0)

        status = None
        for second in range(1, 8):
            smoothed = soc_filter.update(19.5)
            status = monitor.update(smoothed, 30.0 + second)

        self.assertIsNotNone(status)
        self.assertFalse(status.shutdown_due)
        self.assertNotEqual(status.state, CRITICAL)


if __name__ == "__main__":
    unittest.main()
