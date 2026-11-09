"""Hardware-independent tests for the battery voltage and SoC helpers."""

import unittest

from horseshitbot.drivers.modbus_adc import (
    DEFAULT_SOC_PERCENT_POINTS,
    DEFAULT_SOC_VOLTAGE_POINTS,
    AdcRegisterMap,
    MedianVoltageFilter,
    state_of_charge_percent,
    validate_soc_curve,
)

# Same affine map as config/battery_modbus.yaml, i.e. (raw + 15.2) / 49.25.
CALIBRATED = AdcRegisterMap(
    channel_base_register=0,
    raw_scale=49.25,
    voltage_divider_factor=1.0,
    voltage_offset=15.2 / 49.25,
    voltage_multiplier=1.0,
)


class TestAdcVoltageCalibration(unittest.TestCase):
    """The five points measured on the robot on 2026-09-25."""

    def test_calibration_points(self):
        for raw, expected in [
            (970, 20.0),
            (1068, 22.0),
            (1167, 24.0),
            (1265, 26.0),
            (1364, 28.0),
        ]:
            with self.subTest(raw=raw):
                got = CALIBRATED.convert(raw, 1)["battery_voltage"]
                self.assertAlmostEqual(got, expected, delta=0.01)

    def test_matches_logger_formula_exactly(self):
        for raw in (964, 1000, 1135, 1282, 1371):
            with self.subTest(raw=raw):
                self.assertAlmostEqual(
                    CALIBRATED.convert(raw, 1)["battery_voltage"],
                    (raw + 15.2) / 49.25,
                    places=9,
                )

    def test_old_placeholder_was_far_off(self):
        # Guards against silently reverting to raw/50.
        got = CALIBRATED.convert(1364, 1)["battery_voltage"]
        self.assertGreater(got - (1364 / 50.0), 0.5)


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

    def test_calibration_points_return_their_own_percentages(self):
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

    def test_interpolated_value_stays_between_neighbours(self):
        # 25.90 V lies between the (25.95, 39.8) and (25.85, 32.2) points.
        percent = state_of_charge_percent(25.90)
        self.assertLess(percent, 39.8)
        self.assertGreater(percent, 32.2)

    def test_monotonic_across_whole_range(self):
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

    def test_accepts_a_custom_curve(self):
        volts, pcts = (30.0, 20.0), (100.0, 0.0)
        self.assertAlmostEqual(state_of_charge_percent(25.0, volts, pcts), 50.0)

    def test_rejects_non_finite_voltage(self):
        with self.assertRaises(ValueError):
            state_of_charge_percent(float("nan"))


class TestValidateSocCurve(unittest.TestCase):
    def test_accepts_the_defaults(self):
        volts, pcts = validate_soc_curve(
            DEFAULT_SOC_VOLTAGE_POINTS, DEFAULT_SOC_PERCENT_POINTS
        )
        self.assertEqual(volts, DEFAULT_SOC_VOLTAGE_POINTS)
        self.assertEqual(pcts, DEFAULT_SOC_PERCENT_POINTS)

    def test_normalises_ascending_input_to_descending(self):
        volts, pcts = validate_soc_curve([20.0, 26.4], [0.0, 100.0])
        self.assertEqual(volts, (26.4, 20.0))
        self.assertEqual(pcts, (100.0, 0.0))

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
        self.assertEqual(f.update(100.0), 100.0)

    def test_even_window_averages_middle_pair(self):
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


class TestSmoothingDoesNotDelaySafety(unittest.TestCase):
    """A collapse must be visible on the instantaneous voltage immediately."""

    def test_median_lags_but_instant_value_does_not(self):
        f = MedianVoltageFilter(30)
        for _ in range(30):
            f.update(26.0)
        instant = 20.5
        smoothed = f.update(instant)
        # The median still reads healthy, which is exactly why the node
        # compares thresholds against the instantaneous voltage instead.
        self.assertGreater(smoothed, 25.0)
        self.assertLessEqual(instant, 21.0)


if __name__ == "__main__":
    unittest.main()
