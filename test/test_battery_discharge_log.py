"""Hardware-independent tests for the shared-bus battery discharge logger."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch


SCRIPT = (
    Path(__file__).parents[1]
    / "test_scripts"
    / "battery_discharge_log.py"
)
SPEC = importlib.util.spec_from_file_location("battery_discharge_log", SCRIPT)
LOGGER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LOGGER)


class _Reader:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.calls = 0

    def read(self, _slave, _address):
        self.calls += 1
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome


class TestBatteryDischargeLogger(unittest.TestCase):
    def test_calibration_reproduces_measured_points(self):
        for raw, expected in (
            (970, 20.0),
            (1068, 22.0),
            (1167, 24.0),
            (1265, 26.0),
            (1364, 28.0),
        ):
            with self.subTest(raw=raw):
                self.assertAlmostEqual(
                    LOGGER.raw_to_voltage(raw), expected, delta=0.01
                )

    def test_sample_retries_busy_bus_and_transient_error(self):
        reader = _Reader(
            [LOGGER.BusBusy("motor transaction"), RuntimeError("retry"), 1265]
        )
        args = SimpleNamespace(slave=33, retries=2, retry_delay=0.01)

        with patch.object(LOGGER.time, "sleep") as sleep:
            result = LOGGER.sample_once(reader, 0, args)

        self.assertEqual(result, 1265)
        self.assertEqual(reader.calls, 3)
        self.assertEqual(sleep.call_count, 2)

    def test_sample_raises_after_retry_budget_is_exhausted(self):
        reader = _Reader([LOGGER.BusBusy("busy"), RuntimeError("read failed")])
        args = SimpleNamespace(slave=33, retries=1, retry_delay=0.01)

        with (
            patch.object(LOGGER.time, "sleep"),
            self.assertRaisesRegex(RuntimeError, "read failed"),
        ):
            LOGGER.sample_once(reader, 0, args)

        self.assertEqual(reader.calls, 2)

    def test_log_flushes_and_fsyncs_each_line(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "battery.log"
            log = LOGGER.VoltageLog(path)
            try:
                with patch.object(LOGGER.os, "fsync") as fsync:
                    log.write("sample")
                fsync.assert_called_once()
            finally:
                log.close()

            self.assertEqual(path.read_text(encoding="utf-8"), "sample\n")

    def test_numeric_argument_validators_reject_unsafe_values(self):
        self.assertEqual(LOGGER.positive_float("0.5"), 0.5)
        self.assertEqual(LOGGER.non_negative_int("0"), 0)
        with self.assertRaises(LOGGER.argparse.ArgumentTypeError):
            LOGGER.positive_float("0")
        with self.assertRaises(LOGGER.argparse.ArgumentTypeError):
            LOGGER.non_negative_int("-1")


if __name__ == "__main__":
    unittest.main()
