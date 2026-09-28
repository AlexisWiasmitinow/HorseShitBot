import unittest
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
TOOL_ROOT = REPO_ROOT / "test_scripts"
sys.path.insert(0, str(TOOL_ROOT))

from modbus_adc_tool import run_stress_loop  # noqa: E402


class FakeClock:
    """Monotonic clock that only advances when the loop sleeps or reads.

    Counts whole milliseconds so repeated 0.1 s steps cannot drift below an
    exact deadline the way accumulated floats do.
    """

    def __init__(self, step_ms=100):
        self.elapsed_ms = 0
        self.step_ms = step_ms

    def now(self):
        return self.elapsed_ms / 1000.0

    def tick(self):
        self.elapsed_ms += self.step_ms

    def sleep(self, seconds):
        self.elapsed_ms += round(seconds * 1000)


class StressLoopTest(unittest.TestCase):
    def test_counts_successful_reads(self):
        clock = FakeClock(step_ms=100)

        def read_once():
            clock.tick()

        result = run_stress_loop(
            read_once, duration_sec=1.0, now=clock.now, sleep=clock.sleep
        )
        self.assertEqual(result.attempts, 10)
        self.assertEqual(result.successes, 10)
        self.assertEqual(result.failures, 0)
        self.assertFalse(result.interrupted)

    def test_counts_failures_without_aborting(self):
        clock = FakeClock(step_ms=100)
        calls = []

        def read_once():
            clock.tick()
            calls.append(len(calls))
            if len(calls) % 2 == 0:
                raise RuntimeError("modbus timeout")

        result = run_stress_loop(
            read_once, duration_sec=1.0, now=clock.now, sleep=clock.sleep
        )
        self.assertEqual(result.attempts, 10)
        self.assertEqual(result.successes, 5)
        self.assertEqual(result.failures, 5)

    def test_attempts_always_equal_successes_plus_failures(self):
        clock = FakeClock(step_ms=100)

        def read_once():
            clock.tick()
            raise ValueError("crc error")

        result = run_stress_loop(
            read_once, duration_sec=0.5, now=clock.now, sleep=clock.sleep
        )
        self.assertEqual(result.attempts, result.successes + result.failures)
        self.assertEqual(result.successes, 0)

    def test_delay_reduces_read_count(self):
        clock = FakeClock(step_ms=100)

        def read_once():
            clock.tick()

        result = run_stress_loop(
            read_once,
            duration_sec=1.0,
            delay_sec=0.1,
            now=clock.now,
            sleep=clock.sleep,
        )
        self.assertEqual(result.attempts, 5)

    def test_keyboard_interrupt_during_read_preserves_counts(self):
        clock = FakeClock(step_ms=100)
        calls = []

        def read_once():
            clock.tick()
            calls.append(1)
            if len(calls) == 4:
                raise KeyboardInterrupt
        result = run_stress_loop(
            read_once, duration_sec=10.0, now=clock.now, sleep=clock.sleep
        )
        self.assertTrue(result.interrupted)
        self.assertEqual(result.attempts, 3)
        self.assertEqual(result.successes, 3)
        self.assertEqual(result.failures, 0)

    def test_keyboard_interrupt_during_sleep_preserves_counts(self):
        clock = FakeClock(step_ms=100)

        def read_once():
            clock.tick()

        def sleep(seconds):
            raise KeyboardInterrupt

        result = run_stress_loop(
            read_once,
            duration_sec=10.0,
            delay_sec=0.1,
            now=clock.now,
            sleep=sleep,
        )
        self.assertTrue(result.interrupted)
        self.assertEqual(result.attempts, 1)
        self.assertEqual(result.successes, 1)

    def test_reads_per_sec(self):
        clock = FakeClock(step_ms=100)

        def read_once():
            clock.tick()

        result = run_stress_loop(
            read_once, duration_sec=1.0, now=clock.now, sleep=clock.sleep
        )
        self.assertAlmostEqual(result.reads_per_sec, 10.0, places=6)

    def test_reads_per_sec_is_zero_without_elapsed_time(self):
        result = run_stress_loop(
            lambda: None, duration_sec=0.0, now=lambda: 0.0, sleep=lambda _: None
        )
        self.assertEqual(result.attempts, 0)
        self.assertEqual(result.reads_per_sec, 0.0)


if __name__ == "__main__":
    unittest.main()
