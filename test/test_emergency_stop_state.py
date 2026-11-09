"""Hardware-free tests for fail-closed MKS emergency-stop release."""

from pathlib import Path
import sys
import unittest
from unittest.mock import patch


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "horseshitbot"
sys.path.insert(0, str(PACKAGE_ROOT))

from horseshitbot.drivers.emergency_stop_state import (  # noqa: E402
    EmergencyStopPhase,
    EmergencyStopState,
    resolve_trigger_future,
)
from horseshitbot.drivers.mks_bus import (  # noqa: E402
    MksBus,
    REG_EMERGENCY_STOP,
)


class EmergencyStopStateTest(unittest.TestCase):
    def setUp(self):
        self.state = EmergencyStopState()
        self.state.activate()

    def test_success_requires_matching_hardware_acknowledgement(self):
        token = self.state.begin_release(now=10.0, timeout=1.0)
        self.assertEqual(self.state.snapshot().phase, EmergencyStopPhase.RELEASING)
        self.assertTrue(self.state.inhibited)
        self.assertTrue(self.state.confirm_release(token))
        self.assertEqual(self.state.snapshot().phase, EmergencyStopPhase.CLEAR)

    def test_failed_release_remains_inhibited(self):
        token = self.state.begin_release(now=10.0, timeout=1.0)
        self.assertTrue(self.state.fail_release(token, "motor 2 did not acknowledge"))
        snapshot = self.state.snapshot()
        self.assertEqual(snapshot.phase, EmergencyStopPhase.ACTIVE)
        self.assertTrue(snapshot.inhibited)
        self.assertIn("motor 2", snapshot.error)

    def test_timeout_remains_inhibited_and_late_success_is_ignored(self):
        token = self.state.begin_release(now=10.0, timeout=0.5)
        self.assertFalse(self.state.expire(10.49))
        self.assertTrue(self.state.expire(10.5))
        self.assertFalse(self.state.confirm_release(token))
        self.assertEqual(self.state.snapshot().phase, EmergencyStopPhase.ACTIVE)

    def test_concurrent_estop_invalidates_release(self):
        token = self.state.begin_release(now=10.0, timeout=1.0)
        self.state.activate("new emergency stop")
        self.assertFalse(self.state.confirm_release(token))
        self.assertTrue(self.state.inhibited)

    def test_commands_during_release_are_not_replayed(self):
        token = self.state.begin_release(now=10.0, timeout=1.0)
        self.assertFalse(self.state.allows_velocity(500.0, 500.0))
        self.assertTrue(self.state.allows_velocity(0.0, 0.0))
        self.assertTrue(self.state.confirm_release(token))
        # A new post-release command may be accepted; the blocked command was
        # never stored by the wheel node and therefore cannot replay itself.
        self.assertTrue(self.state.allows_velocity(200.0, -200.0))

    def test_duplicate_release_request_reuses_current_attempt(self):
        first = self.state.begin_release(now=10.0, timeout=1.0)
        second = self.state.begin_release(now=10.2, timeout=1.0)
        self.assertEqual(first, second)


class FakeTriggerResponse:
    def __init__(self, success, message=""):
        self.success = success
        self.message = message


class FakeTriggerFuture:
    def __init__(self, response=None, error=None):
        self.response = response
        self.error = error

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response


class MockedMksReleaseResponseTest(unittest.TestCase):
    def test_successful_mocked_service_response_is_confirmed(self):
        success, message = resolve_trigger_future(
            FakeTriggerFuture(FakeTriggerResponse(True, "both motors released"))
        )
        self.assertTrue(success)
        self.assertEqual(message, "both motors released")

    def test_failed_mocked_service_response_is_not_confirmed(self):
        success, message = resolve_trigger_future(
            FakeTriggerFuture(FakeTriggerResponse(False, "motor 2 timeout"))
        )
        self.assertFalse(success)
        self.assertEqual(message, "motor 2 timeout")

    def test_mocked_service_exception_is_not_confirmed(self):
        success, message = resolve_trigger_future(
            FakeTriggerFuture(error=TimeoutError("release timed out"))
        )
        self.assertFalse(success)
        self.assertIn("timed out", message)

    def test_malformed_mocked_response_is_not_confirmed(self):
        success, message = resolve_trigger_future(FakeTriggerFuture(object()))
        self.assertFalse(success)
        self.assertIn("valid success", message)


class MksReleaseSequenceTest(unittest.TestCase):
    def make_bus(self):
        bus = object.__new__(MksBus)
        calls = []
        bus.set_enable = lambda motor_id, enabled: calls.append(
            ("enable", motor_id, enabled)
        )
        bus.write_reg = lambda motor_id, register, value: calls.append(
            ("write", motor_id, register, value)
        )
        bus.set_speed_signed = lambda motor_id, rpm, acc=10: calls.append(
            ("speed", motor_id, rpm, acc)
        )
        return bus, calls

    @patch("horseshitbot.drivers.mks_bus.time.sleep", return_value=None)
    def test_release_writes_zero_before_reenable(self, _sleep):
        bus, calls = self.make_bus()
        bus.release_emergency_stop(2)
        self.assertEqual(
            calls,
            [
                ("enable", 2, False),
                ("write", 2, REG_EMERGENCY_STOP, 0),
                ("speed", 2, 0.0, 0),
                ("enable", 2, True),
            ],
        )

    @patch("horseshitbot.drivers.mks_bus.time.sleep", return_value=None)
    def test_release_failure_does_not_reenable_motor(self, _sleep):
        bus, calls = self.make_bus()

        def fail_release(motor_id, register, value):
            calls.append(("write", motor_id, register, value))
            raise RuntimeError("no acknowledgement")

        bus.write_reg = fail_release
        with self.assertRaisesRegex(RuntimeError, "no acknowledgement"):
            bus.release_emergency_stop(2)
        self.assertEqual(
            calls,
            [
                ("enable", 2, False),
                ("write", 2, REG_EMERGENCY_STOP, 0),
            ],
        )


if __name__ == "__main__":
    unittest.main()
