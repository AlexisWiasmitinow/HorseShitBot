import sys
import unittest
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "horseshitbot"
sys.path.insert(0, str(PACKAGE_ROOT))

from horseshitbot.drivers.mks_command_state import (  # noqa: E402
    MksCommandHighWater,
    MksCommandIdGenerator,
    MksCommandDispatcher,
    MksCommandState,
    MksSpeedCommand,
    apply_velocity_watchdog,
)


def command(rpm: int, motor_id: int = 1) -> MksSpeedCommand:
    return MksSpeedCommand(
        motor_id=motor_id,
        rpm=rpm,
        accel=0 if rpm == 0 else 255,
        invert_dir=False,
    )


class Response:
    def __init__(self, success, stale=False):
        self.success = success
        self.stale = stale


class MissingSuccessResponse:
    pass


class FakeFuture:
    def __init__(self, response=None, error: Exception | None = None):
        self._response = response
        self._error = error
        self._callback = None
        self.cancelled = False

    def add_done_callback(self, callback):
        self._callback = callback

    def result(self):
        if self._error is not None:
            raise self._error
        return self._response

    def resolve(self):
        self._callback(self)

    def cancel(self):
        self.cancelled = True


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class Submitter:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.commands = []
        self.command_ids = []

    def __call__(self, sent, command_id):
        self.commands.append(sent)
        self.command_ids.append(command_id)
        outcome = self.outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome


def dispatcher(
    submitter: Submitter,
    clock=None,
    timeout=0.5,
    cancel_request=None,
    command_ids=None,
):
    failures = []
    instance = MksCommandDispatcher(
        submit=submitter,
        service_is_ready=lambda: True,
        on_failure=lambda sent, reason: failures.append((sent, reason)),
        request_timeout_sec=timeout,
        monotonic=clock or __import__("time").monotonic,
        cancel_request=cancel_request,
        command_ids=command_ids,
    )
    return instance, failures


class MksCommandStateTest(unittest.TestCase):
    def test_command_ids_are_strictly_increasing(self):
        values = iter((100, 100, 99, 200))
        generator = MksCommandIdGenerator(monotonic_ns=lambda: next(values))
        self.assertEqual(
            [generator.next_id() for _ in range(4)],
            [100, 101, 102, 200],
        )

    def test_restart_style_command_ids_continue_with_system_monotonic_clock(self):
        first = MksCommandIdGenerator(monotonic_ns=lambda: 1_000)
        second = MksCommandIdGenerator(monotonic_ns=lambda: 2_000)
        self.assertGreater(second.next_id(), first.next_id())

    def test_command_id_is_generated_only_when_command_dispatches(self):
        values = iter((100, 200))
        ids = MksCommandIdGenerator(monotonic_ns=lambda: next(values))
        future = FakeFuture(Response(True))
        submitter = Submitter([future])
        manager, _ = dispatcher(submitter, command_ids=ids)

        manager.offer(command(500))
        manager.offer(command(500))
        future.resolve()
        manager.offer(command(500))

        self.assertEqual(submitter.command_ids, [100])

    def test_newer_command_id_is_accepted(self):
        high_water = MksCommandHighWater()
        self.assertTrue(high_water.admit(1, 100))
        self.assertTrue(high_water.admit(1, 200))
        self.assertEqual(high_water.highest(1), 200)

    def test_lower_and_equal_command_ids_are_rejected_without_write(self):
        high_water = MksCommandHighWater()
        writes = []

        def admit_and_write(command_id):
            if high_water.admit(1, command_id):
                writes.append(command_id)

        admit_and_write(200)
        admit_and_write(100)
        admit_and_write(200)
        self.assertEqual(writes, [200])

    def test_newer_zero_blocks_delayed_old_nonzero(self):
        high_water = MksCommandHighWater()
        writes = []

        for command_id, rpm in ((100, 500), (200, 0), (100, 500)):
            if high_water.admit(1, command_id):
                writes.append(rpm)

        self.assertEqual(writes, [500, 0])
        self.assertEqual(high_water.highest(1), 200)

    def test_high_water_advances_before_failed_modbus_attempt(self):
        high_water = MksCommandHighWater()
        self.assertTrue(high_water.admit(1, 200))
        # The admitted command's physical write fails, but its ID remains
        # authoritative so an older request can never execute afterward.
        self.assertFalse(high_water.admit(1, 100))
        self.assertEqual(high_water.highest(1), 200)

    def test_successful_command_is_deduplicated(self):
        state = MksCommandState()
        sent = state.offer(command(500))
        self.assertEqual(sent, command(500))
        self.assertIsNone(state.complete(sent, True))
        self.assertEqual(state.last_success(1), 500)
        self.assertIsNone(state.offer(command(500)))

    def test_failed_zero_remains_pending_and_is_retried(self):
        state = MksCommandState()
        moving = state.offer(command(500))
        state.complete(moving, True)

        stopping = state.offer(command(0))
        self.assertIsNone(state.offer(command(0)))
        self.assertIsNone(state.complete(stopping, False))
        self.assertEqual(state.last_success(1), 500)

        retry = state.offer(command(0))
        self.assertEqual(retry, command(0))
        self.assertIsNone(state.complete(retry, True))
        self.assertEqual(state.last_success(1), 0)
        self.assertIsNone(state.offer(command(0)))

    def test_failed_nonzero_does_not_poison_success_cache(self):
        state = MksCommandState()
        sent = state.offer(command(700))
        self.assertIsNone(state.offer(command(700)))
        self.assertIsNone(state.complete(sent, False))
        self.assertIsNone(state.last_success(1))
        self.assertEqual(state.offer(command(700)), command(700))

    def test_pending_zero_precedes_newer_nonzero(self):
        state = MksCommandState()
        moving = state.offer(command(400))
        self.assertIsNone(state.offer(command(0)))
        self.assertIsNone(state.offer(command(800)))

        stopping = state.complete(moving, True)
        self.assertEqual(stopping, command(0))
        self.assertIsNone(state.offer(command(800)))

        resumed = state.complete(stopping, True)
        self.assertEqual(resumed, command(800))

    def test_left_and_right_state_is_independent(self):
        state = MksCommandState()
        left = command(300, motor_id=1)
        right = command(450, motor_id=2)
        self.assertEqual(state.offer(left), left)
        self.assertEqual(state.offer(right), right)
        state.complete(left, True)
        self.assertEqual(state.last_success(1), 300)
        self.assertIsNone(state.last_success(2))

    def test_deferred_zero_survives_new_nonzero_command(self):
        state = MksCommandState()
        state.defer(command(0))
        self.assertEqual(state.offer(command(600)), command(0))

    def test_same_motor_with_different_inversion_has_one_inflight_slot(self):
        first = FakeFuture(Response(True))
        second = FakeFuture(Response(True))
        submitter = Submitter([first, second])
        manager, _ = dispatcher(submitter)
        normal = command(500)
        inverted = MksSpeedCommand(1, 500, 255, True)
        manager.offer(normal)
        manager.offer(inverted)
        self.assertEqual(len(submitter.commands), 1)
        first.resolve()
        self.assertEqual(len(submitter.commands), 2)
        self.assertEqual(submitter.commands, [normal, inverted])

    def test_last_success_changes_only_after_future_success(self):
        future = FakeFuture(Response(True))
        submitter = Submitter([future])
        manager, _ = dispatcher(submitter)

        manager.offer(command(500))
        self.assertIsNone(manager.last_success(1))
        future.resolve()
        self.assertEqual(manager.last_success(1), 500)
        manager.offer(command(500))
        self.assertEqual(len(submitter.commands), 1)

    def test_failed_zero_service_response_retries_zero(self):
        moving = FakeFuture(Response(True))
        failed_stop = FakeFuture(Response(False))
        successful_stop = FakeFuture(Response(True))
        submitter = Submitter([moving, failed_stop, successful_stop])
        manager, _ = dispatcher(submitter)

        manager.offer(command(500))
        moving.resolve()
        manager.offer(command(0))
        manager.offer(command(0))
        failed_stop.resolve()
        self.assertEqual(manager.last_success(1), 500)
        manager.offer(command(0))
        self.assertEqual(submitter.commands[-1], command(0))
        successful_stop.resolve()
        self.assertEqual(manager.last_success(1), 0)

    def test_failed_service_response_keeps_command_retryable(self):
        first = FakeFuture(Response(False))
        second = FakeFuture(Response(True))
        submitter = Submitter([first, second])
        manager, failures = dispatcher(submitter)

        manager.offer(command(500))
        first.resolve()
        self.assertIsNone(manager.last_success(1))
        manager.offer(command(500))
        self.assertEqual(len(submitter.commands), 2)
        self.assertEqual(len(failures), 1)

    def test_retry_is_dispatched_with_newer_command_id(self):
        ids = MksCommandIdGenerator(monotonic_ns=lambda: 100)
        first = FakeFuture(Response(False))
        second = FakeFuture(Response(True))
        submitter = Submitter([first, second])
        manager, _ = dispatcher(submitter, command_ids=ids)

        manager.offer(command(500))
        first.resolve()
        manager.offer(command(500))

        self.assertEqual(len(submitter.command_ids), 2)
        self.assertGreater(submitter.command_ids[1], submitter.command_ids[0])

    def test_stale_response_does_not_update_or_retry(self):
        stale = FakeFuture(Response(False, stale=True))
        submitter = Submitter([stale])
        manager, failures = dispatcher(submitter)

        manager.offer(command(500))
        stale.resolve()

        self.assertIsNone(manager.last_success(1))
        self.assertEqual(manager.queue_depth(1), (0, 0))
        self.assertEqual(submitter.commands, [command(500)])
        self.assertEqual(failures, [])

    def test_future_exception_keeps_command_retryable(self):
        first = FakeFuture(error=RuntimeError("future failed"))
        second = FakeFuture(Response(True))
        submitter = Submitter([first, second])
        manager, failures = dispatcher(submitter)

        manager.offer(command(500))
        first.resolve()
        self.assertIsNone(manager.last_success(1))
        manager.offer(command(500))
        self.assertEqual(len(submitter.commands), 2)
        self.assertIn("future failed", failures[0][1])

    def test_invalid_service_response_keeps_command_retryable(self):
        for response in (None, MissingSuccessResponse(), Response("yes")):
            with self.subTest(response=response):
                first = FakeFuture(response)
                second = FakeFuture(Response(True))
                submitter = Submitter([first, second])
                manager, failures = dispatcher(submitter)

                manager.offer(command(500))
                first.resolve()
                self.assertIsNone(manager.last_success(1))
                manager.offer(command(500))
                self.assertEqual(len(submitter.commands), 2)
                self.assertIn("no valid success field", failures[0][1])

    def test_call_async_exception_releases_inflight_for_retry(self):
        second = FakeFuture(Response(True))
        submitter = Submitter([RuntimeError("submit failed"), second])
        manager, failures = dispatcher(submitter)

        manager.offer(command(500))
        self.assertIsNone(manager.last_success(1))
        manager.offer(command(500))
        self.assertEqual(len(submitter.commands), 2)
        second.resolve()
        self.assertEqual(manager.last_success(1), 500)
        self.assertIn("submit failed", failures[0][1])

    def test_nonzero_future_timeout_releases_slot(self):
        clock = FakeClock()
        future = FakeFuture(Response(True))
        submitter = Submitter([future])
        cancelled = []
        manager, failures = dispatcher(
            submitter,
            clock=clock,
            cancel_request=lambda item: (cancelled.append(item), item.cancel()),
        )

        manager.offer(command(500))
        self.assertEqual(manager.queue_depth(1), (1, 0))
        clock.advance(0.51)
        self.assertEqual(manager.expire_timeouts(), 1)

        self.assertEqual(manager.queue_depth(1), (0, 1))
        self.assertIsNone(manager.last_success(1))
        self.assertEqual(cancelled, [future])
        self.assertTrue(future.cancelled)
        self.assertIn("timed out", failures[0][1])

    def test_pending_zero_dispatches_after_nonzero_timeout(self):
        clock = FakeClock()
        old = FakeFuture(Response(True))
        stopping = FakeFuture(Response(True))
        submitter = Submitter([old, stopping])
        manager, _ = dispatcher(submitter, clock=clock)

        manager.offer(command(500))
        manager.offer(command(0))
        clock.advance(0.51)
        manager.expire_timeouts()

        self.assertEqual(submitter.commands, [command(500), command(0)])
        self.assertEqual(manager.queue_depth(1), (1, 0))

    def test_timed_out_zero_remains_retryable(self):
        clock = FakeClock()
        first = FakeFuture(Response(True))
        retry = FakeFuture(Response(True))
        submitter = Submitter([first, retry])
        manager, _ = dispatcher(submitter, clock=clock)

        manager.offer(command(0))
        clock.advance(0.51)
        manager.expire_timeouts()
        self.assertEqual(manager.queue_depth(1), (0, 1))

        manager.offer(command(0))
        self.assertEqual(submitter.commands, [command(0), command(0)])

    def test_late_success_from_timed_out_future_is_ignored(self):
        clock = FakeClock()
        old = FakeFuture(Response(True))
        submitter = Submitter([old])
        manager, failures = dispatcher(submitter, clock=clock)

        manager.offer(command(500))
        clock.advance(0.51)
        manager.expire_timeouts()
        old.resolve()

        self.assertIsNone(manager.last_success(1))
        self.assertEqual(len(failures), 1)
        self.assertEqual(manager.queue_depth(1), (0, 1))

    def test_late_failure_from_timed_out_future_is_ignored(self):
        clock = FakeClock()
        old = FakeFuture(Response(False))
        submitter = Submitter([old])
        manager, failures = dispatcher(submitter, clock=clock)

        manager.offer(command(500))
        clock.advance(0.51)
        manager.expire_timeouts()
        old.resolve()

        self.assertIsNone(manager.last_success(1))
        self.assertEqual(len(failures), 1)
        self.assertEqual(manager.queue_depth(1), (0, 1))

    def test_late_nonzero_cannot_overwrite_successful_zero(self):
        clock = FakeClock()
        old = FakeFuture(Response(True))
        stopping = FakeFuture(Response(True))
        submitter = Submitter([old, stopping])
        manager, _ = dispatcher(submitter, clock=clock)

        manager.offer(command(500))
        manager.offer(command(0))
        clock.advance(0.51)
        manager.expire_timeouts()
        stopping.resolve()
        self.assertEqual(manager.last_success(1), 0)

        old.resolve()
        self.assertEqual(manager.last_success(1), 0)
        self.assertEqual(manager.queue_depth(1), (0, 0))

    def test_queue_remains_one_inflight_and_one_pending(self):
        clock = FakeClock()
        old = FakeFuture(Response(True))
        submitter = Submitter([old])
        manager, _ = dispatcher(submitter, clock=clock)

        manager.offer(command(100))
        for rpm in (200, 300, 0, 400, 500):
            manager.offer(command(rpm))

        self.assertEqual(len(submitter.commands), 1)
        self.assertEqual(manager.queue_depth(1), (1, 1))

    def test_watchdog_expiry_returns_zero_stored_targets(self):
        left, right, expired = apply_velocity_watchdog(
            desired_left=500.0,
            desired_right=500.0,
            command_age=0.21,
            timeout=0.2,
        )
        self.assertTrue(expired)
        self.assertEqual((left, right), (0.0, 0.0))
        self.assertEqual((command(int(left)).rpm, command(int(right), 2).rpm), (0, 0))


if __name__ == "__main__":
    unittest.main()
