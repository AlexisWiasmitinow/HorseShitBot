"""Bounded per-motor state for asynchronous MKS speed commands."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Callable


@dataclass(frozen=True)
class MksSpeedCommand:
    motor_id: int
    rpm: int
    accel: int
    invert_dir: bool

    @property
    def key(self) -> int:
        return self.motor_id


class MksCommandState:
    """Track one in-flight and one latest pending command per motor."""

    def __init__(self):
        self._lock = threading.Lock()
        self._last_successful: dict[int, MksSpeedCommand] = {}
        self._inflight: dict[int, MksSpeedCommand] = {}
        self._pending: dict[int, MksSpeedCommand] = {}

    def offer(self, command: MksSpeedCommand) -> MksSpeedCommand | None:
        """Queue a command and return it only when it may be dispatched now."""
        key = command.key
        with self._lock:
            if key in self._inflight:
                self._pending[key] = self._prefer_stop(
                    self._pending.get(key), command
                )
                return None

            pending = self._pending.pop(key, None)
            selected = self._select_pending(pending, command)
            if self._last_successful.get(key) == selected:
                return None

            self._inflight[key] = selected
            return selected

    def defer(self, command: MksSpeedCommand) -> None:
        """Keep a command pending while its ROS service is unavailable."""
        with self._lock:
            self._pending[command.key] = self._prefer_stop(
                self._pending.get(command.key), command
            )

    def complete(
        self, command: MksSpeedCommand, success: bool
    ) -> MksSpeedCommand | None:
        """Record completion and optionally return a priority command to send."""
        key = command.key
        with self._lock:
            if self._inflight.get(key) != command:
                return None
            self._inflight.pop(key, None)

            pending = self._pending.pop(key, None)
            if success:
                self._last_successful[key] = command
                if pending is None or pending == command:
                    return None
                self._inflight[key] = pending
                return pending

            # A failed command is not successful state. Preserve a pending STOP
            # ahead of it; otherwise retain the latest pending command.
            if command.rpm == 0:
                retry = command
            elif pending is not None:
                retry = pending
            else:
                retry = command
            if retry.rpm == 0 and command.rpm != 0:
                self._inflight[key] = retry
                return retry
            self._pending[key] = retry
            return None

    def clear_success(self) -> None:
        """Force future commands to be transmitted without losing in-flight state."""
        with self._lock:
            self._last_successful.clear()

    def discard(self, command: MksSpeedCommand) -> MksSpeedCommand | None:
        """Drop a stale command without recording success or retrying it."""
        key = command.key
        with self._lock:
            if self._inflight.get(key) != command:
                return None
            self._inflight.pop(key, None)
            pending = self._pending.pop(key, None)
            if pending is None or self._last_successful.get(key) == pending:
                return None
            self._inflight[key] = pending
            return pending

    def is_stopped(self, motor_ids: list[int]) -> bool:
        with self._lock:
            return all(
                self._last_successful.get(motor_id) is not None
                and self._last_successful[motor_id].rpm == 0
                and motor_id not in self._inflight
                and motor_id not in self._pending
                for motor_id in motor_ids
            )

    def last_success(self, motor_id: int) -> int | None:
        with self._lock:
            command = self._last_successful.get(motor_id)
            return command.rpm if command is not None else None

    def queue_depth(self, motor_id: int) -> tuple[int, int]:
        """Return bounded (in-flight, pending) slot counts for tests/diagnostics."""
        with self._lock:
            return (
                int(motor_id in self._inflight),
                int(motor_id in self._pending),
            )

    @staticmethod
    def _prefer_stop(
        existing: MksSpeedCommand | None, new: MksSpeedCommand
    ) -> MksSpeedCommand:
        if existing is not None and existing.rpm == 0 and new.rpm != 0:
            return existing
        return new

    @staticmethod
    def _select_pending(
        pending: MksSpeedCommand | None, current: MksSpeedCommand
    ) -> MksSpeedCommand:
        if pending is not None and pending.rpm == 0:
            return pending
        return current


class MksCommandDispatcher:
    """Connect bounded command state to an asynchronous service client."""

    def __init__(
        self,
        submit: Callable[[MksSpeedCommand, int], object],
        service_is_ready: Callable[[], bool],
        on_failure: Callable[[MksSpeedCommand, str], None],
        request_timeout_sec: float = 1.5,
        monotonic: Callable[[], float] = time.monotonic,
        cancel_request: Callable[[object], None] | None = None,
        command_ids: MksCommandIdGenerator | None = None,
    ):
        self._submit = submit
        self._service_is_ready = service_is_ready
        self._on_failure = on_failure
        self._request_timeout_sec = max(0.001, float(request_timeout_sec))
        self._monotonic = monotonic
        self._cancel_request = cancel_request
        self._command_ids = command_ids or MksCommandIdGenerator()
        self._state = MksCommandState()
        self._active_lock = threading.Lock()
        self._active: dict[int, _ActiveRequest] = {}

    def offer(self, command: MksSpeedCommand) -> bool:
        self.expire_timeouts()
        if not self._service_is_ready():
            self._state.defer(command)
            self._on_failure(command, "service unavailable")
            return False

        dispatch = self._state.offer(command)
        if dispatch is not None:
            self._dispatch(dispatch)
        return True

    def _dispatch(self, command: MksSpeedCommand) -> None:
        command_id = self._command_ids.next_id()
        try:
            future = self._submit(command, command_id)
        except Exception as exc:
            self._complete(command, False, str(exc))
            return

        active = _ActiveRequest(
            token=object(),
            command=command,
            command_id=command_id,
            deadline=self._monotonic() + self._request_timeout_sec,
            future=future,
        )
        with self._active_lock:
            self._active[command.key] = active
        try:
            future.add_done_callback(
                lambda completed, sent=active: self._on_done(sent, completed)
            )
        except Exception as exc:
            if self._claim(active):
                self._cancel(active.future)
                self._complete(active.command, False, str(exc))

    def _on_done(self, active: _ActiveRequest, future) -> None:
        if not self._claim(active):
            return
        try:
            response = future.result()
            value = getattr(response, "success", None)
            stale = getattr(response, "stale", None)
            if not isinstance(value, bool):
                raise ValueError("service response has no valid success field")
            if not isinstance(stale, bool):
                raise ValueError("service response has no valid stale field")
            if stale and value:
                raise ValueError("service response cannot be successful and stale")
            if stale:
                self._discard(active.command)
                return
            success = value
            reason = "" if success else "service reported failure"
        except Exception as exc:
            success = False
            reason = str(exc)
        self._complete(active.command, success, reason)

    def _discard(self, command: MksSpeedCommand) -> None:
        next_command = self._state.discard(command)
        if next_command is not None:
            if self._service_is_ready():
                self._dispatch(next_command)
            else:
                self._state.complete(next_command, False)
                self._on_failure(next_command, "service unavailable")

    def _claim(self, active: _ActiveRequest) -> bool:
        with self._active_lock:
            current = self._active.get(active.command.key)
            if current is not active or current.token is not active.token:
                return False
            self._active.pop(active.command.key, None)
            return True

    def _complete(
        self, command: MksSpeedCommand, success: bool, reason: str
    ) -> None:
        next_command = self._state.complete(command, success)
        if not success:
            self._on_failure(command, reason)
        if next_command is not None:
            if self._service_is_ready():
                self._dispatch(next_command)
            else:
                self._state.complete(next_command, False)
                self._on_failure(next_command, "service unavailable")

    def expire_timeouts(self, now: float | None = None) -> int:
        """Expire active requests and release their per-motor state slots."""
        current_time = self._monotonic() if now is None else float(now)
        with self._active_lock:
            expired = [
                active
                for active in self._active.values()
                if current_time >= active.deadline
            ]
            for active in expired:
                if self._active.get(active.command.key) is active:
                    self._active.pop(active.command.key, None)

        for active in expired:
            self._cancel(active.future)
            self._complete(active.command, False, "service request timed out")
        return len(expired)

    def _cancel(self, future: object) -> None:
        if self._cancel_request is not None:
            try:
                self._cancel_request(future)
            except Exception:
                pass

    def clear_success(self) -> None:
        self._state.clear_success()

    def is_stopped(self, motor_ids: list[int]) -> bool:
        return self._state.is_stopped(motor_ids)

    def last_success(self, motor_id: int) -> int | None:
        return self._state.last_success(motor_id)

    def queue_depth(self, motor_id: int) -> tuple[int, int]:
        return self._state.queue_depth(motor_id)


@dataclass
class _ActiveRequest:
    token: object
    command: MksSpeedCommand
    command_id: int
    deadline: float
    future: object


class MksCommandIdGenerator:
    """Generate IDs ordered across processes during one system boot."""

    def __init__(self, monotonic_ns: Callable[[], int] = time.monotonic_ns):
        self._monotonic_ns = monotonic_ns
        self._lock = threading.Lock()
        self._last_id = 0

    def next_id(self) -> int:
        with self._lock:
            command_id = max(int(self._monotonic_ns()), self._last_id + 1)
            self._last_id = command_id
            return command_id


class MksCommandHighWater:
    """Admission-time stale-command filter keyed by physical motor ID."""

    def __init__(self):
        self._lock = threading.Lock()
        self._highest: dict[int, int] = {}

    def admit(self, motor_id: int, command_id: int) -> bool:
        motor_id = int(motor_id)
        command_id = int(command_id)
        with self._lock:
            highest = self._highest.get(motor_id, 0)
            if command_id <= highest:
                return False
            self._highest[motor_id] = command_id
            return True

    def highest(self, motor_id: int) -> int:
        with self._lock:
            return self._highest.get(int(motor_id), 0)


def apply_velocity_watchdog(
    desired_left: float,
    desired_right: float,
    command_age: float,
    timeout: float,
    inhibited: bool = False,
) -> tuple[float, float, bool]:
    """Return stored targets after applying the cmd_vel watchdog."""
    expired = not inhibited and command_age > timeout
    if expired:
        return 0.0, 0.0, True
    return desired_left, desired_right, False
