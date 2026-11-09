"""Thread-safe, fail-closed emergency-stop release state."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import threading


class EmergencyStopPhase(str, Enum):
    CLEAR = "clear"
    ACTIVE = "active"
    RELEASING = "releasing"


@dataclass(frozen=True)
class EmergencyStopSnapshot:
    phase: EmergencyStopPhase
    release_token: int | None
    release_deadline: float | None
    error: str

    @property
    def inhibited(self) -> bool:
        return self.phase is not EmergencyStopPhase.CLEAR


class EmergencyStopState:
    """Track an e-stop without ever clearing it speculatively.

    A release token identifies one hardware-release attempt. Late responses
    and responses superseded by a new e-stop cannot clear the current latch.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._phase = EmergencyStopPhase.CLEAR
        self._generation = 0
        self._release_token: int | None = None
        self._release_deadline: float | None = None
        self._error = ""

    def snapshot(self) -> EmergencyStopSnapshot:
        with self._lock:
            return EmergencyStopSnapshot(
                phase=self._phase,
                release_token=self._release_token,
                release_deadline=self._release_deadline,
                error=self._error,
            )

    @property
    def inhibited(self) -> bool:
        with self._lock:
            return self._phase is not EmergencyStopPhase.CLEAR

    def activate(self, reason: str = "") -> None:
        """Latch the stop and invalidate every in-flight release response."""
        with self._lock:
            self._generation += 1
            self._phase = EmergencyStopPhase.ACTIVE
            self._release_token = None
            self._release_deadline = None
            self._error = reason

    def begin_release(self, now: float, timeout: float) -> int | None:
        """Start one release, returning its token or ``None`` if not allowed."""
        now = float(now)
        timeout = float(timeout)
        if not math.isfinite(now):
            raise ValueError("now must be finite")
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("timeout must be positive and finite")
        with self._lock:
            if self._phase is EmergencyStopPhase.CLEAR:
                return None
            if self._phase is EmergencyStopPhase.RELEASING:
                return self._release_token
            self._generation += 1
            self._release_token = self._generation
            self._release_deadline = now + timeout
            self._phase = EmergencyStopPhase.RELEASING
            self._error = ""
            return self._release_token

    def confirm_release(self, token: int) -> bool:
        """Clear only when ``token`` is the current releasing attempt."""
        with self._lock:
            if (
                self._phase is not EmergencyStopPhase.RELEASING
                or self._release_token != int(token)
            ):
                return False
            self._phase = EmergencyStopPhase.CLEAR
            self._release_token = None
            self._release_deadline = None
            self._error = ""
            return True

    def fail_release(self, token: int, reason: str) -> bool:
        """Return a current release attempt to the inhibited active state."""
        with self._lock:
            if (
                self._phase is not EmergencyStopPhase.RELEASING
                or self._release_token != int(token)
            ):
                return False
            self._generation += 1
            self._phase = EmergencyStopPhase.ACTIVE
            self._release_token = None
            self._release_deadline = None
            self._error = str(reason)
            return True

    def expire(self, now: float) -> bool:
        """Fail a release whose acknowledgement deadline has elapsed."""
        now = float(now)
        with self._lock:
            if (
                self._phase is not EmergencyStopPhase.RELEASING
                or self._release_deadline is None
                or now < self._release_deadline
            ):
                return False
            self._generation += 1
            self._phase = EmergencyStopPhase.ACTIVE
            self._release_token = None
            self._release_deadline = None
            self._error = "hardware release acknowledgement timed out"
            return True

    def allows_velocity(self, left_rpm: float, right_rpm: float) -> bool:
        """Allow any command when clear, but only zero while inhibited."""
        with self._lock:
            return (
                self._phase is EmergencyStopPhase.CLEAR
                or (float(left_rpm) == 0.0 and float(right_rpm) == 0.0)
            )


def resolve_trigger_future(future: object) -> tuple[bool, str]:
    """Validate a mocked or rclpy Trigger future without assuming success."""
    try:
        result = future.result()
    except Exception as exc:
        return False, str(exc)
    success = getattr(result, "success", None)
    if not isinstance(success, bool):
        return False, "hardware release response has no valid success field"
    message = str(getattr(result, "message", "") or "")
    return success, message or (
        "hardware release acknowledged" if success else "hardware release failed"
    )
