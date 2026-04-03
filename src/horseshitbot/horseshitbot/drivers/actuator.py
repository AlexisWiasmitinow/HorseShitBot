"""
Generic open/close actuator driver with stall-based referencing.

Used by lift_node, brush_node, and bin_door_node. Each actuator is a simple
linear motion (open/close, up/down, in/out) controlled by one or two MKS
stepper motors via the MKS bus node service interface.

Stall detection relies on the MKS servo's locked-rotor protection: when the
motor hits a mechanical end stop, the servo flags a locked-rotor condition
which we detect by polling.
"""

from __future__ import annotations

import enum
import logging
import time
import threading
from dataclasses import dataclass, field
from typing import Callable, Optional

_log = logging.getLogger(__name__)


class ActuatorState(enum.IntEnum):
    IDLE = 0
    REFERENCING = 1
    REFERENCED = 2
    MOVING_OPEN = 3
    MOVING_CLOSE = 4
    ERROR = 5


@dataclass
class ActuatorConfig:
    motor_ids: list[int] = field(default_factory=lambda: [4])
    dual_motor: bool = False
    invert_dir: bool = False

    open_speed_rpm: float = 300.0
    close_speed_rpm: float = 200.0
    open_accel: int = 3
    close_accel: int = 3

    ref_speed_rpm: float = 100.0
    ref_direction: str = "close"  # "open" or "close"

    stall_confirm_ms: int = 200
    watchdog_sec: float = 0.8


class Actuator:
    """
    Generic open/close actuator with stall-based homing.

    Parameters
    ----------
    cfg : ActuatorConfig
        Per-actuator settings.
    set_speed_fn : callable(motor_id, rpm, accel, invert_dir) -> bool
        Sends a speed command via the MKS bus node service.
    """

    def __init__(
        self,
        cfg: ActuatorConfig,
        set_speed_fn: Callable[[int, float, int, bool], bool],
    ):
        self.cfg = cfg
        self._set_speed = set_speed_fn

        self._state = ActuatorState.IDLE
        self._is_referenced = False
        self._direction = "stopped"
        self._speed_rpm = 0.0
        self._error_message = ""

        self._lock = threading.Lock()
        self._last_cmd_ts = 0.0
        self._stall_start: Optional[float] = None

    # ── public state ──────────────────────────────────────────────

    @property
    def state(self) -> ActuatorState:
        with self._lock:
            return self._state

    @property
    def is_referenced(self) -> bool:
        with self._lock:
            return self._is_referenced

    @property
    def direction(self) -> str:
        with self._lock:
            return self._direction

    @property
    def speed_rpm(self) -> float:
        with self._lock:
            return self._speed_rpm

    @property
    def error_message(self) -> str:
        with self._lock:
            return self._error_message

    def get_state_dict(self) -> dict:
        with self._lock:
            return {
                "state": int(self._state),
                "is_referenced": self._is_referenced,
                "direction": self._direction,
                "speed_rpm": self._speed_rpm,
                "error_message": self._error_message,
            }

    # ── commands ──────────────────────────────────────────────────

    def open(self, speed_override: float = 0.0) -> tuple[bool, str]:
        speed = speed_override if speed_override > 0 else self.cfg.open_speed_rpm
        return self._start_move("open", speed, self.cfg.open_accel)

    def close(self, speed_override: float = 0.0) -> tuple[bool, str]:
        speed = speed_override if speed_override > 0 else self.cfg.close_speed_rpm
        return self._start_move("close", speed, self.cfg.close_accel)

    def stop(self) -> tuple[bool, str]:
        with self._lock:
            self._send_stop()
            self._direction = "stopped"
            self._speed_rpm = 0.0
            if self._state in (
                ActuatorState.MOVING_OPEN,
                ActuatorState.MOVING_CLOSE,
                ActuatorState.REFERENCING,
            ):
                self._state = (
                    ActuatorState.REFERENCED
                    if self._is_referenced
                    else ActuatorState.IDLE
                )
            return True, "stopped"

    def reference(self) -> tuple[bool, str]:
        with self._lock:
            self._send_stop()
            self._state = ActuatorState.REFERENCING
            self._is_referenced = False
            self._stall_start = None

            direction = self.cfg.ref_direction
            speed = self.cfg.ref_speed_rpm
            accel = (
                self.cfg.close_accel if direction == "close" else self.cfg.open_accel
            )
            rpm = -speed if direction == "close" else speed
            self._direction = direction
            self._speed_rpm = speed

        self._send_rpm(rpm, accel)
        return True, f"referencing ({direction})"

    def clear_error(self) -> tuple[bool, str]:
        with self._lock:
            self._state = ActuatorState.IDLE
            self._error_message = ""
            return True, "error cleared"

    # ── periodic tick (called by the node's timer) ────────────────

    def tick(self, stall_detected: bool):
        """
        Called at ~10-20 Hz by the owning node.

        Parameters
        ----------
        stall_detected : bool
            True when the MKS servo reports a locked-rotor / stall condition.
        """
        now = time.monotonic()

        with self._lock:
            if self._state not in (
                ActuatorState.MOVING_OPEN,
                ActuatorState.MOVING_CLOSE,
                ActuatorState.REFERENCING,
            ):
                self._stall_start = None
                return

            if stall_detected:
                if self._stall_start is None:
                    self._stall_start = now
                elif (now - self._stall_start) * 1000 >= self.cfg.stall_confirm_ms:
                    self._on_stall()
            else:
                self._stall_start = None

    # ── internals ────────────────────────────────────────────────

    def _start_move(
        self, direction: str, speed: float, accel: int
    ) -> tuple[bool, str]:
        with self._lock:
            if self._state == ActuatorState.ERROR:
                return False, f"in error state: {self._error_message}"

            rpm = -speed if direction == "close" else speed
            self._direction = direction
            self._speed_rpm = speed
            self._state = (
                ActuatorState.MOVING_OPEN
                if direction == "open"
                else ActuatorState.MOVING_CLOSE
            )
            self._stall_start = None
            self._last_cmd_ts = time.monotonic()

        self._send_rpm(rpm, accel)
        return True, f"moving {direction} at {speed} rpm"

    def _on_stall(self):
        """Handle confirmed stall — must be called with _lock held."""
        self._send_stop()
        prev_state = self._state

        if prev_state == ActuatorState.REFERENCING:
            self._is_referenced = True
            self._state = ActuatorState.REFERENCED
            _log.info("Reference complete (stall detected)")
        else:
            self._state = (
                ActuatorState.REFERENCED
                if self._is_referenced
                else ActuatorState.IDLE
            )
            _log.info("Stall detected during move, stopped")

        self._direction = "stopped"
        self._speed_rpm = 0.0
        self._stall_start = None

    def _send_rpm(self, rpm: float, accel: int):
        invert = self.cfg.invert_dir
        for mid in self.cfg.motor_ids:
            try:
                is_secondary = self.cfg.dual_motor and mid != self.cfg.motor_ids[0]
                actual_rpm = -rpm if is_secondary else rpm
                self._set_speed(mid, actual_rpm, accel, invert)
            except Exception as exc:
                _log.error("set_speed failed for motor %d: %s", mid, exc)

    def _send_stop(self):
        for mid in self.cfg.motor_ids:
            try:
                self._set_speed(mid, 0.0, 0, self.cfg.invert_dir)
            except Exception:
                pass
