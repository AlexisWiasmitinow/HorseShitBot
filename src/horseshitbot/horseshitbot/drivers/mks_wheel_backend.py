"""
MKS stepper wheel backend.

Wraps MksBus to drive two MKS stepper motors as a differential-drive pair.
Called by wheel_driver_node via the WheelBackend interface.
Unlike the old FastAPI controller, this does NOT own the MksBus — it receives
a callable that invokes the mks_bus_node service for each motor command.
"""

from __future__ import annotations

from typing import Callable

from .wheel_backend import WheelBackend


class MksWheelBackend(WheelBackend):
    """
    Parameters
    ----------
    set_speed_fn : callable(motor_id, rpm, acc, invert_dir) -> bool
        Function that sends a set-speed service call to the MKS bus node.
    id_left, id_right : int
        Modbus unit IDs of the left and right wheel motors.
    invert_left, invert_right : bool
        Direction inversion per side.
    acc_reg : int
        Acceleration register value (0-255).
    """

    def __init__(
        self,
        set_speed_fn: Callable[[int, float, int, bool], bool],
        id_left: int,
        id_right: int,
        invert_left: bool = False,
        invert_right: bool = True,
        acc_reg: int = 3,
    ):
        self._set_speed = set_speed_fn
        self.id_left = id_left
        self.id_right = id_right
        self.invert_left = invert_left
        self.invert_right = invert_right
        self.acc_reg = acc_reg

    def connect(self) -> None:
        pass  # MKS bus node owns the connection

    def disconnect(self) -> None:
        self.stop()

    def send_velocity(self, left_rpm: float, right_rpm: float) -> None:
        self._set_speed(self.id_left, left_rpm, self.acc_reg, self.invert_left)
        self._set_speed(self.id_right, right_rpm, self.acc_reg, self.invert_right)

    def stop(self) -> None:
        self.send_velocity(0.0, 0.0)

    def emergency_stop(self) -> None:
        for _ in range(3):
            try:
                self.send_velocity(0.0, 0.0)
            except Exception:
                pass
