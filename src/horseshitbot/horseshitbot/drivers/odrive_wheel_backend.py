"""
ODrive wheel backend.

Wraps ODriveSerial to drive two BLDC motors via axis0/axis1 on a single
ODrive board as a differential-drive pair.
"""

from __future__ import annotations

import logging
import time

from .odrive_serial import ODriveSerial
from .wheel_backend import WheelBackend

_log = logging.getLogger(__name__)


class ODriveWheelBackend(WheelBackend):
    """
    Parameters
    ----------
    port : str
        Serial port for the ODrive (e.g. /dev/ttyACM0, COM18).
    baudrate : int
        Serial baud rate (default 115200).
    vel_limit : float
        ODrive velocity limit in turns/sec.
    current_lim : float
        Motor current limit in amps.
    vel_ramp_rate : float
        Velocity ramp rate in turns/sec^2.
    rpm_to_tps : float
        Conversion factor from RPM to turns-per-second (default 1/60).
    invert_left, invert_right : bool
        Direction inversion per side.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        vel_limit: float = 100.0,
        current_lim: float = 20.0,
        vel_ramp_rate: float = 20.0,
        rpm_to_tps: float = 1.0 / 60.0,
        invert_left: bool = False,
        invert_right: bool = False,
    ):
        self._odrv = ODriveSerial(port, baudrate)
        self._vel_limit = vel_limit
        self._current_lim = current_lim
        self._vel_ramp_rate = vel_ramp_rate
        self._rpm_to_tps = rpm_to_tps
        self._invert_left = invert_left
        self._invert_right = invert_right
        self._connected = False

    def connect(self) -> None:
        if not self._odrv.connect():
            raise RuntimeError(f"Cannot connect to ODrive on {self._odrv.port}")

        vbus = self._odrv.get_vbus_voltage()
        _log.info("ODrive connected — Vbus=%.1fV", vbus)

        ready = {}
        for ax in (0, 1):
            ready[ax] = self._odrv.is_ready_for_closed_loop(ax)
            cal = self._odrv.read_property(f"axis{ax}.motor.is_calibrated")
            enc = self._odrv.read_property(f"axis{ax}.encoder.is_ready")
            _log.info("Axis %d: calibrated=%s  encoder_ready=%s  ready=%s",
                       ax, cal, enc, ready[ax])

        if not all(ready.values()):
            _log.info("Calibration needed — applying defaults …")
            self._odrv.apply_defaults()

        for ax in (0, 1):
            self._odrv.write_property(
                f"axis{ax}.controller.config.vel_limit", str(self._vel_limit)
            )
            self._odrv.write_property(
                f"axis{ax}.motor.config.current_lim", str(self._current_lim)
            )
            self._odrv.write_property(
                f"axis{ax}.controller.config.vel_ramp_rate", str(self._vel_ramp_rate)
            )

        for ax in (0, 1):
            if ready[ax]:
                _log.info("Axis %d already calibrated — entering closed loop …", ax)
            else:
                _log.info("Axis %d needs calibration — starting full sequence …", ax)
            if not self._odrv.start_motor(ax):
                raise RuntimeError(f"ODrive axis{ax} startup failed")
            self._odrv.set_velocity_mode(ax)
            _log.info("Axis %d ready.", ax)

        self._connected = True

    def disconnect(self) -> None:
        if self._connected:
            for ax in (0, 1):
                try:
                    self._odrv.set_velocity(ax, 0.0)
                except Exception:
                    pass
                try:
                    self._odrv.set_idle(ax)
                except Exception:
                    pass
        self._odrv.disconnect()
        self._connected = False

    def send_velocity(self, left_rpm: float, right_rpm: float) -> None:
        left_tps = left_rpm * self._rpm_to_tps
        right_tps = right_rpm * self._rpm_to_tps
        if self._invert_left:
            left_tps = -left_tps
        if self._invert_right:
            right_tps = -right_tps
        self._odrv.set_velocity(0, left_tps)
        self._odrv.set_velocity(1, right_tps)

    def stop(self) -> None:
        self.send_velocity(0.0, 0.0)

    def emergency_stop(self) -> None:
        for _ in range(3):
            try:
                self._odrv.set_velocity(0, 0.0)
                self._odrv.set_velocity(1, 0.0)
            except Exception:
                pass
        for ax in (0, 1):
            try:
                self._odrv.set_idle(ax)
            except Exception:
                pass

    def resume(self) -> bool:
        """Re-enter closed loop after e-stop."""
        try:
            self._odrv.ser.reset_input_buffer()
            time.sleep(0.2)

            for ax in (0, 1):
                self._odrv.clear_errors(ax)

                cal = self._odrv.read_property(f"axis{ax}.motor.is_calibrated")
                enc = self._odrv.read_property(f"axis{ax}.encoder.is_ready")
                _log.info("Axis %d: calibrated=%s encoder_ready=%s", ax, cal, enc)

                if not self._odrv.set_closed_loop(ax):
                    _log.error("set_closed_loop failed on axis %d, trying start_motor", ax)
                    if not self._odrv.start_motor(ax):
                        _log.error("start_motor also failed on axis %d", ax)
                        return False

                self._odrv.set_velocity_mode(ax)
                _log.info("Axis %d resumed", ax)
            return True
        except Exception as e:
            _log.error("Resume failed: %s", e)
            return False

    def get_diagnostics(self) -> dict:
        if not self._connected:
            return {}
        diag: dict = {}
        try:
            diag["vbus"] = self._odrv.get_vbus_voltage()
        except Exception:
            pass
        for ax in (0, 1):
            label = "left" if ax == 0 else "right"
            try:
                t = self._odrv.get_temperature(ax)
                if t is not None:
                    diag[f"fet_temp_{label}"] = round(t, 1)
            except Exception:
                pass
            try:
                t = self._odrv.get_motor_temperature(ax)
                if t is not None:
                    diag[f"motor_temp_{label}"] = round(t, 1)
            except Exception:
                pass
        return diag
