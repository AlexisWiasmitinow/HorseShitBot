from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from ..core.bus import MksBus
from ..core.util import clamp, sign


def ramp_toward(current: float, target: float, rate_rpm_s: float, dt: float) -> float:
    max_delta = max(0.0, float(rate_rpm_s)) * dt
    delta = clamp(target - current, -max_delta, max_delta)
    nxt = current + delta
    if abs(nxt) < 0.5:
        nxt = 0.0
    return nxt


@dataclass
class WheelChannel:
    actual: float = 0.0
    desired: float = 0.0
    reversing: bool = False
    zero_until: float = 0.0

    def step(self, now: float, dt: float, accel_rate: float, decel_rate: float, zero_hold_sec: float):
        cur_s = sign(self.actual)
        des_s = sign(self.desired)

        if self.reversing:
            self.actual = ramp_toward(self.actual, 0.0, decel_rate, dt)
            if self.actual == 0.0:
                if now < self.zero_until:
                    return
                self.reversing = False
            else:
                return

        if cur_s != 0 and des_s != 0 and cur_s != des_s:
            self.reversing = True
            self.zero_until = now + zero_hold_sec
            self.actual = ramp_toward(self.actual, 0.0, decel_rate, dt)
            return

        rate = decel_rate if abs(self.desired) < abs(self.actual) else accel_rate
        self.actual = ramp_toward(self.actual, self.desired, rate, dt)


class WheelsController:
    def __init__(
        self,
        bus: MksBus,
        id_left: int,
        id_right: int,
        invert_left_dir: bool,
        invert_right_dir: bool,
        max_wheel_rpm: float,
        acc_reg: int,
        update_hz: float,
        accel_rpm_s: float,
        decel_rpm_s: float,
        stop_decel_rpm_s: float,
        zero_hold_sec: float,
        watchdog_sec: float,
    ):
        self.bus = bus
        self.id_left = int(id_left)
        self.id_right = int(id_right)
        self.invert_left_dir = bool(invert_left_dir)
        self.invert_right_dir = bool(invert_right_dir)

        self.max_wheel_rpm = float(max_wheel_rpm)
        self.acc_reg = int(acc_reg)

        self.update_hz = float(update_hz)
        self.accel_rpm_s = float(accel_rpm_s)
        self.decel_rpm_s = float(decel_rpm_s)
        self.stop_decel_rpm_s = float(stop_decel_rpm_s)
        self.zero_hold_sec = float(zero_hold_sec)
        self.watchdog_sec = float(watchdog_sec)

        self._lock = threading.Lock()
        self._last_cmd_ts = 0.0
        self._stop_fast_active = False
        self._pause_tx_until = 0.0

        self._desired_left = 0.0
        self._desired_right = 0.0

        self._L = WheelChannel()
        self._R = WheelChannel()

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._running = False

    def start(self):
        if self._running:
            return
        self._running = True
        if not self._thread.is_alive():
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def set_drive(self, x: float, y: float):
        dl = clamp((float(y) - float(x)) * self.max_wheel_rpm, -self.max_wheel_rpm, self.max_wheel_rpm)
        dr = clamp((float(y) + float(x)) * self.max_wheel_rpm, -self.max_wheel_rpm, self.max_wheel_rpm)

        with self._lock:
            if self._stop_fast_active:
                return
            self._desired_left = float(dl)
            self._desired_right = float(dr)
            self._last_cmd_ts = time.time()

    def set_stop(self):
        with self._lock:
            self._desired_left = 0.0
            self._desired_right = 0.0
            self._last_cmd_ts = time.time()
            self._stop_fast_active = False

    def set_stop_fast(self):
        now = time.time()
        with self._lock:
            self._desired_left = 0.0
            self._desired_right = 0.0
            self._last_cmd_ts = now
            self._stop_fast_active = True

        for _ in range(3):
            self._send(0.0, 0.0)
            time.sleep(0.02)

    def clear_errors_pause(self):
        now = time.time()
        with self._lock:
            self._pause_tx_until = now + 0.9
            self._stop_fast_active = True
            self._desired_left = 0.0
            self._desired_right = 0.0
            self._L = WheelChannel()
            self._R = WheelChannel()
            self._last_cmd_ts = now

        for _ in range(3):
            self._send(0.0, 0.0)
            time.sleep(0.02)

        with self._lock:
            self._stop_fast_active = False
            self._last_cmd_ts = time.time()

    def get_status(self) -> dict:
        with self._lock:
            return {
                "left_rpm": float(self._L.actual),
                "right_rpm": float(self._R.actual),
                "stopping": bool(self._stop_fast_active),
            }

    def _send(self, left_rpm: float, right_rpm: float):
        self.bus.set_speed_signed(self.id_left, left_rpm, acc=self.acc_reg, invert_dir=self.invert_left_dir)
        self.bus.set_speed_signed(self.id_right, right_rpm, acc=self.acc_reg, invert_dir=self.invert_right_dir)

    def _loop(self):
        period = 1.0 / max(1.0, self.update_hz)
        last = time.time()

        while self._running:
            time.sleep(period)
            now = time.time()
            dt = now - last
            last = now

            with self._lock:
                age = now - self._last_cmd_ts
                stop_fast = self._stop_fast_active
                pause_tx = (now < self._pause_tx_until)

                if stop_fast:
                    dl = 0.0
                    dr = 0.0
                    self._desired_left = 0.0
                    self._desired_right = 0.0
                else:
                    dl = self._desired_left
                    dr = self._desired_right

            if (not stop_fast) and age > self.watchdog_sec:
                dl, dr = 0.0, 0.0
                with self._lock:
                    self._desired_left = 0.0
                    self._desired_right = 0.0

            self._L.desired = dl
            self._R.desired = dr

            decel = self.stop_decel_rpm_s if stop_fast else self.decel_rpm_s
            self._L.step(now, dt, self.accel_rpm_s, decel, self.zero_hold_sec)
            self._R.step(now, dt, self.accel_rpm_s, decel, self.zero_hold_sec)

            if not pause_tx:
                try:
                    self._send(self._L.actual, self._R.actual)
                except Exception:
                    pass

            if stop_fast and self._L.actual == 0.0 and self._R.actual == 0.0:
                with self._lock:
                    self._stop_fast_active = False
