from __future__ import annotations
import threading
import time
from ..core.bus import MksBus
from ..core.util import clamp

class AuxSpeedController:
    """Spring slider controller: pct in [-100,100] => speed command (rpm). Stale commands auto-stop."""

    def __init__(
        self,
        bus: MksBus,
        motor_id: int,
        max_rpm: float,
        acc_reg: int,
        invert_dir: bool,
        watchdog_sec: float,
        deadzone_pct: float,
    ):
        self.bus = bus
        self.motor_id = int(motor_id)
        self.max_rpm = float(max_rpm)
        self.acc_reg = int(acc_reg)
        self.invert_dir = bool(invert_dir)
        self.watchdog_sec = float(watchdog_sec)
        self.deadzone_pct = float(deadzone_pct)

        self._lock = threading.Lock()
        self._last_ts = 0.0
        self._rpm = 0.0

        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._running = False

    def start(self):
        if self._running:
            return
        self._running = True
        if not self._thread.is_alive():
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self):
        self.set_pct(0.0)
        self._send_rpm(0.0)

    def set_pct(self, pct: float):
        pct = clamp(float(pct), -100.0, 100.0)
        if abs(pct) < self.deadzone_pct:
            rpm = 0.0
        else:
            rpm = (pct / 100.0) * self.max_rpm

        with self._lock:
            self._rpm = float(rpm)
            self._last_ts = time.time()

        self._send_rpm(rpm)

    def get_status(self) -> dict:
        with self._lock:
            return {"rpm": float(self._rpm)}

    def _send_rpm(self, rpm: float):
        self.bus.set_speed_signed(self.motor_id, rpm, acc=self.acc_reg, invert_dir=self.invert_dir)

    def _loop(self):
        while self._running:
            time.sleep(0.10)
            now = time.time()
            with self._lock:
                rpm = self._rpm
                age = now - self._last_ts if self._last_ts else 9999.0

            if rpm != 0.0 and age > self.watchdog_sec:
                with self._lock:
                    self._rpm = 0.0
                try:
                    self._send_rpm(0.0)
                except Exception:
                    pass
