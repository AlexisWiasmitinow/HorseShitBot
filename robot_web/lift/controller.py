from __future__ import annotations
import threading
import time
from ..core.bus import MksBus
from ..core.util import clamp

class LiftController:
    """Spring slider: pct in [-100,100]. Motor B counter-rotates motor A."""

    def __init__(
        self,
        bus: MksBus,
        id_a: int,
        id_b: int,
        max_rpm: float,
        acc_reg: int,
        invert_dir_a: bool,
        watchdog_sec: float,
        deadzone_pct: float,
    ):
        self.bus = bus
        self.id_a = int(id_a)
        self.id_b = int(id_b)
        self.max_rpm = float(max_rpm)
        self.acc_reg = int(acc_reg)
        self.invert_dir_a = bool(invert_dir_a)
        self.watchdog_sec = float(watchdog_sec)
        self.deadzone_pct = float(deadzone_pct)

        self._lock = threading.Lock()
        self._last_ts = 0.0
        self._rpm_a = 0.0
        self._rpm_b = 0.0

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

    def set_pct(self, pct: float):
        pct = clamp(float(pct), -100.0, 100.0)
        if abs(pct) < self.deadzone_pct:
            rpm_a = 0.0
        else:
            rpm_a = (pct / 100.0) * self.max_rpm

        rpm_b = -rpm_a

        with self._lock:
            self._rpm_a = float(rpm_a)
            self._rpm_b = float(rpm_b)
            self._last_ts = time.time()

        self.bus.set_speed_signed(self.id_a, rpm_a, acc=self.acc_reg, invert_dir=self.invert_dir_a)
        self.bus.set_speed_signed(self.id_b, rpm_b, acc=self.acc_reg, invert_dir=self.invert_dir_a)

    def get_status(self) -> dict:
        with self._lock:
            return {"rpm_a": float(self._rpm_a), "rpm_b": float(self._rpm_b)}

    def _loop(self):
        while self._running:
            time.sleep(0.10)
            now = time.time()
            with self._lock:
                rpm_a = self._rpm_a
                rpm_b = self._rpm_b
                age = now - self._last_ts if self._last_ts else 9999.0

            if (rpm_a != 0.0 or rpm_b != 0.0) and age > self.watchdog_sec:
                with self._lock:
                    self._rpm_a = 0.0
                    self._rpm_b = 0.0
                try:
                    self.bus.set_speed_signed(self.id_a, 0.0, acc=self.acc_reg, invert_dir=self.invert_dir_a)
                    self.bus.set_speed_signed(self.id_b, 0.0, acc=self.acc_reg, invert_dir=self.invert_dir_a)
                except Exception:
                    pass
