"""
MKS Servo Modbus RTU driver.

Ported from robot_web/core/bus.py — standalone, no FastAPI dependencies.
Provides thread-safe Modbus RTU communication with MKS servo motors.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient

_pymodbus_logger = logging.getLogger("pymodbus")

REG_WORKMODE = 0x0082
REG_RUN_CURRENT = 0x0083
REG_MICROSTEPS = 0x0084
REG_HOLD_CURRENT_PCT = 0x009B
REG_ENABLE = 0x00F3
REG_SPEED = 0x00F6
REG_POS_ABS = 0x00F5
REG_AXIS_ZERO = 0x0092
REG_RELEASE_PROTECT = 0x003D

MODE_SR_OPEN = 3
MODE_SR_CLOSE = 4
MODE_SR_VFOC = 5

COUNTS_PER_REV = 0x4000


def _split_i32_to_u16s(val: int):
    val &= 0xFFFFFFFF
    return [(val >> 16) & 0xFFFF, val & 0xFFFF]


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


@dataclass
class BusCfg:
    port: str
    baud: int = 38400
    timeout: float = 0.35
    retries: int = 3
    inter_delay: float = 0.002


class MksBus:
    """Thread-safe Modbus RTU client for MKS servo motors."""

    def __init__(self, cfg: BusCfg):
        self.cfg = cfg
        self.lock = threading.Lock()

        self.client = ModbusSerialClient(
            port=cfg.port,
            framer=FramerType.RTU,
            baudrate=cfg.baud,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=cfg.timeout,
            retries=cfg.retries,
        )

    def connect(self):
        if not self.client.connect():
            raise RuntimeError(f"Cannot open Modbus port: {self.cfg.port}")

    def close(self):
        try:
            self.client.close()
        except Exception:
            pass

    def _ensure_connected(self):
        try:
            if hasattr(self.client, "connected") and not self.client.connected:
                self.client.connect()
        except Exception:
            pass

    def _retry(self, call, err_ctx: str):
        """Execute a Modbus call. Pymodbus handles protocol-level retries;
        we only reconnect if the serial port itself drops."""
        self._ensure_connected()
        try:
            with self.lock:
                rr = call()
            if hasattr(rr, "isError") and rr.isError():
                raise RuntimeError(f"{err_ctx}: {rr}")
            if self.cfg.inter_delay and self.cfg.inter_delay > 0:
                time.sleep(self.cfg.inter_delay)
            return rr
        except Exception as first_err:
            # One reconnect attempt in case the port dropped
            try:
                self.client.close()
            except Exception:
                pass
            try:
                self.client.connect()
            except Exception:
                pass
            time.sleep(0.02)
            try:
                with self.lock:
                    rr = call()
                if hasattr(rr, "isError") and rr.isError():
                    raise RuntimeError(f"{err_ctx}: {rr}")
                if self.cfg.inter_delay and self.cfg.inter_delay > 0:
                    time.sleep(self.cfg.inter_delay)
                return rr
            except Exception:
                raise RuntimeError(f"{err_ctx}: {first_err}") from None

    def write_reg(self, unit_id: int, addr: int, value: int):
        def _call():
            return self.client.write_register(
                address=addr,
                value=int(value) & 0xFFFF,
                device_id=unit_id,
            )
        return self._retry(_call, f"write_reg failed unit={unit_id} addr=0x{addr:04X}")

    def write_regs(self, unit_id: int, addr: int, values: list[int]):
        def _call():
            return self.client.write_registers(
                address=addr,
                values=[int(v) & 0xFFFF for v in values],
                device_id=unit_id,
            )
        return self._retry(_call, f"write_regs failed unit={unit_id} addr=0x{addr:04X}")

    def read_regs(self, unit_id: int, addr: int, count: int = 1):
        def _call():
            return self.client.read_holding_registers(
                address=addr,
                count=count,
                device_id=unit_id,
            )
        rr = self._retry(_call, f"read_regs failed unit={unit_id} addr=0x{addr:04X}")
        return rr.registers if hasattr(rr, "registers") else []

    def read_input_regs(self, unit_id: int, addr: int, count: int = 1):
        def _call():
            return self.client.read_input_registers(
                address=addr,
                count=count,
                device_id=unit_id,
            )
        rr = self._retry(_call, f"read_input_regs failed unit={unit_id} addr=0x{addr:04X}")
        return rr.registers if hasattr(rr, "registers") else []

    def probe(self, unit_id: int) -> bool:
        """Quick single-attempt connectivity check (no retries, no reconnect).
        Suppresses pymodbus log noise for expected timeouts."""
        self._ensure_connected()
        prev_level = _pymodbus_logger.level
        prev_retries = getattr(self.client, "retries", 0)
        _pymodbus_logger.setLevel(logging.CRITICAL)
        try:
            self.client.retries = 0
            with self.lock:
                rr = self.client.read_input_registers(
                    address=0x3A, count=1, device_id=unit_id,
                )
            return not (hasattr(rr, "isError") and rr.isError())
        except Exception:
            return False
        finally:
            self.client.retries = prev_retries
            _pymodbus_logger.setLevel(prev_level)

    def get_run_current(self, unit_id: int) -> int | None:
        """Read run current in mA. Returns None if firmware doesn't support readback."""
        try:
            regs = self.read_input_regs(unit_id, REG_RUN_CURRENT, 1)
            if regs:
                return regs[0]
        except Exception:
            pass
        return None

    def set_run_current(self, unit_id: int, ma: int):
        """Set run current in mA (clamped to 10–5200)."""
        self.write_reg(unit_id, REG_RUN_CURRENT, int(clamp(ma, 10, 5200)))

    def get_hold_current_pct(self, unit_id: int) -> int | None:
        """Read hold current as percentage (10–100). Returns None if firmware doesn't support readback."""
        try:
            regs = self.read_input_regs(unit_id, REG_HOLD_CURRENT_PCT, 1)
            if regs:
                return (regs[0] + 1) * 10
        except Exception:
            pass
        return None

    def set_hold_current_pct(self, unit_id: int, pct: int):
        """Set hold current percentage (10–100, in steps of 10)."""
        pct = int(clamp(pct, 10, 100))
        self.write_reg(unit_id, REG_HOLD_CURRENT_PCT, (pct - 10) // 10)

    def init_servo(self, unit_id: int, mode: int = MODE_SR_VFOC, enable: bool = True):
        self.write_reg(unit_id, REG_WORKMODE, int(mode))
        if enable:
            self.write_reg(unit_id, REG_ENABLE, 1)

    def set_enable(self, unit_id: int, enable: bool):
        self.write_reg(unit_id, REG_ENABLE, 1 if enable else 0)

    def axis_zero(self, unit_id: int):
        self.write_reg(unit_id, REG_AXIS_ZERO, 1)

    def release_locked_rotor(self, unit_id: int):
        self.write_reg(unit_id, REG_RELEASE_PROTECT, 1)

    def set_speed_signed(
        self, unit_id: int, rpm_signed: float, acc: int = 10, invert_dir: bool = False
    ):
        rpm_signed = float(rpm_signed)
        acc = int(clamp(acc, 0, 255))
        rpm_mag = int(clamp(abs(round(rpm_signed)), 0, 3000))

        dir_bit = 0 if rpm_signed >= 0 else 1
        if invert_dir:
            dir_bit ^= 1

        reg0 = ((dir_bit & 0xFF) << 8) | (acc & 0xFF)
        self.write_regs(unit_id, REG_SPEED, [reg0, rpm_mag])

    def move_abs_axis(
        self, unit_id: int, abs_axis_i32: int, speed_rpm: int = 600, acc: int = 2
    ):
        speed_rpm = int(clamp(speed_rpm, 0, 3000))
        acc = int(clamp(acc, 0, 65535))
        hi, lo = _split_i32_to_u16s(int(abs_axis_i32))
        self.write_regs(unit_id, REG_POS_ABS, [acc, speed_rpm, hi, lo])

    def move_turns(
        self, unit_id: int, turns: float, speed_rpm: int = 300,
        acc: int = 3, invert_dir: bool = False,
    ):
        """Relative move: zero the axis, then move to turns × COUNTS_PER_REV."""
        self.axis_zero(unit_id)
        time.sleep(0.05)
        counts = int(round(turns * COUNTS_PER_REV))
        if invert_dir:
            counts = -counts
        self.move_abs_axis(unit_id, counts, speed_rpm=speed_rpm, acc=acc)

    def clear_error_state(self, unit_id: int, mode: int | None = None):
        try:
            self.set_speed_signed(unit_id, 0.0, acc=0, invert_dir=False)
        except Exception:
            pass
        try:
            self.release_locked_rotor(unit_id)
        except Exception:
            pass
        try:
            self.axis_zero(unit_id)
        except Exception:
            pass
        try:
            self.set_enable(unit_id, False)
            time.sleep(0.05)
        except Exception:
            pass
        if mode is not None:
            try:
                self.write_reg(unit_id, REG_WORKMODE, int(mode))
            except Exception:
                pass
        try:
            self.set_enable(unit_id, True)
        except Exception:
            pass
