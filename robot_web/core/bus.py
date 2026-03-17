import inspect
import threading
import time
from dataclasses import dataclass

from pymodbus.client import ModbusSerialClient

try:
    from pymodbus import FramerType
    _FRAMER = FramerType.RTU
except Exception:
    _FRAMER = None

REG_WORKMODE         = 0x0082
REG_ENABLE           = 0x00F3
REG_SPEED            = 0x00F6
REG_POS_ABS          = 0x00F5
REG_AXIS_ZERO        = 0x0092
REG_RELEASE_PROTECT  = 0x003D

MODE_SR_OPEN = 3
MODE_SR_VFOC = 5

COUNTS_PER_REV = 0x4000

def _id_kw(method):
    try:
        params = inspect.signature(method).parameters
        for key in ("device_id", "unit", "slave"):
            if key in params:
                return key
    except Exception:
        pass
    return None

def _with_unit(method, unit_id: int):
    kw = _id_kw(method)
    return {kw: unit_id} if kw else {}

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
    def __init__(self, cfg: BusCfg):
        self.cfg = cfg
        self.lock = threading.Lock()

        if _FRAMER is not None:
            self.client = ModbusSerialClient(
                port=cfg.port,
                framer=_FRAMER,
                baudrate=cfg.baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=cfg.timeout,
                retries=0,
            )
        else:
            self.client = ModbusSerialClient(
                method="rtu",
                port=cfg.port,
                baudrate=cfg.baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=cfg.timeout,
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
        last_exc = None
        attempts = max(1, int(self.cfg.retries) + 1)

        for i in range(attempts):
            try:
                self._ensure_connected()
                with self.lock:
                    rr = call()
                if hasattr(rr, "isError") and rr.isError():
                    raise RuntimeError(f"{err_ctx}: {rr}")

                if self.cfg.inter_delay and self.cfg.inter_delay > 0:
                    time.sleep(self.cfg.inter_delay)
                return rr

            except Exception as e:
                last_exc = e
                try:
                    self.client.close()
                except Exception:
                    pass
                try:
                    self.client.connect()
                except Exception:
                    pass
                time.sleep(0.02 * (i + 1))

        raise RuntimeError(f"{err_ctx}: {last_exc}")

    def write_reg(self, unit_id: int, addr: int, value: int):
        def _call():
            return self.client.write_register(
                address=addr,
                value=int(value) & 0xFFFF,
                **_with_unit(self.client.write_register, unit_id),
            )
        return self._retry(_call, f"write_reg failed unit={unit_id} addr=0x{addr:04X}")

    def write_regs(self, unit_id: int, addr: int, values: list[int]):
        def _call():
            return self.client.write_registers(
                address=addr,
                values=[int(v) & 0xFFFF for v in values],
                **_with_unit(self.client.write_registers, unit_id),
            )
        return self._retry(_call, f"write_regs failed unit={unit_id} addr=0x{addr:04X}")

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

    def set_speed_signed(self, unit_id: int, rpm_signed: float, acc: int = 10, invert_dir: bool = False):
        rpm_signed = float(rpm_signed)
        acc = int(clamp(acc, 0, 255))
        rpm_mag = int(clamp(abs(round(rpm_signed)), 0, 3000))

        dir_bit = 0 if rpm_signed >= 0 else 1
        if invert_dir:
            dir_bit ^= 1

        reg0 = ((dir_bit & 0xFF) << 8) | (acc & 0xFF)
        self.write_regs(unit_id, REG_SPEED, [reg0, rpm_mag])

    def move_abs_axis(self, unit_id: int, abs_axis_i32: int, speed_rpm: int = 600, acc: int = 2):
        speed_rpm = int(clamp(speed_rpm, 0, 3000))
        acc = int(clamp(acc, 0, 65535))
        hi, lo = _split_i32_to_u16s(int(abs_axis_i32))
        self.write_regs(unit_id, REG_POS_ABS, [acc, speed_rpm, hi, lo])

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
