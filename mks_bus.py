import inspect
import threading
import time
from dataclasses import dataclass

from pymodbus.client import ModbusSerialClient

# pymodbus v3 uses FramerType, v2 uses method="rtu"
try:
    from pymodbus import FramerType
    _FRAMER = FramerType.RTU
except Exception:
    _FRAMER = None

# ----------------- MKS SERVO57D registers -----------------
REG_WORKMODE  = 0x0082
REG_ENABLE    = 0x00F3
REG_SPEED     = 0x00F6            # [dir<<8|acc, speed_rpm]
REG_POS_ABS   = 0x00F5            # [acc, speed, axis_hi, axis_lo]
REG_AXIS_ZERO = 0x0092

# Work modes:
MODE_SR_OPEN = 3
MODE_SR_VFOC = 5

COUNTS_PER_REV = 0x4000          # 360° == 0x4000


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
    inter_delay: float = 0.002   # small gap between RTU frames (helps reliability)


class MksBus:
    def __init__(self, cfg: BusCfg):
        self.cfg = cfg
        self.lock = threading.Lock()

        if _FRAMER is not None:
            # pymodbus v3
            self.client = ModbusSerialClient(
                port=cfg.port,
                framer=_FRAMER,
                baudrate=cfg.baud,
                bytesize=8,
                parity="N",
                stopbits=1,
                timeout=cfg.timeout,
                retries=0,  # we do our own retries for both v2/v3
            )
        else:
            # pymodbus v2
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
        # pymodbus differs, so just try connect if needed
        try:
            # v3 has .connected
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
                # small backoff + try to reconnect
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

    # --------- High-level helpers ---------
    def init_servo(self, unit_id: int, mode: int = MODE_SR_VFOC, enable: bool = True):
        self.write_reg(unit_id, REG_WORKMODE, int(mode))
        if enable:
            self.write_reg(unit_id, REG_ENABLE, 1)

    def axis_zero(self, unit_id: int):
        self.write_reg(unit_id, REG_AXIS_ZERO, 1)

    def set_speed_signed(self, unit_id: int, rpm_signed: float, acc: int = 10, invert_dir: bool = False):
        """
        Speed mode: REG_SPEED (0x00F6) write 2 regs:
          reg0 = (dir<<8) | acc
          reg1 = speed_rpm (0..3000)
        Convention: rpm>=0 => dir=0, rpm<0 => dir=1
        """
        rpm_signed = float(rpm_signed)
        acc = int(clamp(acc, 0, 255))
        rpm_mag = int(clamp(abs(round(rpm_signed)), 0, 3000))

        dir_bit = 0 if rpm_signed >= 0 else 1
        if invert_dir:
            dir_bit ^= 1

        reg0 = ((dir_bit & 0xFF) << 8) | (acc & 0xFF)
        self.write_regs(unit_id, REG_SPEED, [reg0, rpm_mag])

    def move_abs_axis(self, unit_id: int, abs_axis_i32: int, speed_rpm: int = 600, acc: int = 2):
        """
        Position mode absolute axis: REG_POS_ABS (0x00F5) write 4 regs:
          [acc, speed_rpm, axis_hi, axis_lo]
        axis is signed i32.
        """
        speed_rpm = int(clamp(speed_rpm, 0, 3000))
        acc = int(clamp(acc, 0, 65535))
        hi, lo = _split_i32_to_u16s(int(abs_axis_i32))
        self.write_regs(unit_id, REG_POS_ABS, [acc, speed_rpm, hi, lo])
