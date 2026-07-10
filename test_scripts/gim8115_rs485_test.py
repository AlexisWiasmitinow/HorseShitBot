#!/usr/bin/env python3
"""
Standalone test script for the GIM8115-9 joint motor over RS485.

IMPORTANT: out of the box this motor speaks SteadyWin's *custom* RS485 protocol
(header 0xAE host / 0xAC slave), NOT Modbus RTU. That is why a Modbus-only
client gets no answer. Protocol reference:

    docs/Motor_GIM8115-9/自定义RS485通信协议_3.03b0.pdf

The Modbus register CSV in the same folder is a separate map that only applies
if the driver reports a non-zero RS485-Modbus protocol version (see cmd 0x0A).

Defaults from the PDF:
    baud 115200, 8N1, little-endian multi-byte fields, CRC16-Modbus
    device address 0x01 (green LED blink pattern on the driver)

Position units: 16384 counts = 1 revolution = 360°.

Dependencies:
    pip install pyserial

Usage:
    python gim8115_rs485_test.py --port COM9 --info
    python gim8115_rs485_test.py --port COM9 --move 90 --speed 10
    python gim8115_rs485_test.py --port COM9 --velocity 20
    python gim8115_rs485_test.py --port COM9 --torture 90 --speed 20
    python gim8115_rs485_test.py --port COM9 --torture 180 --speed 30 --temp-every 10
    python gim8115_rs485_test.py --port COM9 --bus-test
    python gim8115_rs485_test.py --port COM9 --bus-test --rate 5 --duration 60
    python gim8115_rs485_test.py --port COM9 --probe
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

import serial

# ── custom RS485 protocol (PDF V3.03b0) ─────────────────────────
HDR_HOST = 0xAE
HDR_SLAVE = 0xAC

CMD_READ_VERSION = 0x0A
CMD_READ_STATUS = 0x0B
CMD_CLEAR_FAULT = 0x0F
CMD_VELOCITY = 0x21          # target vel (0.01 RPM) + accel (0.01 RPM/s; 0 = max)
CMD_ABS_POSITION = 0x22
CMD_REL_POSITION = 0x23
CMD_TRAP_POSITION = 0x26  # trapezoid: type + pos + vmax + accel + decel
CMD_DISABLE = 0x2F

POS_TYPE_ABS = 0x00
POS_TYPE_REL = 0x01

COUNTS_PER_REV = 16384
DEG_PER_COUNT = 360.0 / COUNTS_PER_REV

RUN_STATE_NAMES = {
    0: "off",
    1: "voltage",
    2: "Iq",
    3: "velocity",
    4: "position",
}

FAULT_BITS = (
    (0, "voltage"),
    (1, "current"),
    (2, "temperature"),
    (3, "encoder"),
    (6, "hardware"),
    (7, "software"),
)


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def degrees_to_counts(degrees: float) -> int:
    return int(round(degrees / DEG_PER_COUNT))


def counts_to_degrees(counts: int) -> float:
    return counts * DEG_PER_COUNT


def format_fault(code: int) -> str:
    if code == 0:
        return "none"
    bits = [name for bit, name in FAULT_BITS if code & (1 << bit)]
    extra = code & ~sum(1 << b for b, _ in FAULT_BITS)
    if extra:
        bits.append(f"other=0x{extra:02X}")
    return ",".join(bits) if bits else f"0x{code:02X}"


class GIM8115RS485:
    """SteadyWin custom RS485 framing (not Modbus)."""

    def __init__(self, ser: serial.Serial, addr: int = 1):
        self.ser = ser
        self.addr = addr & 0xFF
        self._seq = 0
        self._last_tx_end = 0.0
        self.inter_frame_gap_s = 0.02  # bus turnaround between commands
        # Snapshot at init — never use ser.timeout later (it is mutated during reads).
        self.default_timeout = float(ser.timeout) if ser.timeout else 0.5

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def build_frame(self, cmd: int, payload: bytes = b"", seq: int | None = None) -> bytes:
        if seq is None:
            seq = self._next_seq()
        body = bytes([HDR_HOST, seq & 0xFF, self.addr, cmd & 0xFF, len(payload) & 0xFF]) + payload
        crc = crc16_modbus(body)
        return body + struct.pack("<H", crc)

    def _read_exact(self, n: int, deadline: float) -> bytes:
        """Read n bytes without permanently mutating ser.timeout (that caused fail cascades)."""
        buf = bytearray()
        saved_timeout = self.ser.timeout
        try:
            while len(buf) < n:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                self.ser.timeout = remaining
                chunk = self.ser.read(n - len(buf))
                if not chunk:
                    break
                buf.extend(chunk)
        finally:
            self.ser.timeout = saved_timeout
        return bytes(buf)

    def _wait_bus_idle(self):
        gap = self.inter_frame_gap_s - (time.time() - self._last_tx_end)
        if gap > 0:
            time.sleep(gap)

    def _drain_rx(self, quiet_s: float = 0.05, max_s: float = 0.35):
        """Discard late/garbage RX until the line is quiet (half-duplex recovery)."""
        t_end = time.time() + max_s
        last_data = time.time()
        saved_timeout = self.ser.timeout
        try:
            self.ser.timeout = 0.01
            while time.time() < t_end:
                n = getattr(self.ser, "in_waiting", 0) or 0
                if n:
                    self.ser.read(n)
                    last_data = time.time()
                    continue
                chunk = self.ser.read(64)
                if chunk:
                    last_data = time.time()
                    continue
                if time.time() - last_data >= quiet_s:
                    break
                time.sleep(0.005)
        finally:
            self.ser.timeout = saved_timeout
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def transact(
        self,
        cmd: int,
        payload: bytes = b"",
        timeout: float | None = None,
        retries: int = 2,
    ) -> bytes:
        """Send a command and return the full reply frame (including header/CRC)."""
        # Never trust ser.timeout as the command deadline — _read_exact may have
        # temporarily lowered it; use the snapshot from init.
        if timeout is None:
            timeout = self.default_timeout
        last_err: Exception | None = None
        for attempt in range(retries + 1):
            try:
                return self._transact_once(cmd, payload, timeout)
            except (TimeoutError, ValueError) as e:
                last_err = e
                # Let late bytes finish, then flush — common after truncated frames.
                self._drain_rx(quiet_s=0.04 + 0.02 * attempt, max_s=0.25 + 0.1 * attempt)
                time.sleep(0.03 * (attempt + 1))
        assert last_err is not None
        raise last_err

    def _transact_once(self, cmd: int, payload: bytes, timeout: float) -> bytes:
        self._wait_bus_idle()
        frame = self.build_frame(cmd, payload)
        self.ser.reset_input_buffer()
        self.ser.write(frame)
        self.ser.flush()
        bit_time = 10.0 / max(self.ser.baudrate, 1)
        time.sleep(bit_time * len(frame) + 0.002)
        self._last_tx_end = time.time()

        deadline = time.time() + timeout
        buf = bytearray()
        # Buffer + search: reject false 0xAC syncs (payload bytes can equal 0xAC).
        while time.time() < deadline:
            n_wait = getattr(self.ser, "in_waiting", 0) or 0
            chunk = self._read_exact(max(n_wait, 1), deadline)
            if chunk:
                buf.extend(chunk)
            while True:
                try:
                    idx = buf.index(HDR_SLAVE)
                except ValueError:
                    buf.clear()
                    break
                if idx:
                    del buf[:idx]
                if len(buf) < 5:
                    break
                length = buf[4]
                # Protocol payloads are small; absurd len => false sync on noise.
                if length > 64:
                    del buf[0]
                    continue
                frame_len = 5 + length + 2
                if len(buf) < frame_len:
                    break
                reply = bytes(buf[:frame_len])
                del buf[:frame_len]
                body, crc_rx = reply[:-2], reply[-2:]
                crc_calc = crc16_modbus(body)
                crc_got = struct.unpack("<H", crc_rx)[0]
                if crc_got != crc_calc:
                    # Bad frame — keep scanning remaining buffer for another 0xAC.
                    continue
                r_addr, r_cmd = reply[2], reply[3]
                if r_cmd != cmd:
                    continue
                if self.addr not in (0xFF, r_addr) and r_addr != 0xFF:
                    continue
                return reply
            if not chunk:
                time.sleep(0.001)

        if buf:
            raise TimeoutError(
                f"truncated reply for cmd 0x{cmd:02X} "
                f"({len(buf)} bytes buffered: {bytes(buf)[:24].hex(' ')}"
                f"{'…' if len(buf) > 24 else ''})"
            )
        raise TimeoutError(
            f"no reply to cmd 0x{cmd:02X} (sent {frame.hex(' ')}). "
            f"Check port/wiring/baud/address, and that the motor is on the custom RS485 protocol."
        )

    def ping(self) -> bool:
        try:
            self.read_status()
            return True
        except Exception:
            return False

    def read_version(self) -> dict:
        reply = self.transact(CMD_READ_VERSION)
        data = reply[5:-2]
        if len(data) < 22:
            raise ValueError(f"short 0x0A reply ({len(data)} bytes)")
        boot, sw, hw = struct.unpack_from("<HHH", data, 0)
        rs485_custom = data[6]
        rs485_modbus = data[7]
        can_custom = data[8]
        can_open = data[9]
        uid = data[10:22].hex().upper()
        return {
            "boot": boot,
            "software": sw,
            "hardware": hw,
            "rs485_custom": rs485_custom,
            "rs485_modbus": rs485_modbus,
            "can_custom": can_custom,
            "can_open": can_open,
            "uid": uid,
        }

    def parse_status_payload(self, data: bytes) -> dict:
        # 0x0B reply data length 0x16 = 22 bytes
        if len(data) < 22:
            raise ValueError(f"short status payload ({len(data)} bytes)")
        single = struct.unpack_from("<H", data, 0)[0]
        multi = struct.unpack_from("<i", data, 2)[0]
        vel = struct.unpack_from("<i", data, 6)[0]
        iq = struct.unpack_from("<i", data, 10)[0]
        vbus = struct.unpack_from("<H", data, 14)[0]
        ibus = struct.unpack_from("<H", data, 16)[0]
        temp = data[18]
        run_state = data[19]
        motor_state = data[20]
        fault = data[21]
        return {
            "single_counts": single,
            "single_deg": counts_to_degrees(single),
            "multi_counts": multi,
            "multi_deg": counts_to_degrees(multi),
            "velocity_rpm": vel / 100.0,
            "iq_a": iq / 1000.0,
            "vbus_v": vbus / 100.0,
            "ibus_a": ibus / 100.0,
            "temp_c": temp,
            "run_state": run_state,
            "run_state_name": RUN_STATE_NAMES.get(run_state, f"unknown({run_state})"),
            "motor_state": motor_state,
            "enabled": motor_state != 0,
            "fault": fault,
        }

    def read_status(self, retries: int = 2) -> dict:
        reply = self.transact(CMD_READ_STATUS, retries=retries)
        return self.parse_status_payload(reply[5:-2])

    def clear_fault(self) -> int:
        reply = self.transact(CMD_CLEAR_FAULT)
        data = reply[5:-2]
        return data[0] if data else 0

    def disable(self, settle_s: float = 0.05) -> dict:
        """
        Freewheel / disable output (cmd 0x2F).
        After velocity streaming the bus is hot — wait briefly, optionally ramp to 0,
        then disable with a longer timeout and retries.
        """
        time.sleep(settle_s)
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        # Soft stop first so 0x2F isn't fighting a high-rate velocity loop.
        try:
            self.set_velocity(0.0, accel_rpm_s=0.0)
            time.sleep(0.05)
        except Exception:
            pass
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        time.sleep(0.02)
        reply = self.transact(CMD_DISABLE, timeout=1.0, retries=4)
        return self.parse_status_payload(reply[5:-2])

    def set_velocity(self, speed_rpm: float, accel_rpm_s: float = 0.0) -> dict:
        """
        Velocity mode (cmd 0x21).
        speed_rpm: signed target speed (PDF unit 0.01 RPM).
        accel_rpm_s: accel in RPM/s; 0 means maximum accel (PDF).
        """
        vel = int(round(speed_rpm * 100.0))
        accel = 0 if accel_rpm_s <= 0 else max(1, int(round(accel_rpm_s * 100.0)))
        payload = struct.pack("<iI", vel, accel)
        reply = self.transact(CMD_VELOCITY, payload)
        return self.parse_status_payload(reply[5:-2])

    def move_relative_counts(self, counts: int) -> dict:
        """Simple relative move (cmd 0x23) — driver uses its internal max speed."""
        payload = struct.pack("<i", int(counts))
        reply = self.transact(CMD_REL_POSITION, payload)
        return self.parse_status_payload(reply[5:-2])

    def move_relative_trapezoid(
        self,
        counts: int,
        speed_rpm: float,
        accel_rpm_s: float | None = None,
        decel_rpm_s: float | None = None,
    ) -> dict:
        """
        Trapezoidal relative move (cmd 0x26).
        Units from PDF: speed/accel/decel are 0.01 RPM and 0.01 RPM/s.
        """
        if accel_rpm_s is None:
            accel_rpm_s = speed_rpm
        if decel_rpm_s is None:
            decel_rpm_s = accel_rpm_s
        vmax = max(1, int(round(speed_rpm * 100.0)))
        accel = max(1, int(round(accel_rpm_s * 100.0)))
        decel = max(1, int(round(decel_rpm_s * 100.0)))
        payload = (
            bytes([POS_TYPE_REL])
            + struct.pack("<i", int(counts))
            + struct.pack("<I", vmax)
            + struct.pack("<I", accel)
            + struct.pack("<I", decel)
        )
        reply = self.transact(CMD_TRAP_POSITION, payload)
        return self.parse_status_payload(reply[5:-2])

    def move_relative_degrees(
        self,
        degrees: float,
        speed_rpm: float | None = None,
        accel_rpm_s: float | None = None,
        decel_rpm_s: float | None = None,
    ) -> tuple[int, dict]:
        counts = degrees_to_counts(degrees)
        if speed_rpm is None:
            return counts, self.move_relative_counts(counts)
        return counts, self.move_relative_trapezoid(
            counts, speed_rpm, accel_rpm_s=accel_rpm_s, decel_rpm_s=decel_rpm_s
        )


def print_info(motor: GIM8115RS485):
    print()
    print("=" * 50)
    print("GIM8115-9 DEVICE INFO (custom RS485)")
    print("=" * 50)
    try:
        ver = motor.read_version()
        print(f"  Boot / SW / HW : {ver['boot']} / {ver['software']} / {ver['hardware']}")
        print(f"  RS485 custom   : v{ver['rs485_custom']}")
        print(f"  RS485 Modbus   : v{ver['rs485_modbus']}"
              + ("  (0 = not supported on this driver)" if ver["rs485_modbus"] == 0 else ""))
        print(f"  CAN custom/Open: v{ver['can_custom']} / v{ver['can_open']}")
        print(f"  UID            : {ver['uid']}")
    except Exception as e:
        print(f"  Version (0x0A) : failed ({e})")

    st = motor.read_status()
    print(f"  Multi-turn     : {st['multi_counts']} counts ({st['multi_deg']:+.2f}°)")
    print(f"  Single-turn    : {st['single_counts']} counts ({st['single_deg']:.2f}°)")
    print(f"  Velocity       : {st['velocity_rpm']:+.2f} RPM")
    print(f"  Iq / Vbus/Ibus : {st['iq_a']:+.3f} A / {st['vbus_v']:.2f} V / {st['ibus_a']:.2f} A")
    print(f"  Temperature    : {st['temp_c']} °C")
    print(f"  Run state      : {st['run_state']} ({st['run_state_name']})")
    print(f"  Motor state    : {st['motor_state']} ({'enabled' if st['enabled'] else 'disabled/free'})")
    print(f"  Fault          : {format_fault(st['fault'])}")


def _read_status_retry(motor: GIM8115RS485, attempts: int = 3) -> dict:
    """Status poll that tolerates occasional truncated RS485 frames."""
    last_err: Exception | None = None
    for i in range(attempts):
        try:
            return motor.read_status()
        except (TimeoutError, ValueError) as e:
            last_err = e
            try:
                motor.ser.reset_input_buffer()
            except Exception:
                pass
            # Back off — don't hammer the slave after a bad frame.
            time.sleep(0.08 * (i + 1))
    assert last_err is not None
    raise last_err


def wait_move(
    motor: GIM8115RS485,
    target_counts: int,
    timeout_s: float,
    settle_rpm: float = 1.0,
    quiet: bool = False,
    poll_s: float = 0.1,
    initial_delay_s: float = 0.0,
):
    if initial_delay_s > 0:
        time.sleep(initial_delay_s)
    t0 = time.time()
    last = None
    consecutive_fail = 0
    while time.time() - t0 < timeout_s:
        try:
            st = _read_status_retry(motor, attempts=3)
            consecutive_fail = 0
        except (TimeoutError, ValueError) as e:
            consecutive_fail += 1
            # Don't abort the whole move on one bad poll — back off harder.
            if not quiet:
                print(f"  (status poll retry: {e})")
            time.sleep(min(0.6, poll_s * (1 + consecutive_fail)))
            continue
        last = st
        err = st["multi_counts"] - target_counts
        if not quiet:
            print(
                f"  pos={st['multi_counts']:8d} ({st['multi_deg']:+8.2f}°)  "
                f"err={err:+6d}  vel={st['velocity_rpm']:+7.2f} RPM  "
                f"state={st['run_state_name']}  fault={format_fault(st['fault'])}"
            )
        if st["fault"]:
            raise RuntimeError(f"motor fault during move: {format_fault(st['fault'])}")
        if abs(err) <= 20 and abs(st["velocity_rpm"]) < settle_rpm:
            return st
        time.sleep(poll_s)
    raise TimeoutError(
        f"move did not settle within {timeout_s:.1f}s "
        f"(last pos={last['multi_counts'] if last else '?'} counts)"
    )


def run_torture(
    motor: GIM8115RS485,
    degrees: float,
    speed_rpm: float,
    accel_rpm_s: float,
    decel_rpm_s: float,
    wait_s: float,
    temp_every: int,
    max_cycles: int,
    disable_after: bool,
):
    """
    Oscillate +degrees then -degrees until Ctrl+C (or --cycles N).
    One cycle = out and back. Temperature is logged every temp_every cycles.
    """
    degrees = abs(degrees)
    if degrees <= 0:
        raise ValueError("torture angle must be > 0")
    if speed_rpm <= 0:
        raise ValueError("torture --speed must be > 0")
    if temp_every < 1:
        raise ValueError("--temp-every must be >= 1")

    # Be gentle on the bus: longer gaps, slower polls. Fast hammering causes
    # truncated 0x0B replies on many USB-RS485 adapters.
    prev_gap = motor.inter_frame_gap_s
    motor.inter_frame_gap_s = max(prev_gap, 0.10)

    # Expected one-leg time (rough): angle/360 * 60/rpm, plus accel overhead.
    leg_est_s = (degrees / 360.0) * (60.0 / max(speed_rpm, 0.1)) + 1.0
    # First status poll only after most of the move should be done.
    initial_delay_s = max(0.6, leg_est_s * 0.65)
    poll_s = 0.35

    st0 = _read_status_retry(motor)
    start_counts = st0["multi_counts"]
    start_temp = st0["temp_c"]
    t0 = time.time()
    cycles = 0
    bus_glitches = 0
    temps: list[tuple[int, float, int]] = []  # cycle, elapsed_s, temp_c

    print()
    print("=" * 50)
    print("TORTURE MODE (back-and-forth)")
    print("=" * 50)
    print(f"  Angle        : ±{degrees:.2f}° per leg")
    print(f"  Speed        : {speed_rpm:.1f} RPM")
    print(f"  Accel/decel  : {accel_rpm_s:.1f} / {decel_rpm_s:.1f} RPM/s")
    print(f"  Start pos    : {start_counts} counts ({counts_to_degrees(start_counts):+.2f}°)")
    print(f"  Start temp   : {start_temp} °C")
    print(f"  Temp log     : every {temp_every} cycle(s)")
    print(f"  Bus pacing   : gap={motor.inter_frame_gap_s*1000:.0f}ms  "
          f"first-poll≈{initial_delay_s:.2f}s  poll={poll_s:.2f}s")
    if max_cycles > 0:
        print(f"  Max cycles   : {max_cycles}")
    else:
        print("  Max cycles   : until Ctrl+C")
    print("  Cycle = +angle then -angle. Keep clear of the joint.")
    print()

    def one_leg(signed_deg: float):
        """
        Command the relative move once. On bus errors, only retry status
        polling — never re-issue the move (that would stack another ±angle).
        """
        nonlocal bus_glitches
        st = _read_status_retry(motor, attempts=4)
        target = st["multi_counts"] + degrees_to_counts(signed_deg)
        motor.move_relative_degrees(
            signed_deg,
            speed_rpm=speed_rpm,
            accel_rpm_s=accel_rpm_s,
            decel_rpm_s=decel_rpm_s,
        )
        # Let the motor run — don't poll immediately after the command.
        time.sleep(0.25)

        last_err: Exception | None = None
        for attempt in range(5):
            try:
                # First attempt: wait out most of the move before polling.
                # Later attempts: motor may already be there — poll sooner.
                delay = initial_delay_s if attempt == 0 else 0.15
                return wait_move(
                    motor,
                    target,
                    timeout_s=wait_s if attempt == 0 else min(wait_s, 8.0),
                    quiet=True,
                    poll_s=poll_s,
                    initial_delay_s=delay,
                )
            except (TimeoutError, ValueError) as e:
                last_err = e
                bus_glitches += 1
                print(f" [bus glitch, retry {attempt + 1}/5]", end="", flush=True)
                try:
                    motor.ser.reset_input_buffer()
                except Exception:
                    pass
                time.sleep(0.4 * (attempt + 1))
                # If we can still read and we're near target, treat as success.
                try:
                    st = _read_status_retry(motor, attempts=3)
                    if abs(st["multi_counts"] - target) <= 40 and abs(st["velocity_rpm"]) < 2.0:
                        print(" [at target]", end="", flush=True)
                        return st
                except (TimeoutError, ValueError):
                    pass
        assert last_err is not None
        raise last_err

    try:
        while True:
            leg_t0 = time.time()
            print(f"  cycle {cycles + 1}: +{degrees:.1f}° ...", end="", flush=True)
            st = one_leg(+degrees)
            # Pause between legs so the previous reply is fully done.
            time.sleep(0.4)
            print(" back ...", end="", flush=True)
            st = one_leg(-degrees)
            cycles += 1  # only count fully completed out+back
            elapsed = time.time() - t0
            leg_dt = time.time() - leg_t0
            drift = st["multi_counts"] - start_counts
            print(
                f" done  ({leg_dt:.2f}s)  pos={st['multi_counts']}  "
                f"drift={drift:+d} counts ({counts_to_degrees(drift):+.2f}°)  "
                f"temp={st['temp_c']}°C"
            )

            if cycles % temp_every == 0:
                # Extra settle before the temp read — this is where glitches often hit.
                time.sleep(0.5)
                try:
                    motor.ser.reset_input_buffer()
                except Exception:
                    pass
                try:
                    st = _read_status_retry(motor, attempts=4)
                except (TimeoutError, ValueError) as e:
                    bus_glitches += 1
                    print(f"  *** TEMP @ cycle {cycles}: (read failed: {e}) ***")
                else:
                    temps.append((cycles, elapsed, st["temp_c"]))
                    print(
                        f"  *** TEMP @ cycle {cycles}: {st['temp_c']} °C  "
                        f"(Δ from start: {st['temp_c'] - start_temp:+d} °C, "
                        f"elapsed {elapsed:.0f}s) ***"
                    )
                # Rest after temp read before next cycle.
                time.sleep(0.5)

            if st["fault"]:
                raise RuntimeError(f"motor fault: {format_fault(st['fault'])}")

            # Brief rest between cycles.
            time.sleep(0.4)

            if max_cycles > 0 and cycles >= max_cycles:
                print(f"\n  Reached --cycles {max_cycles}.")
                break
    except KeyboardInterrupt:
        print(f"\n  Stopped by user after {cycles} complete cycle(s).")
    except (TimeoutError, ValueError, RuntimeError) as e:
        print(f"\n  Aborting after {cycles} complete cycle(s): {e}")
    finally:
        motor.inter_frame_gap_s = prev_gap
        elapsed = time.time() - t0
        st = None
        time.sleep(0.4)
        try:
            st = _read_status_retry(motor, attempts=4)
        except Exception:
            pass
        print()
        print("=" * 50)
        print("TORTURE SUMMARY")
        print("=" * 50)
        print(f"  Cycles       : {cycles}")
        print(f"  Elapsed      : {elapsed:.1f}s")
        if cycles > 0:
            print(f"  Avg cycle    : {elapsed / cycles:.2f}s")
        print(f"  Bus glitches : {bus_glitches} (retried)")
        print(f"  Start temp   : {start_temp} °C")
        if st is not None:
            print(f"  End temp     : {st['temp_c']} °C  (Δ {st['temp_c'] - start_temp:+d} °C)")
            drift = st["multi_counts"] - start_counts
            print(
                f"  End pos      : {st['multi_counts']} counts "
                f"(drift {drift:+d} / {counts_to_degrees(drift):+.2f}°)"
            )
            print(f"  Fault        : {format_fault(st['fault'])}")
        if temps:
            print("  Temp log:")
            for cyc, t_el, temp in temps:
                print(f"    cycle {cyc:5d}  t={t_el:7.1f}s  {temp} °C")
        if disable_after:
            print("  Disabling motor (cmd 0x2F)...")
            try:
                time.sleep(0.4)
                motor.disable()
            except Exception as e:
                print(f"  WARNING: disable failed: {e}")
        print("Torture done.")


def run_velocity(
    motor: GIM8115RS485,
    speed_rpm: float,
    accel_rpm_s: float,
    duration_s: float,
    until_deg: float | None,
    rate_hz: float,
    disable_after: bool,
):
    """
    Command constant velocity and stream encoder distance until Ctrl+C,
    --duration, or --until-deg traveled.
    """
    st0 = motor.read_status()
    start_counts = st0["multi_counts"]
    period = 1.0 / rate_hz if rate_hz > 0 else 0.1
    print()
    print("=" * 50)
    print("VELOCITY MODE (cmd 0x21)")
    print("=" * 50)
    print(f"  Target speed : {speed_rpm:+.2f} RPM")
    print(f"  Accel        : {'max' if accel_rpm_s <= 0 else f'{accel_rpm_s:.1f} RPM/s'}")
    print(f"  Start encoder: {start_counts} counts ({counts_to_degrees(start_counts):+.2f}°)")
    if duration_s > 0:
        print(f"  Duration     : {duration_s:.1f}s")
    else:
        print("  Duration     : until Ctrl+C")
    if until_deg is not None:
        print(f"  Stop after   : {until_deg:.1f}° traveled (|delta|)")
    print("  Streaming encoder distance — Ctrl+C to stop.")
    print()

    motor.set_velocity(speed_rpm, accel_rpm_s=accel_rpm_s)
    t0 = time.time()
    last_print = 0.0
    try:
        while True:
            now = time.time()
            st = motor.read_status()
            delta_counts = st["multi_counts"] - start_counts
            delta_deg = counts_to_degrees(delta_counts)
            delta_rev = delta_counts / COUNTS_PER_REV
            elapsed = now - t0
            if st["fault"]:
                raise RuntimeError(f"motor fault: {format_fault(st['fault'])}")
            if now - last_print >= period:
                last_print = now
                print(
                    f"  t={elapsed:6.2f}s  vel={st['velocity_rpm']:+7.2f} RPM  "
                    f"pos={st['multi_counts']:8d}  "
                    f"Δ={delta_counts:+8d} counts  "
                    f"Δ={delta_deg:+8.2f}°  "
                    f"Δ={delta_rev:+7.3f} rev  "
                    f"Iq={st['iq_a']:+6.3f}A"
                )
            if duration_s > 0 and elapsed >= duration_s:
                print(f"\n  Duration {duration_s:.1f}s reached.")
                break
            if until_deg is not None and abs(delta_deg) >= abs(until_deg):
                print(f"\n  Traveled {delta_deg:+.2f}° (>= {until_deg:.1f}°).")
                break
            time.sleep(min(period, 0.05))
    except KeyboardInterrupt:
        print("\n  Stopped by user.")
    finally:
        if disable_after:
            print("  Disabling motor (cmd 0x2F)...")
            try:
                st = motor.disable()
            except Exception as e:
                print(f"  WARNING: disable failed: {e}")
                st = motor.read_status()
        else:
            # Command zero velocity but leave enabled if requested
            try:
                motor.set_velocity(0.0, accel_rpm_s=accel_rpm_s)
                st = motor.read_status()
            except Exception:
                st = motor.read_status()
        delta_counts = st["multi_counts"] - start_counts
        print(
            f"  Final: Δ={delta_counts:+d} counts "
            f"({counts_to_degrees(delta_counts):+.2f}°, "
            f"{delta_counts / COUNTS_PER_REV:+.3f} rev)  "
            f"vel={st['velocity_rpm']:+.2f} RPM"
        )


def run_bus_test(
    motor: GIM8115RS485,
    rate_hz: float,
    duration_s: float,
    gap_s: float | None = None,
):
    """
    Continuously poll status (cmd 0x0B) with no motion — stress-test the RS485 link.
    Reports OK/fail counts and recent error strings until Ctrl+C or --duration.
    """
    prev_gap = motor.inter_frame_gap_s
    if gap_s is not None:
        motor.inter_frame_gap_s = gap_s

    period = 1.0 / rate_hz if rate_hz > 0 else 0.2
    # Don't request faster than the gap allows.
    period = max(period, motor.inter_frame_gap_s + 0.02)

    print()
    print("=" * 50)
    print("BUS TEST (status poll only, no motion)")
    print("=" * 50)
    print(f"  Poll rate    : ~{1.0 / period:.1f} Hz  (period {period*1000:.0f}ms)")
    print(f"  Inter-frame  : {motor.inter_frame_gap_s*1000:.0f}ms")
    if duration_s > 0:
        print(f"  Duration     : {duration_s:.1f}s")
    else:
        print("  Duration     : until Ctrl+C")
    print("  Command      : 0x0B read status")
    print("  Note         : PDF has no poll-rate recommendation; default baud 115200.")
    print("                 cmd 0x2D is a host-watchdog (default 5000ms), not a max Hz.")
    print()

    ok = 0
    fail = 0
    t0 = time.time()
    last_report = t0
    last_err = ""
    err_counts: dict[str, int] = {}
    consecutive_fail = 0
    max_consecutive_fail = 0

    try:
        while True:
            now = time.time()
            try:
                # Single attempt — measure raw link quality (no hidden retries).
                st = motor.read_status(retries=0)
                if consecutive_fail:
                    print(f"  RECOVERED after {consecutive_fail} fails")
                ok += 1
                consecutive_fail = 0
            except (TimeoutError, ValueError, OSError) as e:
                fail += 1
                consecutive_fail += 1
                max_consecutive_fail = max(max_consecutive_fail, consecutive_fail)
                last_err = str(e)
                key = last_err.split("(")[0].strip()[:60]
                err_counts[key] = err_counts.get(key, 0) + 1
                print(f"  FAIL #{ok + fail:<6d}  {last_err}")
                # Half-duplex recovery: wait for late bytes, then quiet the line.
                motor._drain_rx(
                    quiet_s=0.05 + 0.03 * min(consecutive_fail, 6),
                    max_s=0.4 + 0.1 * min(consecutive_fail, 6),
                )
                st = None

            # Same report cadence for all rates (avoids 5Hz looking "faster" than 10Hz
            # just because it printed every poll while 10Hz summarized once/sec).
            if time.time() - last_report >= 1.0:
                total = ok + fail
                elapsed = time.time() - t0
                pct = 100.0 * ok / total if total else 0.0
                eff = total / elapsed if elapsed > 0 else 0.0
                if st is not None:
                    tail = f"last: pos={st['multi_counts']} temp={st['temp_c']}C"
                else:
                    tail = f"last_err={last_err[:40]}" if last_err else "last: (fail)"
                print(
                    f"  … {total} polls  OK={ok}  FAIL={fail}  "
                    f"success={pct:.1f}%  eff={eff:.1f}Hz (req {1.0 / period:.1f})  "
                    f"streak_fail_max={max_consecutive_fail}  {tail}"
                )
                last_report = time.time()

            if duration_s > 0 and (time.time() - t0) >= duration_s:
                print(f"\n  Duration {duration_s:.1f}s reached.")
                break
            # Pace to the requested period (transact already waits inter_frame_gap).
            # After fails, enforce a minimum backoff so we don't hammer a wedged adapter.
            elapsed_cmd = time.time() - now
            min_period = period
            if consecutive_fail:
                min_period = max(period, 0.15 * consecutive_fail)
            sleep_for = min_period - elapsed_cmd
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        print("\n  Stopped by user.")
    finally:
        motor.inter_frame_gap_s = prev_gap
        elapsed = time.time() - t0
        total = ok + fail
        pct = 100.0 * ok / total if total else 0.0
        print()
        print("=" * 50)
        print("BUS TEST SUMMARY")
        print("=" * 50)
        print(f"  Elapsed      : {elapsed:.1f}s")
        print(f"  Polls        : {total}  (OK={ok}  FAIL={fail})")
        print(f"  Success rate : {pct:.2f}%")
        print(f"  Effective Hz : {total / elapsed:.2f}" if elapsed > 0 else "  Effective Hz : n/a")
        print(f"  Max fail streak: {max_consecutive_fail}")
        if err_counts:
            print("  Error breakdown:")
            for msg, n in sorted(err_counts.items(), key=lambda kv: -kv[1]):
                print(f"    {n:5d}  {msg}")
        print("Bus test done.")


def scan_addresses(ser: serial.Serial, start: int = 1, end: int = 16) -> list[int]:
    found = []
    for addr in range(start, end + 1):
        sys.stdout.write(f"\rScanning address {addr}/{end} ...")
        sys.stdout.flush()
        motor = GIM8115RS485(ser, addr)
        if motor.ping():
            found.append(addr)
            print(f"\r  Found device at address {addr}          ")
        time.sleep(0.05)
    # Public address 0xFF can help when the real address is unknown (single motor only)
    sys.stdout.write("\rTrying public address 0xFF ...")
    sys.stdout.flush()
    motor = GIM8115RS485(ser, 0xFF)
    if motor.ping():
        print("\r  Public address 0xFF answered (single-motor bus only)          ")
        if not found:
            found.append(0xFF)
    else:
        print("\r" + " " * 40 + "\r", end="")
    return found


def _tx_frame(ser: serial.Serial, frame: bytes, rts_de: bool, turnaround_s: float = 0.002):
    """Write one frame; optionally drive RTS as RS485 DE (data enable)."""
    ser.reset_input_buffer()
    if rts_de:
        ser.rts = True
        time.sleep(0.001)
    ser.write(frame)
    ser.flush()
    # Wait for the last bit to leave the UART before releasing DE / listening.
    bit_time = 10.0 / max(ser.baudrate, 1)  # start + 8 data + stop
    time.sleep(bit_time * len(frame) + turnaround_s)
    if rts_de:
        ser.rts = False
        time.sleep(0.001)


def raw_exchange(ser: serial.Serial, addr: int, rts_de: bool, listen_s: float = 0.3) -> tuple[bytes, bytes]:
    motor = GIM8115RS485(ser, addr)
    frame = motor.build_frame(CMD_READ_STATUS, seq=0)
    _tx_frame(ser, frame, rts_de=rts_de)
    deadline = time.time() + listen_s
    rx = bytearray()
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            rx.extend(chunk)
        else:
            time.sleep(0.01)
    return frame, bytes(rx)


def interpret_rx(rx: bytes) -> None:
    if not rx:
        print("  Interpret: empty RX — no electrical reply. Check power, A/B, GND, DE/RE.")
        return
    if rx == b"\x00" or set(rx) == {0}:
        print("  Interpret: only 0x00 — usually idle/noise on a floating bus, NOT a motor reply.")
        print("             A real reply is ~29 bytes starting with AC (e.g. AC xx 01 0B 16 ...).")
        print("             Next: swap A/B, confirm motor power + common GND, try --rts-de, try --probe.")
        return
    if HDR_HOST in rx and HDR_SLAVE not in rx:
        print("  Interpret: saw host header 0xAE in RX — adapter is echoing TX (half-duplex echo).")
        print("             Echo alone is OK; we still need an 0xAC reply after it.")
    if HDR_SLAVE in rx:
        i = rx.index(HDR_SLAVE)
        print(f"  Interpret: found slave header 0xAC at offset {i} — motor is talking.")
        print(f"             candidate: {rx[i:i+32].hex(' ')}")
    else:
        print("  Interpret: bytes received but no 0xAC — wrong baud, A/B swapped, or not this protocol.")


def probe_bus(port: str, timeout: float, rts_de: bool) -> None:
    """Try common bauds × addresses and dump raw RX for each."""
    bauds = (115200, 460800, 921600, 57600, 38400, 19200, 9600)
    addrs = (1, 2, 3, 255)
    print("=== Bus probe (baud × address) ===")
    print("Looking for RX containing AC (slave header). TX frame is always cmd 0x0B status.")
    hits = []
    for baud in bauds:
        try:
            ser = serial.Serial(
                port=port, baudrate=baud, bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                timeout=0.05, write_timeout=timeout,
            )
        except serial.SerialException as e:
            print(f"  {baud}: cannot open ({e})")
            continue
        try:
            if rts_de:
                ser.rts = False
            for addr in addrs:
                tx, rx = raw_exchange(ser, addr, rts_de=rts_de, listen_s=0.15)
                mark = ""
                if HDR_SLAVE in rx:
                    mark = "  <-- AC seen"
                    hits.append((baud, addr, rx))
                elif rx and set(rx) != {0}:
                    mark = "  (noise/echo?)"
                print(f"  baud={baud:6d} addr={addr:3d}  TX={tx.hex(' ')}  "
                      f"RX={rx.hex(' ') if rx else '(empty)'}{mark}")
        finally:
            ser.close()
    if hits:
        print("\nHits (AC in RX):")
        for baud, addr, rx in hits:
            print(f"  try: python {sys.argv[0]} -p {port} -b {baud} -a {addr} --info"
                  + (" --rts-de" if rts_de else ""))
            print(f"       RX={rx.hex(' ')}")
    else:
        print("\nNo AC reply on any baud/address.")
        print("Hardware checklist (LED blinking ≠ RS485 working):")
        print("  1. Confirm driver series on the board silkscreen:")
        print("       GDZ*  = RS485 OK (this script)")
        print("       GDS*  = usually CAN / CP2102 — NOT this RS485 protocol")
        print("       GDM*  = MIT / USB2CAN — NOT this RS485 protocol")
        print("  2. Wire THREE wires: adapter A↔motor A, B↔motor B, GND↔motor GND")
        print("     (A/B alone without common GND often gives RX: 00 / silence)")
        print("  3. Swap A and B, then re-run --probe")
        print("  4. Retry with RTS as DE:  --probe --rts-de")
        print("  5. SteadyWin notes many 3rd-party USB-485 dongles are incompatible with GDZ;")
        print("     their own USB-to-485 adapter is the known-good option")
        print("  6. Do not use the CANH/CANL pins by mistake — need the RS485 pair")


def open_serial(port: str, baud: int, timeout: float, rts_de: bool) -> serial.Serial:
    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=timeout,
        write_timeout=timeout,
    )
    if rts_de:
        ser.rts = False
    return ser


def main():
    parser = argparse.ArgumentParser(
        description="GIM8115-9 RS485 test (SteadyWin custom protocol, relative angle move)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
examples:
  python gim8115_rs485_test.py --port COM9 --info
  python gim8115_rs485_test.py --port COM9 --move 90 --speed 10
  python gim8115_rs485_test.py --port COM9 --velocity 20
  python gim8115_rs485_test.py --port COM9 --torture 90 --speed 20
  python gim8115_rs485_test.py --port COM9 --torture 180 --speed 30 --temp-every 10
  python gim8115_rs485_test.py --port COM9 --bus-test
  python gim8115_rs485_test.py --port COM9 --bus-test --rate 5 --duration 60
  python gim8115_rs485_test.py --port COM9 --probe

This is NOT Modbus. The PDF protocol uses header 0xAE/0xAC frames.

--torture DEG: back-and-forth ±DEG at --speed until Ctrl+C (or --cycles N).
One cycle = out + back. Temperature is printed every --temp-every cycles.

--bus-test: poll status (0x0B) only — no motion. Use to measure RS485 reliability.
Stop with Ctrl+C or --duration. --rate sets poll Hz.
""",
    )
    parser.add_argument("-p", "--port", required=True,
                        help="Serial port (Windows: COM9, Linux: /dev/ttyUSB0)")
    parser.add_argument("-b", "--baud", type=int, default=115200,
                        help="Baud rate (default: 115200 per PDF)")
    parser.add_argument("-a", "--addr", type=int, default=1,
                        help="Device address 1-254 (default: 1; PDF default). "
                             "Use 255/0xFF as public address for a single motor.")
    parser.add_argument("-t", "--timeout", type=float, default=0.5,
                        help="Reply timeout seconds (default: 0.5)")
    parser.add_argument("--scan", action="store_true",
                        help="Scan addresses 1-16 (+ public 0xFF) and exit")
    parser.add_argument("--probe", action="store_true",
                        help="Try common bauds × addresses with raw TX/RX dump and exit")
    parser.add_argument("--info", action="store_true",
                        help="Read identity/status and exit (default if no motion command)")
    parser.add_argument("--move", type=float, default=None, metavar="DEG",
                        help="Relative move in degrees (positive/negative)")
    parser.add_argument("--velocity", type=float, default=None, metavar="RPM",
                        help="Velocity mode (cmd 0x21): run at this signed RPM while "
                             "streaming encoder distance (Ctrl+C / --duration / --until-deg to stop)")
    parser.add_argument("--torture", type=float, default=None, metavar="DEG",
                        help="Torture mode: oscillate ±DEG at --speed until Ctrl+C "
                             "(or --cycles). Counts cycles; logs temp every --temp-every cycles.")
    parser.add_argument("--bus-test", action="store_true",
                        help="Poll status (cmd 0x0B) continuously with no motion — "
                             "stress-test RS485 reliability. Stop with Ctrl+C or --duration.")
    parser.add_argument("--speed", type=float, default=30.0, metavar="RPM",
                        help="With --move/--torture: max trapezoid speed in RPM (default: 30). "
                             "Use --speed 0 with --move for simple cmd 0x23. Ignored by --velocity.")
    parser.add_argument("--accel", type=float, default=None, metavar="RPM_S",
                        help="Accel in RPM/s. For --move/--torture: trapezoid accel (default=speed). "
                             "For --velocity: ramp accel; 0 or omit = max accel (PDF).")
    parser.add_argument("--decel", type=float, default=None, metavar="RPM_S",
                        help="With --move/--torture: trapezoid decel in RPM/s (default: same as --accel)")
    parser.add_argument("--cycles", type=int, default=0, metavar="N",
                        help="With --torture: stop after N cycles (default: 0 = until Ctrl+C)")
    parser.add_argument("--temp-every", type=int, default=10, metavar="N",
                        help="With --torture: print temperature every N cycles (default: 10)")
    parser.add_argument("--duration", type=float, default=0.0, metavar="SEC",
                        help="With --velocity/--bus-test: stop after N seconds "
                             "(default: 0 = until Ctrl+C)")
    parser.add_argument("--until-deg", type=float, default=None, metavar="DEG",
                        help="With --velocity: stop after |encoder delta| reaches this many degrees")
    parser.add_argument("--rate", type=float, default=10.0, metavar="HZ",
                        help="With --velocity/--bus-test: poll/print rate in Hz (default: 10)")
    parser.add_argument("--gap", type=float, default=None, metavar="SEC",
                        help="With --bus-test: override inter-frame gap seconds "
                             "(default: motor class default)")
    parser.add_argument("--wait", type=float, default=15.0, metavar="SEC",
                        help="With --move/--torture: seconds to wait for each leg to settle (default: 15)")
    parser.add_argument("--no-disable", action="store_true",
                        help="Leave motor enabled after motion (velocity: command 0 RPM instead)")
    parser.add_argument("--clear-fault", action="store_true",
                        help="Clear fault latch before moving")
    parser.add_argument("--raw", action="store_true",
                        help="Print raw TX/RX hex for the first status read (debug)")
    parser.add_argument("--rts-de", action="store_true",
                        help="Drive RTS high while transmitting (for USB-RS485 adapters "
                             "that use RTS as DE/RE direction control)")
    args = parser.parse_args()

    motion_cmds = sum(x is not None for x in (args.move, args.velocity, args.torture))
    if motion_cmds > 1:
        print("ERROR: use only one of --move, --velocity, or --torture", file=sys.stderr)
        sys.exit(2)
    if args.bus_test and motion_cmds:
        print("ERROR: --bus-test cannot be combined with --move/--velocity/--torture",
              file=sys.stderr)
        sys.exit(2)

    if (args.move is None and args.velocity is None and args.torture is None
            and not args.scan and not args.probe and not args.bus_test):
        args.info = True

    print(f"Port: {args.port}  Baud: {args.baud}  Addr: {args.addr}"
          + ("  RTS-as-DE: on" if args.rts_de else ""))
    print("Protocol: SteadyWin custom RS485 (0xAE/0xAC) — NOT Modbus RTU")
    print("Docs: docs/Motor_GIM8115-9/自定义RS485通信协议_3.03b0.pdf")

    if args.probe:
        probe_bus(args.port, args.timeout, rts_de=args.rts_de)
        return

    try:
        ser = open_serial(args.port, args.baud, args.timeout, rts_de=args.rts_de)
    except serial.SerialException as e:
        print(f"Cannot open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    # Monkey-patch write path when RTS-DE is needed: wrap motor.transact via subclassing write.
    if args.rts_de:
        _orig_write = ser.write

        def _write_with_de(data):
            # pyserial write may be called with bytes; assert DE around the whole write.
            ser.rts = True
            time.sleep(0.001)
            n = _orig_write(data)
            bit_time = 10.0 / max(ser.baudrate, 1)
            time.sleep(bit_time * max(len(data), 1) + 0.002)
            ser.rts = False
            time.sleep(0.001)
            return n

        ser.write = _write_with_de  # type: ignore[method-assign]

    try:
        if args.scan:
            found = scan_addresses(ser)
            print(f"Found motors at addresses: {found if found else 'none'}")
            if not found:
                print("Check:")
                print("  - RS485 A/B polarity (try swapping), common GND, motor powered")
                print("  - Baud 115200 8N1 (PDF default)")
                print("  - Green LED blink pattern = device address")
                print("  - Try: --probe   and/or   --rts-de")
                print("  - This script uses the custom protocol, not Modbus")
            return

        motor = GIM8115RS485(ser, args.addr)

        if args.raw:
            print()
            motor_raw = GIM8115RS485(ser, args.addr)
            tx = motor_raw.build_frame(CMD_READ_STATUS, seq=0)
            ser.reset_input_buffer()
            # ser.write may already be wrapped with --rts-de
            if args.rts_de:
                ser.write(tx)
            else:
                _tx_frame(ser, tx, rts_de=False)
            deadline = time.time() + max(0.3, args.timeout)
            rx = bytearray()
            while time.time() < deadline:
                chunk = ser.read(64)
                if chunk:
                    rx.extend(chunk)
                else:
                    time.sleep(0.01)
            rx = bytes(rx)
            print(f"TX: {tx.hex(' ')}")
            print(f"RX: {rx.hex(' ') if rx else '(empty)'}  ({len(rx)} bytes)")
            interpret_rx(rx)
            if HDR_SLAVE not in rx:
                print("\nNo valid slave frame. Run a wider search:")
                print(f"  python {sys.argv[0]} -p {args.port} --probe")
                print(f"  python {sys.argv[0]} -p {args.port} --probe --rts-de")
                sys.exit(1)

        if not motor.ping():
            print(f"No response from address {args.addr}.", file=sys.stderr)
            print("Check:", file=sys.stderr)
            print("  - Motor powered; RS485 A/B (swap if silent); common GND", file=sys.stderr)
            print("  - Baud 115200 8N1 (PDF default)", file=sys.stderr)
            print("  - Address (LED blink / try --scan / try -a 255 public addr)", file=sys.stderr)
            print(f"  - python {sys.argv[0]} -p {args.port} --probe", file=sys.stderr)
            print(f"  - python {sys.argv[0]} -p {args.port} --probe --rts-de", file=sys.stderr)
            print("  - Do NOT use Modbus RTU against a stock GIM8115-9 — it uses 0xAE frames", file=sys.stderr)
            sys.exit(1)

        if args.bus_test:
            print_info(motor)
            run_bus_test(
                motor,
                rate_hz=args.rate,
                duration_s=args.duration,
                gap_s=args.gap,
            )
            return

        if args.info or args.move is not None or args.velocity is not None or args.torture is not None:
            print_info(motor)

        if args.move is None and args.velocity is None and args.torture is None:
            print("\nMotor is responding. Use --move DEG, --velocity RPM, --torture DEG, or --bus-test.")
            return

        if args.clear_fault:
            print("\nClearing fault...")
            code = motor.clear_fault()
            print(f"  Fault after clear: {format_fault(code)}")

        st0 = motor.read_status()
        if st0["fault"]:
            print(f"\nWARNING: fault present ({format_fault(st0['fault'])}). "
                  f"Retry with --clear-fault if safe.")

        if args.velocity is not None:
            # For velocity mode, omit --accel => max accel (0). Explicit value = RPM/s.
            vel_accel = 0.0 if args.accel is None else args.accel
            run_velocity(
                motor,
                speed_rpm=args.velocity,
                accel_rpm_s=vel_accel,
                duration_s=args.duration,
                until_deg=args.until_deg,
                rate_hz=args.rate,
                disable_after=not args.no_disable,
            )
            return

        if args.torture is not None:
            accel = args.accel if args.accel is not None else args.speed
            decel = args.decel if args.decel is not None else accel
            # Longer wait for large/slow torture legs
            wait_s = max(args.wait, abs(args.torture) / max(args.speed, 0.1) * 60.0 / 360.0 * 3.0 + 5.0)
            run_torture(
                motor,
                degrees=args.torture,
                speed_rpm=args.speed,
                accel_rpm_s=accel,
                decel_rpm_s=decel,
                wait_s=wait_s,
                temp_every=args.temp_every,
                max_cycles=args.cycles,
                disable_after=not args.no_disable,
            )
            return

        counts = degrees_to_counts(args.move)
        target = st0["multi_counts"] + counts
        use_trap = args.speed > 0
        accel = args.accel if args.accel is not None else args.speed
        decel = args.decel if args.decel is not None else accel
        print()
        print("=" * 50)
        print("RELATIVE MOVE (cmd 0x26 trapezoid)" if use_trap else "RELATIVE MOVE (cmd 0x23 simple)")
        print("=" * 50)
        print(f"  Command      : {args.move:+.2f}°  ({counts:+d} counts)")
        print(f"  Start        : {st0['multi_counts']} counts ({st0['multi_deg']:+.2f}°)")
        print(f"  Target       : {target} counts ({counts_to_degrees(target):+.2f}°)")
        if use_trap:
            print(f"  Max speed    : {args.speed:.1f} RPM")
            print(f"  Accel/decel  : {accel:.1f} / {decel:.1f} RPM/s")
        else:
            print("  Speed        : driver default (simple 0x23)")
        print("  Keep clear of the joint — motor will move.")

        print("\n  Sending relative position command...")
        if use_trap:
            motor.move_relative_degrees(
                args.move, speed_rpm=args.speed, accel_rpm_s=accel, decel_rpm_s=decel
            )
        else:
            motor.move_relative_degrees(args.move)

        print("  Waiting for settle...")
        try:
            st1 = wait_move(motor, target, timeout_s=args.wait)
            print(f"\n  Settled at {st1['multi_counts']} counts ({st1['multi_deg']:+.2f}°)")
            print("Move OK.")
        finally:
            if not args.no_disable:
                print("  Disabling motor (cmd 0x2F)...")
                try:
                    motor.disable()
                except Exception as e:
                    print(f"  WARNING: disable failed: {e}")

    except KeyboardInterrupt:
        print("\nStopped by user.")
        try:
            GIM8115RS485(ser, args.addr).disable()
        except Exception:
            pass
        sys.exit(130)
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        try:
            GIM8115RS485(ser, args.addr).disable()
        except Exception:
            pass
        sys.exit(1)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
