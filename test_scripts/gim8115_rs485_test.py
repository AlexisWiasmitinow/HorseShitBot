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
    python gim8115_rs485_test.py --port COM9 --velocity 15 --until-deg 360
    python gim8115_rs485_test.py --port COM9 --velocity -10 --duration 5
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
        buf = bytearray()
        while len(buf) < n:
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            self.ser.timeout = remaining
            chunk = self.ser.read(n - len(buf))
            if not chunk:
                break
            buf.extend(chunk)
        return bytes(buf)

    def _wait_bus_idle(self):
        gap = self.inter_frame_gap_s - (time.time() - self._last_tx_end)
        if gap > 0:
            time.sleep(gap)

    def transact(
        self,
        cmd: int,
        payload: bytes = b"",
        timeout: float | None = None,
        retries: int = 2,
    ) -> bytes:
        """Send a command and return the full reply frame (including header/CRC)."""
        if timeout is None:
            timeout = self.ser.timeout if self.ser.timeout else 0.5
        last_err: Exception | None = None
        for attempt in range(retries + 1):
            try:
                return self._transact_once(cmd, payload, timeout)
            except (TimeoutError, ValueError) as e:
                last_err = e
                # Drain garbage, back off, retry — common after fast polling / direction switch.
                try:
                    self.ser.reset_input_buffer()
                except Exception:
                    pass
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
        # Sync to slave header 0xAC (ignore idle 0x00 / noise)
        while time.time() < deadline:
            b = self._read_exact(1, deadline)
            if not b:
                break
            if b[0] == HDR_SLAVE:
                hdr = b
                break
        else:
            raise TimeoutError(
                f"no reply to cmd 0x{cmd:02X} (sent {frame.hex(' ')}). "
                f"Check port/wiring/baud/address, and that the motor is on the custom RS485 protocol."
            )

        rest_head = self._read_exact(4, deadline)  # seq, addr, cmd, len
        if len(rest_head) < 4:
            raise TimeoutError(f"truncated reply header for cmd 0x{cmd:02X}")
        length = rest_head[3]
        rest = self._read_exact(length + 2, deadline)  # payload + CRC
        if len(rest) < length + 2:
            raise TimeoutError(f"truncated reply payload for cmd 0x{cmd:02X}")

        reply = hdr + rest_head + rest
        body, crc_rx = reply[:-2], reply[-2:]
        crc_calc = crc16_modbus(body)
        crc_got = struct.unpack("<H", crc_rx)[0]
        if crc_got != crc_calc:
            raise ValueError(
                f"CRC mismatch on reply to 0x{cmd:02X}: got 0x{crc_got:04X}, "
                f"calc 0x{crc_calc:04X}, frame={reply.hex(' ')}"
            )
        if reply[3] != cmd:
            raise ValueError(f"reply cmd mismatch: expected 0x{cmd:02X}, got 0x{reply[3]:02X}")
        return reply

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

    def read_status(self) -> dict:
        reply = self.transact(CMD_READ_STATUS)
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


def wait_move(motor: GIM8115RS485, target_counts: int, timeout_s: float, settle_rpm: float = 1.0):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout_s:
        st = motor.read_status()
        last = st
        err = st["multi_counts"] - target_counts
        print(
            f"  pos={st['multi_counts']:8d} ({st['multi_deg']:+8.2f}°)  "
            f"err={err:+6d}  vel={st['velocity_rpm']:+7.2f} RPM  "
            f"state={st['run_state_name']}  fault={format_fault(st['fault'])}"
        )
        if st["fault"]:
            raise RuntimeError(f"motor fault during move: {format_fault(st['fault'])}")
        if abs(err) <= 20 and abs(st["velocity_rpm"]) < settle_rpm:
            return st
        time.sleep(0.1)
    raise TimeoutError(
        f"move did not settle within {timeout_s:.1f}s "
        f"(last pos={last['multi_counts'] if last else '?'} counts)"
    )


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
  python gim8115_rs485_test.py --port COM9 --velocity 15 --until-deg 360 --accel 30
  python gim8115_rs485_test.py --port COM9 --velocity -10 --duration 5
  python gim8115_rs485_test.py --port COM9 --probe

This is NOT Modbus. The PDF protocol uses header 0xAE/0xAC frames.

Velocity mode (--velocity) is cmd 0x21: run at constant RPM while streaming
encoder distance (good for tracked-robot drive tests). Ctrl+C stops and disables.
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
    parser.add_argument("--speed", type=float, default=30.0, metavar="RPM",
                        help="With --move: max trapezoid speed in RPM (default: 30). "
                             "Use --speed 0 for simple cmd 0x23. Ignored by --velocity.")
    parser.add_argument("--accel", type=float, default=None, metavar="RPM_S",
                        help="Accel in RPM/s. For --move: trapezoid accel (default=speed). "
                             "For --velocity: ramp accel; 0 or omit = max accel (PDF).")
    parser.add_argument("--decel", type=float, default=None, metavar="RPM_S",
                        help="With --move: trapezoid decel in RPM/s (default: same as --accel)")
    parser.add_argument("--duration", type=float, default=0.0, metavar="SEC",
                        help="With --velocity: stop after N seconds (default: 0 = until Ctrl+C)")
    parser.add_argument("--until-deg", type=float, default=None, metavar="DEG",
                        help="With --velocity: stop after |encoder delta| reaches this many degrees")
    parser.add_argument("--rate", type=float, default=10.0, metavar="HZ",
                        help="With --velocity: status print rate in Hz (default: 10)")
    parser.add_argument("--wait", type=float, default=15.0, metavar="SEC",
                        help="With --move: seconds to wait for settle (default: 15)")
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

    if (args.move is None and args.velocity is None
            and not args.scan and not args.probe):
        args.info = True

    if args.move is not None and args.velocity is not None:
        print("ERROR: use either --move or --velocity, not both", file=sys.stderr)
        sys.exit(2)

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

        if args.info or args.move is not None or args.velocity is not None:
            print_info(motor)

        if args.move is None and args.velocity is None:
            print("\nMotor is responding. Use --move DEG or --velocity RPM.")
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
