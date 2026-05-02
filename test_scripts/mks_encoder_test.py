#!/usr/bin/env python3
"""
Quick test: read encoder from an MKS SERVO57D over RS485 using Modbus RTU.

Requires Modbus RTU enabled on the motor (menu: Mb_RTU -> Enable).

Usage:
    python3 mks_encoder_test.py                          # defaults
    python3 mks_encoder_test.py -p /dev/ttyUSB1          # different port
    python3 mks_encoder_test.py -b 115200 -a 2           # different baud/address
    python3 mks_encoder_test.py --scan                   # try addresses 1-16
"""

import argparse
import struct
import sys
import time

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient


def make_client(port: str, baud: int, timeout: float) -> ModbusSerialClient:
    return ModbusSerialClient(
        port=port, framer=FramerType.RTU,
        baudrate=baud, bytesize=8, parity="N", stopbits=1,
        timeout=timeout, retries=0,
    )


def _unit_kw(client, unit_id: int) -> dict:
    return {"device_id": unit_id}


def read_encoder_carry(client, addr: int):
    """Function 04, register 0x30, 3 regs -> carry(i32) + value(u16)."""
    rr = client.read_input_registers(address=0x30, count=3, **_unit_kw(client, addr))
    if rr.isError():
        return None
    regs = rr.registers
    carry = struct.unpack(">i", struct.pack(">HH", regs[0], regs[1]))[0]
    value = regs[2]
    return carry, value


def read_encoder_addition(client, addr: int):
    """Function 04, register 0x31, 3 regs -> encoder cumulative (int48)."""
    rr = client.read_input_registers(address=0x31, count=3, **_unit_kw(client, addr))
    if rr.isError():
        return None
    raw_bytes = struct.pack(">HHH", *rr.registers)
    return int.from_bytes(raw_bytes, byteorder="big", signed=True)


def read_speed(client, addr: int):
    """Function 04, register 0x32, 1 reg -> speed (int16 RPM)."""
    rr = client.read_input_registers(address=0x32, count=1, **_unit_kw(client, addr))
    if rr is None or rr.isError():
        return None
    return struct.unpack(">h", struct.pack(">H", rr.registers[0]))[0]


def read_motor_status(client, addr: int):
    """Function 04, register 0xF1, 1 reg -> motor status."""
    rr = client.read_input_registers(address=0xF1, count=1, **_unit_kw(client, addr))
    if rr is None or rr.isError():
        return None
    status_map = {0: "fail", 1: "stopped", 2: "accel", 3: "decel", 4: "full_speed", 5: "homing/cal"}
    val = rr.registers[0] & 0xFF
    return val, status_map.get(val, f"unknown({val})")


def read_enable_status(client, addr: int):
    """Function 04, register 0x3A, 1 reg -> enable status."""
    rr = client.read_input_registers(address=0x3A, count=1, **_unit_kw(client, addr))
    if rr is None or rr.isError():
        return None
    return bool(rr.registers[0] & 0xFF)


def test_one(client, addr: int) -> bool:
    """Run all read tests on a single address. Returns True if motor responded."""
    print(f"\n--- Address {addr} ---")

    try:
        enc = read_encoder_carry(client, addr)
    except Exception:
        enc = None
    if enc is None:
        print("  No response")
        return False
    carry, value = enc
    total = carry * 0x4000 + value
    degrees = (total / 0x4000) * 360
    print(f"  Encoder carry : carry={carry}, value=0x{value:04X} (total={total}, ~{degrees:.1f} deg)")

    try:
        enc_add = read_encoder_addition(client, addr)
    except Exception:
        enc_add = None
    if enc_add is not None:
        deg2 = (enc_add / 0x4000) * 360
        print(f"  Encoder addit : {enc_add} (~{deg2:.1f} deg)")

    try:
        spd = read_speed(client, addr)
    except Exception:
        spd = None
    if spd is not None:
        print(f"  Speed         : {spd} RPM")

    try:
        en = read_enable_status(client, addr)
    except Exception:
        en = None
    if en is not None:
        print(f"  Enabled       : {en}")

    try:
        status = read_motor_status(client, addr)
    except Exception:
        status = None
    if status is not None:
        print(f"  Motor status  : {status[0]} ({status[1]})")

    return True


def main():
    parser = argparse.ArgumentParser(description="MKS SERVO57D RS485 encoder test (Modbus RTU)")
    parser.add_argument("-p", "--port", default="/dev/mksbus", help="Serial port (default: /dev/mksbus)")
    parser.add_argument("-b", "--baud", type=int, default=38400, help="Baud rate (default: 38400)")
    parser.add_argument("-a", "--addr", type=int, default=1, help="Slave address 1-255 (default: 1)")
    parser.add_argument("-s", "--scan", action="store_true", help="Scan addresses 1-16")
    parser.add_argument("-t", "--timeout", type=float, default=0.3, help="Response timeout in seconds (default: 0.3)")
    args = parser.parse_args()

    print(f"Port: {args.port}  Baud: {args.baud}  Timeout: {args.timeout}s  (Modbus RTU)")

    client = make_client(args.port, args.baud, args.timeout)
    if not client.connect():
        print(f"Cannot open {args.port}", file=sys.stderr)
        sys.exit(1)

    try:
        if args.scan:
            found = []
            for addr in range(1, 17):
                sys.stdout.write(f"\rScanning address {addr}/16 ...")
                sys.stdout.flush()
                if test_one(client, addr):
                    found.append(addr)
                time.sleep(0.05)
            print(f"\n\nFound motors at addresses: {found if found else 'none'}")
        else:
            if not test_one(client, args.addr):
                print(f"\nNo response from address {args.addr}.")
                print("Check:")
                print("  - Mb_RTU is enabled on the motor (menu: Mb_RTU -> Enable)")
                print("  - Wiring (A+/B- and GND)")
                print(f"  - Baud rate matches motor (menu: UartBaud, default 38400)")
                print(f"  - Address matches motor (menu: UartAddr, default 01)")
                print("  - Motor is powered on")
                sys.exit(1)
            else:
                print("\nMotor is responding.")
    finally:
        client.close()


if __name__ == "__main__":
    main()
