#!/usr/bin/env python3
"""
Door endurance test: cycles +4 / -4 turns until Ctrl-C.

Usage:
    python3 test_scripts/door_endurance.py
    python3 test_scripts/door_endurance.py -p /dev/ttyUSB0 -a 6 --turns 4 --speed 300 --acc 100
"""

import argparse
import struct
import sys
import time

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient

ENCODER_CPR = 0x4000  # 16384 counts/rev
REG_POS_REL = 0x00F4
REG_ENABLE = 0x00F3
REG_WORKMODE = 0x0082
MODE_SR_CLOSE = 4


def make_client(port: str, baud: int, timeout: float) -> ModbusSerialClient:
    return ModbusSerialClient(
        port=port, framer=FramerType.RTU,
        baudrate=baud, bytesize=8, parity="N", stopbits=1,
        timeout=timeout, retries=1,
    )


def split_i32(val: int):
    val &= 0xFFFFFFFF
    return [(val >> 16) & 0xFFFF, val & 0xFFFF]


def read_motor_status(client, addr: int):
    rr = client.read_input_registers(address=0xF1, count=1, device_id=addr)
    if rr is None or rr.isError():
        return None
    return rr.registers[0] & 0xFF


def move_rel(client, addr: int, counts: int, speed: int, acc: int):
    hi, lo = split_i32(counts)
    client.write_registers(
        address=REG_POS_REL,
        values=[acc, speed, hi, lo],
        device_id=addr,
    )


def wait_move_done(client, addr: int, timeout: float = 30.0):
    """Poll motor status until stopped (1) or timeout."""
    t0 = time.time()
    time.sleep(0.3)
    while time.time() - t0 < timeout:
        st = read_motor_status(client, addr)
        if st == 1:  # stopped
            return True
        time.sleep(0.2)
    return False


def main():
    parser = argparse.ArgumentParser(description="Door endurance test")
    parser.add_argument("-p", "--port", default="/dev/mksbus")
    parser.add_argument("-b", "--baud", type=int, default=115200)
    parser.add_argument("-a", "--addr", type=int, default=6)
    parser.add_argument("--turns", type=float, default=4.0)
    parser.add_argument("--speed", type=int, default=3000)
    parser.add_argument("--acc", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=0.3)
    args = parser.parse_args()

    counts = int(round(args.turns * ENCODER_CPR))

    print("=" * 50)
    print("  DOOR ENDURANCE TEST")
    print("=" * 50)
    print(f"  Motor addr : {args.addr}")
    print(f"  Turns      : ±{args.turns} ({counts} counts)")
    print(f"  Speed      : {args.speed} RPM")
    print(f"  Acc        : {args.acc}")
    print(f"  Port       : {args.port} @ {args.baud}")
    print("=" * 50)
    print()
    print("  ⚠  Ensure door is FULLY OPEN before start!  ⚠")
    print()
    input("  Press ENTER to begin...")
    print()

    client = make_client(args.port, args.baud, args.timeout)
    if not client.connect():
        print(f"Cannot open {args.port}", file=sys.stderr)
        sys.exit(1)

    # Ensure motor is in SR_CLOSE and enabled
    client.write_register(address=REG_WORKMODE, value=MODE_SR_CLOSE, device_id=args.addr)
    time.sleep(0.05)
    client.write_register(address=REG_ENABLE, value=1, device_id=args.addr)
    time.sleep(0.1)

    cycle = 0
    direction = 1  # +1 = close, -1 = open
    try:
        while True:
            cycle += 1
            offset = counts * direction
            dir_label = "CLOSE" if direction > 0 else "OPEN"
            print(f"  Cycle {cycle:4d} | {dir_label:5s} | {offset:+7d} counts ...", end=" ", flush=True)

            move_rel(client, args.addr, offset, args.speed, args.acc)
            done = wait_move_done(client, args.addr, timeout=30.0)

            if done:
                print("OK")
            else:
                print("TIMEOUT!")
                print("  Motor did not reach target within 30s. Stopping.")
                break

            time.sleep(0.5)
            direction *= -1

    except KeyboardInterrupt:
        print("\n\n  Stopped by user.")
        print(f"  Completed {cycle - 1} full cycles.")
    finally:
        client.close()


if __name__ == "__main__":
    main()
