#!/usr/bin/env python3
"""
Test connection and preconfigure MKS SERVO57D motors over RS485.

Sets work mode, microsteps, and run current on each motor, then verifies
the motor responds.

Usage:
    # Configure all actuator motors (3,4,5,6) with defaults
    python3 mks_configure.py

    # Custom port / baud
    python3 mks_configure.py -p /dev/ttyUSB1 -b 115200

    # Only specific motors
    python3 mks_configure.py -m 3 4

    # Override settings
    python3 mks_configure.py --mode 4 --microsteps 16 --current 1500
"""

import argparse
import sys
import time

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient

REG_WORKMODE    = 0x0082
REG_RUN_CURRENT = 0x0083
REG_MICROSTEPS  = 0x0084
REG_ENABLE      = 0x00F3

WORK_MODES = {
    0: "open-loop pulse",
    1: "closed-loop pulse",
    2: "closed-loop FOC (pulse)",
    3: "open-loop serial",
    4: "closed-loop serial",
    5: "closed-loop FOC serial",
}

DEFAULT_MOTOR_IDS = [3, 4, 5, 6]


def make_client(port: str, baud: int, timeout: float) -> ModbusSerialClient:
    return ModbusSerialClient(
        port=port, framer=FramerType.RTU,
        baudrate=baud, bytesize=8, parity="N", stopbits=1,
        timeout=timeout, retries=1,
    )


def probe(client, addr: int) -> bool:
    try:
        rr = client.read_input_registers(address=0x30, count=3, device_id=addr)
        return not rr.isError()
    except Exception:
        return False


def write_reg(client, addr: int, reg: int, value: int, label: str) -> bool:
    try:
        rr = client.write_register(address=reg, value=value, device_id=addr)
        if hasattr(rr, "isError") and rr.isError():
            print(f"    {label}: WRITE ERROR — {rr}")
            return False
        print(f"    {label}: OK (wrote {value})")
        return True
    except Exception as e:
        print(f"    {label}: FAILED — {e}")
        return False


def read_reg(client, addr: int, reg: int) -> int | None:
    try:
        rr = client.read_holding_registers(address=reg, count=1, device_id=addr)
        if hasattr(rr, "isError") and rr.isError():
            return None
        return rr.registers[0]
    except Exception:
        return None


def configure_motor(client, addr: int, mode: int, microsteps: int, current: int) -> bool:
    print(f"\n=== Motor {addr} ===")

    print(f"  Probing ... ", end="", flush=True)
    if not probe(client, addr):
        print("NO RESPONSE")
        return False
    print("OK")

    print(f"  Configuring:")
    mode_name = WORK_MODES.get(mode, f"unknown({mode})")
    write_reg(client, addr, REG_WORKMODE, mode, f"Work mode → {mode} ({mode_name})")
    time.sleep(0.05)
    write_reg(client, addr, REG_MICROSTEPS, microsteps, f"Microsteps → {microsteps}")
    time.sleep(0.05)
    write_reg(client, addr, REG_RUN_CURRENT, current, f"Run current → {current} mA")
    time.sleep(0.05)

    print(f"  Readback:")
    for reg, label in [(REG_WORKMODE, "Work mode"), (REG_MICROSTEPS, "Microsteps"), (REG_RUN_CURRENT, "Run current")]:
        val = read_reg(client, addr, reg)
        if val is not None:
            extra = f" ({WORK_MODES[val]})" if reg == REG_WORKMODE and val in WORK_MODES else ""
            print(f"    {label}: {val}{extra}")
        else:
            print(f"    {label}: read failed")

    return True


def main():
    parser = argparse.ArgumentParser(description="Test & configure MKS servo motors")
    parser.add_argument("-p", "--port", default="/dev/mksbus",
                        help="Serial port (default: /dev/mksbus)")
    parser.add_argument("-b", "--baud", type=int, default=38400,
                        help="Baud rate (default: 38400)")
    parser.add_argument("-m", "--motors", type=int, nargs="+", default=DEFAULT_MOTOR_IDS,
                        help=f"Motor IDs to configure (default: {DEFAULT_MOTOR_IDS})")
    parser.add_argument("--mode", type=int, default=4,
                        help="Work mode (default: 4 = closed-loop serial)")
    parser.add_argument("--microsteps", type=int, default=16,
                        help="Microstep setting (default: 16)")
    parser.add_argument("--current", type=int, default=1500,
                        help="Run current in mA (default: 1500)")
    parser.add_argument("--timeout", type=float, default=0.3,
                        help="Response timeout in seconds (default: 0.3)")
    args = parser.parse_args()

    mode_name = WORK_MODES.get(args.mode, f"unknown")
    print(f"Port: {args.port}  Baud: {args.baud}")
    print(f"Motors: {args.motors}")
    print(f"Settings: mode={args.mode} ({mode_name}), microsteps={args.microsteps}, current={args.current} mA")

    client = make_client(args.port, args.baud, args.timeout)
    if not client.connect():
        print(f"Cannot open {args.port}", file=sys.stderr)
        sys.exit(1)

    found = []
    failed = []
    try:
        for mid in args.motors:
            if configure_motor(client, mid, args.mode, args.microsteps, args.current):
                found.append(mid)
            else:
                failed.append(mid)
    finally:
        client.close()

    print(f"\n--- Summary ---")
    print(f"  Configured: {found if found else 'none'}")
    if failed:
        print(f"  No response: {failed}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
