#!/usr/bin/env python3
"""
Change the Modbus address of an MKS SERVO57D motor over RS485.

Tests connectivity at the current address, optionally writes a new address,
then verifies the motor responds on the new address.

Usage:
    python3 mks_set_address.py -a 1                        # just test address 1
    python3 mks_set_address.py -a 1 -t 5                   # change address 1 → 5
    python3 mks_set_address.py -a 1 -t 5 -p /dev/ttyUSB1   # different port
    python3 mks_set_address.py -a 1 -t 5 -b 115200         # different baud
"""

import argparse
import sys
import time

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient

REG_ADDRESS = 0x008B


def make_client(port: str, baud: int, timeout: float) -> ModbusSerialClient:
    return ModbusSerialClient(
        port=port, framer=FramerType.RTU,
        baudrate=baud, bytesize=8, parity="N", stopbits=1,
        timeout=timeout, retries=1,
    )


def probe(client, addr: int) -> bool:
    """Try reading encoder carry registers — returns True if motor responds."""
    try:
        rr = client.read_input_registers(address=0x30, count=3, device_id=addr)
        return not rr.isError()
    except Exception:
        return False


def set_address(client, current: int, target: int):
    """Write the new address register (motor won't respond on old address after this)."""
    client.write_register(
        address=REG_ADDRESS, value=target,
        device_id=current, no_response_expected=True,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Test connectivity / change Modbus address of an MKS servo")
    parser.add_argument("-a", "--addr", type=int, required=True,
                        help="Current motor address (1-247)")
    parser.add_argument("-t", "--target", type=int, default=None,
                        help="New address to assign (1-247)")
    parser.add_argument("-p", "--port", default="/dev/mksbus",
                        help="Serial port (default: /dev/mksbus)")
    parser.add_argument("-b", "--baud", type=int, default=38400,
                        help="Baud rate (default: 38400)")
    parser.add_argument("--timeout", type=float, default=0.3,
                        help="Response timeout in seconds (default: 0.3)")
    args = parser.parse_args()

    if args.target is not None and not 1 <= args.target <= 247:
        print("Target address must be 1-247", file=sys.stderr)
        sys.exit(1)

    print(f"Port: {args.port}  Baud: {args.baud}  Timeout: {args.timeout}s")

    client = make_client(args.port, args.baud, args.timeout)
    if not client.connect():
        print(f"Cannot open {args.port}", file=sys.stderr)
        sys.exit(1)

    try:
        # --- test current address ---
        print(f"\nProbing address {args.addr} ... ", end="", flush=True)
        if not probe(client, args.addr):
            print("NO RESPONSE")
            print("Motor not found at that address. Check wiring, baud rate, and power.")
            sys.exit(1)
        print("OK")

        if args.target is None:
            print("No target address given — nothing to change.")
            return

        if args.target == args.addr:
            print(f"Target address is the same as current ({args.addr}) — nothing to do.")
            return

        # --- write new address ---
        print(f"Changing address {args.addr} → {args.target} ... ", end="", flush=True)
        set_address(client, args.addr, args.target)
        time.sleep(0.5)
        print("sent")

        # --- verify new address ---
        print(f"Probing new address {args.target} ... ", end="", flush=True)
        if not probe(client, args.target):
            print("NO RESPONSE")
            print("Motor did not respond on the new address.")
            print("It may need a power cycle, or the write may have failed.")
            sys.exit(1)
        print("OK")

        # --- confirm old address is gone ---
        print(f"Confirming old address {args.addr} is gone ... ", end="", flush=True)
        if probe(client, args.addr):
            print("STILL RESPONDING (another motor on the bus?)")
        else:
            print("OK (no response, as expected)")

        print(f"\nAddress successfully changed to {args.target}.")

    finally:
        client.close()


if __name__ == "__main__":
    main()
