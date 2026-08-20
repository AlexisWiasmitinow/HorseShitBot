#!/usr/bin/env python3
"""
Change the Modbus address or Baud Rate of an MKS SERVO57D motor over RS485.

Tests connectivity at current settings, applies updates, and verifies.

Usage:
    python3 mks_config.py -a 1                          # Just test address 1
    python3 mks_config.py -a 1 -t 5                     # Change address 1 → 5
    python3 mks_config.py -a 1 --target-baud 115200     # Change baud to 115200
    python3 mks_config.py -a 1 -t 5 --target-baud 115200 # Change address AND baud
"""

import argparse
import struct
import sys
import time

from pymodbus import FramerType
from pymodbus.client import ModbusSerialClient

REG_ADDRESS = 0x008B

# Map target baud rates to the MKS proprietary command values (01-07)
BAUD_MAP = {
    9600: 1,
    19200: 2,
    25000: 3,
    38400: 4,
    57600: 5,
    115200: 6,
    256000: 7,
}


def make_client(port: str, baud: int, timeout: float) -> ModbusSerialClient:
    return ModbusSerialClient(
        port=port,
        framer=FramerType.RTU,
        baudrate=baud,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=timeout,
        retries=1,
    )


def calculate_crc16(data: bytes) -> int:
    """Calculate Modbus RTU CRC16 checksum."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


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
        address=REG_ADDRESS,
        value=target,
        device_id=current,
        no_response_expected=True,
    )


def set_baudrate(client, addr: int, target_baud: int) -> bool:
    """
    Send the custom MKS command (FA [Addr] 8A [Baud_Val] [CRC]) to update baud rate.
    """
    if target_baud not in BAUD_MAP:
        print(
            f"Unsupported baud rate: {target_baud}. Valid options: {list(BAUD_MAP.keys())}",
            file=sys.stderr,
        )
        return False

    baud_val = BAUD_MAP[target_baud]

    # Build MKS custom command payload: Header(0xFA) + Addr + Func(0x8A) + Data
    payload = struct.pack("BBBB", 0xFA, addr, 0x8A, baud_val)

    # Compute CRC (Little-endian)
    crc = calculate_crc16(payload)
    frame = payload + struct.pack("<H", crc)

    try:
        # Send raw frame directly over the serial line
        client.send(frame)
        return True
    except Exception as e:
        print(f"Error sending baud change command: {e}", file=sys.stderr)
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Test connectivity / change settings of an MKS servo"
    )
    parser.add_argument(
        "-a",
        "--addr",
        type=int,
        required=True,
        help="Current motor address (1-247)",
    )
    parser.add_argument(
        "-t",
        "--target",
        type=int,
        default=None,
        help="New address to assign (1-247)",
    )
    parser.add_argument(
        "-p",
        "--port",
        default="/dev/mksbus",
        help="Serial port (default: /dev/mksbus)",
    )
    parser.add_argument(
        "-b",
        "--baud",
        type=int,
        default=38400,
        help="Current baud rate (default: 38400)",
    )
    parser.add_argument(
        "--target-baud",
        type=int,
        default=None,
        choices=[9600, 19200, 25000, 38400, 57600, 115200, 256000],
        help="New baud rate to assign",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.3,
        help="Response timeout in seconds (default: 0.3)",
    )
    args = parser.parse_args()

    if args.target is not None and not 1 <= args.target <= 247:
        print("Target address must be 1-247", file=sys.stderr)
        sys.exit(1)

    print(f"Port: {args.port}  Baud: {args.baud}  Timeout: {args.timeout}s")

    client = make_client(args.port, args.baud, args.timeout)
    if not client.connect():
        print(f"Cannot open {args.port}", file=sys.stderr)
        sys.exit(1)

    active_addr = args.addr
    active_baud = args.baud

    try:
        # --- 1. Test current connection ---
        print(f"\nProbing address {active_addr} ... ", end="", flush=True)
        if not probe(client, active_addr):
            print("NO RESPONSE")
            print(
                "Motor not found. Check wiring, baud rate, and power."
            )
            sys.exit(1)
        print("OK")

        if args.target is None and args.target_baud is None:
            print("No target address or baud rate given — nothing to change.")
            return

        # --- 2. Write new address (if requested) ---
        if args.target is not None and args.target != active_addr:
            print(
                f"Changing address {active_addr} → {args.target} ... ",
                end="",
                flush=True,
            )
            set_address(client, active_addr, args.target)
            time.sleep(0.5)
            active_addr = args.target
            print("sent")

        # --- 3. Write new baud rate (if requested) ---
        if args.target_baud is not None and args.target_baud != active_baud:
            print(
                f"Changing baud rate {active_baud} → {args.target_baud} ... ",
                end="",
                flush=True,
            )
            if set_baudrate(client, active_addr, args.target_baud):
                active_baud = args.target_baud
                time.sleep(0.5)
                print("sent")
            else:
                print("FAILED")
                sys.exit(1)

        # --- 4. Re-verify configuration ---
        client.close()

        print(
            f"\nRe-connecting with target settings (Baud: {active_baud}) ... ",
            end="",
            flush=True,
        )
        client = make_client(args.port, active_baud, args.timeout)
        if not client.connect():
            print("FAILED TO REOPEN SERIAL PORT")
            sys.exit(1)
        print("OK")

        print(
            f"Probing motor at address {active_addr} @ {active_baud} baud ... ",
            end="",
            flush=True,
        )
        if not probe(client, active_addr):
            print("NO RESPONSE")
            print(
                "Motor did not respond on new configuration. A power cycle might be required."
            )
            sys.exit(1)
        print("OK")

        print("\nConfiguration successful!")

    finally:
        client.close()


if __name__ == "__main__":
    main()