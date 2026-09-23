#!/usr/bin/env python3
"""Read-only, direct-serial diagnostic for the HorseShitBot N43IC04.

This tool owns the serial port directly. Stop mks_bus_node before using it.
It reads FC03 holding registers only and never writes ADC configuration.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time


PACKAGE_ROOT = Path(__file__).resolve().parent / "src" / "horseshitbot"
sys.path.insert(0, str(PACKAGE_ROOT))

from horseshitbot.drivers.mks_bus import validate_modbus_response  # noqa: E402
from horseshitbot.drivers.pymodbus_compat import (  # noqa: E402
    ModbusSerialClient,
    RTU_FRAMER,
    call_with_device,
)


CHANNEL_START_REGISTER = 0x0000
CHANNEL_COUNT = 4
ADDRESS_REGISTER = 0x00FD
BAUD_REGISTER = 0x00FE
BAUD_CODES = {
    0: 1200,
    1: 2400,
    2: 4800,
    3: 9600,
    4: 19200,
}


def parse_int(value: str) -> int:
    return int(value, 0)


def make_client(port: str, baudrate: int, timeout: float):
    return ModbusSerialClient(
        port=port,
        framer=RTU_FRAMER,
        baudrate=baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=timeout,
        retries=0,
    )


def read_holding(client, device_id: int, address: int, count: int) -> list[int]:
    response = call_with_device(
        client.read_holding_registers,
        device_id,
        address=address,
        count=count,
    )
    validate_modbus_response(
        response,
        f"FC03 read device={device_id} address=0x{address:04X}",
        expected_registers=count,
    )
    return [int(value) for value in response.registers[:count]]


def print_channels(values: list[int]) -> None:
    for index, raw in enumerate(values, start=1):
        register = CHANNEL_START_REGISTER + index - 1
        print(f"CH{index} register=0x{register:04X} raw={raw}")


def read_device(args) -> int:
    client = make_client(args.port, args.baudrate, args.timeout)
    if not client.connect():
        print(f"ERROR: could not open {args.port}", file=sys.stderr)
        return 1

    try:
        while True:
            values = read_holding(
                client,
                args.slave,
                CHANNEL_START_REGISTER,
                CHANNEL_COUNT,
            )
            print_channels(values)

            if args.read_config:
                address = read_holding(
                    client, args.slave, ADDRESS_REGISTER, 1
                )[0]
                baud_code = read_holding(
                    client, args.slave, BAUD_REGISTER, 1
                )[0]
                print(
                    f"device_address_register=0x{ADDRESS_REGISTER:04X} "
                    f"value={address}"
                )
                print(
                    f"baud_register=0x{BAUD_REGISTER:04X} "
                    f"code={baud_code} "
                    f"baud={BAUD_CODES.get(baud_code, 'UNKNOWN')}"
                )

            if not args.watch:
                return 0
            time.sleep(args.period)
    except KeyboardInterrupt:
        print("\nStopped.")
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    finally:
        client.close()


def scan(args) -> int:
    found = []
    for baudrate in args.baudrates:
        print(
            f"Scanning {args.port} at {baudrate} baud, "
            f"IDs {args.start_id}..{args.end_id}",
            flush=True,
        )
        client = make_client(args.port, baudrate, args.timeout)
        if not client.connect():
            print(f"ERROR: could not open {args.port}", file=sys.stderr)
            continue
        try:
            for device_id in range(args.start_id, args.end_id + 1):
                try:
                    values = read_holding(
                        client,
                        device_id,
                        CHANNEL_START_REGISTER,
                        CHANNEL_COUNT,
                    )
                except Exception:
                    continue
                print(
                    f"RESPONDER baud={baudrate} device_id={device_id} "
                    f"raw_channels={values}"
                )
                found.append((baudrate, device_id, values))
        finally:
            client.close()

    if not found:
        print("No FC03 responder found.", file=sys.stderr)
        return 1
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Read-only offline N43IC04 direct-serial diagnostic"
    )
    parser.add_argument(
        "--ros-stopped",
        action="store_true",
        help="confirm mks_bus_node and every other serial owner are stopped",
    )
    parser.add_argument("--port", default="/dev/mksbus")
    parser.add_argument("--baudrate", type=int, default=19200)
    parser.add_argument("--slave", type=parse_int, default=33)
    parser.add_argument("--timeout", type=float, default=0.4)
    parser.add_argument(
        "--read-config",
        action="store_true",
        help="also read device address 0x00FD and baud code 0x00FE",
    )
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--period", type=float, default=1.0)
    parser.add_argument(
        "--scan",
        action="store_true",
        help="scan configurable IDs/baudrates instead of reading one device",
    )
    parser.add_argument("--start-id", type=int, default=1)
    parser.add_argument(
        "--end-id",
        type=int,
        default=40,
        help="default includes verified ID 33 without scanning all 247 IDs",
    )
    parser.add_argument(
        "--baudrates",
        type=int,
        nargs="+",
        default=[19200],
        help="baudrates used by --scan; default is only verified 19200",
    )
    args = parser.parse_args()

    if not args.ros_stopped:
        parser.error(
            "--ros-stopped is required: stop mks_bus_node and all serial owners"
        )
    if not 1 <= args.slave <= 247:
        parser.error("--slave must be in 1..247")
    if not 1 <= args.start_id <= args.end_id <= 247:
        parser.error("scan range must satisfy 1 <= start-id <= end-id <= 247")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.period < 0:
        parser.error("--period must be non-negative")

    print(
        "WARNING: direct serial ownership enabled. mks_bus_node must be stopped.",
        file=sys.stderr,
    )
    print("READ-ONLY: this tool sends FC03 requests and performs no writes.")

    return scan(args) if args.scan else read_device(args)


if __name__ == "__main__":
    raise SystemExit(main())
