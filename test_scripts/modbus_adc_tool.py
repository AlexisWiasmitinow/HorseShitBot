#!/usr/bin/env python3
"""Standalone ADC commissioning tool.

This process opens /dev/mksbus directly. Never run it while mks_bus_node or
any other ROS/legacy bus owner is running.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


PACKAGE_ROOT = Path(__file__).resolve().parents[1] / "src" / "horseshitbot"
sys.path.insert(0, str(PACKAGE_ROOT))

from horseshitbot.drivers.mks_bus import validate_modbus_response  # noqa: E402
from horseshitbot.drivers.modbus_adc import (  # noqa: E402
    AdcRegisterMap,
    BAUD_TO_CODE,
    CODE_TO_BAUD,
)
from horseshitbot.drivers.pymodbus_compat import (  # noqa: E402
    ModbusSerialClient,
    RTU_FRAMER,
    call_with_device,
)


def parse_int(value: str) -> int:
    return int(value, 0)


def make_client(args, baudrate=None):
    return ModbusSerialClient(
        port=args.port,
        framer=RTU_FRAMER,
        baudrate=args.baudrate if baudrate is None else baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=args.timeout,
        retries=0,
    )


def read_register(client, device_id: int, address: int) -> int:
    response = call_with_device(
        client.read_holding_registers,
        device_id,
        address=address,
        count=1,
    )
    validate_modbus_response(response, "ADC read", expected_registers=1)
    return int(response.registers[0])


def write_register(client, device_id: int, address: int, value: int):
    response = call_with_device(
        client.write_register,
        device_id,
        address=address,
        value=value,
    )
    validate_modbus_response(
        response,
        "ADC write",
        expected_address=address,
        expected_value=value,
    )


def register_map(args) -> AdcRegisterMap:
    return AdcRegisterMap(
        channel_base_register=args.channel_base_register,
        address_register=args.address_register,
        baudrate_register=args.baudrate_register,
        raw_scale=args.raw_scale,
        voltage_divider_factor=args.divider_factor,
    )


def connect(client):
    if not client.connect():
        raise RuntimeError("could not open serial port")
    return client


def command_read(args) -> int:
    mapping = register_map(args)
    client = connect(make_client(args))
    try:
        raw = read_register(
            client, args.slave, mapping.channel_register(args.channel)
        )
        reading = mapping.convert(raw, args.channel)
        print(
            f"raw={raw} adc_voltage={reading['adc_voltage']:.3f}V "
            f"battery_voltage={reading['battery_voltage']:.3f}V"
        )
    finally:
        client.close()
    return 0


def command_config(args) -> int:
    mapping = register_map(args)
    client = connect(make_client(args))
    try:
        slave = read_register(client, args.slave, mapping.address_register)
        baud_code = read_register(client, args.slave, mapping.baudrate_register)
        print(f"reported_slave_id={slave}")
        print(f"baudrate_code={baud_code}")
        print(f"baudrate={CODE_TO_BAUD.get(baud_code, 'UNKNOWN')}")
    finally:
        client.close()
    return 0


def command_scan(args) -> int:
    found = []
    mapping = register_map(args)
    for baudrate in args.baudrates:
        client = connect(make_client(args, baudrate))
        try:
            for device_id in range(args.start_id, args.end_id + 1):
                try:
                    raw = read_register(
                        client,
                        device_id,
                        mapping.channel_register(args.channel),
                    )
                except Exception:
                    continue
                print(f"FOUND baud={baudrate} slave={device_id} raw={raw}")
                found.append((baudrate, device_id))
        finally:
            client.close()
    return 0 if found else 1


def require_write_confirmation(args):
    if not args.yes:
        raise RuntimeError("refusing configuration write without --yes")


def command_set_slave(args) -> int:
    require_write_confirmation(args)
    mapping = register_map(args)
    client = connect(make_client(args))
    try:
        write_register(
            client, args.slave, mapping.address_register, args.new_slave
        )
        print(f"wrote slave address {args.slave} -> {args.new_slave}")
    finally:
        client.close()
    return 0


def command_set_baud(args) -> int:
    require_write_confirmation(args)
    mapping = register_map(args)
    code = BAUD_TO_CODE[args.new_baudrate]
    client = connect(make_client(args))
    try:
        write_register(client, args.slave, mapping.baudrate_register, code)
        print(f"wrote baud {args.new_baudrate} as code {code}")
        print("Power-cycle the ADC, then verify it at the new baud.")
    finally:
        client.close()
    return 0


def add_common(parser):
    parser.add_argument("--port", default="/dev/mksbus")
    parser.add_argument("--baudrate", type=int, default=19200)
    parser.add_argument("--slave", type=parse_int, default=7)
    parser.add_argument("--timeout", type=float, default=0.1)
    parser.add_argument("--channel", type=int, default=1)
    parser.add_argument("--channel-base-register", type=parse_int, default=0)
    parser.add_argument("--address-register", type=parse_int, default=0x000E)
    parser.add_argument("--baudrate-register", type=parse_int, default=0x000F)
    parser.add_argument("--raw-scale", type=float, default=100.0)
    parser.add_argument("--divider-factor", type=float, default=0.5)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Offline Modbus ADC read/configuration tool"
    )
    parser.add_argument(
        "--ros-stopped",
        action="store_true",
        help="confirm mks_bus_node and all other serial owners are stopped",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    read_parser = subparsers.add_parser("read")
    add_common(read_parser)
    read_parser.set_defaults(func=command_read)

    config_parser = subparsers.add_parser("config")
    add_common(config_parser)
    config_parser.set_defaults(func=command_config)

    scan_parser = subparsers.add_parser("scan")
    add_common(scan_parser)
    scan_parser.add_argument("--start-id", type=int, default=1)
    scan_parser.add_argument("--end-id", type=int, default=247)
    scan_parser.add_argument(
        "--baudrates",
        type=int,
        nargs="+",
        default=[9600, 19200, 4800, 2400, 1200],
    )
    scan_parser.set_defaults(func=command_scan)

    slave_parser = subparsers.add_parser("set-slave")
    add_common(slave_parser)
    slave_parser.add_argument("--new-slave", type=int, required=True)
    slave_parser.add_argument("--yes", action="store_true")
    slave_parser.set_defaults(func=command_set_slave)

    baud_parser = subparsers.add_parser("set-baud")
    add_common(baud_parser)
    baud_parser.add_argument(
        "--new-baudrate",
        type=int,
        choices=sorted(BAUD_TO_CODE),
        required=True,
    )
    baud_parser.add_argument("--yes", action="store_true")
    baud_parser.set_defaults(func=command_set_baud)

    args = parser.parse_args()
    if not args.ros_stopped:
        parser.error(
            "--ros-stopped is required: stop mks_bus_node before opening the port"
        )
    if hasattr(args, "new_slave") and not 1 <= args.new_slave <= 247:
        parser.error("--new-slave must be in 1..247")

    print(
        "WARNING: direct serial ownership enabled; do not run ROS mks_bus_node "
        "concurrently.",
        file=sys.stderr,
    )
    try:
        return args.func(args)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
