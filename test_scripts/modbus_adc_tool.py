#!/usr/bin/env python3

import argparse
import csv
import pathlib
import statistics
import sys
import time
from typing import List

REPO_ROOT = pathlib.Path(__file__).resolve().parents[1]
PY_PKG = REPO_ROOT / "src" / "horseshitbot"

if PY_PKG.exists():
    sys.path.insert(0, str(PY_PKG))

from horseshitbot.modbus_adc import (  # noqa: E402
    AdcRegisterMap,
    CODE_TO_BAUD,
    ModbusRtuClient,
    N43VD04,
)


def parse_int(value: str) -> int:
    return int(value, 0)


def add_common_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port of USB-RS485 adapter",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=9600,
        help="Serial baudrate",
    )
    parser.add_argument(
        "--slave",
        type=parse_int,
        default=1,
        help="Modbus slave id, for example 1 or 0x01",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.25,
        help="Serial timeout in seconds",
    )

    parser.add_argument(
        "--channel-base-register",
        type=parse_int,
        default=0x0000,
    )
    parser.add_argument(
        "--address-register",
        type=parse_int,
        default=0x000E,
    )
    parser.add_argument(
        "--baudrate-register",
        type=parse_int,
        default=0x000F,
    )

    parser.add_argument(
        "--raw-scale",
        type=float,
        default=100.0,
        help="raw / raw_scale = volts at ADC input",
    )
    parser.add_argument(
        "--divider-factor",
        type=float,
        default=1.0,
        help=(
            "ADC_voltage = real_battery_voltage * divider_factor. "
            "For the final robot use 0.5."
        ),
    )


def make_adc(args):
    reg_map = AdcRegisterMap(
        channel_base_register=args.channel_base_register,
        address_register=args.address_register,
        baudrate_register=args.baudrate_register,
        raw_scale=args.raw_scale,
        voltage_divider_factor=args.divider_factor,
    )

    client = ModbusRtuClient(
        port=args.port,
        baudrate=args.baudrate,
        slave_id=args.slave,
        timeout=args.timeout,
    )

    return client, N43VD04(client, reg_map)


def print_reading(reading: dict) -> None:
    print(
        f"CH{reading['channel']}: "
        f"raw={reading['raw']}  "
        f"adc_voltage={reading['adc_voltage']:.3f} V  "
        f"battery_voltage={reading['battery_voltage']:.3f} V"
    )


def cmd_read(args) -> int:
    client, adc = make_adc(args)

    with client:
        count = args.count
        n = 0

        while count == 0 or n < count:
            print_reading(adc.read_voltage(args.channel))
            n += 1

            if count == 1:
                break

            time.sleep(args.period)

    return 0


def cmd_read_all(args) -> int:
    client, adc = make_adc(args)

    with client:
        for reading in adc.read_all_voltages():
            print_reading(reading)

    return 0


def cmd_config(args) -> int:
    client, adc = make_adc(args)

    with client:
        slave_id = adc.read_slave_id(use_broadcast=args.broadcast)
        baud_code = adc.read_baudrate_code()
        baudrate = CODE_TO_BAUD.get(baud_code)

        print(f"reported_slave_id={slave_id}")
        print(f"baudrate_code={baud_code}")
        print(f"baudrate={baudrate if baudrate is not None else 'UNKNOWN'}")

    return 0


def cmd_set_slave(args) -> int:
    if not args.yes:
        print("Refusing to change slave id without --yes")
        return 2

    client, adc = make_adc(args)

    with client:
        print(f"Writing slave id: old={args.slave} new={args.new_slave}")
        adc.set_slave_id(args.new_slave)

        print("Write completed. Testing read with new slave id...")
        print_reading(adc.read_voltage(args.channel))

    return 0


def cmd_set_baud(args) -> int:
    if not args.yes:
        print("Refusing to change baudrate without --yes")
        return 2

    client, adc = make_adc(args)

    with client:
        code = adc.set_baudrate(args.new_baudrate)

        print(f"Wrote baudrate {args.new_baudrate} as code {code}.")
        print("Power-cycle the ADC module now.")
        print(f"Then test again using --baudrate {args.new_baudrate}")

    return 0


def cmd_scan(args) -> int:
    found = []

    for baudrate in args.baudrates:
        client, adc = make_adc(args)
        client.baudrate = baudrate

        try:
            client.connect()
        except Exception as exc:
            print(f"Could not open {args.port} at {baudrate}: {exc}")
            continue

        try:
            for slave in range(args.start_id, args.end_id + 1):
                client.slave_id = slave

                try:
                    reading = adc.read_voltage(args.channel)
                except Exception:
                    continue

                print(f"FOUND baudrate={baudrate} slave={slave}: ", end="")
                print_reading(reading)
                found.append((baudrate, slave))
        finally:
            client.close()

    if not found:
        print(
            "No devices found. Try swapping RS485 A/B, checking power, "
            "or checking the documented default baud/slave."
        )
        return 1

    return 0


def cmd_stress(args) -> int:
    client, adc = make_adc(args)

    values: List[float] = []
    errors = 0
    first_error = None

    csv_file = None
    writer = None

    if args.csv:
        csv_file = open(args.csv, "w", newline="")
        writer = csv.writer(csv_file)
        writer.writerow(
            [
                "t_sec",
                "raw",
                "adc_voltage",
                "battery_voltage",
            ]
        )

    start = time.monotonic()
    end = start + args.duration

    last_success_t = None
    max_success_gap = 0.0

    try:
        with client:
            while time.monotonic() < end:
                try:
                    reading = adc.read_voltage(args.channel)
                    now = time.monotonic()

                    if last_success_t is not None:
                        max_success_gap = max(
                            max_success_gap,
                            now - last_success_t,
                        )

                    last_success_t = now

                    values.append(reading["battery_voltage"])

                    if writer is not None:
                        writer.writerow(
                            [
                                f"{now - start:.6f}",
                                reading["raw"],
                                f"{reading['adc_voltage']:.6f}",
                                f"{reading['battery_voltage']:.6f}",
                            ]
                        )

                except Exception as exc:
                    errors += 1

                    if first_error is None:
                        first_error = repr(exc)

                    if args.print_errors:
                        print("ERROR:", repr(exc))

                if args.period > 0:
                    time.sleep(args.period)

    finally:
        if csv_file is not None:
            csv_file.close()

    elapsed = time.monotonic() - start
    success = len(values)
    total = success + errors

    print("Stress test summary")
    print(f"  duration_sec={elapsed:.3f}")
    print(f"  total_attempts={total}")
    print(f"  success={success}")
    print(f"  errors={errors}")
    print(f"  success_rate_hz={success / elapsed if elapsed > 0 else 0:.2f}")
    print(f"  max_gap_between_successes_sec={max_success_gap:.3f}")

    if values:
        print(f"  voltage_min={min(values):.3f} V")
        print(f"  voltage_max={max(values):.3f} V")
        print(f"  voltage_mean={statistics.fmean(values):.3f} V")

    if first_error:
        print(f"  first_error={first_error}")

    return 0 if errors == 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Standalone Modbus ADC test/config tool for HSB"
    )

    sub = parser.add_subparsers(
        dest="command",
        required=True,
    )

    p = sub.add_parser(
        "read",
        help="Read one ADC channel",
    )
    add_common_args(p)
    p.add_argument("--channel", type=int, default=1)
    p.add_argument(
        "--count",
        type=int,
        default=1,
        help="1 = once, 0 = forever",
    )
    p.add_argument("--period", type=float, default=1.0)
    p.set_defaults(func=cmd_read)

    p = sub.add_parser(
        "read-all",
        help="Read all 4 ADC channels",
    )
    add_common_args(p)
    p.set_defaults(func=cmd_read_all)

    p = sub.add_parser(
        "config",
        help="Read configured slave id and baudrate",
    )
    add_common_args(p)
    p.add_argument(
        "--broadcast",
        action="store_true",
        help="Use slave id 0xFF for reading address; use only with one device connected",
    )
    p.set_defaults(func=cmd_config)

    p = sub.add_parser(
        "set-slave",
        help="Write a new Modbus slave id",
    )
    add_common_args(p)
    p.add_argument("--new-slave", type=parse_int, required=True)
    p.add_argument(
        "--channel",
        type=int,
        default=1,
        help="Channel to test after changing",
    )
    p.add_argument("--yes", action="store_true")
    p.set_defaults(func=cmd_set_slave)

    p = sub.add_parser(
        "set-baud",
        help="Write a new serial baudrate code",
    )
    add_common_args(p)
    p.add_argument(
        "--new-baudrate",
        type=int,
        required=True,
        choices=[1200, 2400, 4800, 9600, 19200],
    )
    p.add_argument("--yes", action="store_true")
    p.set_defaults(func=cmd_set_baud)

    p = sub.add_parser(
        "scan",
        help="Scan slave ids and baudrates",
    )
    add_common_args(p)
    p.add_argument("--start-id", type=int, default=1)
    p.add_argument("--end-id", type=int, default=247)
    p.add_argument("--channel", type=int, default=1)
    p.add_argument(
        "--baudrates",
        type=int,
        nargs="+",
        default=[9600, 19200, 4800, 2400, 1200],
    )
    p.set_defaults(func=cmd_scan)

    p = sub.add_parser(
        "stress",
        help="Read as fast as possible for a few minutes",
    )
    add_common_args(p)
    p.add_argument("--channel", type=int, default=1)
    p.add_argument("--duration", type=float, default=180.0)
    p.add_argument(
        "--period",
        type=float,
        default=0.0,
        help="0 means no deliberate sleep",
    )
    p.add_argument("--csv", default=None)
    p.add_argument("--print-errors", action="store_true")
    p.set_defaults(func=cmd_stress)

    args = parser.parse_args()

    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())