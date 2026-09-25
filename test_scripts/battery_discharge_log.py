#!/usr/bin/env python3
"""Log battery voltage during a discharge / BMS cutoff test.

Reads one N43IC04 channel through `mks_bus_node`'s shared
`/modbus/read_holding_register` service, converts the raw count to battery
volts, prints every sample and appends it to a log file. Each line is flushed
and fsynced so the log survives the moment the hardware BMS removes power.

The robot keeps running during the test. This tool never opens a serial port:
`mks_bus_node` stays the only owner of /dev/mksbus and keeps arbitrating
between the motors and the ADC, so logging cannot corrupt a motor transaction.
Start the normal stack first (`./scripts/start.sh`), then start this.

This tool deliberately never shuts the Jetson down and keeps logging below
20 V: the point of the test is to observe the voltage at which the BMS itself
cuts power. The automatic low-voltage shutdown lives in the ROS
battery_modbus_node, not here.

Typical use inside GNU screen:

    screen -S battery
    source ~/HorseShitBot/install/setup.bash
    python3 ~/HorseShitBot/test_scripts/battery_discharge_log.py
    # detach with Ctrl-A then D, reattach later with: screen -r battery
"""

from __future__ import annotations

import argparse
from datetime import datetime
import os
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node

from horseshitbot_interfaces.srv import ModbusReadHoldingRegister


READ_SERVICE = "/modbus/read_holding_register"
CHANNEL_START_REGISTER = 0x0000

# Battery divider calibration measured on the robot (2026-09-25):
#   20 V -> 970   22 V -> 1068   24 V -> 1167   26 V -> 1265   28 V -> 1364
# Keep in sync with horseshitbot/drivers/modbus_adc.py, which the ROS battery
# monitor uses.
RAW_OFFSET_COUNTS = 15.2
COUNTS_PER_VOLT = 49.25

# Reported for context only. This script never acts on them.
LOW_VOLTAGE = 22.0
CRITICAL_VOLTAGE = 20.0

DEFAULT_LOG_DIR = Path("/home/hsb/battery_logs")
DEFAULT_LOG_NAME = "battery_discharge.log"


def raw_to_voltage(raw: int) -> float:
    return (int(raw) + RAW_OFFSET_COUNTS) / COUNTS_PER_VOLT


def timestamp() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


class BusBusy(Exception):
    """The bus owner is mid-motor-transaction; the read was not attempted."""


class VoltageLog:
    """Append-only log that is flushed and fsynced after every line."""

    def __init__(self, path: Path):
        self.path = path
        path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = open(path, "a", buffering=1, encoding="utf-8")

    def write(self, line: str) -> None:
        print(line, flush=True)
        self._handle.write(f"{line}\n")
        self._handle.flush()
        # flush() only reaches the page cache; fsync is what survives a BMS
        # cutting power mid-test.
        os.fsync(self._handle.fileno())

    def close(self) -> None:
        self._handle.close()


class SharedBusReader:
    """Single-register reads through the bus owner's service."""

    def __init__(self, node: Node, timeout: float):
        self._node = node
        self._timeout = timeout
        self._client = node.create_client(
            ModbusReadHoldingRegister, READ_SERVICE
        )

    def wait_for_service(self, timeout: float) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout)

    def service_is_ready(self) -> bool:
        return self._client.service_is_ready()

    def read(self, device_id: int, address: int) -> int:
        if not self._client.service_is_ready():
            raise RuntimeError(
                f"{READ_SERVICE} unavailable (is the ROS stack running?)"
            )

        request = ModbusReadHoldingRegister.Request()
        request.device_id = device_id
        request.address = address
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(
            self._node, future, timeout_sec=self._timeout
        )

        if not future.done():
            try:
                self._client.remove_pending_request(future)
            except Exception:
                pass
            future.cancel()
            raise RuntimeError("shared-bus service request timed out")

        response = future.result()
        if response is None:
            raise RuntimeError("empty shared-bus service response")
        if response.busy:
            raise BusBusy(response.message or "bus busy")
        if not response.success:
            raise RuntimeError(response.message or "ADC read failed")
        return int(response.value)


def sample_once(reader: SharedBusReader, address: int, args) -> int:
    """Read one value, retrying transient bus-busy and Modbus errors."""
    last_error = None
    for attempt in range(args.retries + 1):
        try:
            return reader.read(args.slave, address)
        except BusBusy as exc:
            # The motors own the bus for a moment. Not an error: back off
            # briefly and try again inside the same sample slot.
            last_error = exc
        except Exception as exc:
            last_error = exc
        if attempt < args.retries:
            time.sleep(args.retry_delay)
    raise RuntimeError(str(last_error))


def run(args, log: VoltageLog, reader: SharedBusReader) -> int:
    address = CHANNEL_START_REGISTER + args.channel - 1

    log.write(
        f"# session start {timestamp()} service={READ_SERVICE} "
        f"slave={args.slave} channel={args.channel} "
        f"register=0x{address:04X} period={args.period}s"
    )
    log.write(
        f"# voltage = (raw + {RAW_OFFSET_COUNTS}) / {COUNTS_PER_VOLT} "
        f"| low <= {LOW_VOLTAGE} V | critical <= {CRITICAL_VOLTAGE} V "
        f"| automatic shutdown: DISABLED (BMS cutoff test)"
    )

    samples = 0
    errors = 0
    lowest: float | None = None
    lowest_at = ""
    below_critical_logged = False

    try:
        while True:
            started = time.monotonic()
            try:
                raw = sample_once(reader, address, args)
            except Exception as exc:
                errors += 1
                log.write(f"{timestamp()}, error={exc}")
            else:
                voltage = raw_to_voltage(raw)
                samples += 1
                if lowest is None or voltage < lowest:
                    lowest = voltage
                    lowest_at = timestamp()
                log.write(f"{timestamp()}, raw={raw}, voltage={voltage:.3f}")
                if voltage <= CRITICAL_VOLTAGE and not below_critical_logged:
                    below_critical_logged = True
                    log.write(
                        f"# below {CRITICAL_VOLTAGE} V at {timestamp()} — "
                        "this tool keeps logging until the BMS cuts power"
                    )

            remaining = args.period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        log.write(
            f"# session end {timestamp()} samples={samples} errors={errors} "
            f"lowest_voltage="
            + ("none" if lowest is None else f"{lowest:.3f} at {lowest_at}")
        )
        return 130


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def non_negative_int(value: str) -> int:
    parsed = int(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must not be negative")
    return parsed


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "Log battery voltage during a discharge test, through the shared "
            "Modbus bus owner so the robot can keep running. Never shuts the "
            "robot down, so the hardware BMS cutoff voltage can be observed."
        )
    )
    parser.add_argument("--slave", type=int, default=33)
    parser.add_argument("--channel", type=int, default=1, choices=(1, 2, 3, 4))
    parser.add_argument(
        "--timeout",
        type=positive_float,
        default=1.0,
        help="seconds to wait for one service response (default 1.0)",
    )
    parser.add_argument(
        "--retries",
        type=non_negative_int,
        default=3,
        help="extra attempts per sample on a busy bus or read error",
    )
    parser.add_argument(
        "--retry-delay",
        type=positive_float,
        default=0.1,
        help="seconds between attempts (default 0.1)",
    )
    parser.add_argument(
        "--period",
        type=positive_float,
        default=1.0,
        help="seconds between samples (default 1.0)",
    )
    parser.add_argument(
        "--service-wait",
        type=positive_float,
        default=15.0,
        help=f"seconds to wait for {READ_SERVICE} at startup (default 15)",
    )
    parser.add_argument(
        "--log-file",
        type=Path,
        default=DEFAULT_LOG_DIR / DEFAULT_LOG_NAME,
        help=f"appended log file (default {DEFAULT_LOG_DIR / DEFAULT_LOG_NAME})",
    )
    args = parser.parse_args(argv)
    if not 1 <= args.slave <= 247:
        parser.error("--slave must be in 1..247")
    return args


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=None)
    node = rclpy.create_node("battery_discharge_logger")
    try:
        args = parse_args(argv)

        print(f"READ-ONLY: single-register reads through {READ_SERVICE}.")
        print("mks_bus_node remains the only owner of the serial port.")
        print("Automatic low-voltage shutdown is DISABLED in this tool.")
        print(f"Logging to {args.log_file}")

        reader = SharedBusReader(node, args.timeout)
        if not reader.wait_for_service(args.service_wait):
            print(
                f"ERROR: {READ_SERVICE} did not appear within "
                f"{args.service_wait:g}s. Start the ROS stack first:\n"
                "  ./scripts/start.sh",
                file=sys.stderr,
            )
            return 2

        log = VoltageLog(args.log_file)
        try:
            return run(args, log, reader)
        finally:
            log.close()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
