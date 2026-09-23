#!/usr/bin/env python3
"""Read N43IC04 raw channels through the ROS shared-bus service only."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from horseshitbot_interfaces.srv import ModbusReadHoldingRegister


SERVICE_NAME = "/modbus/read_holding_register"
CHANNEL_REGISTERS = (0x0000, 0x0001, 0x0002, 0x0003)


class AdcSharedBusTest(Node):
    def __init__(self):
        super().__init__("adc_shared_bus_test")
        self.client = self.create_client(
            ModbusReadHoldingRegister, SERVICE_NAME
        )

    def read_register(
        self, device_id: int, address: int, timeout_sec: float
    ):
        request = ModbusReadHoldingRegister.Request()
        request.device_id = int(device_id)
        request.address = int(address)
        future = self.client.call_async(request)
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

        if not future.done():
            try:
                self.client.remove_pending_request(future)
            except Exception:
                pass
            future.cancel()
            raise TimeoutError(
                f"service request timed out after {timeout_sec:.3f}s"
            )
        return future.result()


def positive_int(value: str) -> int:
    parsed = int(value, 0)
    if parsed < 1:
        raise argparse.ArgumentTypeError("value must be at least 1")
    return parsed


def nonnegative_float(value: str) -> float:
    parsed = float(value)
    if parsed < 0:
        raise argparse.ArgumentTypeError("value must be non-negative")
    return parsed


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Read N43IC04 raw channels through mks_bus_node without opening "
            "the serial port"
        )
    )
    parser.add_argument("--device-id", type=positive_int, default=33)
    parser.add_argument("--rounds", type=positive_int, default=5)
    parser.add_argument(
        "--delay",
        type=nonnegative_float,
        default=0.1,
        help="delay between service requests in seconds",
    )
    parser.add_argument(
        "--service-timeout",
        type=positive_float,
        default=5.0,
        help="maximum wait for the shared service",
    )
    parser.add_argument(
        "--request-timeout",
        type=positive_float,
        default=1.0,
        help="maximum wait for each service response",
    )
    args = parser.parse_args()
    if args.device_id > 247:
        parser.error("--device-id must be in 1..247")
    return args


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def run(args) -> int:
    node = AdcSharedBusTest()
    total = args.rounds * len(CHANNEL_REGISTERS)
    successful = 0
    busy = 0
    failed = 0

    try:
        if not node.client.wait_for_service(timeout_sec=args.service_timeout):
            print(
                f"ERROR: {SERVICE_NAME} unavailable after "
                f"{args.service_timeout:.1f}s; start mks_bus_node first.",
                file=sys.stderr,
            )
            return 2

        for round_number in range(1, args.rounds + 1):
            for channel, address in enumerate(CHANNEL_REGISTERS, start=1):
                prefix = (
                    f"round={round_number} channel={channel} "
                    f"register=0x{address:04X}"
                )
                try:
                    response = node.read_register(
                        args.device_id, address, args.request_timeout
                    )
                    if response is None:
                        failed += 1
                        print(f"{prefix} status=failure raw=- message=empty response")
                    elif response.busy:
                        busy += 1
                        print(
                            f"{prefix} status=busy raw=- "
                            f"message={response.message!r}"
                        )
                    elif not response.success:
                        failed += 1
                        print(
                            f"{prefix} status=failure raw=- "
                            f"message={response.message!r}"
                        )
                    else:
                        successful += 1
                        print(
                            f"{prefix} status=success raw={int(response.value)}"
                        )
                except Exception as exc:
                    failed += 1
                    print(f"{prefix} status=failure raw=- message={exc!r}")

                if args.delay > 0:
                    time.sleep(args.delay)

        print("\nSummary")
        print(f"  total_reads={total}")
        print(f"  successful_reads={successful}")
        print(f"  busy_responses={busy}")
        print(f"  failed_reads={failed}")
        return 0 if successful == total else 1
    finally:
        node.destroy_node()


def main() -> int:
    args = parse_args()
    rclpy.init()
    try:
        return run(args)
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        return 130
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
