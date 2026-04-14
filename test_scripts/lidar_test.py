#!/usr/bin/env python3
"""
RPLIDAR (SLAMTEC) test script — device info, health check, and short scan.

Uses raw serial protocol (no SDK needed, only pyserial).

Usage:
    python3 lidar_test.py                       # defaults
    python3 lidar_test.py -p /dev/ttyUSB1       # different port
    python3 lidar_test.py --scan                 # run a short scan (~3 rotations)
    python3 lidar_test.py --scan -n 1000         # collect 1000 points
"""

import argparse
import struct
import sys
import time

import serial

# RPLIDAR protocol constants
SYNC_BYTE = 0xA5
RESP_SYNC1 = 0xA5
RESP_SYNC2 = 0x5A

CMD_STOP = 0x25
CMD_RESET = 0x40
CMD_SCAN = 0x20
CMD_GET_INFO = 0x50
CMD_GET_HEALTH = 0x52
CMD_EXPRESS_SCAN = 0x82
CMD_MOTOR_PWM = 0xF0

HEALTH_GOOD = 0
HEALTH_WARNING = 1
HEALTH_ERROR = 2
HEALTH_NAMES = {0: "Good", 1: "Warning", 2: "Error"}

MODEL_NAMES = {
    0x18: "RPLIDAR A1",
    0x28: "RPLIDAR A2",
    0x38: "RPLIDAR A3",
    0x61: "RPLIDAR S1",
    0x62: "RPLIDAR S2",
    0x63: "RPLIDAR S3",
    0xA1: "RPLIDAR T1",
    0xA2: "RPLIDAR T-MINI Plus",
    0xA5: "RPLIDAR T-MINI",
    0xC0: "RPLIDAR C1",
}


class RPLidar:
    """Minimal RPLIDAR driver using raw serial protocol."""

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 3.0):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        time.sleep(0.1)
        self.ser.reset_input_buffer()

    def close(self):
        self.stop_scan()
        self.ser.close()

    def _send_cmd(self, cmd: int, payload: bytes = b""):
        if payload:
            pkt = bytes([SYNC_BYTE, cmd, len(payload)]) + payload
            checksum = 0
            for b in pkt:
                checksum ^= b
            pkt += bytes([checksum])
        else:
            pkt = bytes([SYNC_BYTE, cmd])
        self.ser.write(pkt)

    def _read_descriptor(self) -> tuple[int, bool, int]:
        """Read 7-byte response descriptor. Returns (data_size, is_single, data_type)."""
        header = self.ser.read(7)
        if len(header) < 7:
            raise TimeoutError("No response from lidar (check power and wiring)")
        if header[0] != RESP_SYNC1 or header[1] != RESP_SYNC2:
            raise ValueError(f"Bad response header: {header.hex()}")
        size_and_mode = struct.unpack("<I", header[2:6])[0]
        data_size = size_and_mode & 0x3FFFFFFF
        send_mode = (size_and_mode >> 30) & 0x03
        data_type = header[6]
        return data_size, send_mode == 0x00, data_type

    def _read_response(self, expected_size: int) -> bytes:
        data = self.ser.read(expected_size)
        if len(data) < expected_size:
            raise TimeoutError(f"Incomplete response: got {len(data)}/{expected_size} bytes")
        return data

    def stop_scan(self):
        self._send_cmd(CMD_STOP)
        time.sleep(0.05)
        self.ser.reset_input_buffer()

    def reset(self):
        self._send_cmd(CMD_RESET)
        time.sleep(0.5)
        self.ser.reset_input_buffer()

    def get_info(self) -> dict:
        self.ser.reset_input_buffer()
        self._send_cmd(CMD_GET_INFO)
        data_size, _, _ = self._read_descriptor()
        data = self._read_response(20)
        model = data[0]
        fw_minor = data[1]
        fw_major = data[2]
        hardware = data[3]
        serial_num = data[4:20].hex().upper()
        return {
            "model": model,
            "model_name": MODEL_NAMES.get(model, f"Unknown (0x{model:02X})"),
            "firmware": f"{fw_major}.{fw_minor:02d}",
            "hardware": hardware,
            "serial": serial_num,
        }

    def get_health(self) -> dict:
        self.ser.reset_input_buffer()
        self._send_cmd(CMD_GET_HEALTH)
        data_size, _, _ = self._read_descriptor()
        data = self._read_response(3)
        status = data[0]
        error_code = struct.unpack("<H", data[1:3])[0]
        return {
            "status": status,
            "status_name": HEALTH_NAMES.get(status, f"Unknown ({status})"),
            "error_code": error_code,
        }

    def scan(self, max_points: int = 720) -> list[dict]:
        """Start a scan and collect points. Returns list of {angle, distance, quality}."""
        self.ser.reset_input_buffer()
        self._send_cmd(CMD_SCAN)
        self._read_descriptor()

        points = []
        rotations = 0
        prev_angle = 0.0

        while len(points) < max_points:
            raw = self.ser.read(5)
            if len(raw) < 5:
                break

            quality = (raw[0] >> 2) & 0x3F
            start_flag = raw[0] & 0x01
            angle_raw = ((raw[2] << 8) | raw[1]) >> 1
            angle = angle_raw / 64.0
            distance_raw = (raw[4] << 8) | raw[3]
            distance = distance_raw / 4.0

            if start_flag and len(points) > 0:
                rotations += 1
                if rotations >= 3:
                    break

            if distance > 0 and quality > 0:
                points.append({
                    "angle": round(angle, 2),
                    "distance": round(distance, 1),
                    "quality": quality,
                })

        self.stop_scan()
        return points


def print_info(info: dict):
    print("=" * 50)
    print("LIDAR DEVICE INFO")
    print("=" * 50)
    print(f"  Model      : {info['model_name']}")
    print(f"  Firmware   : {info['firmware']}")
    print(f"  Hardware   : Rev {info['hardware']}")
    print(f"  Serial     : {info['serial']}")


def print_health(health: dict):
    print()
    print(f"  Health     : {health['status_name']}", end="")
    if health["error_code"]:
        print(f"  (error code: 0x{health['error_code']:04X})")
    else:
        print()


def print_scan_summary(points: list[dict]):
    if not points:
        print("\n  No scan data received.")
        return

    distances = [p["distance"] for p in points]
    qualities = [p["quality"] for p in points]
    min_d = min(distances)
    max_d = max(distances)
    avg_d = sum(distances) / len(distances)
    avg_q = sum(qualities) / len(qualities)

    print()
    print("=" * 50)
    print("SCAN RESULTS")
    print("=" * 50)
    print(f"  Points     : {len(points)}")
    print(f"  Distance   : min={min_d:.0f}mm  max={max_d:.0f}mm  avg={avg_d:.0f}mm")
    print(f"  Quality    : avg={avg_q:.1f} / 63")

    # Simple ASCII "radar" — 8 sectors
    sectors = [[] for _ in range(8)]
    for p in points:
        idx = int(p["angle"] / 45) % 8
        sectors[idx].append(p["distance"])

    sector_labels = ["  0° (front)", " 45°", " 90° (right)", "135°",
                     "180° (back)", "225°", "270° (left)", "315°"]
    print()
    print("  Sector averages:")
    for i, label in enumerate(sector_labels):
        if sectors[i]:
            avg = sum(sectors[i]) / len(sectors[i])
            count = len(sectors[i])
            bar_len = min(40, int(avg / 100))
            bar = "█" * bar_len
            print(f"    {label:>14s} : {avg:6.0f}mm ({count:3d}pts) {bar}")
        else:
            print(f"    {label:>14s} :   -- no data --")


def main():
    parser = argparse.ArgumentParser(description="SLAMTEC RPLIDAR test script")
    parser.add_argument("-p", "--port", default="/dev/lidar",
                        help="Serial port (default: /dev/lidar)")
    parser.add_argument("-b", "--baud", type=int, default=230400,
                        help="Baud rate (default: 230400 for T-MINI Plus)")
    parser.add_argument("-s", "--scan", action="store_true",
                        help="Run a short scan (~3 rotations)")
    parser.add_argument("-n", "--points", type=int, default=720,
                        help="Max points to collect during scan (default: 720)")
    args = parser.parse_args()

    print(f"Port: {args.port}  Baud: {args.baud}")
    print()

    try:
        lidar = RPLidar(args.port, args.baud)
    except serial.SerialException as e:
        print(f"Cannot open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        info = lidar.get_info()
        print_info(info)

        health = lidar.get_health()
        print_health(health)

        if health["status"] == HEALTH_ERROR:
            print("\n  Lidar reports error — attempting reset...")
            lidar.reset()
            health = lidar.get_health()
            print_health(health)

        if args.scan:
            print("\n  Scanning...")
            points = lidar.scan(max_points=args.points)
            print_scan_summary(points)
        else:
            print("\n  Use --scan to run a measurement scan.")

        print()
        print("Lidar OK.")

    except TimeoutError as e:
        print(f"\nError: {e}", file=sys.stderr)
        print("Check that the lidar motor is spinning (needs 5V on motor pin).", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        lidar.close()


if __name__ == "__main__":
    main()
