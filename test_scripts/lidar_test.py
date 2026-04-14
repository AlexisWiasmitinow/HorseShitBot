#!/usr/bin/env python3
"""
YDLidar T-MINI Plus test script — device info, health check, and short scan.

Uses the YDLidar serial protocol directly (only needs pyserial).

Usage:
    python3 lidar_test.py                       # defaults
    python3 lidar_test.py -p /dev/ttyUSB1       # different port
    python3 lidar_test.py --scan                 # run a short scan (~3 rotations)
    python3 lidar_test.py --scan -n 2000         # collect 2000 points
    python3 lidar_test.py --raw                  # dump raw serial bytes (debug)
"""

import argparse
import struct
import sys
import time

import serial

# YDLidar protocol — same framing (0xA5/0x5A) as RPLIDAR, different commands
SYNC = 0xA5
RESP1 = 0xA5
RESP2 = 0x5A

CMD_STOP = 0x65
CMD_SCAN = 0x60
CMD_FORCE_SCAN = 0x61
CMD_RESET = 0x80
CMD_GET_INFO = 0x90
CMD_GET_HEALTH = 0x92

# Scan package header (little-endian 0x55AA)
PKG_HEADER = 0x55AA

HEALTH_NAMES = {0: "Good", 1: "Warning", 2: "Error"}


class YDLidar:
    """Minimal YDLidar driver for T-MINI Plus."""

    def __init__(self, port: str, baudrate: int = 230400, timeout: float = 3.0):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout,
                                 parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        self.ser.dtr = False
        time.sleep(0.1)
        self.ser.dtr = True
        time.sleep(1.0)

        # Drain boot banner (e.g. "[Platform]:Welcome to Tmini-Plus\n")
        boot_msg = b""
        deadline = time.time() + 2.0
        while time.time() < deadline:
            chunk = self.ser.read(256)
            if chunk:
                boot_msg += chunk
            elif boot_msg:
                break
        if boot_msg:
            text = boot_msg.decode("ascii", errors="replace").strip()
            print(f"  Boot: {text}")

        # Stop any auto-scan, drain residual
        self._send(CMD_STOP)
        time.sleep(0.1)
        self._send(CMD_STOP)
        time.sleep(0.3)
        self._drain()

    def close(self):
        try:
            self._send(CMD_STOP)
            time.sleep(0.05)
        except Exception:
            pass
        self.ser.close()

    def _send(self, cmd: int):
        self.ser.write(bytes([SYNC, cmd]))

    def _drain(self):
        while self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)
            time.sleep(0.02)
        self.ser.reset_input_buffer()

    def _read_header(self) -> tuple[int, int, int]:
        """Read 7-byte response header. Returns (data_size, send_mode, data_type)."""
        hdr = self.ser.read(7)
        if len(hdr) < 7:
            raise TimeoutError(f"No response (got {len(hdr)} bytes: {hdr.hex()})")
        if hdr[0] != RESP1 or hdr[1] != RESP2:
            raise ValueError(f"Bad header: {hdr.hex()}")
        size_mode = struct.unpack("<I", hdr[2:6])[0]
        return size_mode & 0x3FFFFFFF, (size_mode >> 30) & 0x03, hdr[6]

    def get_info(self) -> dict:
        self._drain()
        self._send(CMD_GET_INFO)
        _, _, _ = self._read_header()
        data = self.ser.read(20)
        if len(data) < 20:
            raise TimeoutError(f"Incomplete device info ({len(data)} bytes)")
        model = data[0]
        fw = f"{data[2]}.{data[1]:02d}"
        hw = data[3]
        sn = data[4:20].hex().upper()
        return {"model": model, "firmware": fw, "hardware": hw, "serial": sn}

    def get_health(self) -> dict:
        self._drain()
        self._send(CMD_GET_HEALTH)
        _, _, _ = self._read_header()
        data = self.ser.read(3)
        if len(data) < 3:
            raise TimeoutError("Incomplete health response")
        status = data[0]
        error_code = struct.unpack("<H", data[1:3])[0]
        return {
            "status": status,
            "status_name": HEALTH_NAMES.get(status, f"Unknown ({status})"),
            "error_code": error_code,
        }

    def scan(self, max_points: int = 720) -> list[dict]:
        """Start scan, collect points. Each point has angle (deg), distance (mm), intensity."""
        self._drain()
        self._send(CMD_SCAN)
        # Read scan response header
        self._read_header()

        points = []
        rotations = 0
        prev_start = False

        while len(points) < max_points:
            # Find package header 0xAA55
            b0 = self.ser.read(1)
            if not b0:
                break
            if b0[0] != 0xAA:
                continue
            b1 = self.ser.read(1)
            if not b1 or b1[0] != 0x55:
                continue

            # Read package metadata: CT(1) + count(1) + firstAngle(2) + lastAngle(2) + checksum(2)
            meta = self.ser.read(8)
            if len(meta) < 8:
                break

            ct = meta[0]
            count = meta[1]
            first_angle = struct.unpack("<H", meta[2:4])[0]
            last_angle = struct.unpack("<H", meta[4:6])[0]

            is_start = (ct & 0x01) == 1
            if is_start and points:
                rotations += 1
                if rotations >= 3:
                    break

            if count == 0:
                continue

            # T-MINI Plus with intensity: 3 bytes per node (intensity:1 + distance:2)
            node_bytes = self.ser.read(count * 3)
            if len(node_bytes) < count * 3:
                break

            fa = (first_angle >> 1) / 64.0
            la = (last_angle >> 1) / 64.0

            for i in range(count):
                off = i * 3
                intensity = node_bytes[off]
                dist = struct.unpack("<H", node_bytes[off + 1:off + 3])[0]
                distance_mm = dist / 4.0

                if count > 1:
                    angle_diff = la - fa
                    if angle_diff < 0:
                        angle_diff += 360.0
                    angle = fa + angle_diff * i / (count - 1)
                else:
                    angle = fa

                if angle >= 360.0:
                    angle -= 360.0

                if distance_mm > 0:
                    points.append({
                        "angle": round(angle, 2),
                        "distance": round(distance_mm, 1),
                        "quality": intensity,
                    })

        self._send(CMD_STOP)
        time.sleep(0.1)
        self._drain()
        return points


def dump_raw(port: str, baud: int, seconds: float = 3.0):
    """Read raw bytes to check if lidar sends anything."""
    print(f"\n  Raw dump ({baud} baud, {seconds}s):")
    ser = serial.Serial(port, baudrate=baud, timeout=0.5)
    ser.dtr = False
    time.sleep(0.1)
    ser.dtr = True
    time.sleep(1.0)
    ser.reset_input_buffer()

    # Send YDLidar reset + get_info
    ser.write(bytes([SYNC, CMD_RESET]))
    time.sleep(0.5)
    ser.write(bytes([SYNC, CMD_GET_INFO]))
    time.sleep(0.3)

    end = time.time() + seconds
    total = 0
    while time.time() < end:
        chunk = ser.read(256)
        if chunk:
            total += len(chunk)
            hex_str = chunk[:48].hex(" ")
            ascii_str = chunk[:48].decode("ascii", errors=".")
            if len(chunk) > 48:
                hex_str += " ..."
            print(f"    [{len(chunk):4d} bytes] {hex_str}")
            printable = "".join(c if 32 <= ord(c) < 127 else "." for c in ascii_str)
            print(f"             ascii: {printable}")
    ser.close()
    if total == 0:
        print("    (no data received)")
    else:
        print(f"    Total: {total} bytes in {seconds}s")
    return total


def print_info(info: dict):
    print()
    print("=" * 50)
    print("LIDAR DEVICE INFO")
    print("=" * 50)
    print(f"  Model      : {info['model']} (0x{info['model']:02X})")
    print(f"  Firmware   : {info['firmware']}")
    print(f"  Hardware   : Rev {info['hardware']}")
    print(f"  Serial     : {info['serial']}")


def print_scan_summary(points: list[dict]):
    if not points:
        print("\n  No scan data received.")
        return

    distances = [p["distance"] for p in points]
    qualities = [p["quality"] for p in points]

    print()
    print("=" * 50)
    print("SCAN RESULTS")
    print("=" * 50)
    print(f"  Points     : {len(points)}")
    print(f"  Distance   : min={min(distances):.0f}mm  max={max(distances):.0f}mm  avg={sum(distances)/len(distances):.0f}mm")
    print(f"  Intensity  : avg={sum(qualities)/len(qualities):.0f} / 255")

    sectors = [[] for _ in range(8)]
    for p in points:
        idx = int(p["angle"] / 45) % 8
        sectors[idx].append(p["distance"])

    labels = ["  0° (front)", " 45°", " 90° (right)", "135°",
              "180° (back)", "225°", "270° (left)", "315°"]
    print()
    print("  Sector averages:")
    for i, label in enumerate(labels):
        if sectors[i]:
            avg = sum(sectors[i]) / len(sectors[i])
            bar = "█" * min(40, int(avg / 100))
            print(f"    {label:>14s} : {avg:6.0f}mm ({len(sectors[i]):3d}pts) {bar}")
        else:
            print(f"    {label:>14s} :   -- no data --")


def main():
    parser = argparse.ArgumentParser(description="YDLidar T-MINI Plus test script")
    parser.add_argument("-p", "--port", default="/dev/lidar",
                        help="Serial port (default: /dev/lidar)")
    parser.add_argument("-b", "--baud", type=int, default=230400,
                        help="Baud rate (default: 230400)")
    parser.add_argument("-s", "--scan", action="store_true",
                        help="Run a short scan (~3 rotations)")
    parser.add_argument("-n", "--points", type=int, default=720,
                        help="Max points to collect (default: 720)")
    parser.add_argument("--raw", action="store_true",
                        help="Dump raw serial data (debug)")
    args = parser.parse_args()

    print(f"Port: {args.port}  Baud: {args.baud}")

    if args.raw:
        dump_raw(args.port, args.baud)
        return

    try:
        lidar = YDLidar(args.port, args.baud)
    except serial.SerialException as e:
        print(f"Cannot open {args.port}: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        info = lidar.get_info()
        print_info(info)

        health = lidar.get_health()
        print(f"\n  Health     : {health['status_name']}", end="")
        if health["error_code"]:
            print(f"  (error: 0x{health['error_code']:04X})")
        else:
            print()

        if args.scan:
            print("\n  Scanning...")
            points = lidar.scan(max_points=args.points)
            print_scan_summary(points)
        else:
            print("\n  Use --scan / -s to run a measurement scan.")

        print("\nLidar OK.")

    except TimeoutError as e:
        print(f"\nError: {e}", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        lidar.close()


if __name__ == "__main__":
    main()
