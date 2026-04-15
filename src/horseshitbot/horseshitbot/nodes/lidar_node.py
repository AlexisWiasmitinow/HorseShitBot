"""
Lidar Node — owns the YDLidar T-MINI Plus serial port, provides
start/stop services, and publishes scan data for the web dashboard
and as standard sensor_msgs/LaserScan for SLAM / mapping.
"""

from __future__ import annotations

import json
import math
import struct
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

import serial

# YDLidar protocol constants
SYNC = 0xA5
RESP1 = 0xA5
RESP2 = 0x5A
CMD_STOP = 0x65
CMD_SCAN = 0x60
CMD_GET_INFO = 0x90
CMD_GET_HEALTH = 0x92


class _LidarDriver:
    """Minimal YDLidar driver (T-MINI Plus)."""

    def __init__(self, port: str, baud: int = 230400, timeout: float = 3.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser: serial.Serial | None = None

    def connect(self) -> bool:
        try:
            self.ser = serial.Serial(
                self.port, baudrate=self.baud, timeout=self.timeout,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
            )
            self.ser.dtr = False
            time.sleep(0.1)
            self.ser.dtr = True
            time.sleep(1.0)
            # Drain boot banner
            deadline = time.time() + 2.0
            while time.time() < deadline:
                chunk = self.ser.read(256)
                if not chunk:
                    break
            self._stop()
            time.sleep(0.3)
            self._drain()
            return True
        except Exception:
            return False

    def disconnect(self):
        if self.ser and self.ser.is_open:
            try:
                self._stop()
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

    def _stop(self):
        self._send(CMD_STOP)
        time.sleep(0.05)
        self._send(CMD_STOP)

    def _read_header(self) -> tuple[int, int, int]:
        hdr = self.ser.read(7)
        if len(hdr) < 7 or hdr[0] != RESP1 or hdr[1] != RESP2:
            raise TimeoutError("No valid header")
        size_mode = struct.unpack("<I", hdr[2:6])[0]
        return size_mode & 0x3FFFFFFF, (size_mode >> 30) & 0x03, hdr[6]

    def get_info(self) -> dict | None:
        try:
            self._drain()
            self._send(CMD_GET_INFO)
            self._read_header()
            data = self.ser.read(20)
            if len(data) < 20:
                return None
            return {
                "model": data[0],
                "firmware": f"{data[2]}.{data[1]:02d}",
                "hardware": data[3],
                "serial": data[4:20].hex().upper(),
            }
        except Exception:
            return None

    def get_health(self) -> dict | None:
        try:
            self._drain()
            self._send(CMD_GET_HEALTH)
            self._read_header()
            data = self.ser.read(3)
            if len(data) < 3:
                return None
            return {"status": data[0], "error_code": struct.unpack("<H", data[1:3])[0]}
        except Exception:
            return None

    def start_scan(self):
        self._drain()
        self._send(CMD_SCAN)
        self._read_header()

    def stop_scan(self):
        self._stop()
        time.sleep(0.1)
        self._drain()

    def read_scan_packages(self) -> list[tuple[float, float, int]]:
        """Read available packages, return list of (angle_deg, distance_mm, intensity)."""
        points = []
        deadline = time.time() + 0.15

        while time.time() < deadline:
            b0 = self.ser.read(1)
            if not b0:
                break
            if b0[0] != 0xAA:
                continue
            b1 = self.ser.read(1)
            if not b1 or b1[0] != 0x55:
                continue

            meta = self.ser.read(8)
            if len(meta) < 8:
                break

            count = meta[1]
            first_angle = struct.unpack("<H", meta[2:4])[0]
            last_angle = struct.unpack("<H", meta[4:6])[0]

            if count == 0:
                continue

            node_bytes = self.ser.read(count * 3)
            if len(node_bytes) < count * 3:
                break

            fa = (first_angle >> 1) / 64.0
            la = (last_angle >> 1) / 64.0

            for i in range(count):
                off = i * 3
                intensity = node_bytes[off]
                dist = struct.unpack("<H", node_bytes[off + 1:off + 3])[0]
                dist_mm = dist / 4.0

                if count > 1:
                    diff = la - fa
                    if diff < 0:
                        diff += 360.0
                    angle = fa + diff * i / (count - 1)
                else:
                    angle = fa
                if angle >= 360.0:
                    angle -= 360.0

                if dist_mm > 0:
                    points.append((angle, dist_mm, intensity))

        return points


class LidarNode(Node):
    def __init__(self):
        super().__init__("lidar_node")

        self.declare_parameter("port", "/dev/lidar")
        self.declare_parameter("baud", 230400)
        self.declare_parameter("auto_start", False)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("range_min", 0.02)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("scan_bins", 720)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        auto_start = self.get_parameter("auto_start").get_parameter_value().bool_value
        pub_hz = self.get_parameter("publish_hz").get_parameter_value().double_value
        self._frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self._range_min = self.get_parameter("range_min").get_parameter_value().double_value
        self._range_max = self.get_parameter("range_max").get_parameter_value().double_value
        self._scan_bins = self.get_parameter("scan_bins").get_parameter_value().integer_value

        self._driver = _LidarDriver(port, baud)
        self._scanning = False
        self._scan_thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._points: list[tuple[float, float, int]] = []
        self._scan_count = 0
        self._connected = False
        self._device_info: dict = {}

        self._status_pub = self.create_publisher(String, "/lidar/status", 10)
        self._points_pub = self.create_publisher(String, "/lidar/points", 10)
        self._scan_pub = self.create_publisher(LaserScan, "/scan", 10)

        self.create_service(Trigger, "~/start_scan", self._srv_start)
        self.create_service(Trigger, "~/stop_scan", self._srv_stop)

        if self._driver.connect():
            self._connected = True
            info = self._driver.get_info()
            if info:
                self._device_info = info
                self.get_logger().info(
                    f"Lidar connected: model=0x{info['model']:02X} fw={info['firmware']} sn={info['serial'][:8]}…"
                )
            else:
                self.get_logger().warn("Lidar connected but GET_INFO failed")

            if auto_start:
                self._start_scanning()
        else:
            self.get_logger().error(f"Cannot connect to lidar on {port}")

        period = 1.0 / max(0.5, pub_hz)
        self.create_timer(period, self._publish_tick)

    def _start_scanning(self) -> tuple[bool, str]:
        if self._scanning:
            return True, "already scanning"
        if not self._connected:
            return False, "lidar not connected"
        try:
            self._driver.start_scan()
            self._scanning = True
            self._scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
            self._scan_thread.start()
            self.get_logger().info("Scan started")
            return True, "scan started"
        except Exception as e:
            return False, str(e)

    def _stop_scanning(self) -> tuple[bool, str]:
        if not self._scanning:
            return True, "already stopped"
        self._scanning = False
        if self._scan_thread:
            self._scan_thread.join(timeout=3.0)
        try:
            self._driver.stop_scan()
        except Exception:
            pass
        with self._lock:
            self._points.clear()
        self.get_logger().info("Scan stopped")
        return True, "scan stopped"

    def _scan_loop(self):
        while self._scanning:
            try:
                new_points = self._driver.read_scan_packages()
                if new_points:
                    with self._lock:
                        self._points = new_points
                        self._scan_count += 1
            except Exception:
                time.sleep(0.1)

    def _publish_tick(self):
        with self._lock:
            pts = list(self._points)
            count = self._scan_count

        # Status
        status = {
            "connected": self._connected,
            "scanning": self._scanning,
            "point_count": len(pts),
            "scan_count": count,
        }
        if self._device_info:
            status["model"] = self._device_info.get("model", 0)
            status["firmware"] = self._device_info.get("firmware", "")
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

        # Points (downsample to ~360 for the web UI)
        if pts and self._scanning:
            step = max(1, len(pts) // 360)
            sampled = pts[::step]
            # Compact format: [[angle, dist, intensity], ...]
            compact = [[round(a, 1), round(d, 0), q] for a, d, q in sampled]
            msg2 = String()
            msg2.data = json.dumps(compact)
            self._points_pub.publish(msg2)

            self._publish_laser_scan(pts)

    def _publish_laser_scan(self, pts: list[tuple[float, float, int]]):
        """Convert raw (angle_deg, dist_mm, intensity) points into a LaserScan."""
        n = self._scan_bins
        angle_increment = 2.0 * math.pi / n

        ranges = [0.0] * n
        intensities = [0.0] * n

        for angle_deg, dist_mm, intensity in pts:
            rad = math.radians(angle_deg)
            if rad < 0.0:
                rad += 2.0 * math.pi
            idx = int(rad / angle_increment)
            if 0 <= idx < n:
                dist_m = dist_mm / 1000.0
                if self._range_min <= dist_m <= self._range_max:
                    if ranges[idx] == 0.0 or dist_m < ranges[idx]:
                        ranges[idx] = dist_m
                        intensities[idx] = float(intensity)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self._frame_id
        scan.angle_min = 0.0
        scan.angle_max = 2.0 * math.pi
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / max(0.5, self.get_parameter("publish_hz").get_parameter_value().double_value)
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.ranges = ranges
        scan.intensities = intensities
        self._scan_pub.publish(scan)

    def _srv_start(self, request, response):
        ok, msg = self._start_scanning()
        response.success = ok
        response.message = msg
        return response

    def _srv_stop(self, request, response):
        ok, msg = self._stop_scanning()
        response.success = ok
        response.message = msg
        return response

    def destroy_node(self):
        self._stop_scanning()
        self._driver.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
