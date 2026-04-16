"""
Wheel Driver Node — subscribes to /cmd_vel, applies ramping, dispatches
to the active wheel backend (MKS steppers or ODrive), publishes
nav_msgs/Odometry, and exposes a service to switch backends at runtime.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from horseshitbot_interfaces.srv import MksSetSpeed, SwitchBackend

from ..drivers.wheel_backend import WheelBackend
from ..drivers.mks_wheel_backend import MksWheelBackend
from ..drivers.odrive_wheel_backend import ODriveWheelBackend


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def _sign(x):
    if x > 0:
        return 1
    if x < 0:
        return -1
    return 0


def _ramp_toward(current, target, rate, dt):
    max_delta = max(0.0, rate) * dt
    delta = _clamp(target - current, -max_delta, max_delta)
    nxt = current + delta
    if abs(nxt) < 0.5:
        nxt = 0.0
    return nxt


class WheelDriverNode(Node):
    def __init__(self):
        super().__init__("wheel_driver_node")

        # Declare parameters
        self.declare_parameter("wheel_backend", "mks")
        self.declare_parameter("id_left", 1)
        self.declare_parameter("id_right", 2)
        self.declare_parameter("invert_left_dir", False)
        self.declare_parameter("invert_right_dir", True)
        self.declare_parameter("max_speed_ms", 3.0)
        self.declare_parameter("wheel_acc_reg", 3)
        self.declare_parameter("update_hz", 50.0)
        self.declare_parameter("accel_rpm_s", 120.0)
        self.declare_parameter("decel_rpm_s", 80.0)
        self.declare_parameter("stop_decel_rpm_s", 500.0)
        self.declare_parameter("watchdog_sec", 0.8)
        self.declare_parameter("odrive_port", "/dev/odrive")
        self.declare_parameter("odrive_baud", 115200)
        self.declare_parameter("odrive_vel_limit", 100.0)
        self.declare_parameter("odrive_current_lim", 20.0)
        self.declare_parameter("odrive_vel_ramp_rate", 20.0)
        # Odometry: differential-drive geometry
        self.declare_parameter("meters_per_motor_turn", 0.093333)
        self.declare_parameter("wheel_separation", 0.40)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("publish_odom_tf", True)

        self._meters_per_turn = self._p_float("meters_per_motor_turn")
        max_speed = self._p_float("max_speed_ms")
        self._max_rpm = (max_speed / self._meters_per_turn) * 60.0
        self._update_hz = self._p_float("update_hz")
        self._accel = self._p_float("accel_rpm_s")
        self._decel = self._p_float("decel_rpm_s")
        self._stop_decel = self._p_float("stop_decel_rpm_s")
        self._watchdog = self._p_float("watchdog_sec")

        self._lock = threading.Lock()
        self._desired_left = 0.0
        self._desired_right = 0.0
        self._actual_left = 0.0
        self._actual_right = 0.0
        self._last_cmd_ts = 0.0
        self._stop_fast = False
        self._estopped = False

        # Odometry state (integrated from commanded RPM)
        self._wheel_sep = self._p_float("wheel_separation")
        self._odom_frame = self._p_str("odom_frame_id")
        self._base_frame = self._p_str("base_frame_id")
        self._publish_odom_tf = self._p_bool("publish_odom_tf")
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_theta = 0.0

        # MKS bus service client (used when backend == mks)
        self._mks_cli = self.create_client(MksSetSpeed, "/mks/set_speed")

        # Active backend
        self._backend: WheelBackend | None = None
        self._backend_name = ""
        self._backend_error = ""
        self._backend_diag: dict = {}

        # Subscriptions and services
        self.create_subscription(Twist, "/cmd_vel", self._cb_cmd_vel, 10)
        self.create_service(SwitchBackend, "~/switch_backend", self._srv_switch)
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.create_service(Trigger, "~/stop_fast", self._srv_stop_fast)

        # Status publisher
        self._status_pub = self.create_publisher(String, "/wheel_status", 10)
        self._odom_pub = self.create_publisher(Odometry, "/odom", 50)
        self._tf_broadcaster = TransformBroadcaster(self)

        # Activate initial backend
        backend_name = self.get_parameter("wheel_backend").get_parameter_value().string_value
        self._activate_backend(backend_name)

        # Control loop timer
        period = 1.0 / max(1.0, self._update_hz)
        self._last_tick = time.monotonic()
        self.create_timer(period, self._control_loop)

        # Slower diagnostics timer (temps, voltages) — every 5 s
        self.create_timer(5.0, self._diag_tick)

        self.get_logger().info(f"Wheel driver started (backend={self._backend_name})")

    # ── helpers ───────────────────────────────────────────────────

    def _p_float(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _p_int(self, name):
        return self.get_parameter(name).get_parameter_value().integer_value

    def _p_bool(self, name):
        return self.get_parameter(name).get_parameter_value().bool_value

    def _p_str(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _mks_set_speed(self, motor_id: int, rpm: float, acc: int, invert: bool) -> bool:
        req = MksSetSpeed.Request()
        req.motor_id = motor_id
        req.rpm = float(rpm)
        req.accel = acc
        req.invert_dir = invert
        future = self._mks_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if future.result() is not None:
            return future.result().success
        return False

    # ── backend management ───────────────────────────────────────

    def _activate_backend(self, name: str) -> tuple[bool, str]:
        if self._backend is not None:
            try:
                self._backend.emergency_stop()
                self._backend.disconnect()
            except Exception:
                pass
            self._backend = None

        name = name.lower().strip()
        try:
            if name == "odrive":
                self._backend = ODriveWheelBackend(
                    port=self._p_str("odrive_port"),
                    baudrate=self._p_int("odrive_baud"),
                    vel_limit=self._p_float("odrive_vel_limit"),
                    current_lim=self._p_float("odrive_current_lim"),
                    vel_ramp_rate=self._p_float("odrive_vel_ramp_rate"),
                    invert_left=self._p_bool("invert_left_dir"),
                    invert_right=self._p_bool("invert_right_dir"),
                )
            else:
                name = "mks"
                self._backend = MksWheelBackend(
                    set_speed_fn=self._mks_set_speed,
                    id_left=self._p_int("id_left"),
                    id_right=self._p_int("id_right"),
                    invert_left=self._p_bool("invert_left_dir"),
                    invert_right=self._p_bool("invert_right_dir"),
                    acc_reg=self._p_int("wheel_acc_reg"),
                )

            self._backend.connect()
            self._backend_name = name
            self._backend_error = ""
            self.get_logger().info(f"Backend activated: {name}")
            return True, f"switched to {name}"
        except Exception as e:
            self._backend = None
            self._backend_name = "none"
            self._backend_error = str(e)
            msg = f"Failed to activate {name}: {e}"
            self.get_logger().error(msg)
            return False, msg

    # ── callbacks ────────────────────────────────────────────────

    def _cb_cmd_vel(self, msg: Twist):
        linear = _clamp(msg.linear.x, -1.0, 1.0)
        angular = _clamp(msg.angular.z, -1.0, 1.0)

        dl = _clamp((linear - angular) * self._max_rpm, -self._max_rpm, self._max_rpm)
        dr = _clamp((linear + angular) * self._max_rpm, -self._max_rpm, self._max_rpm)

        with self._lock:
            if not self._stop_fast and not self._estopped:
                self._desired_left = dl
                self._desired_right = dr
                self._last_cmd_ts = time.monotonic()

    def _srv_switch(self, request, response):
        ok, msg = self._activate_backend(request.backend)
        response.success = ok
        response.message = msg
        return response

    def _srv_stop(self, request, response):
        self.get_logger().info(f"Stop service called (estopped={self._estopped})")
        with self._lock:
            self._desired_left = 0.0
            self._desired_right = 0.0
            self._last_cmd_ts = time.monotonic()
            self._stop_fast = False
            was_estopped = self._estopped

        if was_estopped:
            if self._backend:
                ok = self._backend.resume()
                if ok:
                    self.get_logger().info("E-stop cleared, motors re-enabled")
                else:
                    self.get_logger().error("Failed to resume motors after e-stop")
                    response.success = False
                    response.message = "resume failed"
                    return response
            else:
                self.get_logger().warn("No backend connected — clearing e-stop flag only")

            with self._lock:
                self._estopped = False

        response.success = True
        response.message = "stopped" if not was_estopped else "e-stop cleared"
        return response

    def _srv_stop_fast(self, request, response):
        with self._lock:
            self._desired_left = 0.0
            self._desired_right = 0.0
            self._last_cmd_ts = time.monotonic()
            self._stop_fast = True
            self._estopped = True
        if self._backend:
            self._backend.emergency_stop()
        self.get_logger().warn("E-STOP activated. Press B to resume.")
        response.success = True
        response.message = "emergency stop"
        return response

    # ── diagnostics ────────────────────────────────────────────

    def _diag_tick(self):
        if self._backend:
            try:
                self._backend_diag = self._backend.get_diagnostics()
            except Exception:
                self._backend_diag = {}
        else:
            self._backend_diag = {}

    # ── control loop ─────────────────────────────────────────────

    def _control_loop(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now

        with self._lock:
            stop_fast = self._stop_fast
            age = now - self._last_cmd_ts
            dl = self._desired_left
            dr = self._desired_right

        if not stop_fast and age > self._watchdog:
            dl, dr = 0.0, 0.0

        decel = self._stop_decel if stop_fast else self._decel
        self._actual_left = _ramp_toward(self._actual_left, dl, self._accel if abs(dl) >= abs(self._actual_left) else decel, dt)
        self._actual_right = _ramp_toward(self._actual_right, dr, self._accel if abs(dr) >= abs(self._actual_right) else decel, dt)

        if self._backend:
            try:
                self._backend.send_velocity(self._actual_left, self._actual_right)
            except Exception:
                pass

        # Odometry integration from actual (ramped) RPM
        self._update_odometry(self._actual_left, self._actual_right, dt)

        if stop_fast and self._actual_left == 0.0 and self._actual_right == 0.0:
            with self._lock:
                self._stop_fast = False

        import json
        with self._lock:
            estopped = self._estopped
        payload = {
            "backend": self._backend_name,
            "left_rpm": round(self._actual_left, 1),
            "right_rpm": round(self._actual_right, 1),
            "stopping": stop_fast,
            "estopped": estopped,
            "error": self._backend_error,
        }
        if self._backend_diag:
            payload["diag"] = self._backend_diag
        status = json.dumps(payload)
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)


    def _update_odometry(self, left_rpm: float, right_rpm: float, dt: float):
        """Integrate wheel RPMs into odometry and publish."""
        # RPM -> m/s via drive-train ratio (meters_per_motor_turn)
        v_left = left_rpm * self._meters_per_turn / 60.0
        v_right = right_rpm * self._meters_per_turn / 60.0

        v_linear = (v_right + v_left) / 2.0
        v_angular = (v_right - v_left) / self._wheel_sep

        self._odom_theta += v_angular * dt
        self._odom_x += v_linear * math.cos(self._odom_theta) * dt
        self._odom_y += v_linear * math.sin(self._odom_theta) * dt

        now = self.get_clock().now().to_msg()

        # Quaternion from yaw
        cy = math.cos(self._odom_theta * 0.5)
        sy = math.sin(self._odom_theta * 0.5)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._odom_x
        odom.pose.pose.position.y = self._odom_y
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        odom.twist.twist.linear.x = v_linear
        odom.twist.twist.angular.z = v_angular
        self._odom_pub.publish(odom)

        if self._publish_odom_tf:
            tf = TransformStamped()
            tf.header.stamp = now
            tf.header.frame_id = self._odom_frame
            tf.child_frame_id = self._base_frame
            tf.transform.translation.x = self._odom_x
            tf.transform.translation.y = self._odom_y
            tf.transform.rotation.z = sy
            tf.transform.rotation.w = cy
            self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WheelDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._backend:
            try:
                node._backend.emergency_stop()
                node._backend.disconnect()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
