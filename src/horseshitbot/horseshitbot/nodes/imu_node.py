#!/usr/bin/env python3
"""ROS 2 node for the ICM-20948 IMU."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField, Temperature

from horseshitbot.drivers.icm20948 import ICM20948


GRAVITY_M_S2 = 9.80665
DEGREES_TO_RADIANS = math.pi / 180.0
MICROTESLA_TO_TESLA = 1.0e-6


class ImuNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_node")

        self.declare_parameter("bus", 7)
        self.declare_parameter("address", 105)
        self.declare_parameter("publish_hz", 50.0)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("accel_full_scale_g", 2)
        self.declare_parameter("gyro_full_scale_dps", 250)
        self.declare_parameter("use_magnetometer", True)

        bus = int(self.get_parameter("bus").value)
        address = int(self.get_parameter("address").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        self._frame_id = str(self.get_parameter("frame_id").value)
        self._use_magnetometer = bool(
            self.get_parameter("use_magnetometer").value
        )

        accel_full_scale_g = int(
            self.get_parameter("accel_full_scale_g").value
        )
        gyro_full_scale_dps = int(
            self.get_parameter("gyro_full_scale_dps").value
        )

        if publish_hz <= 0.0:
            raise ValueError("publish_hz must be greater than zero")

        self._driver = ICM20948(
            bus=bus,
            address=address,
        )

        self._driver.initialize(
            accel_full_scale_g=accel_full_scale_g,
            gyro_full_scale_dps=gyro_full_scale_dps,
        )

        self._magnetometer_available = False

        if self._use_magnetometer:
            try:
                self._driver.enable_magnetometer_bypass()
                self._driver.initialize_magnetometer()
                self._magnetometer_available = True
            except (OSError, RuntimeError) as exc:
                self.get_logger().warning(
                    f"Magnetometer unavailable; continuing with "
                    f"accelerometer and gyroscope only: {exc}"
                )

        self._imu_publisher = self.create_publisher(
            Imu,
            "/imu/data_raw",
            qos_profile_sensor_data,
        )

        self._mag_publisher = self.create_publisher(
            MagneticField,
            "/imu/mag",
            qos_profile_sensor_data,
        )

        self._temperature_publisher = self.create_publisher(
            Temperature,
            "/imu/temperature",
            qos_profile_sensor_data,
        )

        self._last_read_error_time = 0.0
        self._last_mag_error_time = 0.0

        self._timer = self.create_timer(
            1.0 / publish_hz,
            self._publish_measurement,
        )

        self.get_logger().info(
            f"ICM-20948 initialized on /dev/i2c-{bus}, "
            f"address=0x{address:02X}, "
            f"rate={publish_hz:.1f} Hz, "
            f"frame={self._frame_id}, "
            f"magnetometer={self._magnetometer_available}"
        )

    def _publish_measurement(self) -> None:
        try:
            sample = self._driver.read_accel_gyro_temp()
        except (OSError, RuntimeError) as exc:
            self._log_throttled_read_error(str(exc))
            return

        stamp = self.get_clock().now().to_msg()

        ax_g, ay_g, az_g = sample["accel_g"]
        gx_dps, gy_dps, gz_dps = sample["gyro_dps"]

        imu_message = Imu()
        imu_message.header.stamp = stamp
        imu_message.header.frame_id = self._frame_id

        # This node publishes raw measurements and does not calculate
        # an orientation quaternion.
        imu_message.orientation.w = 1.0
        imu_message.orientation_covariance[0] = -1.0

        imu_message.linear_acceleration.x = ax_g * GRAVITY_M_S2
        imu_message.linear_acceleration.y = ay_g * GRAVITY_M_S2
        imu_message.linear_acceleration.z = az_g * GRAVITY_M_S2

        imu_message.angular_velocity.x = gx_dps * DEGREES_TO_RADIANS
        imu_message.angular_velocity.y = gy_dps * DEGREES_TO_RADIANS
        imu_message.angular_velocity.z = gz_dps * DEGREES_TO_RADIANS

        self._imu_publisher.publish(imu_message)

        temperature_message = Temperature()
        temperature_message.header.stamp = stamp
        temperature_message.header.frame_id = self._frame_id
        temperature_message.temperature = float(sample["temp_c"])
        temperature_message.variance = 0.0

        self._temperature_publisher.publish(temperature_message)

        if self._magnetometer_available:
            self._publish_magnetometer(stamp)

    def _publish_magnetometer(self, stamp) -> None:
        try:
            magnetometer = self._driver.read_magnetometer(timeout=0.03)
        except (OSError, TimeoutError) as exc:
            now = time.monotonic()

            if now - self._last_mag_error_time >= 5.0:
                self.get_logger().warning(
                    f"Magnetometer read failed: {exc}"
                )
                self._last_mag_error_time = now

            return

        if magnetometer["overflow"]:
            now = time.monotonic()

            if now - self._last_mag_error_time >= 5.0:
                self.get_logger().warning(
                    "Magnetometer reported measurement overflow"
                )
                self._last_mag_error_time = now

            return

        mx_ut, my_ut, mz_ut = magnetometer["mag_ut"]

        magnetic_message = MagneticField()
        magnetic_message.header.stamp = stamp
        magnetic_message.header.frame_id = self._frame_id

        magnetic_message.magnetic_field.x = mx_ut * MICROTESLA_TO_TESLA
        magnetic_message.magnetic_field.y = my_ut * MICROTESLA_TO_TESLA
        magnetic_message.magnetic_field.z = mz_ut * MICROTESLA_TO_TESLA

        self._mag_publisher.publish(magnetic_message)

    def _log_throttled_read_error(self, error: str) -> None:
        now = time.monotonic()

        if now - self._last_read_error_time >= 5.0:
            self.get_logger().error(f"ICM-20948 read failed: {error}")
            self._last_read_error_time = now

    def destroy_node(self) -> None:
        try:
            self._driver.close()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = ImuNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()