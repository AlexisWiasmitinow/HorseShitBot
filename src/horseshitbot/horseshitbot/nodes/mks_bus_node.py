"""
MKS Bus Node — owns the Modbus RTU serial connection and exposes ROS 2
services so other nodes can command MKS servo motors without serial
port contention.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from horseshitbot.srv import MksSetSpeed
from std_srvs.srv import Trigger

from ..drivers.mks_bus import BusCfg, MksBus, MODE_SR_VFOC


class MksBusNode(Node):
    def __init__(self):
        super().__init__("mks_bus_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 38400)
        self.declare_parameter("timeout", 0.35)
        self.declare_parameter("retries", 3)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        timeout = self.get_parameter("timeout").get_parameter_value().double_value
        retries = self.get_parameter("retries").get_parameter_value().integer_value

        self._bus = MksBus(BusCfg(port=port, baud=baud, timeout=timeout, retries=retries))

        self.create_service(MksSetSpeed, "/mks/set_speed", self._srv_set_speed)
        self.create_service(Trigger, "/mks/init_servo", self._srv_init_servo)
        self.create_service(Trigger, "/mks/clear_errors", self._srv_clear_errors)

        try:
            self._bus.connect()
            self.get_logger().info(f"MKS bus connected on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"MKS bus connection failed: {e}")

    def _srv_set_speed(self, request, response):
        try:
            self._bus.set_speed_signed(
                unit_id=int(request.motor_id),
                rpm_signed=float(request.rpm),
                acc=int(request.accel),
                invert_dir=bool(request.invert_dir),
            )
            response.success = True
        except Exception as e:
            self.get_logger().warning(f"set_speed failed motor={request.motor_id}: {e}")
            response.success = False
        return response

    def _srv_init_servo(self, request, response):
        try:
            for mid in range(1, 7):
                try:
                    self._bus.init_servo(mid, mode=MODE_SR_VFOC, enable=True)
                except Exception:
                    pass
            response.success = True
            response.message = "init_servo done"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _srv_clear_errors(self, request, response):
        try:
            for mid in range(1, 7):
                try:
                    self._bus.clear_error_state(mid, mode=MODE_SR_VFOC)
                except Exception:
                    pass
            response.success = True
            response.message = "errors cleared"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self):
        self._bus.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MksBusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
