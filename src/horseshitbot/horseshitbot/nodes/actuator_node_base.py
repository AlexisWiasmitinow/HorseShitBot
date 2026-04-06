"""
Shared base class for actuator nodes (lift, brush, bin_door).

Each concrete node just sets class-level defaults and inherits
all parameter declarations, subscriptions, services, and the tick loop.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from horseshitbot_interfaces.srv import MksSetSpeed, ActuatorCommand
from horseshitbot_interfaces.msg import ActuatorState as ActuatorStateMsg

from ..drivers.actuator import Actuator, ActuatorConfig, ActuatorState


class ActuatorNodeBase(Node):
    NODE_NAME: str = "actuator_node"
    TOPIC_PREFIX: str = "actuator"
    DEFAULT_MOTOR_IDS: list[int] = [4]
    DEFAULT_DUAL: bool = False

    def __init__(self):
        super().__init__(self.NODE_NAME)

        # Declare parameters
        if self.DEFAULT_DUAL:
            self.declare_parameter("motor_id_a", self.DEFAULT_MOTOR_IDS[0])
            self.declare_parameter("motor_id_b", self.DEFAULT_MOTOR_IDS[1] if len(self.DEFAULT_MOTOR_IDS) > 1 else self.DEFAULT_MOTOR_IDS[0])
        else:
            self.declare_parameter("motor_id", self.DEFAULT_MOTOR_IDS[0])

        self.declare_parameter("dual_motor", self.DEFAULT_DUAL)
        self.declare_parameter("invert_dir", False)
        self.declare_parameter("open_speed_rpm", 300.0)
        self.declare_parameter("close_speed_rpm", 200.0)
        self.declare_parameter("open_accel", 3)
        self.declare_parameter("close_accel", 3)
        self.declare_parameter("ref_speed_rpm", 100.0)
        self.declare_parameter("ref_direction", "close")
        self.declare_parameter("stall_confirm_ms", 200)
        self.declare_parameter("watchdog_sec", 0.8)

        dual = self.get_parameter("dual_motor").get_parameter_value().bool_value
        if dual:
            id_a = self.get_parameter("motor_id_a").get_parameter_value().integer_value
            id_b = self.get_parameter("motor_id_b").get_parameter_value().integer_value
            motor_ids = [id_a, id_b]
        else:
            motor_ids = [self.get_parameter("motor_id").get_parameter_value().integer_value]

        cfg = ActuatorConfig(
            motor_ids=motor_ids,
            dual_motor=dual,
            invert_dir=self.get_parameter("invert_dir").get_parameter_value().bool_value,
            open_speed_rpm=self.get_parameter("open_speed_rpm").get_parameter_value().double_value,
            close_speed_rpm=self.get_parameter("close_speed_rpm").get_parameter_value().double_value,
            open_accel=self.get_parameter("open_accel").get_parameter_value().integer_value,
            close_accel=self.get_parameter("close_accel").get_parameter_value().integer_value,
            ref_speed_rpm=self.get_parameter("ref_speed_rpm").get_parameter_value().double_value,
            ref_direction=self.get_parameter("ref_direction").get_parameter_value().string_value,
            stall_confirm_ms=self.get_parameter("stall_confirm_ms").get_parameter_value().integer_value,
            watchdog_sec=self.get_parameter("watchdog_sec").get_parameter_value().double_value,
        )

        # MKS bus service client
        self._mks_cli = self.create_client(MksSetSpeed, "/mks/set_speed")

        self._actuator = Actuator(cfg=cfg, set_speed_fn=self._mks_set_speed)

        # Topic subscription
        self.create_subscription(
            String, f"/{self.TOPIC_PREFIX}/command", self._cb_command, 10
        )

        # State publisher
        self._state_pub = self.create_publisher(
            ActuatorStateMsg, f"/{self.TOPIC_PREFIX}/state", 10
        )

        # Services
        self.create_service(Trigger, "~/stop", self._srv_stop)
        self.create_service(Trigger, "~/reference", self._srv_reference)
        self.create_service(ActuatorCommand, "~/open", self._srv_open)
        self.create_service(ActuatorCommand, "~/close", self._srv_close)

        # Tick timer (~20 Hz for stall detection polling)
        self.create_timer(0.05, self._tick)

        self.get_logger().info(
            f"{self.NODE_NAME} started (motors={motor_ids}, dual={dual})"
        )

    def _mks_set_speed(self, motor_id, rpm, acc, invert) -> bool:
        req = MksSetSpeed.Request()
        req.motor_id = int(motor_id)
        req.rpm = float(rpm)
        req.accel = int(acc)
        req.invert_dir = bool(invert)
        future = self._mks_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=0.5)
        if future.result() is not None:
            return future.result().success
        return False

    # ── callbacks ────────────────────────────────────────────────

    def _cb_command(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == "open":
            self._actuator.open()
        elif cmd == "close":
            self._actuator.close()
        elif cmd == "stop":
            self._actuator.stop()
        elif cmd == "reference":
            self._actuator.reference()

    def _srv_stop(self, request, response):
        ok, msg = self._actuator.stop()
        response.success = ok
        response.message = msg
        return response

    def _srv_reference(self, request, response):
        ok, msg = self._actuator.reference()
        response.success = ok
        response.message = msg
        return response

    def _srv_open(self, request, response):
        ok, msg = self._actuator.open(speed_override=request.speed_override)
        response.success = ok
        response.message = msg
        return response

    def _srv_close(self, request, response):
        ok, msg = self._actuator.close(speed_override=request.speed_override)
        response.success = ok
        response.message = msg
        return response

    # ── tick & publish ───────────────────────────────────────────

    def _tick(self):
        # TODO: read actual stall status from MKS servo register via bus node
        # For now, stall detection is not wired (always False).
        stall_detected = False
        self._actuator.tick(stall_detected)

        st = self._actuator.get_state_dict()
        msg = ActuatorStateMsg()
        msg.state = st["state"]
        msg.is_referenced = st["is_referenced"]
        msg.direction = st["direction"]
        msg.speed_rpm = st["speed_rpm"]
        msg.error_message = st["error_message"]
        self._state_pub.publish(msg)
