"""
Lift Node — dual-motor (3+5, counter-rotating) actuator with open/close
commands and stall-based referencing, using the generic Actuator driver.
"""

from __future__ import annotations

from .actuator_node_base import ActuatorNodeBase


class LiftNode(ActuatorNodeBase):
    NODE_NAME = "lift_node"
    TOPIC_PREFIX = "lift"
    DEFAULT_MOTOR_IDS = [3, 5]
    DEFAULT_DUAL = True


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = LiftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
