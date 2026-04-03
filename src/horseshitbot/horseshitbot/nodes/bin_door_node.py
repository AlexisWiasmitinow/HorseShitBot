"""
Bin Door Node — single-motor actuator with open/close commands and
stall-based referencing, using the generic Actuator driver.
"""

from __future__ import annotations

from .actuator_node_base import ActuatorNodeBase


class BinDoorNode(ActuatorNodeBase):
    NODE_NAME = "bin_door_node"
    TOPIC_PREFIX = "bin_door"
    DEFAULT_MOTOR_IDS = [6]
    DEFAULT_DUAL = False


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = BinDoorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
