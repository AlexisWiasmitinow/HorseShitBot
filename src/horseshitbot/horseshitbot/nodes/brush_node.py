"""
Brush Node — single-motor actuator with open/close commands and
stall-based referencing, using the generic Actuator driver.
"""

from __future__ import annotations

from .actuator_node_base import ActuatorNodeBase


class BrushNode(ActuatorNodeBase):
    NODE_NAME = "brush_node"
    TOPIC_PREFIX = "brush"
    DEFAULT_MOTOR_IDS = [4]
    DEFAULT_DUAL = False


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = BrushNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
