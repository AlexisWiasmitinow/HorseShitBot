"""
Bag Recorder Node -- programmatic rosbag2 recording of RealSense camera topics.

Subscribes to configured camera topics and writes them to timestamped bag files
on demand.  Start / stop recording via Trigger services or the web dashboard.
"""

from __future__ import annotations

import json
import os
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    import rosbag2_py
    _HAS_ROSBAG2 = True
except ImportError:
    _HAS_ROSBAG2 = False


_TOPIC_TYPE_MAP = {
    "/camera/color/image_raw": "sensor_msgs/msg/Image",
    "/camera/aligned_depth_to_color/image_raw": "sensor_msgs/msg/Image",
    "/camera/color/camera_info": "sensor_msgs/msg/CameraInfo",
    "/camera/aligned_depth_to_color/camera_info": "sensor_msgs/msg/CameraInfo",
}


class BagRecorderNode(Node):
    def __init__(self):
        super().__init__("bag_recorder_node")

        self.declare_parameter("output_dir", "~/rosbags")
        self.declare_parameter("bag_prefix", "horseshitbot")
        self.declare_parameter(
            "topics",
            list(_TOPIC_TYPE_MAP.keys()),
        )
        self.declare_parameter("storage_id", "mcap")

        self._output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self._bag_prefix = self.get_parameter("bag_prefix").get_parameter_value().string_value
        self._topics: list[str] = (
            self.get_parameter("topics").get_parameter_value().string_array_value
        )
        self._storage_id = self.get_parameter("storage_id").get_parameter_value().string_value

        self._recording = False
        self._writer: rosbag2_py.SequentialWriter | None = None if _HAS_ROSBAG2 else None
        self._bag_path = ""
        self._start_time = 0.0
        self._frame_count = 0

        self.create_service(Trigger, "~/start_recording", self._srv_start)
        self.create_service(Trigger, "~/stop_recording", self._srv_stop)

        self._status_pub = self.create_publisher(String, "~/status", 10)

        self._subscriptions: list = []
        self._create_topic_subscriptions()

        self.create_timer(0.5, self._publish_status)

        if not _HAS_ROSBAG2:
            self.get_logger().error("rosbag2_py not available -- recording disabled")

        self.get_logger().info(
            f"Bag recorder ready  topics={self._topics}  "
            f"storage={self._storage_id}  dir={self._output_dir}"
        )

    def _create_topic_subscriptions(self):
        """Create generic (raw) subscriptions for each configured topic."""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        topic_msg_modules: dict[str, type] = {}
        for topic in self._topics:
            type_str = _TOPIC_TYPE_MAP.get(topic)
            if type_str is None:
                self.get_logger().warn(f"Unknown topic type for {topic}, skipping subscription")
                continue
            msg_cls = self._import_msg_type(type_str)
            if msg_cls is None:
                self.get_logger().warn(f"Could not import {type_str}, skipping {topic}")
                continue
            topic_msg_modules[topic] = msg_cls

        for topic, msg_cls in topic_msg_modules.items():
            sub = self.create_subscription(
                msg_cls,
                topic,
                lambda msg, t=topic: self._on_message(t, msg),
                qos,
            )
            self._subscriptions.append(sub)

    @staticmethod
    def _import_msg_type(type_str: str):
        """Import a ROS message class from a 'pkg/msg/Type' string."""
        try:
            parts = type_str.split("/")
            mod = __import__(f"{parts[0]}.{parts[1]}", fromlist=[parts[2]])
            return getattr(mod, parts[2])
        except Exception:
            return None

    def _on_message(self, topic: str, msg):
        if not self._recording or self._writer is None:
            return
        try:
            serialized = serialize_message(msg)
            now_ns = self.get_clock().now().nanoseconds
            self._writer.write(topic, serialized, now_ns)
            self._frame_count += 1
        except Exception as exc:
            self.get_logger().error(f"Failed to write to bag: {exc}")

    def _srv_start(self, request, response):
        if self._recording:
            response.success = False
            response.message = "Already recording"
            return response

        if not _HAS_ROSBAG2:
            response.success = False
            response.message = "rosbag2_py not installed"
            return response

        try:
            self._open_bag()
            response.success = True
            response.message = f"Recording to {self._bag_path}"
            self.get_logger().info(f"Recording started -> {self._bag_path}")
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"Failed to start recording: {exc}")

        return response

    def _srv_stop(self, request, response):
        if not self._recording:
            response.success = False
            response.message = "Not recording"
            return response

        self._close_bag()
        response.success = True
        response.message = f"Stopped. Saved {self._frame_count} frames to {self._bag_path}"
        self.get_logger().info(response.message)
        return response

    def _open_bag(self):
        out_dir = os.path.expanduser(self._output_dir)
        os.makedirs(out_dir, exist_ok=True)

        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        bag_name = f"{self._bag_prefix}_{stamp}"
        self._bag_path = os.path.join(out_dir, bag_name)

        storage_options = rosbag2_py.StorageOptions(
            uri=self._bag_path,
            storage_id=self._storage_id,
        )
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )

        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(storage_options, converter_options)

        for topic in self._topics:
            type_str = _TOPIC_TYPE_MAP.get(topic, "std_msgs/msg/String")
            topic_meta = rosbag2_py.TopicMetadata(
                name=topic,
                type=type_str,
                serialization_format="cdr",
            )
            self._writer.create_topic(topic_meta)

        self._recording = True
        self._start_time = time.monotonic()
        self._frame_count = 0

    def _close_bag(self):
        self._recording = False
        if self._writer is not None:
            del self._writer
            self._writer = None

    def _publish_status(self):
        duration = time.monotonic() - self._start_time if self._recording else 0.0
        status = {
            "recording": self._recording,
            "bag_path": self._bag_path,
            "duration_sec": round(duration, 1),
            "frame_count": self._frame_count,
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._recording:
            node._close_bag()
        node.destroy_node()
        rclpy.shutdown()
