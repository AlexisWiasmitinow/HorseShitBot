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
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
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
        self._msg_counts: dict[str, int] = {}
        self._topic_first_logged: set[str] = set()

        self.create_service(Trigger, "~/start_recording", self._srv_start)
        self.create_service(Trigger, "~/stop_recording", self._srv_stop)

        self._status_pub = self.create_publisher(String, "~/status", 10)

        self._subscriptions: list = []
        self._create_topic_subscriptions()

        self.create_timer(0.5, self._publish_status)
        self.create_timer(5.0, self._diag_topic_check)

        if not _HAS_ROSBAG2:
            self.get_logger().error("rosbag2_py not available -- recording disabled")

        self.get_logger().info(
            f"Bag recorder ready  topics={self._topics}  "
            f"storage={self._storage_id}  dir={self._output_dir}"
        )

    def _create_topic_subscriptions(self):
        """Create subscriptions for each configured topic.

        Tries two QoS profiles per topic: BEST_EFFORT (for sensor-data
        publishers) and RELIABLE (for system-default publishers).  At least
        one will match the camera node regardless of its QoS setting.
        """
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        created = 0
        for topic in self._topics:
            type_str = _TOPIC_TYPE_MAP.get(topic)
            if type_str is None:
                self.get_logger().warn(f"Unknown topic type for {topic}, skipping")
                continue
            msg_cls = self._import_msg_type(type_str)
            if msg_cls is None:
                self.get_logger().warn(f"Could not import {type_str}, skipping {topic}")
                continue

            self._msg_counts[topic] = 0
            for label, qos in [("best_effort", qos_best_effort),
                                ("reliable", qos_reliable)]:
                sub = self.create_subscription(
                    msg_cls,
                    topic,
                    lambda msg, t=topic: self._on_message(t, msg),
                    qos,
                )
                self._subscriptions.append(sub)
                created += 1

        self.get_logger().info(
            f"Created {created} subscriptions for {len(self._msg_counts)} topics"
        )

    @staticmethod
    def _import_msg_type(type_str: str):
        """Import a ROS message class from a 'pkg/msg/Type' string."""
        try:
            parts = type_str.split("/")
            mod = __import__(f"{parts[0]}.{parts[1]}", fromlist=[parts[2]])
            return getattr(mod, parts[2])
        except Exception as exc:
            return None

    def _diag_topic_check(self):
        """Periodically log which target topics have actual publishers."""
        missing = []
        found = []
        for t in self._topics:
            pubs = self.get_publishers_info_by_topic(t)
            if pubs:
                qos = pubs[0].qos_profile
                found.append(f"{t} (reliability={qos.reliability})")
            else:
                missing.append(t)

        if missing:
            all_topics = [n for n, _ in self.get_topic_names_and_types()]
            camera_topics = [n for n in all_topics if "camera" in n.lower()]
            self.get_logger().warn(
                f"NO publishers for: {missing}  |  "
                f"Camera topics on network: {camera_topics}"
            )
        elif found and not self._topic_first_logged:
            self.get_logger().info(f"Publishers found: {found}")

    def _on_message(self, topic: str, msg):
        self._msg_counts[topic] = self._msg_counts.get(topic, 0) + 1
        if topic not in self._topic_first_logged:
            self._topic_first_logged.add(topic)
            self.get_logger().info(
                f"First message received on {topic} "
                f"(type={type(msg).__module__}.{type(msg).__name__})"
            )

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

        rx_summary = {t: c for t, c in self._msg_counts.items()}
        self.get_logger().info(f"Messages received so far: {rx_summary}")

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
