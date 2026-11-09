"""
Bag Recorder Node -- programmatic rosbag2 recording of RealSense camera and mapping topics.

Subscribes to configured topics and writes them to timestamped bag files
on demand.  Start / stop recording via Trigger services or the web dashboard.
"""

from __future__ import annotations

import errno
import fcntl
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
    # Camera — color
    "/camera/color/image_raw": "sensor_msgs/msg/Image",
    "/camera/color/camera_info": "sensor_msgs/msg/CameraInfo",
    # Camera — aligned depth
    "/camera/aligned_depth_to_color/image_raw": "sensor_msgs/msg/Image",
    "/camera/aligned_depth_to_color/camera_info": "sensor_msgs/msg/CameraInfo",
    # Camera — raw depth (not aligned)
    "/camera/depth/image_rect_raw": "sensor_msgs/msg/Image",
    "/camera/depth/camera_info": "sensor_msgs/msg/CameraInfo",
    # Camera — point cloud (if enabled)
    "/camera/depth/color/points": "sensor_msgs/msg/PointCloud2",
    # Mapping — lidar + odometry
    "/scan": "sensor_msgs/msg/LaserScan",
    "/odom": "nav_msgs/msg/Odometry",
    "/imu/data_raw": "sensor_msgs/msg/Imu",
    "/imu/mag": "sensor_msgs/msg/MagneticField",
    "/imu/temperature": "sensor_msgs/msg/Temperature",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/tf_static": "tf2_msgs/msg/TFMessage",
    # Navigation
    "/cmd_vel": "geometry_msgs/msg/Twist",
    # Gamepad
    "/gamepad/status": "std_msgs/msg/String",
    "/gamepad/button": "std_msgs/msg/String",
}

_TOPIC_GROUPS = {
    "Camera — Color": [
        "/camera/color/image_raw",
        "/camera/color/camera_info",
    ],
    "Camera — Depth (aligned)": [
        "/camera/aligned_depth_to_color/image_raw",
        "/camera/aligned_depth_to_color/camera_info",
    ],
    "Camera — Depth (raw)": [
        "/camera/depth/image_rect_raw",
        "/camera/depth/camera_info",
    ],
    "Camera — Point Cloud": [
        "/camera/depth/color/points",
    ],
    "Mapping": [
        "/scan",
        "/odom",
        "/imu/data_raw",
        "/imu/mag",
        "/imu/temperature",
        "/tf",
        "/tf_static",
    ],
    "Navigation": [
        "/cmd_vel",
    ],
    "Gamepad": [
        "/gamepad/status",
        "/gamepad/button",
    ],
}

_FALLBACK_TOPICS = [
    "/camera/color/image_raw",
    "/camera/aligned_depth_to_color/image_raw",
    "/camera/color/camera_info",
    "/camera/aligned_depth_to_color/camera_info",
]

# Topics that publish latched data with TRANSIENT_LOCAL durability
_TRANSIENT_LOCAL_TOPICS = {"/tf_static"}


_CONFIG_DIR = Path.home() / ".config" / "horseshitbot"


class BagRecorderNode(Node):
    def __init__(self, *, node_name: str = "bag_recorder_node", **kwargs):
        super().__init__(node_name, **kwargs)

        self.declare_parameter("output_dir", "~/rosbags")
        self.declare_parameter("bag_prefix", "horseshitbot")
        self.declare_parameter("storage_id", "mcap")
        self.declare_parameter("topics_config_name", "bag_topics.json")
        self.declare_parameter("default_topics", _FALLBACK_TOPICS)

        self._output_dir = self.get_parameter("output_dir").get_parameter_value().string_value
        self._bag_prefix = self.get_parameter("bag_prefix").get_parameter_value().string_value
        self._storage_id = self.get_parameter("storage_id").get_parameter_value().string_value
        self._bag_topics_file = _CONFIG_DIR / self.get_parameter(
            "topics_config_name"
        ).get_parameter_value().string_value
        self._default_topics = list(
            self.get_parameter("default_topics").get_parameter_value().string_array_value
        )

        self._topics: list[str] = self._load_topics_from_file()

        self._recording = False
        self._writer: rosbag2_py.SequentialWriter | None = None if _HAS_ROSBAG2 else None
        self._bag_path = ""
        self._lock_files: list = []
        self._start_time = 0.0
        self._frame_count = 0
        self._msg_counts: dict[str, int] = {}
        self._topic_first_logged: set[str] = set()
        self._latched_tf_static_by_pair: dict[tuple[str, str], object] = {}
        self._latched_messages: dict[str, list] = {}

        self.create_service(Trigger, "~/start_recording", self._srv_start)
        self.create_service(Trigger, "~/stop_recording", self._srv_stop)

        self._status_pub = self.create_publisher(String, "~/status", 10)

        self._topic_subscriptions: list = []
        self._subscriptions_by_topic: dict[str, object] = {}
        self._create_topic_subscriptions()

        self.create_timer(0.5, self._publish_status)
        self.create_timer(5.0, self._diag_topic_check)
        self.create_timer(3.0, self._reload_topics_if_changed)

        if not _HAS_ROSBAG2:
            self.get_logger().error("rosbag2_py not available -- recording disabled")

        self.get_logger().info(
            f"Bag recorder ready  topics={self._topics}  "
            f"storage={self._storage_id}  dir={self._output_dir}"
        )

    def _load_topics_from_file(self) -> list[str]:
        """Load selected topics from the config file, or return deduplicated defaults."""
        raw_topics: list[str] = []
        try:
            if self._bag_topics_file.is_file():
                data = json.loads(self._bag_topics_file.read_text())
                raw_topics = [t for t in data if t in _TOPIC_TYPE_MAP]
        except Exception:
            pass
        if not raw_topics:
            raw_topics = [
                t for t in self._default_topics if t in _TOPIC_TYPE_MAP
            ] or list(_FALLBACK_TOPICS)
        return list(dict.fromkeys(raw_topics))

    def _destroy_topic_subscriptions(self):
        """Destroy all active topic subscriptions."""
        for sub in list(self._topic_subscriptions):
            self.destroy_subscription(sub)
        self._topic_subscriptions.clear()
        self._subscriptions_by_topic.clear()

    def _reload_topics_if_changed(self):
        """Periodically check config file and update topics if changed."""
        if self._recording:
            return
        new_topics = self._load_topics_from_file()
        if set(new_topics) != set(self._topics):
            self._topics = new_topics
            self._destroy_topic_subscriptions()
            self._msg_counts.clear()
            self._topic_first_logged.clear()
            self._latched_tf_static_by_pair.clear()
            self._latched_messages.clear()
            self._create_topic_subscriptions()
            self.get_logger().info(f"Topics updated from config: {self._topics}")

    def _build_subscription_qos(self, topic: str) -> QoSProfile:
        """Return a single QoSProfile for `topic` compatible with its publishers.

        - `/tf_static` uses RELIABLE + TRANSIENT_LOCAL to receive latched static
          transforms from `tf2_ros/static_transform_publisher`.
        - All other topics use a single BEST_EFFORT + VOLATILE subscription, which
          is RMW-compatible with both BEST_EFFORT (e.g. `qos_profile_sensor_data`
          on `/imu/*`) and RELIABLE publishers (`/odom`, `/scan`, `/tf`, `/cmd_vel`,
          camera topics) without duplicating messages.
        """
        if topic in _TRANSIENT_LOCAL_TOPICS:
            return QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            )
        return QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _create_topic_subscriptions(self):
        """Create exactly one subscription for each configured topic."""
        self._destroy_topic_subscriptions()

        created = 0
        for topic in dict.fromkeys(self._topics):
            type_str = _TOPIC_TYPE_MAP.get(topic)
            if type_str is None:
                self.get_logger().warn(f"Unknown topic type for {topic}, skipping")
                continue
            msg_cls = self._import_msg_type(type_str)
            if msg_cls is None:
                self.get_logger().warn(f"Could not import {type_str}, skipping {topic}")
                continue

            self._msg_counts[topic] = 0
            qos = self._build_subscription_qos(topic)
            sub = self.create_subscription(
                msg_cls,
                topic,
                lambda msg, t=topic: self._on_message(t, msg),
                qos,
            )
            self._topic_subscriptions.append(sub)
            self._subscriptions_by_topic[topic] = sub
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
        except Exception:
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

    def _cache_transient_local_message(self, topic: str, msg):
        """Cache latest latched messages without bounding or duplicating static transforms."""
        if topic == "/tf_static" and hasattr(msg, "transforms"):
            for transform in msg.transforms:
                parent = getattr(getattr(transform, "header", None), "frame_id", "")
                child = getattr(transform, "child_frame_id", "")
                self._latched_tf_static_by_pair[(parent, child)] = transform
            return
        latched = self._latched_messages.setdefault(topic, [])
        latched.append(msg)
        if len(latched) > 64:
            del latched[:-64]

    def _get_latched_messages_for_topic(self, topic: str) -> list:
        """Return deduplicated latched messages to write at the start of a recording."""
        if topic == "/tf_static":
            if not self._latched_tf_static_by_pair:
                return []
            tf_cls = self._import_msg_type(_TOPIC_TYPE_MAP["/tf_static"])
            if tf_cls is None:
                return []
            consolidated = tf_cls()
            consolidated.transforms = list(self._latched_tf_static_by_pair.values())
            return [consolidated]
        return list(self._latched_messages.get(topic, []))

    def _on_message(self, topic: str, msg):
        self._msg_counts[topic] = self._msg_counts.get(topic, 0) + 1
        if topic in _TRANSIENT_LOCAL_TOPICS:
            self._cache_transient_local_message(topic, msg)

        if topic not in self._topic_first_logged:
            self._topic_first_logged.add(topic)
            self.get_logger().info(
                f"First message received on {topic} "
                f"(type={type(msg).__module__}.{type(msg).__name__})"
            )

        if not self._recording or self._writer is None:
            return
        self._write_serialized_message(topic, msg)

    def _write_serialized_message(self, topic: str, msg):
        if self._writer is None:
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

    @staticmethod
    def _try_acquire_lockfile(lock_path: str):
        """Acquire a non-blocking exclusive flock on `lock_path`."""
        lock_file = open(lock_path, "a+")
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            lock_file.seek(0)
            lock_file.truncate()
            lock_file.write(f"{os.getpid()}\n")
            lock_file.flush()
            return lock_file
        except OSError as exc:
            lock_file.close()
            if exc.errno in (errno.EACCES, errno.EAGAIN):
                raise RuntimeError(f"Lock already held: {lock_path}") from exc
            raise

    def _release_locks(self):
        """Release all held lockfiles."""
        while self._lock_files:
            lock_file = self._lock_files.pop()
            try:
                fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
            except Exception:
                pass
            try:
                lock_file.close()
            except Exception:
                pass

    def _reserve_bag_path(self, out_dir: str, explicit_bag_path: str | None = None) -> str:
        """Reserve an exclusive target bag path and prevent concurrent writes.

        1. Acquires an exclusive per-prefix lock in `out_dir` so two recorders
           with the same `bag_prefix` cannot simultaneously record in `out_dir`.
        2. Acquires an exclusive per-bag lock and verifies the target bag path
           does not already exist on disk (preventing overwrites or concurrent
           writes to the same MCAP file).
        """
        prefix_lock_path = os.path.join(out_dir, f".{self._bag_prefix}.recording.lock")
        try:
            prefix_lock = self._try_acquire_lockfile(prefix_lock_path)
        except RuntimeError as exc:
            raise RuntimeError(
                f"Another '{self._bag_prefix}' recorder is already active in {out_dir}"
            ) from exc
        self._lock_files.append(prefix_lock)

        if explicit_bag_path is not None:
            target_path = os.path.abspath(os.path.expanduser(explicit_bag_path))
            bag_lock_path = f"{target_path}.lock"
            try:
                bag_lock = self._try_acquire_lockfile(bag_lock_path)
            except RuntimeError as exc:
                self._release_locks()
                raise RuntimeError(
                    f"Target bag path is already locked by another recorder: {target_path}"
                ) from exc
            self._lock_files.append(bag_lock)
            if os.path.exists(target_path):
                self._release_locks()
                raise FileExistsError(
                    f"Target bag path already exists, refusing to overwrite: {target_path}"
                )
            return target_path

        stamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        base_name = f"{self._bag_prefix}_{stamp}"
        for suffix in range(0, 1000):
            bag_name = base_name if suffix == 0 else f"{base_name}_{suffix}"
            candidate = os.path.join(out_dir, bag_name)
            if os.path.exists(candidate):
                continue
            bag_lock_path = f"{candidate}.lock"
            try:
                bag_lock = self._try_acquire_lockfile(bag_lock_path)
            except RuntimeError:
                continue
            if os.path.exists(candidate):
                try:
                    fcntl.flock(bag_lock.fileno(), fcntl.LOCK_UN)
                    bag_lock.close()
                except Exception:
                    pass
                continue
            self._lock_files.append(bag_lock)
            return candidate

        self._release_locks()
        raise RuntimeError(f"Could not reserve a unique bag path in {out_dir}")

    def _open_bag(self, explicit_bag_path: str | None = None):
        out_dir = os.path.expanduser(self._output_dir)
        os.makedirs(out_dir, exist_ok=True)

        self._bag_path = self._reserve_bag_path(out_dir, explicit_bag_path=explicit_bag_path)

        try:
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

            # Write any latched TRANSIENT_LOCAL messages (e.g. /tf_static) received prior to start
            for topic in self._topics:
                if topic in _TRANSIENT_LOCAL_TOPICS:
                    for latched_msg in self._get_latched_messages_for_topic(topic):
                        self._write_serialized_message(topic, latched_msg)
        except Exception:
            self._recording = False
            if self._writer is not None:
                del self._writer
                self._writer = None
            self._release_locks()
            raise

    def _close_bag(self):
        self._recording = False
        if self._writer is not None:
            del self._writer
            self._writer = None
        self._release_locks()

    def destroy_node(self):
        if self._recording or self._writer is not None or self._lock_files:
            self._close_bag()
        return super().destroy_node()

    def _publish_status(self):
        duration = time.monotonic() - self._start_time if self._recording else 0.0
        status = {
            "recording": self._recording,
            "bag_path": self._bag_path,
            "duration_sec": round(duration, 1),
            "frame_count": self._frame_count,
            "selected_topics": self._topics,
            "available_topics": _TOPIC_TYPE_MAP,
            "topic_groups": _TOPIC_GROUPS,
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
