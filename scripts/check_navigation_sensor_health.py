#!/usr/bin/env python3
"""Bounded, subscription-only health check for navigation ROS 2 topics.

This utility never creates command publishers, service clients, or action
clients. It observes the existing ROS graph for a fixed duration and prints
machine-readable JSON.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass, field
import json
import math
import statistics
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, LaserScan, MagneticField
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


@dataclass(frozen=True)
class TopicSpec:
    message_type: type
    expected_frame: str | None
    minimum_hz: float | None
    required: bool = True
    qos: QoSProfile = qos_profile_sensor_data


@dataclass
class Observation:
    received_monotonic: list[float] = field(default_factory=list)
    source_stamps_ns: list[int] = field(default_factory=list)
    frames: set[str] = field(default_factory=set)
    invalid_samples: int = 0
    nonzero_commands: int = 0
    notes: set[str] = field(default_factory=set)


TF_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=100,
)
TF_STATIC_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
STATUS_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

TOPICS = {
    "/odom": TopicSpec(Odometry, "odom", 40.0),
    "/imu/data_raw": TopicSpec(Imu, "imu_link", 40.0),
    "/imu/mag": TopicSpec(MagneticField, "imu_link", 40.0),
    "/scan": TopicSpec(LaserScan, "laser", 4.0),
    "/tf": TopicSpec(TFMessage, None, 40.0, qos=TF_QOS),
    "/tf_static": TopicSpec(
        TFMessage, None, None, qos=TF_STATIC_QOS
    ),
    "/wheel_status": TopicSpec(String, None, 40.0, qos=STATUS_QOS),
    "/mks_bus/status": TopicSpec(String, None, 0.25, qos=STATUS_QOS),
    "/cmd_vel": TopicSpec(Twist, None, None, required=False, qos=STATUS_QOS),
}


def _stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _finite(*values: float) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _frequency(received: list[float]) -> float | None:
    if len(received) < 2:
        return None
    intervals = [
        later - earlier
        for earlier, later in zip(received, received[1:])
        if later > earlier
    ]
    return 1.0 / statistics.median(intervals) if intervals else None


def assess(
    spec: TopicSpec,
    observation: Observation,
    publisher_count: int,
) -> dict[str, Any]:
    """Classify one bounded observation without accessing ROS."""
    rate = _frequency(observation.received_monotonic)
    regressions = sum(
        later <= earlier
        for earlier, later in zip(
            observation.source_stamps_ns, observation.source_stamps_ns[1:]
        )
    )
    reasons = []
    level = "PASS"
    if publisher_count == 0:
        level = "FAIL" if spec.required else "WARN"
        reasons.append("no publisher")
    elif not observation.received_monotonic:
        level = "FAIL" if spec.required else "WARN"
        reasons.append("publisher present but no messages received")
    if observation.invalid_samples:
        level = "FAIL"
        reasons.append(f"{observation.invalid_samples} invalid sample(s)")
    if regressions:
        level = "WARN" if level == "PASS" else level
        reasons.append(f"{regressions} non-increasing source timestamp(s)")
    if (
        spec.minimum_hz is not None
        and rate is not None
        and rate < spec.minimum_hz
    ):
        level = "WARN" if level == "PASS" else level
        reasons.append(f"rate below {spec.minimum_hz:g} Hz")
    if (
        spec.expected_frame is not None
        and observation.frames
        and observation.frames != {spec.expected_frame}
    ):
        level = "WARN" if level == "PASS" else level
        reasons.append(
            f"unexpected frame(s): {', '.join(sorted(observation.frames))}"
        )
    if observation.notes and level == "PASS":
        level = "WARN"
    reasons.extend(sorted(observation.notes))
    return {
        "status": level,
        "publisher_count": publisher_count,
        "message_count": len(observation.received_monotonic),
        "frequency_hz": round(rate, 3) if rate is not None else None,
        "frames": sorted(observation.frames),
        "timestamp_regressions": regressions,
        "nonzero_commands": observation.nonzero_commands,
        "reasons": reasons,
    }


class NavigationHealthProbe(Node):
    def __init__(self) -> None:
        super().__init__("navigation_sensor_health_probe")
        self.observations = {topic: Observation() for topic in TOPICS}
        self._subscriptions = [
            self.create_subscription(
                spec.message_type,
                topic,
                lambda message, name=topic: self._observe(name, message),
                spec.qos,
            )
            for topic, spec in TOPICS.items()
        ]

    def _observe(self, topic: str, message: Any) -> None:
        observation = self.observations[topic]
        observation.received_monotonic.append(time.monotonic())
        header = getattr(message, "header", None)
        if header is not None:
            observation.source_stamps_ns.append(_stamp_ns(header.stamp))
            if header.frame_id:
                observation.frames.add(header.frame_id)

        if topic == "/odom":
            pose = message.pose.pose
            twist = message.twist.twist
            if not _finite(
                pose.position.x,
                pose.position.y,
                pose.orientation.z,
                pose.orientation.w,
                twist.linear.x,
                twist.angular.z,
            ):
                observation.invalid_samples += 1
            if not any(message.pose.covariance) and not any(message.twist.covariance):
                observation.notes.add("pose and twist covariances are all zero")
        elif topic == "/imu/data_raw":
            gyro = message.angular_velocity
            accel = message.linear_acceleration
            if not _finite(gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z):
                observation.invalid_samples += 1
            if message.orientation_covariance[0] < 0.0:
                observation.notes.add("orientation intentionally unavailable")
            if not any(message.angular_velocity_covariance):
                observation.notes.add("angular-velocity covariance is all zero")
        elif topic == "/imu/mag":
            field = message.magnetic_field
            if not _finite(field.x, field.y, field.z):
                observation.invalid_samples += 1
        elif topic == "/scan":
            valid = sum(
                math.isfinite(value)
                and message.range_min <= value <= message.range_max
                for value in message.ranges
            )
            if (
                not message.ranges
                or valid == 0
                or message.angle_increment == 0.0
            ):
                observation.invalid_samples += 1
            if message.time_increment == 0.0:
                observation.notes.add("scan time_increment is zero")
        elif topic in ("/tf", "/tf_static"):
            for transform in message.transforms:
                observation.source_stamps_ns.append(
                    _stamp_ns(transform.header.stamp)
                )
                observation.frames.add(
                    f"{transform.header.frame_id}->{transform.child_frame_id}"
                )
        elif topic in ("/wheel_status", "/mks_bus/status"):
            try:
                json.loads(message.data)
            except (TypeError, json.JSONDecodeError):
                observation.invalid_samples += 1
        elif topic == "/cmd_vel":
            if abs(message.linear.x) > 1e-9 or abs(message.angular.z) > 1e-9:
                observation.nonzero_commands += 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument(
        "--discovery-grace",
        type=float,
        default=2.0,
        help="seconds allowed for DDS discovery before the observation interval",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        raise SystemExit("--duration must be positive and finite")
    if not math.isfinite(args.discovery_grace) or args.discovery_grace < 0.0:
        raise SystemExit("--discovery-grace must be finite and non-negative")
    rclpy.init()
    node = NavigationHealthProbe()
    try:
        discovery_deadline = time.monotonic() + args.discovery_grace
        while rclpy.ok() and time.monotonic() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        static_observation = node.observations["/tf_static"]
        node.observations = {topic: Observation() for topic in TOPICS}
        node.observations["/tf_static"] = static_observation
        deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        report = {
            "duration_s": args.duration,
            "discovery_grace_s": args.discovery_grace,
            "read_only": True,
            "topics": {
                topic: assess(
                    spec,
                    node.observations[topic],
                    len(node.get_publishers_info_by_topic(topic)),
                )
                for topic, spec in TOPICS.items()
            },
        }
        print(json.dumps(report, indent=2, sort_keys=True))
        return int(any(item["status"] == "FAIL" for item in report["topics"].values()))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
