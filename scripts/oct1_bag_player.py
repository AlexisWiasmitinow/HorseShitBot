#!/usr/bin/env python3
"""Deduplicated, isolated playback for the October 1 mapping recording.

Only /scan, /odom, and /tf are replayed.  The missing configured static
base_link -> laser transform is supplied explicitly.  In corrected mode the
range array is reflected so that output index j represents the physical ray
from historical index (-j mod N), while the standard positive LaserScan angle
increment is retained.
"""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
import sys
import time
from typing import Any

import rclpy
import rosbag2_py
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster


TOPICS = {"/scan", "/odom", "/tf"}


def stamp_key(message: Any) -> tuple[int, int]:
    return int(message.header.stamp.sec), int(message.header.stamp.nanosec)


def reflected_indices(length: int) -> list[int]:
    """Map positive output angles onto the historical clockwise scan array."""
    return [(-index) % length for index in range(length)]


def correct_scan_handedness(message: LaserScan) -> LaserScan:
    corrected = copy.deepcopy(message)
    indices = reflected_indices(len(message.ranges))
    corrected.ranges = [message.ranges[index] for index in indices]
    if message.intensities:
        corrected.intensities = [message.intensities[index] for index in indices]
    return corrected


def scan_signature(message: LaserScan) -> tuple[Any, ...]:
    return (
        message.header.frame_id,
        float(message.angle_min),
        float(message.angle_max),
        float(message.angle_increment),
        float(message.time_increment),
        float(message.scan_time),
        float(message.range_min),
        float(message.range_max),
        tuple(message.ranges),
        tuple(message.intensities),
    )


def odom_signature(message: Odometry) -> tuple[Any, ...]:
    return (
        message.header.frame_id,
        message.child_frame_id,
        tuple(message.pose.pose.position.__getattribute__(axis) for axis in ("x", "y", "z")),
        tuple(
            message.pose.pose.orientation.__getattribute__(axis)
            for axis in ("x", "y", "z", "w")
        ),
        tuple(message.twist.twist.linear.__getattribute__(axis) for axis in ("x", "y", "z")),
        tuple(message.twist.twist.angular.__getattribute__(axis) for axis in ("x", "y", "z")),
    )


def transform_signature(message: TFMessage) -> tuple[Any, ...]:
    return tuple(
        (
            transform.header.frame_id,
            transform.child_frame_id,
            int(transform.header.stamp.sec),
            int(transform.header.stamp.nanosec),
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        for transform in message.transforms
    )


def configured_static_transform() -> TransformStamped:
    """Return the transform committed before the October 1 recording."""
    transform = TransformStamped()
    transform.header.frame_id = "base_link"
    transform.child_frame_id = "laser"
    transform.transform.translation.x = 0.0
    transform.transform.translation.y = 0.0
    transform.transform.translation.z = 0.15
    yaw = 3.14159
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    return transform


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True)
    parser.add_argument(
        "--mode",
        choices=("raw", "corrected", "corrected_no_odom", "corrected_icp_odom"),
        required=True,
    )
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--summary", required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.rate <= 0:
        raise ValueError("--rate must be positive")

    scan_matched_poses: dict[int, tuple[float, float, float]] = {}
    if args.mode == "corrected_icp_odom":
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from validate_lidar_and_map import estimate_trajectory, load_bag

        diagnostic_data = load_bag(Path(args.bag))
        diagnostic_poses, _, _, _ = estimate_trajectory(diagnostic_data.scans)
        scan_matched_poses = {
            scan.stamp_ns: tuple(map(float, pose))
            for scan, pose in zip(diagnostic_data.scans, diagnostic_poses)
        }

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )

    rclpy.init()
    node = rclpy.create_node("oct1_deduplicated_bag_player")
    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    tf_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=100,
    )
    latched_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    scan_publisher = node.create_publisher(LaserScan, "/scan", sensor_qos)
    odom_publisher = node.create_publisher(Odometry, "/odom", 10)
    odom_visualization_publisher = node.create_publisher(
        Odometry, "/odom_corrected", 10
    )
    path_publisher = node.create_publisher(
        NavPath, "/trajectory_path", latched_qos
    )
    tf_publisher = node.create_publisher(TFMessage, "/tf", tf_qos)
    clock_publisher = node.create_publisher(Clock, "/clock", 10)
    static_broadcaster = StaticTransformBroadcaster(node)
    static_broadcaster.sendTransform(configured_static_transform())

    first_bag_stamp: int | None = None
    first_wall_time: float | None = None
    seen: dict[tuple[str, Any], tuple[Any, ...]] = {}
    raw_counts = {topic: 0 for topic in TOPICS}
    published_counts = {topic: 0 for topic in TOPICS}
    skipped_counts = {topic: 0 for topic in TOPICS}
    suppressed_counts = {topic: 0 for topic in TOPICS}
    synthetic_odom_count = 0
    trajectory = NavPath()
    trajectory.header.frame_id = "odom"

    # Allow discovery and transient-local static-transform delivery before time moves.
    discovery_end = time.monotonic() + 2.0
    while time.monotonic() < discovery_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    try:
        while reader.has_next() and rclpy.ok():
            topic, payload, bag_stamp = reader.read_next()
            if topic not in TOPICS:
                continue
            raw_counts[topic] += 1

            if topic == "/scan":
                message = deserialize_message(payload, LaserScan)
                identity = stamp_key(message)
                signature = scan_signature(message)
            elif topic == "/odom":
                message = deserialize_message(payload, Odometry)
                identity = stamp_key(message)
                signature = odom_signature(message)
            else:
                message = deserialize_message(payload, TFMessage)
                identity = tuple(
                    (
                        int(transform.header.stamp.sec),
                        int(transform.header.stamp.nanosec),
                        transform.header.frame_id,
                        transform.child_frame_id,
                    )
                    for transform in message.transforms
                )
                signature = transform_signature(message)

            deduplication_key = (topic, identity)
            previous = seen.get(deduplication_key)
            if previous is not None:
                if previous != signature:
                    raise RuntimeError(
                        f"Conflicting {topic} messages share identity {identity}"
                    )
                skipped_counts[topic] += 1
                continue
            seen[deduplication_key] = signature

            if first_bag_stamp is None:
                first_bag_stamp = int(bag_stamp)
                first_wall_time = time.monotonic()
            target_elapsed = (int(bag_stamp) - first_bag_stamp) / 1e9 / args.rate
            while time.monotonic() - first_wall_time < target_elapsed:
                rclpy.spin_once(node, timeout_sec=min(0.01, target_elapsed))

            seconds, nanoseconds = divmod(int(bag_stamp), 1_000_000_000)
            clock = Clock()
            clock.clock.sec = seconds
            clock.clock.nanosec = nanoseconds
            clock_publisher.publish(clock)

            if topic == "/scan":
                if args.mode.startswith("corrected"):
                    message = correct_scan_handedness(message)
                scan_publisher.publish(message)
                if args.mode in ("corrected_no_odom", "corrected_icp_odom"):
                    source_stamp = (
                        int(message.header.stamp.sec) * 1_000_000_000
                        + int(message.header.stamp.nanosec)
                    )
                    pose_x, pose_y, pose_yaw = scan_matched_poses.get(
                        source_stamp, (0.0, 0.0, 0.0)
                    )
                    synthetic_odom = Odometry()
                    synthetic_odom.header.stamp = message.header.stamp
                    synthetic_odom.header.frame_id = "odom"
                    synthetic_odom.child_frame_id = "base_link"
                    synthetic_odom.pose.pose.position.x = pose_x
                    synthetic_odom.pose.pose.position.y = pose_y
                    synthetic_odom.pose.pose.orientation.z = math.sin(pose_yaw / 2.0)
                    synthetic_odom.pose.pose.orientation.w = math.cos(pose_yaw / 2.0)
                    odom_publisher.publish(synthetic_odom)
                    odom_visualization_publisher.publish(synthetic_odom)
                    synthetic_transform = TransformStamped()
                    synthetic_transform.header.stamp = message.header.stamp
                    synthetic_transform.header.frame_id = "odom"
                    synthetic_transform.child_frame_id = "base_link"
                    synthetic_transform.transform.translation.x = pose_x
                    synthetic_transform.transform.translation.y = pose_y
                    synthetic_transform.transform.rotation.z = math.sin(
                        pose_yaw / 2.0
                    )
                    synthetic_transform.transform.rotation.w = math.cos(
                        pose_yaw / 2.0
                    )
                    synthetic_tf = TFMessage()
                    synthetic_tf.transforms = [synthetic_transform]
                    tf_publisher.publish(synthetic_tf)
                    if synthetic_odom_count % 5 == 0:
                        path_pose = PoseStamped()
                        path_pose.header = synthetic_odom.header
                        path_pose.pose = synthetic_odom.pose.pose
                        trajectory.header.stamp = synthetic_odom.header.stamp
                        trajectory.poses.append(path_pose)
                        if len(trajectory.poses) % 10 == 0:
                            path_publisher.publish(trajectory)
                    synthetic_odom_count += 1
            elif topic == "/odom":
                if args.mode in ("corrected_no_odom", "corrected_icp_odom"):
                    suppressed_counts[topic] += 1
                    continue
                odom_publisher.publish(message)
                odom_visualization_publisher.publish(message)
                if published_counts[topic] % 10 == 0:
                    pose = PoseStamped()
                    pose.header = message.header
                    pose.pose = message.pose.pose
                    trajectory.header.stamp = message.header.stamp
                    trajectory.poses.append(pose)
                    if len(trajectory.poses) % 10 == 0:
                        path_publisher.publish(trajectory)
            else:
                if args.mode in ("corrected_no_odom", "corrected_icp_odom"):
                    suppressed_counts[topic] += 1
                    continue
                tf_publisher.publish(message)
            published_counts[topic] += 1
            rclpy.spin_once(node, timeout_sec=0.0)
        if trajectory.poses:
            path_publisher.publish(trajectory)
    finally:
        summary = {
            "bag": str(Path(args.bag)),
            "mode": args.mode,
            "rate": args.rate,
            "deduplication": (
                "topic plus source header identity; duplicate semantic payloads "
                "must match or playback aborts"
            ),
            "raw_counts": raw_counts,
            "published_counts": published_counts,
            "skipped_duplicate_counts": skipped_counts,
            "suppressed_recorded_counts": suppressed_counts,
            "synthetic_odom_tf_samples": synthetic_odom_count,
            "synthetic_odom_source": (
                "corrected scan-to-scan trimmed ICP"
                if args.mode == "corrected_icp_odom"
                else "identity"
                if args.mode == "corrected_no_odom"
                else None
            ),
            "static_transform": {
                "parent": "base_link",
                "child": "laser",
                "translation_m": [0.0, 0.0, 0.15],
                "rpy_rad": [0.0, 0.0, 3.14159],
                "source": "recording-date repository commit a2f72fd",
            },
        }
        summary_path = Path(args.summary)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
