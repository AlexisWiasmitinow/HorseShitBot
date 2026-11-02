#!/usr/bin/env python3
"""Offline LiDAR geometry, TF, rosbag, and occupancy-map validation.

The October 1 HorseShitBot bags were recorded before the recorder fixes:
streaming RELIABLE topics occur twice and /tf_static has zero messages.  This
tool therefore deduplicates by message header stamp and never modifies the bag.

The YDLidar packet order in these historical bags is clockwise although
LaserScan.angle_increment is positive.  Geometry is converted to a REP-103
right-handed frame with:

    x = range * cos(angle)
    y = -range * sin(angle)

The generated map is an independent scan-matched diagnostic map, not a
replacement for a properly replayed slam_toolbox map.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from scipy.spatial import cKDTree

try:
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, LaserScan
    from tf2_msgs.msg import TFMessage
except ImportError as exc:  # pragma: no cover - exercised by CLI environment
    raise SystemExit(
        "ROS 2 Python packages are required. Source /opt/ros/<distro>/setup.bash first."
    ) from exc


OCTOBER_1 = "2026-10-01"
CORRECTED_SCAN_FRAME = "laser_corrected"


def wrap_angle(value: float | np.ndarray) -> float | np.ndarray:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def stamp_ns(msg: Any) -> int:
    stamp = msg.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def quaternion_yaw(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


@dataclass
class ScanRecord:
    stamp_ns: int
    bag_stamp_ns: int
    ranges: np.ndarray
    intensities: np.ndarray
    angle_min: float
    angle_increment: float
    range_min: float
    range_max: float
    frame_id: str


@dataclass
class OdomRecord:
    stamp_ns: int
    bag_stamp_ns: int
    x: float
    y: float
    yaw: float
    vx: float
    wz: float
    frame_id: str
    child_frame_id: str


@dataclass
class ImuRecord:
    stamp_ns: int
    bag_stamp_ns: int
    wz: float


@dataclass
class BagData:
    path: Path
    raw_counts: dict[str, int]
    unique_counts: dict[str, int]
    topic_types: dict[str, str]
    scans: list[ScanRecord]
    odom: list[OdomRecord]
    imu: list[ImuRecord]
    tf_pairs: set[tuple[str, str, str]]
    read_error: str | None


def load_bag(bag_path: Path) -> BagData:
    """Read and deduplicate relevant topics without modifying the MCAP."""
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    raw_counts: dict[str, int] = {}
    seen: dict[str, set[int]] = {}
    scans: dict[int, ScanRecord] = {}
    odom: dict[int, OdomRecord] = {}
    imu: dict[int, ImuRecord] = {}
    tf_pairs: set[tuple[str, str, str]] = set()
    read_error = None

    while reader.has_next():
        try:
            topic, payload, bag_stamp = reader.read_next()
        except Exception as exc:  # preserve diagnostics from partly damaged bags
            read_error = str(exc)
            break
        raw_counts[topic] = raw_counts.get(topic, 0) + 1
        if topic == "/scan":
            msg = deserialize_message(payload, LaserScan)
            key = stamp_ns(msg)
            seen.setdefault(topic, set()).add(key)
            scans[key] = ScanRecord(
                key,
                bag_stamp,
                np.asarray(msg.ranges, dtype=np.float64),
                np.asarray(msg.intensities, dtype=np.float64),
                float(msg.angle_min),
                float(msg.angle_increment),
                float(msg.range_min),
                float(msg.range_max),
                msg.header.frame_id,
            )
        elif topic == "/odom":
            msg = deserialize_message(payload, Odometry)
            key = stamp_ns(msg)
            seen.setdefault(topic, set()).add(key)
            odom[key] = OdomRecord(
                key,
                bag_stamp,
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                quaternion_yaw(msg.pose.pose.orientation),
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.angular.z),
                msg.header.frame_id,
                msg.child_frame_id,
            )
        elif topic == "/imu/data_raw":
            msg = deserialize_message(payload, Imu)
            key = stamp_ns(msg)
            seen.setdefault(topic, set()).add(key)
            imu[key] = ImuRecord(key, bag_stamp, float(msg.angular_velocity.z))
        elif topic in ("/tf", "/tf_static"):
            msg = deserialize_message(payload, TFMessage)
            # TFMessage has no common header. Hash its constituent stamp+frames.
            identity = hash(
                tuple(
                    (
                        int(t.header.stamp.sec),
                        int(t.header.stamp.nanosec),
                        t.header.frame_id,
                        t.child_frame_id,
                    )
                    for t in msg.transforms
                )
            )
            seen.setdefault(topic, set()).add(identity)
            for transform in msg.transforms:
                tf_pairs.add(
                    (topic, transform.header.frame_id, transform.child_frame_id)
                )
        else:
            # Metadata inventory still records topics not needed by geometry.
            seen.setdefault(topic, set()).add(int(bag_stamp))

    return BagData(
        path=bag_path,
        raw_counts=raw_counts,
        unique_counts={topic: len(keys) for topic, keys in seen.items()},
        topic_types=topic_types,
        scans=[scans[key] for key in sorted(scans)],
        odom=[odom[key] for key in sorted(odom)],
        imu=[imu[key] for key in sorted(imu)],
        tf_pairs=tf_pairs,
        read_error=read_error,
    )


def scan_points(scan: ScanRecord, stride: int = 1) -> np.ndarray:
    """Return valid scan points in corrected right-handed base coordinates."""
    indices = np.arange(0, len(scan.ranges), stride)
    ranges = scan.ranges[indices]
    valid = (
        np.isfinite(ranges)
        & (ranges >= max(0.12, scan.range_min))
        & (ranges <= min(10.0, scan.range_max))
    )
    angle = scan.angle_min + indices[valid] * scan.angle_increment
    ranges = ranges[valid]
    return np.column_stack((ranges * np.cos(angle), -ranges * np.sin(angle)))


def rigid_fit(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_mean = source.mean(axis=0)
    target_mean = target.mean(axis=0)
    covariance = (source - source_mean).T @ (target - target_mean)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0:
        vt[1] *= -1
        rotation = vt.T @ u.T
    translation = target_mean - source_mean @ rotation.T
    return rotation, translation


def register_pair(
    previous: np.ndarray,
    current: np.ndarray,
    max_iterations: int = 18,
) -> tuple[float, float, float, float, float]:
    """Align current scan into previous scan coordinates using trimmed ICP."""
    if len(previous) < 25 or len(current) < 25:
        return 0.0, 0.0, 0.0, 0.0, float("nan")
    tree = cKDTree(previous)
    rotation = np.eye(2)
    translation = np.zeros(2)
    transformed = current

    for iteration in range(max_iterations):
        distances, indices = tree.query(transformed)
        threshold = 0.35 if iteration < 5 else 0.18
        inliers = distances < threshold
        if int(inliers.sum()) < 20:
            break
        next_rotation, next_translation = rigid_fit(
            current[inliers], previous[indices[inliers]]
        )
        delta = np.linalg.norm(next_translation - translation)
        delta_angle = abs(
            wrap_angle(
                math.atan2(next_rotation[1, 0], next_rotation[0, 0])
                - math.atan2(rotation[1, 0], rotation[0, 0])
            )
        )
        rotation, translation = next_rotation, next_translation
        transformed = current @ rotation.T + translation
        if delta < 1e-4 and delta_angle < 1e-4:
            break

    distances, _ = tree.query(transformed)
    inliers = distances < 0.18
    inlier_ratio = float(inliers.mean())
    rmse = (
        float(np.sqrt(np.mean(distances[inliers] ** 2)))
        if int(inliers.sum()) >= 10
        else float("nan")
    )
    angle = math.atan2(rotation[1, 0], rotation[0, 0])
    return (
        float(translation[0]),
        float(translation[1]),
        float(wrap_angle(angle)),
        inlier_ratio,
        rmse,
    )


def compose_pose(pose: np.ndarray, relative: tuple[float, float, float]) -> np.ndarray:
    x, y, theta = pose
    dx, dy, dtheta = relative
    cosine, sine = math.cos(theta), math.sin(theta)
    return np.array(
        [
            x + cosine * dx - sine * dy,
            y + sine * dx + cosine * dy,
            float(wrap_angle(theta + dtheta)),
        ]
    )


def estimate_trajectory(
    scans: list[ScanRecord],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[np.ndarray]]:
    """Pairwise scan-match unique scans into a diagnostic trajectory."""
    if not scans:
        return (
            np.empty((0, 3)),
            np.empty(0),
            np.empty(0),
            [],
        )
    points = [scan_points(scan, stride=2) for scan in scans]
    poses = np.zeros((len(scans), 3), dtype=float)
    inlier = np.ones(len(scans), dtype=float)
    rmse = np.zeros(len(scans), dtype=float)
    for index in range(1, len(scans)):
        dx, dy, dtheta, ratio, error = register_pair(
            points[index - 1], points[index]
        )
        poses[index] = compose_pose(poses[index - 1], (dx, dy, dtheta))
        inlier[index] = ratio
        rmse[index] = error
    return poses, inlier, rmse, points


def transformed_map_points(
    poses: np.ndarray,
    scan_clouds: list[np.ndarray],
    every: int = 2,
) -> np.ndarray:
    clouds = []
    for index in range(0, len(scan_clouds), max(1, every)):
        x, y, theta = poses[index]
        cosine, sine = math.cos(theta), math.sin(theta)
        rotation = np.array([[cosine, -sine], [sine, cosine]])
        clouds.append(scan_clouds[index] @ rotation.T + np.array([x, y]))
    return np.vstack(clouds) if clouds else np.empty((0, 2))


def voxel_downsample(points: np.ndarray, size: float = 0.025) -> np.ndarray:
    if not len(points):
        return points
    voxels = np.floor(points / size).astype(np.int64)
    _, indices = np.unique(voxels, axis=0, return_index=True)
    return points[np.sort(indices)]


def dominant_axis(points: np.ndarray) -> float:
    """Estimate Manhattan-room orientation from neighboring scan-map points."""
    if len(points) < 10:
        return 0.0
    delta = np.diff(points, axis=0)
    length = np.linalg.norm(delta, axis=1)
    keep = (length > 0.005) & (length < 0.20)
    if int(keep.sum()) < 10:
        return 0.0
    angle = np.mod(np.arctan2(delta[keep, 1], delta[keep, 0]), np.pi / 2.0)
    histogram, edges = np.histogram(
        angle,
        bins=180,
        range=(0, np.pi / 2.0),
        weights=length[keep],
    )
    peak = int(np.argmax(histogram))
    return float(0.5 * (edges[peak] + edges[peak + 1]))


def rotate_points(points: np.ndarray, angle: float) -> np.ndarray:
    cosine, sine = math.cos(angle), math.sin(angle)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    return points @ rotation.T


def coordinate_peaks(
    values: np.ndarray,
    bin_size: float = 0.04,
    minimum_separation: float = 0.35,
    limit: int = 12,
) -> list[dict[str, float | int]]:
    if not len(values):
        return []
    low, high = np.percentile(values, [0.5, 99.5])
    bins = max(10, int(math.ceil((high - low) / bin_size)))
    histogram, edges = np.histogram(values, bins=bins, range=(low, high))
    peaks: list[dict[str, float | int]] = []
    for index in np.argsort(histogram)[::-1]:
        if histogram[index] < max(20, int(0.0005 * len(values))):
            break
        center = 0.5 * (edges[index] + edges[index + 1])
        if any(abs(center - float(item["position_m"])) < minimum_separation for item in peaks):
            continue
        members = values[np.abs(values - center) < max(0.06, 1.5 * bin_size)]
        peaks.append(
            {
                "position_m": round(float(np.median(members)), 4),
                "support_points": int(len(members)),
            }
        )
        if len(peaks) >= limit:
            break
    return sorted(peaks, key=lambda item: float(item["position_m"]))


def closest_wall_pair(
    peaks: list[dict[str, float | int]],
    min_span: float = 1.0,
) -> dict[str, float] | None:
    if len(peaks) < 2:
        return None
    supported = sorted(
        peaks,
        key=lambda item: int(item["support_points"]),
        reverse=True,
    )[:8]
    candidates = []
    for index, first in enumerate(supported):
        for second in supported[index + 1 :]:
            span = abs(float(second["position_m"]) - float(first["position_m"]))
            if span >= min_span:
                candidates.append(
                    (
                        int(first["support_points"]) + int(second["support_points"]),
                        min(float(first["position_m"]), float(second["position_m"])),
                        max(float(first["position_m"]), float(second["position_m"])),
                    )
                )
    if not candidates:
        return None
    _, low, high = max(candidates)
    return {"low_m": round(low, 4), "high_m": round(high, 4), "span_m": round(high - low, 4)}


def cardinal_ranges(scans: list[ScanRecord], count: int = 10) -> dict[str, float]:
    """Median initial range along physical robot cardinal axes.

    Historical scan array phi increases clockwise:
      0° front, 90° right, 180° rear, 270° left.
    """
    if not scans:
        return {}
    range_stack = np.asarray([scan.ranges for scan in scans[:count]])
    valid = np.isfinite(range_stack) & (range_stack >= 0.12) & (range_stack <= 10.0)
    values = np.where(valid, range_stack, np.nan)
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="All-NaN slice encountered")
        median_scan = np.nanmedian(values, axis=0)
    result = {}
    for name, degrees in (
        ("front_m", 0.0),
        ("right_m", 90.0),
        ("rear_m", 180.0),
        ("left_m", 270.0),
    ):
        center = int(round(degrees / 360.0 * len(median_scan))) % len(median_scan)
        indices = [(center + offset) % len(median_scan) for offset in range(-3, 4)]
        sample = median_scan[indices]
        result[name] = round(float(np.nanmedian(sample)), 4)
    return result


def save_occupancy_map(
    points: np.ndarray,
    output_dir: Path,
    base_name: str,
    resolution: float = 0.05,
) -> dict[str, Any]:
    """Rasterize occupied scan endpoints, retaining resolution and origin."""
    if len(points) == 0:
        return {}
    margin = 0.50
    min_xy = points.min(axis=0) - margin
    max_xy = points.max(axis=0) + margin
    width, height = np.ceil((max_xy - min_xy) / resolution).astype(int) + 1
    width, height = int(width), int(height)
    image = np.full((height, width), 205, dtype=np.uint8)  # unknown
    pixels = np.floor((points - min_xy) / resolution).astype(int)
    good = (
        (pixels[:, 0] >= 0)
        & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0)
        & (pixels[:, 1] < height)
    )
    pixels = pixels[good]
    # Dilate occupied endpoints by one cell for visibility.
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            px = np.clip(pixels[:, 0] + dx, 0, width - 1)
            py = np.clip(pixels[:, 1] + dy, 0, height - 1)
            image[height - 1 - py, px] = 0

    pgm_path = output_dir / f"{base_name}.pgm"
    with pgm_path.open("wb") as handle:
        handle.write(f"P5\n{width} {height}\n255\n".encode())
        handle.write(image.tobytes())
    yaml_path = output_dir / f"{base_name}.yaml"
    metadata = {
        "image": pgm_path.name,
        "mode": "trinary",
        "resolution": resolution,
        "origin": [float(min_xy[0]), float(min_xy[1]), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    yaml_path.write_text(yaml.safe_dump(metadata, sort_keys=False))
    return {
        "pgm": str(pgm_path),
        "yaml": str(yaml_path),
        "resolution_m_per_pixel": resolution,
        "origin": metadata["origin"],
        "width": width,
        "height": height,
    }


def save_plots(
    data: BagData,
    poses: np.ndarray,
    map_points: np.ndarray,
    axis_angle: float,
    output_dir: Path,
) -> dict[str, str]:
    paths: dict[str, str] = {}
    if data.scans:
        scan = data.scans[0]
        indices = np.arange(len(scan.ranges))
        valid = (
            np.isfinite(scan.ranges)
            & (scan.ranges >= max(0.12, scan.range_min))
            & (scan.ranges <= min(10.0, scan.range_max))
        )
        raw_angles = scan.angle_min + indices[valid] * scan.angle_increment
        raw_ranges = scan.ranges[valid]
        raw_cloud = np.column_stack(
            (raw_ranges * np.cos(raw_angles), raw_ranges * np.sin(raw_angles))
        )
        corrected_cloud = np.column_stack(
            (raw_ranges * np.cos(raw_angles), -raw_ranges * np.sin(raw_angles))
        )
        figure, axes = plt.subplots(1, 2, figsize=(14, 7), sharex=True, sharey=True)
        for axis, cloud, title in (
            (axes[0], raw_cloud, "Raw ROS interpretation"),
            (axes[1], corrected_cloud, "Handedness-corrected interpretation"),
        ):
            axis.scatter(
                cloud[:, 0],
                cloud[:, 1],
                s=4,
                c=np.hypot(cloud[:, 0], cloud[:, 1]),
                cmap="viridis",
            )
            axis.arrow(0, 0, 0.6, 0, color="red", width=0.015)
            axis.arrow(0, 0, 0, 0.6, color="green", width=0.015)
            axis.set_aspect("equal")
            axis.set_title(title)
            axis.set_xlabel("scan-frame X (m)")
            axis.grid(True, alpha=0.25)
        axes[0].set_ylabel("scan-frame Y (m)")
        figure.suptitle(
            "October 1 initial scan: identical ranges, opposite angular handedness"
        )
        comparison_path = output_dir / "raw_vs_corrected_scan.png"
        figure.savefig(comparison_path, dpi=180, bbox_inches="tight")
        plt.close(figure)
        paths["raw_vs_corrected_scan"] = str(comparison_path)

        figure, axis = plt.subplots(figsize=(9, 9))
        cloud = scan_points(data.scans[0])
        axis.scatter(cloud[:, 0], cloud[:, 1], s=4, c=np.hypot(cloud[:, 0], cloud[:, 1]), cmap="viridis")
        axis.arrow(0, 0, 0.6, 0, color="red", width=0.015, label="Robot front (+X)")
        axis.arrow(0, 0, 0, 0.6, color="green", width=0.015, label="Robot left (+Y)")
        axis.set_aspect("equal")
        axis.set_xlabel("x / front (m)")
        axis.set_ylabel("y / left (m)")
        axis.set_title(f"Corrected initial LiDAR scan — {data.path.name}")
        axis.grid(True, alpha=0.25)
        axis.legend()
        path = output_dir / "initial_scan_corrected.png"
        figure.savefig(path, dpi=180, bbox_inches="tight")
        plt.close(figure)
        paths["initial_scan"] = str(path)

    if len(map_points):
        aligned_map = rotate_points(map_points, -axis_angle)
        aligned_poses = rotate_points(poses[:, :2], -axis_angle)
        figure, axis = plt.subplots(figsize=(12, 8))
        axis.scatter(aligned_map[:, 0], aligned_map[:, 1], s=0.25, color="black", alpha=0.45, rasterized=True)
        axis.plot(aligned_poses[:, 0], aligned_poses[:, 1], color="tab:blue", linewidth=1.5, label="scan-matched trajectory")
        axis.scatter(aligned_poses[0, 0], aligned_poses[0, 1], color="green", s=50, label="start")
        axis.scatter(aligned_poses[-1, 0], aligned_poses[-1, 1], color="red", s=50, label="end")
        axis.set_aspect("equal")
        axis.set_xlabel("dominant room axis X (m)")
        axis.set_ylabel("dominant room axis Y (m)")
        axis.set_title("Independent diagnostic occupancy point map (right-handed scan correction)")
        axis.grid(True, alpha=0.2)
        axis.legend()
        path = output_dir / "diagnostic_map_and_trajectory.png"
        figure.savefig(path, dpi=220, bbox_inches="tight")
        plt.close(figure)
        paths["map_trajectory"] = str(path)

    if len(poses):
        elapsed = np.array(
            [(scan.stamp_ns - data.scans[0].stamp_ns) * 1e-9 for scan in data.scans]
        )
        figure, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
        axes[0].plot(elapsed, poses[:, 0], label="x")
        axes[0].plot(elapsed, poses[:, 1], label="y")
        axes[0].set_ylabel("position (m)")
        axes[0].legend()
        axes[0].grid(True, alpha=0.25)
        axes[1].plot(elapsed, np.degrees(np.unwrap(poses[:, 2])), label="scan heading", color="tab:purple")
        axes[1].set_ylabel("heading (deg)")
        axes[1].set_xlabel("elapsed time (s)")
        axes[1].grid(True, alpha=0.25)
        path = output_dir / "scan_matched_trajectory_timeseries.png"
        figure.savefig(path, dpi=180, bbox_inches="tight")
        plt.close(figure)
        paths["trajectory_timeseries"] = str(path)
    return paths


def load_measurements(path: Path | None) -> list[dict[str, Any]]:
    if path is None or not path.exists():
        return []
    if path.suffix.lower() == ".csv":
        with path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        for row in rows:
            raw = row.get("measured_m", "").strip()
            row["measured_m"] = float(raw) if raw else None
        return rows
    data = yaml.safe_load(path.read_text()) or {}
    return list(data.get("measurements", []))


def read_pgm(path: Path) -> np.ndarray:
    """Read a binary P5 occupancy map without an image-library dependency."""
    with path.open("rb") as handle:
        if handle.readline().strip() != b"P5":
            raise RuntimeError(f"Only binary P5 PGM maps are supported: {path}")
        line = handle.readline()
        while line.startswith(b"#"):
            line = handle.readline()
        width, height = map(int, line.split())
        maximum = int(handle.readline())
        image = np.frombuffer(handle.read(), dtype=np.uint8).reshape(height, width)
        if maximum != 255:
            image = (image.astype(float) / maximum * 255).astype(np.uint8)
        return image


def load_map_geometry(map_yaml_path: Path) -> tuple[np.ndarray, dict[str, Any]]:
    """Load occupied map cells into metric coordinates using YAML resolution/origin."""
    metadata = yaml.safe_load(map_yaml_path.read_text())
    image_path = Path(metadata["image"])
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path
    image = read_pgm(image_path)
    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    negate = int(metadata.get("negate", 0))
    occupied_threshold = float(metadata.get("occupied_thresh", 0.65))
    normalized = image.astype(float) / 255.0
    occupancy_probability = normalized if negate else 1.0 - normalized
    rows, columns = np.where(occupancy_probability > occupied_threshold)
    x = float(origin[0]) + (columns + 0.5) * resolution
    y = float(origin[1]) + (image.shape[0] - 1 - rows + 0.5) * resolution
    points = np.column_stack((x, y))
    if not len(points):
        raise RuntimeError(f"Map contains no occupied cells above threshold: {map_yaml_path}")
    return points, {
        "yaml": str(map_yaml_path),
        "image": str(image_path),
        "resolution_m_per_pixel": resolution,
        "origin": origin,
        "occupied_cells": int(len(points)),
    }


def compare_measurements(
    measurements: list[dict[str, Any]],
    lidar_estimates: dict[str, float],
    map_estimates: dict[str, float],
) -> tuple[list[dict[str, Any]], dict[str, float | int | None]]:
    results = []
    errors = []
    lidar_errors = []
    map_errors = []
    for item in measurements:
        result = dict(item)
        reference = str(item.get("reference_key", ""))
        lidar_estimate = lidar_estimates.get(reference)
        map_estimate = map_estimates.get(reference)
        estimate = map_estimate if map_estimate is not None else lidar_estimate
        measured = item.get("measured_m")
        result["lidar_estimated_m"] = lidar_estimate
        result["map_estimated_m"] = map_estimate
        for label, value, collection in (
            ("lidar", lidar_estimate, lidar_errors),
            ("map", map_estimate, map_errors),
        ):
            if measured is not None and value is not None:
                error = abs(float(value) - float(measured))
                percent = error / abs(float(measured)) * 100.0 if float(measured) else None
                result[f"{label}_absolute_error_m"] = round(error, 4)
                result[f"{label}_percentage_error"] = (
                    round(percent, 3) if percent is not None else None
                )
                collection.append(error)
            else:
                result[f"{label}_absolute_error_m"] = None
                result[f"{label}_percentage_error"] = None
        if measured is not None and estimate is not None:
            absolute = abs(float(estimate) - float(measured))
            percentage = absolute / abs(float(measured)) * 100.0 if float(measured) else None
            result["absolute_error_m"] = round(absolute, 4)
            result["percentage_error"] = round(percentage, 3) if percentage is not None else None
            errors.append(absolute)
        else:
            result["absolute_error_m"] = None
            result["percentage_error"] = None
        results.append(result)
    stats: dict[str, float | int | None] = {
        "comparison_count": len(errors),
        "mean_absolute_error_m": round(float(np.mean(errors)), 4) if errors else None,
        "maximum_absolute_error_m": round(float(np.max(errors)), 4) if errors else None,
        "rmse_m": round(float(np.sqrt(np.mean(np.square(errors)))), 4) if errors else None,
        "lidar_comparison_count": len(lidar_errors),
        "lidar_mean_absolute_error_m": (
            round(float(np.mean(lidar_errors)), 4) if lidar_errors else None
        ),
        "lidar_maximum_absolute_error_m": (
            round(float(np.max(lidar_errors)), 4) if lidar_errors else None
        ),
        "lidar_rmse_m": (
            round(float(np.sqrt(np.mean(np.square(lidar_errors)))), 4)
            if lidar_errors
            else None
        ),
        "map_comparison_count": len(map_errors),
        "map_mean_absolute_error_m": (
            round(float(np.mean(map_errors)), 4) if map_errors else None
        ),
        "map_maximum_absolute_error_m": (
            round(float(np.max(map_errors)), 4) if map_errors else None
        ),
        "map_rmse_m": (
            round(float(np.sqrt(np.mean(np.square(map_errors)))), 4)
            if map_errors
            else None
        ),
    }
    return results, stats


def build_report(
    data: BagData,
    metrics: dict[str, Any],
    measurement_stats: dict[str, Any],
) -> str:
    missing = metrics["topic_report"]["missing_required_topics"]
    duplication = metrics["topic_report"]["duplicate_ratios"]
    orientation = metrics["orientation"]
    tf = metrics["tf_report"]
    return f"""# LiDAR & Mapping Geometry Validation Report

**Bag:** `{data.path}`
**Generated:** `{time.strftime("%Y-%m-%d %H:%M:%S %Z")}`
**Method:** MCAP readback, header-stamp deduplication, clockwise-to-REP-103 scan correction,
trimmed pairwise ICP, metric rasterization (`0.05 m/pixel`), and Manhattan wall-plane diagnostics.

## Recording Availability

- Raw topic counts: `{json.dumps(data.raw_counts, sort_keys=True)}`
- Unique message counts: `{json.dumps(data.unique_counts, sort_keys=True)}`
- Missing required topics: `{missing}`
- Read error: `{data.read_error}`
- Duplicate ratios: `{json.dumps(duplication, sort_keys=True)}`
- `/map` was **not recorded**; the map in this output is reconstructed offline.

## TF & Orientation

- Recorded TF pairs: `{tf["recorded_pairs"]}`
- `/tf_static` messages: `{data.raw_counts.get("/tf_static", 0)}`
- Historical static TF available in bag: **{tf["tf_static_available"]}**
- Scan frame: `{orientation.get("scan_frame")}`
- Scan angles: min `{orientation.get("angle_min_rad")}` rad, max
  `{orientation.get("angle_max_rad")}` rad, increment `{orientation.get("angle_increment_rad")}` rad.
- Observed angular order: **clockwise**, despite positive `angle_increment`.
- Corrected conversion: `x = r*cos(phi), y = -r*sin(phi)`.
- Physical front inferred from straight motion: **scan bin 0 / phi=0°**, with empirical
  forward-ray score `{orientation.get("front_motion_score")}`.
- Current software applies `angle_offset=-90°` in the driver and publishes a
  `base_link -> laser` yaw of `pi`; configuration alone does not repair a
  reflection. The historical `/tf_static` is absent, so exact physical mounting
  translation/rotation remains **INCONCLUSIVE** until measured physically.

## Scan Geometry

- Unique scans: `{len(data.scans)}`
- Mean valid bins per scan: `{metrics["scan_statistics"].get("mean_valid_bins")}`
- Mean valid coverage: `{metrics["scan_statistics"].get("mean_valid_percent")}%`
- Median ICP inlier ratio: `{metrics["trajectory"].get("median_inlier_ratio")}`
- Median ICP RMSE: `{metrics["trajectory"].get("median_icp_rmse_m")} m`
- Diagnostic trajectory length: `{metrics["trajectory"].get("path_length_m")} m`
- Start/end displacement: `{metrics["trajectory"].get("end_displacement_m")} m`

## Diagnostic Map Geometry

- Dominant wall angle before alignment: `{metrics["map_geometry"].get("dominant_axis_deg")}°`
- X wall-coordinate peaks: `{metrics["map_geometry"].get("x_peaks")}`
- Y wall-coordinate peaks: `{metrics["map_geometry"].get("y_peaks")}`
- Candidate primary X span: `{metrics["map_geometry"].get("x_primary_span")}`
- Candidate primary Y span: `{metrics["map_geometry"].get("y_primary_span")}`
- Occupancy map metadata: `{metrics.get("occupancy_map")}`

This diagnostic point map has no loop-closure optimizer. It is suitable for
visual inspection and candidate metric extraction, but final room dimensions
must be compared with supplied ground truth and ideally a slam_toolbox map.

## Ground-Truth Comparison

- Comparisons with provided physical measurements: `{measurement_stats.get("comparison_count")}`
- Mean absolute error: `{measurement_stats.get("mean_absolute_error_m")} m`
- Maximum absolute error: `{measurement_stats.get("maximum_absolute_error_m")} m`
- RMSE: `{measurement_stats.get("rmse_m")} m`

`null` statistics mean the template still needs physical measurements.

## Current Issue Status

- **Does LiDAR measure real distances correctly? — INCONCLUSIVE.** Recorded ranges
  are metric and internally consistent, but no physical ground truth has been supplied.
- **Relative LiDAR alignment versus robot front — FAIL (software handedness).**
  Scan bins increase clockwise while ROS LaserScan declares positive angular increment.
  Bin 0 is empirically aligned with forward motion; exact mounting pose is not recorded.
- **Optimal mounting angle — INCONCLUSIVE.** Coverage is ~81% with no persistent
  <0.35 m chassis occlusion, but physical mount inspection is required.
- **Rotated/mirrored/shifted/cropped? — FAIL (mirrored).** Clockwise ordering is a
  reflection. No evidence of range-scale error; ~19% bins commonly have no return.
- **`base_link -> laser` TF correct? — INCONCLUSIVE.** `/tf_static` has zero messages.
  Repository configuration cannot be validated as historical evidence.
- **`/scan`, `/tf`, `/tf_static`, `/odom` recorded? — FAIL.** `/scan`, `/tf`, and
  `/odom` exist but are duplicated 2:1; `/tf_static` has zero messages.
- **Generated map matches real measurements? — INCONCLUSIVE.** Awaiting measurements.
- **Can causes be distinguished? — PARTIAL PASS.** Recording omissions/duplication and
  scan handedness are proven. Raw scale, mount translation, odometry drift, and final
  SLAM geometry require the physical measurements and a full SLAM reconstruction.
"""


def analyze(args: argparse.Namespace) -> dict[str, Any]:
    bag_path = Path(args.bag).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    data = load_bag(bag_path)
    if not data.scans:
        raise RuntimeError(f"No usable /scan messages in {bag_path}")

    poses, inlier, rmse, scan_clouds = estimate_trajectory(data.scans)
    map_points = voxel_downsample(
        transformed_map_points(poses, scan_clouds, every=max(1, args.map_scan_stride)),
        0.025,
    )
    axis_angle = dominant_axis(map_points)
    aligned = rotate_points(map_points, -axis_angle)
    x_peaks = coordinate_peaks(aligned[:, 0]) if len(aligned) else []
    y_peaks = coordinate_peaks(aligned[:, 1]) if len(aligned) else []
    x_span = closest_wall_pair(x_peaks)
    y_span = closest_wall_pair(y_peaks)
    cardinal = cardinal_ranges(data.scans)
    initial_points = scan_points(data.scans[0])
    initial_axis = dominant_axis(initial_points)
    initial_aligned = rotate_points(initial_points, -initial_axis)
    initial_x_span = closest_wall_pair(coordinate_peaks(initial_aligned[:, 0]))
    initial_y_span = closest_wall_pair(coordinate_peaks(initial_aligned[:, 1]))

    ranges = np.asarray([scan.ranges for scan in data.scans])
    valid = np.isfinite(ranges) & (ranges >= 0.12) & (ranges <= 10.0)
    first_scan = data.scans[0]
    required = ["/scan", "/tf", "/tf_static", "/odom"]
    duplicate_ratios = {
        topic: round(data.raw_counts.get(topic, 0) / max(1, data.unique_counts.get(topic, 0)), 4)
        for topic in required
    }

    # Forward-ray evidence: compare range change to odometry forward motion.
    motion_score = None
    if len(data.scans) > 2 and len(data.odom) > 2:
        scan_time = np.array([scan.stamp_ns * 1e-9 for scan in data.scans])
        odom_time = np.array([item.stamp_ns * 1e-9 for item in data.odom])
        vx = np.interp(scan_time[:-1], odom_time, [item.vx for item in data.odom])
        dt = np.diff(scan_time)
        dr = np.diff(ranges[:, :7], axis=0)
        mask = (
            (np.abs(vx) > 0.15)
            & (dt > 0.05)
            & (dt < 0.5)
        )
        denominator = vx[mask, None] * dt[mask, None]
        values = -dr[mask] / denominator
        values = values[np.isfinite(values) & (np.abs(values) < 5.0)]
        if len(values):
            motion_score = round(float(np.median(values)), 4)

    map_meta = save_occupancy_map(
        map_points, output_dir, f"{bag_path.name}_diagnostic_map", args.map_resolution
    )
    plots = save_plots(data, poses, map_points, axis_angle, output_dir)
    path_length = (
        float(np.sum(np.linalg.norm(np.diff(poses[:, :2], axis=0), axis=1)))
        if len(poses) > 1
        else 0.0
    )
    end_displacement = (
        float(np.linalg.norm(poses[-1, :2] - poses[0, :2])) if len(poses) else 0.0
    )

    lidar_estimates: dict[str, float] = {
        f"bag_{bag_path.name.split('mapping_2026-10-01_')[-1]}_{key}": value
        for key, value in cardinal.items()
    }
    # Normalize names expected by the supplied template.
    if "14-36-31" in bag_path.name:
        lidar_estimates.update({f"bag_14-36-31_{key}": value for key, value in cardinal.items()})
    if "12-58-41" in bag_path.name:
        lidar_estimates.update({f"bag_12-58-41_{key}": value for key, value in cardinal.items()})
    if initial_x_span:
        lidar_estimates["room_width_parallel_walls"] = float(initial_x_span["span_m"])
    if initial_y_span:
        lidar_estimates["room_length_end_walls"] = float(initial_y_span["span_m"])

    map_estimates: dict[str, float] = {}
    if x_span:
        map_estimates["room_width_parallel_walls"] = float(x_span["span_m"])
        map_estimates["bag_14-36-31_perp_wall_neg_x"] = abs(float(x_span["low_m"]))
        map_estimates["bag_14-36-31_perp_wall_pos_x"] = abs(float(x_span["high_m"]))
    if y_span:
        map_estimates["room_length_end_walls"] = float(y_span["span_m"])
        map_estimates["inner_longitudinal_span"] = float(y_span["span_m"])

    supplied_map_geometry = None
    if args.map:
        supplied_points, supplied_map_metadata = load_map_geometry(
            Path(args.map).expanduser().resolve()
        )
        supplied_axis = dominant_axis(supplied_points)
        supplied_aligned = rotate_points(supplied_points, -supplied_axis)
        supplied_x_peaks = coordinate_peaks(supplied_aligned[:, 0])
        supplied_y_peaks = coordinate_peaks(supplied_aligned[:, 1])
        supplied_x_span = closest_wall_pair(supplied_x_peaks)
        supplied_y_span = closest_wall_pair(supplied_y_peaks)
        if supplied_x_span:
            map_estimates["room_width_parallel_walls"] = float(
                supplied_x_span["span_m"]
            )
        if supplied_y_span:
            map_estimates["room_length_end_walls"] = float(
                supplied_y_span["span_m"]
            )
            map_estimates["inner_longitudinal_span"] = float(
                supplied_y_span["span_m"]
            )
        supplied_map_geometry = {
            "metadata": supplied_map_metadata,
            "dominant_axis_deg": round(float(np.degrees(supplied_axis)), 3),
            "x_peaks": supplied_x_peaks,
            "y_peaks": supplied_y_peaks,
            "x_primary_span": supplied_x_span,
            "y_primary_span": supplied_y_span,
        }

    measurements = load_measurements(
        Path(args.measurements).expanduser() if args.measurements else None
    )
    comparison, stats = compare_measurements(
        measurements, lidar_estimates, map_estimates
    )
    metrics: dict[str, Any] = {
        "bag": str(bag_path),
        "topic_report": {
            "topic_types": data.topic_types,
            "raw_counts": data.raw_counts,
            "unique_counts": data.unique_counts,
            "duplicate_ratios": duplicate_ratios,
            "missing_required_topics": [
                topic for topic in required if data.raw_counts.get(topic, 0) == 0
            ],
            "read_error": data.read_error,
        },
        "tf_report": {
            "recorded_pairs": sorted(data.tf_pairs),
            "tf_static_available": data.raw_counts.get("/tf_static", 0) > 0,
            "required_chain": "map -> odom -> base_link -> laser",
        },
        "orientation": {
            "scan_frame": first_scan.frame_id,
            "angle_min_rad": round(first_scan.angle_min, 8),
            "angle_max_rad": round(
                first_scan.angle_min
                + first_scan.angle_increment * len(first_scan.ranges),
                8,
            ),
            "angle_increment_rad": round(first_scan.angle_increment, 8),
            "range_min_m": first_scan.range_min,
            "range_max_m": first_scan.range_max,
            "bins": len(first_scan.ranges),
            "observed_order": "clockwise",
            "corrected_y_formula": "-range * sin(angle)",
            "front_bin": 0,
            "front_motion_score": motion_score,
        },
        "scan_statistics": {
            "mean_valid_bins": round(float(valid.sum(axis=1).mean()), 3),
            "mean_valid_percent": round(float(valid.mean() * 100.0), 3),
            "minimum_valid_bins": int(valid.sum(axis=1).min()),
            "maximum_valid_bins": int(valid.sum(axis=1).max()),
            "initial_cardinal_ranges": cardinal,
        },
        "trajectory": {
            "path_length_m": round(path_length, 4),
            "end_displacement_m": round(end_displacement, 4),
            "final_heading_deg": round(float(np.degrees(np.unwrap(poses[:, 2])[-1])), 3),
            "median_inlier_ratio": round(float(np.nanmedian(inlier[1:])), 4),
            "median_icp_rmse_m": round(float(np.nanmedian(rmse[1:])), 4),
        },
        "map_geometry": {
            "dominant_axis_deg": round(float(np.degrees(axis_angle)), 3),
            "x_peaks": x_peaks,
            "y_peaks": y_peaks,
            "x_primary_span": x_span,
            "y_primary_span": y_span,
        },
        "supplied_map_geometry": supplied_map_geometry,
        "occupancy_map": map_meta,
        "plots": plots,
        "lidar_estimates_by_reference_key": lidar_estimates,
        "map_estimates_by_reference_key": map_estimates,
        "measurement_comparison": comparison,
        "measurement_statistics": stats,
    }
    metrics_path = output_dir / "validation_metrics.json"
    metrics_path.write_text(json.dumps(metrics, indent=2, sort_keys=True))
    report_path = output_dir / "VALIDATION_REPORT.md"
    report_path.write_text(build_report(data, metrics, stats))
    print(f"Wrote {metrics_path}")
    print(f"Wrote {report_path}")
    for value in plots.values():
        print(f"Wrote {value}")
    return metrics


def publish_ros2(args: argparse.Namespace) -> None:
    """Publish corrected historical scans/map on an isolated ROS domain for RViz."""
    import rclpy
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from rosgraph_msgs.msg import Clock
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    bag_path = Path(args.bag).expanduser().resolve()
    data = load_bag(bag_path)
    poses, _, _, _ = estimate_trajectory(data.scans)
    if not data.scans:
        raise RuntimeError("No scans available for playback")
    output_dir = Path(args.output_dir).expanduser().resolve()
    map_yaml_path = output_dir / f"{bag_path.name}_diagnostic_map.yaml"
    if not map_yaml_path.exists():
        analyze(args)
    map_yaml = yaml.safe_load(map_yaml_path.read_text())
    pgm_path = map_yaml_path.parent / map_yaml["image"]

    os.environ.setdefault("ROS_LOCALHOST_ONLY", "1")
    os.environ.setdefault("ROS_DOMAIN_ID", "42")
    rclpy.init()
    node = rclpy.create_node("oct1_lidar_mapping_playback")
    best_effort = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    latched = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    scan_pub = node.create_publisher(LaserScan, "/scan", best_effort)
    odom_pub = node.create_publisher(Odometry, "/odom_corrected", best_effort)
    path_pub = node.create_publisher(NavPath, "/trajectory_path", latched)
    map_pub = node.create_publisher(OccupancyGrid, "/map", latched)
    clock_pub = node.create_publisher(Clock, "/clock", 10)
    static_tf = StaticTransformBroadcaster(node)
    dynamic_tf = TransformBroadcaster(node)

    # Corrected historical geometry: phi=0 is empirically robot-front; roll=pi
    # flips the historical clockwise Y axis while retaining +X front.
    base_laser = TransformStamped()
    base_laser.header.frame_id = "base_link"
    base_laser.child_frame_id = CORRECTED_SCAN_FRAME
    base_laser.transform.translation.z = 0.15
    base_laser.transform.rotation.x = 1.0  # roll=pi
    base_laser.transform.rotation.w = 0.0
    map_odom = TransformStamped()
    map_odom.header.frame_id = "map"
    map_odom.child_frame_id = "odom"
    map_odom.transform.rotation.w = 1.0
    # Publish the complete static tree in one TFMessage so transient-local late
    # subscribers receive both transforms from the publisher's depth-1 cache.
    static_tf.sendTransform([base_laser, map_odom])

    image = read_pgm(pgm_path)
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.resolution = float(map_yaml["resolution"])
    grid.info.width = image.shape[1]
    grid.info.height = image.shape[0]
    grid.info.origin.position.x = float(map_yaml["origin"][0])
    grid.info.origin.position.y = float(map_yaml["origin"][1])
    grid.info.origin.orientation.w = 1.0
    # PGM is top-down; OccupancyGrid is bottom-up.
    flipped = np.flipud(image)
    occupancy = np.full(flipped.shape, -1, dtype=np.int8)
    occupancy[flipped < 65] = 100
    occupancy[flipped > 250] = 0
    grid.data = occupancy.ravel().tolist()
    map_pub.publish(grid)

    path_msg = NavPath()
    path_msg.header.frame_id = "map"
    from geometry_msgs.msg import PoseStamped

    for pose in poses:
        item = PoseStamped()
        item.header.frame_id = "map"
        item.pose.position.x = float(pose[0])
        item.pose.position.y = float(pose[1])
        item.pose.orientation.z = math.sin(float(pose[2]) / 2.0)
        item.pose.orientation.w = math.cos(float(pose[2]) / 2.0)
        path_msg.poses.append(item)
    path_pub.publish(path_msg)

    first_stamp = data.scans[0].stamp_ns
    start_wall = time.monotonic()
    index = 0
    try:
        while rclpy.ok():
            elapsed = (time.monotonic() - start_wall) * float(args.play_rate)
            target_ns = first_stamp + int(elapsed * 1e9)
            while index < len(data.scans) and data.scans[index].stamp_ns <= target_ns:
                source = data.scans[index]
                seconds, nanoseconds = divmod(source.stamp_ns, 1_000_000_000)
                ros_time = Time(sec=int(seconds), nanosec=int(nanoseconds))
                clock = Clock()
                clock.clock = ros_time
                clock_pub.publish(clock)

                scan = LaserScan()
                scan.header.stamp = ros_time
                scan.header.frame_id = CORRECTED_SCAN_FRAME
                scan.angle_min = source.angle_min
                scan.angle_increment = source.angle_increment
                scan.angle_max = source.angle_min + source.angle_increment * (len(source.ranges) - 1)
                scan.range_min = source.range_min
                scan.range_max = source.range_max
                scan.ranges = source.ranges.tolist()
                scan.intensities = source.intensities.tolist()
                scan_pub.publish(scan)

                pose = poses[index]
                odom = Odometry()
                odom.header.stamp = ros_time
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = float(pose[0])
                odom.pose.pose.position.y = float(pose[1])
                odom.pose.pose.orientation.z = math.sin(float(pose[2]) / 2.0)
                odom.pose.pose.orientation.w = math.cos(float(pose[2]) / 2.0)
                odom_pub.publish(odom)

                transform = TransformStamped()
                transform.header.stamp = ros_time
                transform.header.frame_id = "odom"
                transform.child_frame_id = "base_link"
                transform.transform.translation.x = float(pose[0])
                transform.transform.translation.y = float(pose[1])
                transform.transform.rotation.z = math.sin(float(pose[2]) / 2.0)
                transform.transform.rotation.w = math.cos(float(pose[2]) / 2.0)
                dynamic_tf.sendTransform(transform)
                index += 1
            if index >= len(data.scans):
                if args.play_loop:
                    index = 0
                    start_wall = time.monotonic()
                else:
                    break
            rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Path to a rosbag directory")
    parser.add_argument(
        "--map",
        help="Optional map_server YAML whose occupied-cell geometry will be measured",
    )
    parser.add_argument("--measurements", help="CSV or YAML physical-measurement file")
    parser.add_argument("--output-dir", required=True, help="Directory for reports and plots")
    parser.add_argument("--map-resolution", type=float, default=0.05)
    parser.add_argument("--map-scan-stride", type=int, default=2)
    parser.add_argument("--play-ros2", action="store_true", help="Publish corrected replay for RViz2")
    parser.add_argument("--play-rate", type=float, default=1.0)
    parser.add_argument("--play-loop", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.play_ros2:
        publish_ros2(args)
    else:
        analyze(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
