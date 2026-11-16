#!/usr/bin/env python3
"""Offline wheel/IMU/LiDAR consistency analysis for GitHub issue #23.

The script is read-only with respect to ROS bags.  It deduplicates messages by
source stamp, corrects the historical clockwise LaserScan convention only in
memory, estimates scan-to-scan motion with point-to-plane ICP, and compares
that motion with open-loop wheel odometry and raw IMU gyroscope measurements.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import cKDTree


BENCHMARKS = {
    "mapping_2026-07-10_17-38-11": {
        "role": "structured indoor healthy baseline",
        "straight": [(6.87, 14.88)],
        "turn": [(4.14, 5.48), (16.80, 21.00), (24.35, 26.20)],
    },
    "mapping_2026-07-13_10-54-00": {
        "role": "outdoor drive/traction-loss case",
        "straight": [(0.00, 1.95), (12.86, 15.48), (61.46, 63.64)],
        "turn": [(21.24, 24.32), (77.22, 86.08), (86.36, 88.78)],
    },
    "mapping_2026-07-13_10-57-59": {
        "role": "outdoor long straight and turn-loss case",
        "straight": [(22.38, 31.24), (57.80, 59.58)],
        "turn": [(59.60, 66.66), (69.64, 78.64)],
    },
    "mapping_2026-10-01_14-36-31": {
        "role": "dense indoor high-command saturation baseline",
        "straight": [(122.06, 123.06), (124.52, 125.61)],
        "turn": [
            (20.58, 23.19),
            (50.14, 52.64),
            (127.05, 132.64),
            (136.62, 144.43),
        ],
    },
}


def wrap_angle(value: float | np.ndarray) -> float | np.ndarray:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def trapezoid(t: np.ndarray, values: np.ndarray) -> float:
    """Integrate values over strictly increasing timestamps."""
    if len(t) < 2:
        return 0.0
    return float(np.trapz(values, t))


def integrate_window(
    t: np.ndarray,
    values: np.ndarray,
    start: float,
    end: float,
) -> float:
    """Integrate with linearly interpolated values at exact window boundaries."""
    if len(t) < 2 or end <= start:
        return 0.0
    clipped_start = max(float(start), float(t[0]))
    clipped_end = min(float(end), float(t[-1]))
    if clipped_end <= clipped_start:
        return 0.0
    interior = (t > clipped_start) & (t < clipped_end)
    window_t = np.concatenate(
        ([clipped_start], t[interior], [clipped_end])
    )
    window_values = np.concatenate(
        (
            [np.interp(clipped_start, t, values)],
            values[interior],
            [np.interp(clipped_end, t, values)],
        )
    )
    return trapezoid(window_t, window_values)


def quaternion_yaw(q: Any) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def robust_bias(values: np.ndarray) -> float:
    """Median bias estimate, robust to a minority of moving samples."""
    if not len(values):
        return 0.0
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    if mad <= 1e-9:
        return median
    keep = np.abs(values - median) <= 4.0 * 1.4826 * mad
    return float(np.mean(values[keep])) if np.any(keep) else median


def scan_seed_yaw(
    imu: np.ndarray,
    t0: float,
    t1: float,
    gyro_bias: float,
    seed_mode: str,
) -> float:
    """Return the requested ICP yaw seed for one adjacent scan pair."""
    if seed_mode == "zero":
        return 0.0
    if seed_mode != "imu":
        raise ValueError(f"unsupported ICP seed mode: {seed_mode}")
    return integrate_window(imu[:, 0], imu[:, 6] - gyro_bias, t0, t1)


def corrected_scan_points(
    ranges: np.ndarray,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    stride: int = 2,
) -> np.ndarray:
    """Project historical clockwise scan bins into REP-103 XY coordinates."""
    indices = np.arange(0, len(ranges), max(1, stride))
    selected = ranges[indices]
    valid = (
        np.isfinite(selected)
        & (selected >= max(0.12, range_min))
        & (selected <= min(10.0, range_max))
    )
    angle = angle_min + indices[valid] * angle_increment
    distance = selected[valid]
    return np.column_stack(
        (distance * np.cos(angle), -distance * np.sin(angle))
    )


def point_to_plane_icp(
    source: np.ndarray,
    target: np.ndarray,
    initial_yaw: float = 0.0,
    max_iterations: int = 30,
) -> tuple[float, float, float, float, int]:
    """Align current/source scan into previous/target scan coordinates."""
    if len(source) < 25 or len(target) < 25:
        return 0.0, 0.0, 0.0, float("nan"), 0

    tree = cKDTree(target)
    _, neighbors = tree.query(target, k=min(6, len(target)))
    normals = np.zeros_like(target)
    for index in range(len(target)):
        local = target[neighbors[index]]
        covariance = np.cov(local.T)
        _, eigenvectors = np.linalg.eigh(covariance)
        normals[index] = eigenvectors[:, 0]

    cosine, sine = math.cos(initial_yaw), math.sin(initial_yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    translation = np.zeros(2)
    inlier_count = 0
    rmse = float("nan")

    for _ in range(max_iterations):
        transformed = source @ rotation.T + translation
        distances, indices = tree.query(transformed)
        median = float(np.median(distances))
        threshold = min(0.35, max(0.08, 2.5 * median))
        inliers = distances < threshold
        inlier_count = int(inliers.sum())
        if inlier_count < 15:
            break
        point = transformed[inliers]
        matched = target[indices[inliers]]
        normal = normals[indices[inliers]]
        matrix = np.column_stack(
            (
                normal[:, 0],
                normal[:, 1],
                -point[:, 1] * normal[:, 0] + point[:, 0] * normal[:, 1],
            )
        )
        rhs = np.sum(normal * (matched - point), axis=1)
        update, _, _, _ = np.linalg.lstsq(matrix, rhs, rcond=None)
        delta_cosine, delta_sine = math.cos(update[2]), math.sin(update[2])
        delta_rotation = np.array(
            [[delta_cosine, -delta_sine], [delta_sine, delta_cosine]]
        )
        rotation = delta_rotation @ rotation
        translation = delta_rotation @ translation + update[:2]
        rmse = float(np.sqrt(np.mean(rhs**2)))
        if np.linalg.norm(update) < 1e-4:
            break

    # Report quality for the final transform rather than the pre-update state
    # from the last iteration.
    transformed = source @ rotation.T + translation
    distances, indices = tree.query(transformed)
    median = float(np.median(distances))
    threshold = min(0.35, max(0.08, 2.5 * median))
    inliers = distances < threshold
    inlier_count = int(inliers.sum())
    if inlier_count >= 15:
        point = transformed[inliers]
        matched = target[indices[inliers]]
        normal = normals[indices[inliers]]
        residual = np.sum(normal * (matched - point), axis=1)
        rmse = float(np.sqrt(np.mean(residual**2)))

    return (
        float(translation[0]),
        float(translation[1]),
        float(wrap_angle(math.atan2(rotation[1, 0], rotation[0, 0]))),
        rmse,
        inlier_count,
    )


def compose_pose(
    pose: np.ndarray, relative: tuple[float, float, float]
) -> np.ndarray:
    x, y, yaw = pose
    dx, dy, dyaw = relative
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            x + cosine * dx - sine * dy,
            y + sine * dx + cosine * dy,
            float(wrap_angle(yaw + dyaw)),
        ]
    )


@dataclass
class BagData:
    name: str
    topic_types: dict[str, str]
    raw_counts: dict[str, int]
    unique_counts: dict[str, int]
    odom: np.ndarray
    imu: np.ndarray
    scans: list[tuple[float, np.ndarray]]
    scan_frame: str
    odom_frame: str
    base_frame: str
    imu_frame: str
    record_delay_s: dict[str, np.ndarray]
    command_times: np.ndarray
    read_error: str | None


def require_sensor_streams(
    odom_count: int, imu_count: int, scan_count: int, source: str
) -> None:
    """Reject recordings that cannot support the three-way comparison."""
    missing = [
        topic
        for topic, count in (
            ("/odom", odom_count),
            ("/imu/data_raw", imu_count),
            ("/scan", scan_count),
        )
        if count == 0
    ]
    if missing:
        raise ValueError(
            f"{source} lacks required usable topic(s): {', '.join(missing)}"
        )


def load_bag(path: Path) -> BagData:
    """Read selected topics and deduplicate them by source header stamp."""
    try:
        import rosbag2_py
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Imu, LaserScan
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Python packages are required; source /opt/ros/humble/setup.bash"
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    raw_counts: dict[str, int] = {}
    seen: dict[str, set[int]] = {}
    odom: dict[int, list[float]] = {}
    imu: dict[int, list[float]] = {}
    scans: dict[int, tuple[float, np.ndarray]] = {}
    record_delays: dict[str, list[float]] = {}
    command_times: list[float] = []
    command_count = 0
    scan_frame = odom_frame = base_frame = imu_frame = ""
    read_error = None

    while reader.has_next():
        try:
            topic, payload, bag_stamp = reader.read_next()
        except Exception as exc:  # preserve partial-bag diagnosis
            read_error = str(exc)
            break
        raw_counts[topic] = raw_counts.get(topic, 0) + 1
        if topic == "/odom":
            message = deserialize_message(payload, Odometry)
            stamp = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            seen.setdefault(topic, set()).add(stamp)
            record_delays.setdefault(topic, []).append(bag_stamp * 1e-9 - stamp * 1e-9)
            pose = message.pose.pose
            odom[stamp] = [
                stamp * 1e-9,
                float(pose.position.x),
                float(pose.position.y),
                quaternion_yaw(pose.orientation),
                float(message.twist.twist.linear.x),
                float(message.twist.twist.angular.z),
            ]
            odom_frame = message.header.frame_id
            base_frame = message.child_frame_id
        elif topic == "/imu/data_raw":
            message = deserialize_message(payload, Imu)
            stamp = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            seen.setdefault(topic, set()).add(stamp)
            record_delays.setdefault(topic, []).append(bag_stamp * 1e-9 - stamp * 1e-9)
            imu[stamp] = [
                stamp * 1e-9,
                float(message.linear_acceleration.x),
                float(message.linear_acceleration.y),
                float(message.linear_acceleration.z),
                float(message.angular_velocity.x),
                float(message.angular_velocity.y),
                float(message.angular_velocity.z),
            ]
            imu_frame = message.header.frame_id
        elif topic == "/scan":
            message = deserialize_message(payload, LaserScan)
            stamp = (
                int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec)
            )
            seen.setdefault(topic, set()).add(stamp)
            record_delays.setdefault(topic, []).append(bag_stamp * 1e-9 - stamp * 1e-9)
            points = corrected_scan_points(
                np.asarray(message.ranges, dtype=np.float64),
                float(message.angle_min),
                float(message.angle_increment),
                float(message.range_min),
                float(message.range_max),
            )
            scans[stamp] = (stamp * 1e-9, points)
            scan_frame = message.header.frame_id
        elif topic == "/cmd_vel":
            deserialize_message(payload, Twist)
            command_count += 1
            command_times.append(bag_stamp * 1e-9)
            seen.setdefault(topic, set()).add(int(bag_stamp))
        else:
            seen.setdefault(topic, set()).add(int(bag_stamp))

    raw_counts.setdefault("/cmd_vel", command_count)
    odom_array = np.asarray([odom[key] for key in sorted(odom)], dtype=float)
    imu_array = np.asarray([imu[key] for key in sorted(imu)], dtype=float)
    scan_list = [scans[key] for key in sorted(scans)]
    require_sensor_streams(
        len(odom_array), len(imu_array), len(scan_list), str(path)
    )

    start = min(odom_array[0, 0], imu_array[0, 0], scan_list[0][0])
    odom_array[:, 0] -= start
    imu_array[:, 0] -= start
    scan_list = [(stamp - start, points) for stamp, points in scan_list]
    return BagData(
        name=path.name,
        topic_types=topic_types,
        raw_counts=raw_counts,
        unique_counts={topic: len(keys) for topic, keys in seen.items()},
        odom=odom_array,
        imu=imu_array,
        scans=scan_list,
        scan_frame=scan_frame,
        odom_frame=odom_frame,
        base_frame=base_frame,
        imu_frame=imu_frame,
        record_delay_s={
            topic: np.asarray(values, dtype=float)
            for topic, values in record_delays.items()
        },
        command_times=np.asarray(command_times, dtype=float) - start,
        read_error=read_error,
    )


def relevant_scan_motion(
    data: BagData,
    windows: list[tuple[float, float]],
    gyro_bias: float,
    seed_mode: str,
) -> list[dict[str, float | int | None]]:
    """Register scan pairs overlapping benchmark windows."""
    output = []
    scans = data.scans
    for index in range(len(scans) - 1):
        t0, previous = scans[index]
        t1, current = scans[index + 1]
        dt = t1 - t0
        if dt <= 0.0 or dt > 1.0:
            continue
        if not any(t1 >= start - 0.3 and t0 <= end + 0.3 for start, end in windows):
            continue
        initial_yaw = scan_seed_yaw(
            data.imu,
            t0,
            t1,
            gyro_bias,
            seed_mode,
        )
        dx, dy, dyaw, rmse, inliers = point_to_plane_icp(
            current, previous, initial_yaw=initial_yaw
        )
        output.append(
            {
                "t0": t0,
                "t1": t1,
                "dt": dt,
                "dx": dx,
                "dy": dy,
                "dyaw": dyaw,
                "rmse": rmse if math.isfinite(rmse) else None,
                "inliers": inliers,
            }
        )
    return output


def integrate_lidar_window(
    motion: list[dict[str, float | int | None]],
    start: float,
    end: float,
    rmse_max: float,
    min_inliers: int,
) -> tuple[np.ndarray, float, int]:
    pose = np.zeros(3, dtype=float)
    covered = 0.0
    accepted = 0
    for pair in motion:
        t0 = float(pair["t0"])
        t1 = float(pair["t1"])
        overlap = max(0.0, min(t1, end) - max(t0, start))
        duration = t1 - t0
        rmse = pair["rmse"]
        if (
            overlap <= 0.0
            or duration <= 0.0
            or rmse is None
            or float(rmse) > rmse_max
            or int(pair["inliers"]) < min_inliers
        ):
            continue
        weight = overlap / duration
        pose = compose_pose(
            pose,
            (
                float(pair["dx"]) * weight,
                float(pair["dy"]) * weight,
                float(pair["dyaw"]) * weight,
            ),
        )
        covered += overlap
        accepted += 1
    return pose, covered / max(end - start, 1e-9), accepted


def lidar_window_quality(
    motion: list[dict[str, float | int | None]],
    start: float,
    end: float,
    rmse_max: float,
    min_inliers: int,
) -> dict[str, float | int | None]:
    accepted = [
        pair
        for pair in motion
        if float(pair["t1"]) > start
        and float(pair["t0"]) < end
        and pair["rmse"] is not None
        and float(pair["rmse"]) <= rmse_max
        and int(pair["inliers"]) >= min_inliers
    ]
    if not accepted:
        return {
            "accepted_pairs": 0,
            "final_rmse_median_m": None,
            "final_rmse_p95_m": None,
            "inliers_minimum": None,
            "inliers_median": None,
        }
    residuals = np.asarray([float(pair["rmse"]) for pair in accepted])
    inliers = np.asarray([int(pair["inliers"]) for pair in accepted])
    return {
        "accepted_pairs": len(accepted),
        "final_rmse_median_m": round(float(np.median(residuals)), 5),
        "final_rmse_p95_m": round(float(np.percentile(residuals, 95)), 5),
        "inliers_minimum": int(np.min(inliers)),
        "inliers_median": round(float(np.median(inliers)), 1),
    }


def segment_metrics(
    data: BagData,
    motion: list[dict[str, float | int | None]],
    windows: list[tuple[float, float]],
    kind: str,
    gyro_bias: float,
    rmse_max: float,
    min_inliers: int,
    immobility_rate: float,
    continuous_turn_rate: float,
    minimum_command_rate: float,
    boundary_shifts: list[float] | None = None,
) -> list[dict[str, Any]]:
    results = []
    for index, (start, end) in enumerate(windows, 1):
        odom_mask = (data.odom[:, 0] >= start) & (data.odom[:, 0] <= end)
        imu_mask = (data.imu[:, 0] >= start) & (data.imu[:, 0] <= end)
        if int(odom_mask.sum()) < 2 or int(imu_mask.sum()) < 2:
            continue
        odom = data.odom[odom_mask]
        imu = data.imu[imu_mask]
        duration = end - start
        odom_distance = integrate_window(
            data.odom[:, 0], data.odom[:, 4], start, end
        )
        odom_yaw = integrate_window(
            data.odom[:, 0], data.odom[:, 5], start, end
        )
        pose_chord = math.copysign(
            math.hypot(odom[-1, 1] - odom[0, 1], odom[-1, 2] - odom[0, 2]),
            odom_distance,
        )
        imu_yaw = integrate_window(
            data.imu[:, 0], data.imu[:, 6] - gyro_bias, start, end
        )
        imu_yaw_rate = imu_yaw / duration
        gravity = np.median(imu[:, 1:4], axis=0)
        vibration = np.linalg.norm(imu[:, 1:4] - gravity, axis=1)
        vibration_rms = float(np.sqrt(np.mean(vibration**2)))
        lidar_pose, coverage, pair_count = integrate_lidar_window(
            motion, start, end, rmse_max, min_inliers
        )
        lidar_quality_metrics = lidar_window_quality(
            motion, start, end, rmse_max, min_inliers
        )
        lidar_distance = math.copysign(
            math.hypot(lidar_pose[0], lidar_pose[1]),
            lidar_pose[0],
        )
        lidar_yaw_rate = lidar_pose[2] / duration
        base = {
            "index": index,
            "start_s": start,
            "end_s": end,
            "duration_s": round(duration, 3),
            "odom_distance_m": round(odom_distance, 4),
            "odom_pose_chord_m": round(pose_chord, 4),
            "odom_yaw_deg": round(math.degrees(odom_yaw), 2),
            "imu_yaw_deg": round(math.degrees(imu_yaw), 2),
            "lidar_distance_m": round(lidar_distance, 4),
            "lidar_lateral_m": round(float(lidar_pose[1]), 4),
            "lidar_yaw_deg": round(math.degrees(lidar_pose[2]), 2),
            "lidar_coverage": round(coverage, 3),
            "lidar_pair_count": pair_count,
            "lidar_quality": lidar_quality_metrics,
            "mean_odom_linear_m_s": round(float(np.mean(odom[:, 4])), 4),
            "mean_odom_yaw_rad_s": round(float(np.mean(odom[:, 5])), 4),
            "mean_imu_yaw_rad_s": round(imu_yaw_rate, 4),
            "mean_lidar_yaw_rad_s": round(lidar_yaw_rate, 4),
            "imu_vibration_rms_m_s2": round(vibration_rms, 4),
        }
        if kind == "straight":
            lidar_quality = coverage >= 0.45 and pair_count >= 2
            ratio = (
                lidar_distance / odom_distance
                if lidar_quality and abs(odom_distance) > 0.05
                else None
            )
            base["lidar_to_odom_distance_ratio"] = (
                round(ratio, 4) if ratio is not None else None
            )
            base["classification"] = (
                "INSUFFICIENT_LIDAR_EVIDENCE"
                if not lidar_quality
                else "OBSERVED_ODOM_LIDAR_DISTANCE_AGREEMENT"
                if ratio is not None
                and 0.90 <= abs(ratio) <= 1.10
                and abs(imu_yaw) <= math.radians(3.0)
                else "OBSERVED_REDUCED_LIDAR_TRANSLATION"
                if ratio is not None and 0.20 <= abs(ratio) < 0.90
                else "OBSERVED_NEAR_ZERO_LIDAR_TRANSLATION"
                if ratio is not None and abs(ratio) < 0.20
                else "INSUFFICIENT_OR_DISAGREEING_EVIDENCE"
            )
        else:
            command_rate = abs(float(np.mean(odom[:, 5])))
            imu_rate = abs(imu_yaw_rate)
            lidar_rate = abs(lidar_yaw_rate)
            corroborated_immobility = (
                imu_rate < immobility_rate
                and lidar_rate < immobility_rate
                and coverage >= 0.45
            )
            if coverage < 0.45 or pair_count < 2:
                classification = "INSUFFICIENT_LIDAR_EVIDENCE"
            elif command_rate >= minimum_command_rate and corroborated_immobility:
                classification = "OBSERVED_LOW_ROTATION_UNDER_COMMAND"
            elif imu_rate >= continuous_turn_rate and (
                abs(imu_rate - lidar_rate) <= max(0.12, 0.4 * imu_rate)
            ):
                classification = "OBSERVED_CONTINUOUS_ROTATION_UNDER_COMMAND"
            else:
                classification = "OBSERVED_INTERMEDIATE_ROTATION_UNDER_COMMAND"
            base["classification"] = classification
            base["imu_to_odom_yaw_ratio"] = round(
                imu_yaw / odom_yaw, 4
            ) if abs(odom_yaw) > math.radians(5.0) else None
        if boundary_shifts:
            shifted_results = []
            for shift in boundary_shifts:
                shifted = segment_metrics(
                    data,
                    motion,
                    [(start + shift, end + shift)],
                    kind,
                    gyro_bias,
                    rmse_max,
                    min_inliers,
                    immobility_rate,
                    continuous_turn_rate,
                    minimum_command_rate,
                    boundary_shifts=None,
                )
                if shifted:
                    shifted_results.append((float(shift), shifted[0]))
            base["boundary_shift_sensitivity"] = {
                "shifts_s": [item[0] for item in shifted_results],
                "classifications": [
                    item[1]["classification"] for item in shifted_results
                ],
                "classification_stable": all(
                    item[1]["classification"] == base["classification"]
                    for item in shifted_results
                ),
                "odom_distance_range_m": _metric_range(
                    shifted_results, "odom_distance_m"
                ),
                "imu_yaw_range_deg": _metric_range(
                    shifted_results, "imu_yaw_deg"
                ),
                "lidar_distance_range_m": _metric_range(
                    shifted_results, "lidar_distance_m"
                ),
                "lidar_yaw_range_deg": _metric_range(
                    shifted_results, "lidar_yaw_deg"
                ),
            }
        results.append(base)
    return results


def _metric_range(
    shifted_results: list[tuple[float, dict[str, Any]]],
    key: str,
) -> list[float] | None:
    values = [
        float(result[key])
        for _, result in shifted_results
        if result.get(key) is not None
    ]
    return [round(min(values), 4), round(max(values), 4)] if values else None


def topic_frequency(times: np.ndarray) -> float | None:
    if len(times) < 2:
        return None
    differences = np.diff(times)
    valid = differences[(differences > 0.0) & (differences < 1.0)]
    return float(1.0 / np.median(valid)) if len(valid) else None


def _distribution(values: np.ndarray) -> dict[str, float | int | None]:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if not len(finite):
        return {
            "count": 0,
            "median": None,
            "p95": None,
            "minimum": None,
            "maximum": None,
            "absolute_p95": None,
            "absolute_maximum": None,
        }
    return {
        "count": int(len(finite)),
        "median": round(float(np.median(finite)), 6),
        "p95": round(float(np.percentile(finite, 95)), 6),
        "minimum": round(float(np.min(finite)), 6),
        "maximum": round(float(np.max(finite)), 6),
        "absolute_p95": round(float(np.percentile(np.abs(finite), 95)), 6),
        "absolute_maximum": round(float(np.max(np.abs(finite))), 6),
    }


def _nearest_timestamp_offsets(
    reference: np.ndarray, samples: np.ndarray
) -> np.ndarray:
    """Return nearest ``sample - reference`` timestamp offsets."""
    if not len(reference) or not len(samples):
        return np.asarray([], dtype=float)
    indices = np.searchsorted(samples, reference)
    indices = np.clip(indices, 0, len(samples) - 1)
    previous = np.clip(indices - 1, 0, len(samples) - 1)
    choose_previous = (
        np.abs(samples[previous] - reference)
        <= np.abs(samples[indices] - reference)
    )
    nearest = np.where(choose_previous, samples[previous], samples[indices])
    return nearest - reference


def estimate_imu_lidar_lag(
    imu: np.ndarray,
    motion: list[dict[str, float | int | None]],
    gyro_bias: float,
    rmse_max: float,
    min_inliers: int,
    lag_limit_s: float = 0.3,
    lag_step_s: float = 0.005,
) -> dict[str, Any]:
    """Compare unseeded ICP yaw rate with gyro Z over candidate timestamp lags.

    Positive lag means the IMU value sampled at ``lidar_midpoint + lag`` best
    matches the LiDAR pair. This is a diagnostic correlation, not a correction.
    """
    accepted = [
        pair
        for pair in motion
        if pair["rmse"] is not None
        and float(pair["rmse"]) <= rmse_max
        and int(pair["inliers"]) >= min_inliers
        and float(pair["dt"]) > 0.0
    ]
    if len(accepted) < 5:
        return {
            "pair_count": len(accepted),
            "best_imu_sample_offset_s": None,
            "minimum_rmse_rad_s": None,
            "near_minimum_lag_range_s": None,
        }
    lidar_t = np.asarray(
        [
            (float(pair["t0"]) + float(pair["t1"])) / 2.0
            for pair in accepted
        ]
    )
    lidar_rate = np.asarray(
        [float(pair["dyaw"]) / float(pair["dt"]) for pair in accepted]
    )
    lags = np.arange(
        -lag_limit_s,
        lag_limit_s + lag_step_s / 2.0,
        lag_step_s,
    )
    errors = np.full_like(lags, np.nan, dtype=float)
    for index, lag in enumerate(lags):
        shifted = lidar_t + lag
        valid = (shifted >= imu[0, 0]) & (shifted <= imu[-1, 0])
        if int(valid.sum()) < 5:
            continue
        imu_rate = np.interp(shifted[valid], imu[:, 0], imu[:, 6] - gyro_bias)
        errors[index] = float(
            np.sqrt(np.mean((imu_rate - lidar_rate[valid]) ** 2))
        )
    if not np.any(np.isfinite(errors)):
        return {
            "pair_count": len(accepted),
            "best_imu_sample_offset_s": None,
            "minimum_rmse_rad_s": None,
            "near_minimum_lag_range_s": None,
        }
    best_index = int(np.nanargmin(errors))
    minimum = float(errors[best_index])
    near = np.isfinite(errors) & (errors <= minimum * 1.10 + 0.005)
    return {
        "pair_count": len(accepted),
        "best_imu_sample_offset_s": round(float(lags[best_index]), 4),
        "minimum_rmse_rad_s": round(minimum, 5),
        "near_minimum_lag_range_s": [
            round(float(np.min(lags[near])), 4),
            round(float(np.max(lags[near])), 4),
        ],
        "lag_definition": "compare IMU(t_lidar_midpoint + lag) with LiDAR",
        "search_range_s": [-lag_limit_s, lag_limit_s],
        "step_s": lag_step_s,
    }


def timing_metrics(
    data: BagData,
    motion: list[dict[str, float | int | None]],
    gyro_bias: float,
    rmse_max: float,
    min_inliers: int,
) -> dict[str, Any]:
    scan_times = np.asarray([item[0] for item in data.scans], dtype=float)
    intervals = {
        "/odom": _distribution(np.diff(data.odom[:, 0])),
        "/imu/data_raw": _distribution(np.diff(data.imu[:, 0])),
        "/scan": _distribution(np.diff(scan_times)),
        "/cmd_vel": _distribution(np.diff(data.command_times)),
    }
    record_latency = {
        topic: _distribution(values)
        for topic, values in data.record_delay_s.items()
    }
    return {
        "clock_basis": (
            "header and rosbag record stamps are ROS/system-clock epoch values; "
            "cmd_vel has no header stamp"
        ),
        "first_source_stamp_offset_from_bag_start_s": {
            "/odom": round(float(data.odom[0, 0]), 6),
            "/imu/data_raw": round(float(data.imu[0, 0]), 6),
            "/scan": round(float(scan_times[0]), 6),
            "/cmd_vel": (
                round(float(data.command_times[0]), 6)
                if len(data.command_times)
                else None
            ),
        },
        "source_intervals_s": intervals,
        "record_time_minus_source_stamp_s": record_latency,
        "nearest_scan_to_imu_offset_s": _distribution(
            _nearest_timestamp_offsets(scan_times, data.imu[:, 0])
        ),
        "nearest_scan_to_odom_offset_s": _distribution(
            _nearest_timestamp_offsets(scan_times, data.odom[:, 0])
        ),
        "imu_lidar_yaw_lag": estimate_imu_lidar_lag(
            data.imu,
            motion,
            gyro_bias,
            rmse_max,
            min_inliers,
        ),
        "lidar_scan_timing_uncertainty_s": 0.1,
        "lidar_scan_timing_note": (
            "LaserScan time_increment is zero; at 5 Hz, an individual beam's "
            "acquisition time is unknown within roughly half a 0.2 s scan."
        ),
        "cmd_vel_limitation": (
            "not recorded"
            if not len(data.command_times)
            else "record time only; geometry_msgs/Twist has no header"
        ),
    }


def _attach_seed_sensitivity(
    primary: list[dict[str, Any]],
    alternate: list[dict[str, Any]],
    alternate_seed: str,
) -> None:
    for result, comparison in zip(primary, alternate):
        result["icp_seed_sensitivity"] = {
            "alternate_seed": alternate_seed,
            "alternate_lidar_distance_m": comparison["lidar_distance_m"],
            "alternate_lidar_yaw_deg": comparison["lidar_yaw_deg"],
            "distance_delta_m": round(
                float(result["lidar_distance_m"])
                - float(comparison["lidar_distance_m"]),
                4,
            ),
            "yaw_delta_deg": round(
                float(result["lidar_yaw_deg"])
                - float(comparison["lidar_yaw_deg"]),
                2,
            ),
            "alternate_classification": comparison["classification"],
        }


def analyze_bag(
    bag_path: Path,
    spec: dict[str, Any],
    args: argparse.Namespace,
) -> tuple[dict[str, Any], BagData, list[dict[str, float | int | None]]]:
    data = load_bag(bag_path)
    stationary = (
        np.abs(np.interp(data.imu[:, 0], data.odom[:, 0], data.odom[:, 4])) < 0.005
    ) & (
        np.abs(np.interp(data.imu[:, 0], data.odom[:, 0], data.odom[:, 5])) < 0.005
    )
    stationary_gyro = data.imu[stationary, 6]
    gyro_bias = robust_bias(stationary_gyro)
    windows = list(spec["straight"]) + list(spec["turn"])
    expanded_windows = [
        (start + shift, end + shift)
        for start, end in windows
        for shift in [0.0, *args.boundary_shifts]
    ]
    motion = relevant_scan_motion(
        data, expanded_windows, gyro_bias, args.icp_seed
    )
    straight = segment_metrics(
        data,
        motion,
        spec["straight"],
        "straight",
        gyro_bias,
        args.lidar_rmse_max,
        args.lidar_min_inliers,
        args.immobility_yaw_rate,
        args.continuous_turn_yaw_rate,
        args.minimum_command_yaw_rate,
        args.boundary_shifts,
    )
    turn = segment_metrics(
        data,
        motion,
        spec["turn"],
        "turn",
        gyro_bias,
        args.lidar_rmse_max,
        args.lidar_min_inliers,
        args.immobility_yaw_rate,
        args.continuous_turn_yaw_rate,
        args.minimum_command_yaw_rate,
        args.boundary_shifts,
    )
    alternate_seed = "imu" if args.icp_seed == "zero" else "zero"
    alternate_motion = relevant_scan_motion(
        data, expanded_windows, gyro_bias, alternate_seed
    )
    alternate_straight = segment_metrics(
        data,
        alternate_motion,
        spec["straight"],
        "straight",
        gyro_bias,
        args.lidar_rmse_max,
        args.lidar_min_inliers,
        args.immobility_yaw_rate,
        args.continuous_turn_yaw_rate,
        args.minimum_command_yaw_rate,
    )
    alternate_turn = segment_metrics(
        data,
        alternate_motion,
        spec["turn"],
        "turn",
        gyro_bias,
        args.lidar_rmse_max,
        args.lidar_min_inliers,
        args.immobility_yaw_rate,
        args.continuous_turn_yaw_rate,
        args.minimum_command_yaw_rate,
    )
    _attach_seed_sensitivity(straight, alternate_straight, alternate_seed)
    _attach_seed_sensitivity(turn, alternate_turn, alternate_seed)
    duration = max(
        data.odom[-1, 0], data.imu[-1, 0], data.scans[-1][0]
    )
    inventory = {
        "role": spec["role"],
        "duration_s": round(float(duration), 3),
        "topic_types": data.topic_types,
        "raw_counts": data.raw_counts,
        "unique_counts": data.unique_counts,
        "frequencies_hz": {
            "/odom": round(topic_frequency(data.odom[:, 0]) or 0.0, 3),
            "/imu/data_raw": round(topic_frequency(data.imu[:, 0]) or 0.0, 3),
            "/scan": round(
                topic_frequency(np.asarray([scan[0] for scan in data.scans])) or 0.0,
                3,
            ),
        },
        "frames": {
            "/odom.header": data.odom_frame,
            "/odom.child": data.base_frame,
            "/imu/data_raw": data.imu_frame,
            "/scan": data.scan_frame,
        },
        "gyro_bias_z_rad_s": round(gyro_bias, 6),
        "gyro_bias_diagnostics": {
            "stationary_sample_count": int(len(stationary_gyro)),
            "stationary_span_s": (
                round(
                    float(data.imu[stationary, 0][-1] - data.imu[stationary, 0][0]),
                    3,
                )
                if int(stationary.sum()) >= 2
                else 0.0
            ),
            "stationary_gyro_z_std_rad_s": (
                round(float(np.std(stationary_gyro)), 6)
                if len(stationary_gyro)
                else None
            ),
            "stationary_gyro_z_mad_rad_s": (
                round(
                    float(
                        np.median(
                            np.abs(stationary_gyro - np.median(stationary_gyro))
                        )
                    ),
                    6,
                )
                if len(stationary_gyro)
                else None
            ),
            "selection_limitation": (
                "stationary samples are selected from open-loop odom near-zero "
                "commands, not independently observed chassis stillness"
            ),
        },
        "icp_primary_seed": args.icp_seed,
        "icp_alternate_seed": alternate_seed,
        "timing": timing_metrics(
            data,
            motion,
            gyro_bias,
            args.lidar_rmse_max,
            args.lidar_min_inliers,
        ),
        "read_error": data.read_error,
        "straight_segments": straight,
        "turn_segments": turn,
    }
    return inventory, data, motion


def make_plots(
    results: dict[str, dict[str, Any]],
    series: dict[str, BagData],
    motions: dict[str, list[dict[str, float | int | None]]],
    output: Path,
) -> list[str]:
    output.mkdir(parents=True, exist_ok=True)
    generated = []

    figure, axes = plt.subplots(1, 2, figsize=(14, 6))
    for bag_name, bag_result in results.items():
        straight = bag_result["straight_segments"]
        x = [item["odom_distance_m"] for item in straight]
        y = [item["lidar_distance_m"] for item in straight]
        axes[0].scatter(x, y, s=55, label=bag_name.replace("mapping_", ""))
    limits = [-4.5, 4.5]
    axes[0].plot(limits, limits, "k--", label="1:1 agreement")
    axes[0].set(xlim=limits, ylim=limits, xlabel="Open-loop odom distance [m]",
                ylabel="LiDAR scan-matched distance [m]",
                title="Straight-line displacement")
    axes[0].legend(fontsize=8)
    for bag_name, bag_result in results.items():
        straight = bag_result["straight_segments"]
        imu = [item["imu_yaw_deg"] for item in straight]
        lidar = [item["lidar_yaw_deg"] for item in straight]
        axes[1].scatter(imu, lidar, s=55, label=bag_name.replace("mapping_", ""))
    axes[1].plot([-15, 15], [-15, 15], "k--", label="1:1 agreement")
    axes[1].set(xlim=(-15, 15), ylim=(-15, 15),
                xlabel="IMU integrated yaw [deg]",
                ylabel="LiDAR scan-matched yaw [deg]",
                title="Straight-line heading drift")
    axes[1].legend(fontsize=8)
    figure.suptitle("Issue #23 — Straight-Line Wheel/IMU/LiDAR Consistency")
    figure.tight_layout()
    path = output / "straight_line_consistency.png"
    figure.savefig(path, dpi=170)
    plt.close(figure)
    generated.append(str(path))

    figure, axes = plt.subplots(1, 2, figsize=(14, 6))
    labels, odom_rates, imu_rates, lidar_rates, colors = [], [], [], [], []
    palette = {
        "OBSERVED_LOW_ROTATION_UNDER_COMMAND": "#d62728",
        "OBSERVED_CONTINUOUS_ROTATION_UNDER_COMMAND": "#2ca02c",
        "OBSERVED_INTERMEDIATE_ROTATION_UNDER_COMMAND": "#ff7f0e",
    }
    for bag_name, bag_result in results.items():
        for segment in bag_result["turn_segments"]:
            labels.append(f"{bag_name[8:18]}\n{segment['start_s']:.1f}s")
            odom_rates.append(segment["mean_odom_yaw_rad_s"])
            imu_rates.append(segment["mean_imu_yaw_rad_s"])
            lidar_rates.append(segment["mean_lidar_yaw_rad_s"])
            colors.append(palette.get(segment["classification"], "#7f7f7f"))
    positions = np.arange(len(labels))
    width = 0.25
    axes[0].bar(positions - width, odom_rates, width, label="open-loop /odom")
    axes[0].bar(positions, imu_rates, width, label="IMU")
    axes[0].bar(positions + width, lidar_rates, width, label="LiDAR ICP")
    axes[0].set_xticks(positions, labels, rotation=55, ha="right", fontsize=7)
    axes[0].set_ylabel("Mean yaw rate [rad/s]")
    axes[0].set_title("Turning rate by source")
    axes[0].legend()
    for position, (imu, lidar, color) in enumerate(
        zip(imu_rates, lidar_rates, colors)
    ):
        axes[1].scatter(abs(imu), abs(lidar), color=color, s=65)
        axes[1].annotate(labels[position].replace("\n", " "), (abs(imu), abs(lidar)),
                         fontsize=6, xytext=(3, 3), textcoords="offset points")
    axes[1].plot([0, 0.6], [0, 0.6], "k--")
    axes[1].set(xlim=(0, 0.6), ylim=(0, 0.6),
                xlabel="|IMU yaw rate| [rad/s]",
                ylabel="|LiDAR yaw rate| [rad/s]",
                title="Independent physical rotation estimates")
    figure.suptitle("Issue #23 — Turning Consistency & Drive/Traction Anomalies")
    figure.tight_layout()
    path = output / "turning_consistency.png"
    figure.savefig(path, dpi=170)
    plt.close(figure)
    generated.append(str(path))

    bag_name = "mapping_2026-07-13_10-54-00"
    data = series[bag_name]
    gyro_bias = results[bag_name]["gyro_bias_z_rad_s"]
    figure, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    axes[0].plot(data.odom[:, 0], data.odom[:, 5], alpha=0.55,
                 label="open-loop odom yaw rate")
    axes[0].plot(data.imu[:, 0], data.imu[:, 6] - gyro_bias,
                 label="IMU measured yaw rate")
    motion = motions[bag_name]
    good = [
        item for item in motion
        if item["rmse"] is not None and float(item["rmse"]) <= 0.25
        and int(item["inliers"]) >= 20
    ]
    axes[0].scatter(
        [(float(item["t0"]) + float(item["t1"])) / 2 for item in good],
        [float(item["dyaw"]) / float(item["dt"]) for item in good],
        s=12,
        color="#2ca02c",
        label="LiDAR ICP yaw rate",
    )
    for segment in results[bag_name]["turn_segments"]:
        if segment["classification"] == "OBSERVED_LOW_ROTATION_UNDER_COMMAND":
            for axis in axes:
                axis.axvspan(segment["start_s"], segment["end_s"],
                             color="#d62728", alpha=0.18)
    axes[0].set_ylabel("Yaw rate [rad/s]")
    axes[0].set_title("July 13 command vs physical rotation (red = corroborated immobility)")
    axes[0].legend(ncol=3, fontsize=8)
    accel = data.imu[:, 1:4]
    gravity = np.median(accel, axis=0)
    vibration = np.linalg.norm(accel - gravity, axis=1)
    axes[1].plot(data.imu[:, 0], vibration, color="#9467bd")
    axes[1].set(xlabel="Time since bag start [s]",
                ylabel="IMU dynamic acceleration [m/s²]",
                title="Mechanical excitation (not proof of belt slip)")
    figure.tight_layout()
    path = output / "july13_drive_loss_timeline.png"
    figure.savefig(path, dpi=170)
    plt.close(figure)
    generated.append(str(path))
    return generated


def summarize(results: dict[str, dict[str, Any]], args: argparse.Namespace) -> dict[str, Any]:
    turns = [
        segment
        for result in results.values()
        for segment in result["turn_segments"]
    ]
    straight = [
        segment
        for result in results.values()
        for segment in result["straight_segments"]
    ]
    counts: dict[str, int] = {}
    for segment in turns:
        key = segment["classification"]
        counts[key] = counts.get(key, 0) + 1
    healthy_straight = [
        item
        for item in straight
        if item["classification"] == "OBSERVED_ODOM_LIDAR_DISTANCE_AGREEMENT"
    ]
    ratios = [
        abs(item["lidar_to_odom_distance_ratio"])
        for item in straight
        if item["lidar_to_odom_distance_ratio"] is not None
    ]
    return {
        "architecture": {
            "sensor_fusion_implemented": False,
            "wheel_odometry": "open-loop integration of commanded motor RPM",
            "imu": "raw accel/gyro only; orientation covariance[0] = -1; not fused",
            "lidar": "Nav2 obstacle/AMCL input or offline scan matching; not fused into /odom",
            "navigation_odom_input": "/odom from wheel_driver_node",
        },
        "turn_classification_counts": counts,
        "straight_segment_count": len(straight),
        "consistent_straight_count": len(healthy_straight),
        "straight_lidar_to_odom_ratio_range": (
            [round(min(ratios), 4), round(max(ratios), 4)] if ratios else None
        ),
        "thresholds": {
            "immobility_yaw_rate_rad_s": args.immobility_yaw_rate,
            "continuous_turn_yaw_rate_rad_s": args.continuous_turn_yaw_rate,
            "minimum_command_yaw_rate_rad_s": args.minimum_command_yaw_rate,
            "lidar_rmse_max_m": args.lidar_rmse_max,
            "lidar_min_inliers": args.lidar_min_inliers,
        },
        "belt_slip_conclusion": (
            "IMU and LiDAR can corroborate low or reduced physical motion under "
            "command, but existing bags cannot isolate belt slip "
            "from track-ground skid, MKS motor stall/protection, or historical "
            "RS485 command-queue loss because no motor encoder feedback is recorded."
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bags-dir", type=Path, default=Path("/home/hsb/rosbags"))
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("docs/analysis/generated/issue23"),
    )
    parser.add_argument(
        "--bags",
        nargs="*",
        choices=sorted(BENCHMARKS),
        default=sorted(BENCHMARKS),
    )
    parser.add_argument("--immobility-yaw-rate", type=float, default=0.08)
    parser.add_argument("--continuous-turn-yaw-rate", type=float, default=0.25)
    parser.add_argument("--minimum-command-yaw-rate", type=float, default=0.80)
    parser.add_argument("--lidar-rmse-max", type=float, default=0.25)
    parser.add_argument("--lidar-min-inliers", type=int, default=20)
    parser.add_argument(
        "--icp-seed",
        choices=("zero", "imu"),
        default="zero",
        help="initial yaw for each scan registration; zero keeps LiDAR independent",
    )
    parser.add_argument(
        "--boundary-shifts",
        nargs="*",
        type=float,
        default=[-1.0, -0.5, 0.5, 1.0],
        help="uniform segment-boundary shifts used for sensitivity ranges",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results: dict[str, dict[str, Any]] = {}
    series: dict[str, BagData] = {}
    motions: dict[str, list[dict[str, float | int | None]]] = {}
    for bag_name in args.bags:
        path = args.bags_dir / bag_name
        if not path.is_dir():
            raise FileNotFoundError(f"Required benchmark bag not found: {path}")
        print(f"Analyzing {bag_name} ({BENCHMARKS[bag_name]['role']})...")
        inventory, data, motion = analyze_bag(path, BENCHMARKS[bag_name], args)
        results[bag_name] = inventory
        series[bag_name] = data
        motions[bag_name] = motion
    plots = make_plots(results, series, motions, args.output_dir)
    payload = {
        "method": {
            "deduplication": "topic + source header stamp",
            "wheel_reference": "open-loop commanded-RPM odometry (not encoder feedback)",
            "imu_yaw": "trapezoidal integration after stationary robust gyro-z bias removal",
            "lidar": (
                "historical clockwise scan reflected in-memory; point-to-plane "
                f"SE(2) ICP with {args.icp_seed} yaw seed; "
                "overlap-weighted composition"
            ),
            "correlation_warning": (
                "Adjacent scans within one segment are temporally correlated. "
                + (
                    "LiDAR registration is independent of IMU yaw."
                    if args.icp_seed == "zero"
                    else "LiDAR registration is initialized from IMU yaw and is "
                    "therefore not an independent yaw estimate."
                )
            ),
        },
        "summary": summarize(results, args),
        "bags": results,
        "plots": plots,
    }
    metrics_path = args.output_dir / "metrics.json"
    metrics_path.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"Wrote {metrics_path}")
    for plot in plots:
        print(f"Wrote {plot}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
