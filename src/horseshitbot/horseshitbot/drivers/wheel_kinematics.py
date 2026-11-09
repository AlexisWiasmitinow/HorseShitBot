"""Pure differential-drive kinematics in standard ROS SI units."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WheelSpeeds:
    """Commanded wheel speeds after common-factor saturation."""

    left_m_s: float
    right_m_s: float
    left_rpm: float
    right_rpm: float
    saturation_scale: float


def _positive_finite(name: str, value: float) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be positive and finite")
    return value


def twist_to_wheel_speeds(
    linear_m_s: float,
    angular_rad_s: float,
    wheel_separation_m: float,
    meters_per_motor_turn: float,
    max_wheel_speed_m_s: float,
) -> WheelSpeeds:
    """Convert a ROS ``Twist`` command to differential wheel speeds.

    ``linear_m_s`` and ``angular_rad_s`` are physical SI values, not normalized
    joystick inputs. If either wheel exceeds its speed limit, both are scaled
    by the same factor so the requested curvature is preserved.
    """
    linear_m_s = float(linear_m_s)
    angular_rad_s = float(angular_rad_s)
    if not math.isfinite(linear_m_s) or not math.isfinite(angular_rad_s):
        raise ValueError("twist values must be finite")

    separation = _positive_finite("wheel_separation_m", wheel_separation_m)
    distance_per_turn = _positive_finite(
        "meters_per_motor_turn", meters_per_motor_turn
    )
    speed_limit = _positive_finite("max_wheel_speed_m_s", max_wheel_speed_m_s)

    half_differential = angular_rad_s * separation / 2.0
    left_m_s = linear_m_s - half_differential
    right_m_s = linear_m_s + half_differential

    peak = max(abs(left_m_s), abs(right_m_s))
    scale = min(1.0, speed_limit / peak) if peak > 0.0 else 1.0
    left_m_s *= scale
    right_m_s *= scale

    rpm_per_m_s = 60.0 / distance_per_turn
    return WheelSpeeds(
        left_m_s=left_m_s,
        right_m_s=right_m_s,
        left_rpm=left_m_s * rpm_per_m_s,
        right_rpm=right_m_s * rpm_per_m_s,
        saturation_scale=scale,
    )


def wheel_rpm_to_twist(
    left_rpm: float,
    right_rpm: float,
    wheel_separation_m: float,
    meters_per_motor_turn: float,
) -> tuple[float, float]:
    """Return ``(linear_m_s, angular_rad_s)`` for wheel RPM values."""
    separation = _positive_finite("wheel_separation_m", wheel_separation_m)
    distance_per_turn = _positive_finite(
        "meters_per_motor_turn", meters_per_motor_turn
    )
    left_m_s = float(left_rpm) * distance_per_turn / 60.0
    right_m_s = float(right_rpm) * distance_per_turn / 60.0
    if not math.isfinite(left_m_s) or not math.isfinite(right_m_s):
        raise ValueError("wheel RPM values must be finite")
    return (
        (right_m_s + left_m_s) / 2.0,
        (right_m_s - left_m_s) / separation,
    )
