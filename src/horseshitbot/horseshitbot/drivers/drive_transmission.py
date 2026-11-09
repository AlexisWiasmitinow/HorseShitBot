"""Drive-transmission conversions shared by motor control and odometry."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class DriveTransmission:
    """Mechanical relationship between motor turns and driven-wheel travel."""

    motor_pulley_teeth: int
    driven_pulley_teeth: int
    meters_per_wheel_turn: float

    def __post_init__(self) -> None:
        if self.motor_pulley_teeth <= 0:
            raise ValueError("motor_pulley_teeth must be positive")
        if self.driven_pulley_teeth <= 0:
            raise ValueError("driven_pulley_teeth must be positive")
        if self.meters_per_wheel_turn <= 0.0:
            raise ValueError("meters_per_wheel_turn must be positive")

    @property
    def motor_to_wheel_ratio(self) -> float:
        return self.motor_pulley_teeth / self.driven_pulley_teeth

    @property
    def reduction_ratio(self) -> float:
        return self.driven_pulley_teeth / self.motor_pulley_teeth

    @property
    def meters_per_motor_turn(self) -> float:
        return self.motor_to_wheel_ratio * self.meters_per_wheel_turn

    def motor_to_wheel_revolutions(self, motor_revolutions: float) -> float:
        return motor_revolutions * self.motor_to_wheel_ratio

    def wheel_to_motor_revolutions(self, wheel_revolutions: float) -> float:
        return wheel_revolutions * self.reduction_ratio

    def motor_to_wheel_rpm(self, motor_rpm: float) -> float:
        return self.motor_to_wheel_revolutions(motor_rpm)

    def wheel_to_motor_rpm(self, wheel_rpm: float) -> float:
        return self.wheel_to_motor_revolutions(wheel_rpm)

    def motor_revolutions_to_distance(self, motor_revolutions: float) -> float:
        return (
            self.motor_to_wheel_revolutions(motor_revolutions)
            * self.meters_per_wheel_turn
        )

    def motor_rpm_to_linear_velocity(self, motor_rpm: float) -> float:
        return self.motor_revolutions_to_distance(motor_rpm) / 60.0

    def linear_velocity_to_motor_rpm(self, velocity_m_s: float) -> float:
        return velocity_m_s / self.meters_per_motor_turn * 60.0

    def motor_rpms_to_twist(
        self, left_motor_rpm: float, right_motor_rpm: float, wheel_separation: float
    ) -> tuple[float, float]:
        """Return differential-drive ``(linear, angular)`` velocity."""
        if wheel_separation <= 0.0:
            raise ValueError("wheel_separation must be positive")
        v_left = self.motor_rpm_to_linear_velocity(left_motor_rpm)
        v_right = self.motor_rpm_to_linear_velocity(right_motor_rpm)
        return (v_right + v_left) / 2.0, (v_right - v_left) / wheel_separation
