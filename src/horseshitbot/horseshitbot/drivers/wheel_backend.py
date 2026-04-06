"""Abstract base class for wheel drive backends."""

from __future__ import annotations

from abc import ABC, abstractmethod


class WheelBackend(ABC):
    """
    Interface that both MKS-stepper and ODrive wheel backends implement.
    Velocities are in RPM (positive = forward).
    """

    @abstractmethod
    def connect(self) -> None:
        """Initialise hardware connection. Raises on failure."""

    @abstractmethod
    def disconnect(self) -> None:
        """Release hardware resources."""

    @abstractmethod
    def send_velocity(self, left_rpm: float, right_rpm: float) -> None:
        """Command instantaneous wheel velocities (RPM, signed)."""

    @abstractmethod
    def stop(self) -> None:
        """Graceful stop — ramp to zero if the backend supports it."""

    @abstractmethod
    def emergency_stop(self) -> None:
        """Immediate stop — cut power / set zero as fast as possible."""

    def resume(self) -> bool:
        """Re-enable motors after emergency stop. Returns True if successful."""
        return True
