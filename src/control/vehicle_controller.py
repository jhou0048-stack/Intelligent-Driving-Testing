"""Simulator-agnostic vehicle-control interface."""

from __future__ import annotations

from abc import ABC, abstractmethod


class VehicleController(ABC):
    """Abstract interface for sending commands to an ego vehicle.

    Implementations are expected to translate simulator-agnostic speed and
    steering commands into simulator-specific messages (e.g. ``gz.msgs.Twist``
    for Gazebo's Ackermann steering plugin).
    """

    @abstractmethod
    def connect(self) -> None:
        """Open the connection to the vehicle control channel."""

    @abstractmethod
    def disconnect(self) -> None:
        """Release the connection and any associated resources."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Return ``True`` when the controller is ready to send commands."""

    @abstractmethod
    def send_command(self, speed: float, steering_angle: float) -> None:
        """Send a speed and steering command to the vehicle.

        Args:
            speed: Target longitudinal speed in m/s. Positive values move the
                vehicle forward; negative values move it backward.
            steering_angle: Target steering angle in radians. Positive values
                typically turn left and negative values turn right for an
                Ackermann-steered vehicle.
        """

    def stop(self) -> None:
        """Send a zero-speed, zero-steering command.

        This is a convenience wrapper around :meth:`send_command`.
        """
        self.send_command(0.0, 0.0)

    def __enter__(self) -> "VehicleController":
        self.connect()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.disconnect()
