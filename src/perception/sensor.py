"""Simulator-agnostic sensor interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class SensorReading:
    """A single timestamped reading from any sensor."""

    timestamp: float
    sensor_name: str
    data: Any


class Sensor(ABC):
    """Abstract interface for subscribing to a simulated sensor stream."""

    @abstractmethod
    def connect(self) -> None:
        """Start receiving sensor data."""

    @abstractmethod
    def disconnect(self) -> None:
        """Stop receiving sensor data and release resources."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Return ``True`` when the sensor stream is active."""

    @abstractmethod
    def get_latest(self) -> SensorReading | None:
        """Return the most recent reading, or ``None`` if nothing received yet."""

    def __enter__(self) -> Sensor:
        self.connect()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.disconnect()
