"""Lidar sensor interface and Gazebo implementation."""

from __future__ import annotations

import logging
import math
import threading
from abc import abstractmethod
from typing import Any

import numpy as np

from perception.sensor import Sensor, SensorReading
from simulation import GazeboSimulator

logger = logging.getLogger(__name__)


def _scan_topic(world: str, model: str, link: str, sensor: str) -> str:
    return f"/world/{world}/model/{model}/link/{link}/sensor/{sensor}/scan"


def ranges_to_point_cloud(
    ranges: np.ndarray,
    angle_min: float,
    angle_step: float,
) -> np.ndarray:
    """Convert 1-D range array to Nx3 point cloud in the sensor frame."""
    n = len(ranges)
    angles = angle_min + np.arange(n) * angle_step
    valid = np.isfinite(ranges) & (ranges > 0)
    r = ranges[valid]
    a = angles[valid]
    x = r * np.cos(a)
    y = r * np.sin(a)
    z = np.zeros_like(r)
    return np.column_stack([x, y, z])


class Lidar(Sensor):
    """Abstract lidar sensor that produces range scans."""

    @abstractmethod
    def get_ranges(self) -> np.ndarray | None:
        """Return 1-D float array of range values in metres."""

    @abstractmethod
    def get_point_cloud(self) -> np.ndarray | None:
        """Return Nx3 float array of (x, y, z) points in the sensor frame."""


class GazeboLidar(Lidar):
    """Subscribe to a Gazebo lidar sensor and decode scans."""

    def __init__(
        self,
        model_name: str = "ego_vehicle",
        link_name: str = "lidar_link",
        sensor_name: str = "front_lidar",
        world_name: str = "simple_road",
        simulator: GazeboSimulator | None = None,
        own_simulator: bool = False,
    ) -> None:
        self._model_name = model_name
        self._link_name = link_name
        self._sensor_name = sensor_name
        self._world_name = world_name
        self._simulator = simulator if simulator is not None else GazeboSimulator()
        self._own_simulator = own_simulator or simulator is None
        self._topic = _scan_topic(world_name, model_name, link_name, sensor_name)
        self._lock = threading.Lock()
        self._latest_ranges: np.ndarray | None = None
        self._angle_min: float = 0.0
        self._angle_step: float = 0.0
        self._latest_timestamp: float = 0.0

    def connect(self) -> None:
        if self._own_simulator:
            self._simulator.connect()
        self._simulator.subscribe(self._topic, "LaserScan", self._on_scan)
        logger.info("Lidar connected on %s", self._topic)

    def disconnect(self) -> None:
        with self._lock:
            self._latest_ranges = None
        if self._own_simulator:
            self._simulator.disconnect()
        logger.info("Lidar disconnected from %s", self._topic)

    def is_connected(self) -> bool:
        return self._simulator.is_connected()

    def _on_scan(self, msg: Any) -> None:
        try:
            ranges = np.array(list(msg.ranges), dtype=np.float64)
            angle_min = msg.angle_min
            angle_step = msg.angle_step
            timestamp = 0.0
            if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                stamp = msg.header.stamp
                timestamp = stamp.sec + stamp.nsec * 1e-9
            with self._lock:
                self._latest_ranges = ranges
                self._angle_min = angle_min
                self._angle_step = angle_step
                self._latest_timestamp = timestamp
        except Exception:
            logger.exception("Failed to decode lidar scan")

    def get_ranges(self) -> np.ndarray | None:
        with self._lock:
            if self._latest_ranges is None:
                return None
            return self._latest_ranges.copy()

    def get_point_cloud(self) -> np.ndarray | None:
        with self._lock:
            if self._latest_ranges is None:
                return None
            return ranges_to_point_cloud(
                self._latest_ranges, self._angle_min, self._angle_step
            )

    def get_latest(self) -> SensorReading | None:
        with self._lock:
            if self._latest_ranges is None:
                return None
            return SensorReading(
                timestamp=self._latest_timestamp,
                sensor_name=self._sensor_name,
                data=self._latest_ranges.copy(),
            )

    @property
    def topic(self) -> str:
        return self._topic
