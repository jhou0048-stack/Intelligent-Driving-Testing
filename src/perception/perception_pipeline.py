"""Integrated perception pipeline combining camera, lidar, and object detection."""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class PerceivedObject:
    """A detected object with estimated position and distance."""

    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]
    estimated_distance: float
    bearing_angle: float  # radians from vehicle forward


@dataclass
class PerceptionResult:
    """Combined perception output from one processing cycle."""

    timestamp: float
    objects: list[PerceivedObject] = field(default_factory=list)
    lateral_offset: float = 0.0
    lane_confidence: float = 0.0
    min_forward_distance: float = float("inf")
    left_line_detected: bool = False
    right_line_detected: bool = False


class PerceptionPipeline:
    """Fuse camera and lidar data for comprehensive perception.

    Combines:
    - Lane detection (from LaneDetector)
    - Object detection (from ObjectDetector / YOLODetector)
    - Distance estimation (from lidar ranges or bbox heuristic)
    """

    def __init__(
        self,
        camera: Any = None,
        lidar: Any = None,
        lane_detector: Any = None,
        object_detector: Any = None,
        image_width: int = 640,
        camera_hfov: float = 1.047,  # ~60 degrees
    ) -> None:
        self._camera = camera
        self._lidar = lidar
        self._lane_detector = lane_detector
        self._object_detector = object_detector
        self._image_width = image_width
        self._camera_hfov = camera_hfov
        self._lock = threading.Lock()
        self._latest_result: PerceptionResult | None = None

    @property
    def latest_result(self) -> PerceptionResult | None:
        with self._lock:
            return self._latest_result

    def process(self) -> PerceptionResult | None:
        """Run one perception cycle and return the fused result."""
        timestamp = time.monotonic()
        result = PerceptionResult(timestamp=timestamp)

        image = None
        if self._camera is not None:
            image = self._camera.get_image()

        # Lane detection
        if image is not None and self._lane_detector is not None:
            lane_result = self._lane_detector.detect(image)
            result.lateral_offset = lane_result.lateral_offset
            result.lane_confidence = lane_result.confidence
            result.left_line_detected = lane_result.left_line_detected
            result.right_line_detected = lane_result.right_line_detected

        # Object detection
        if image is not None and self._object_detector is not None:
            detections = self._object_detector.detect(image)
            ranges = None
            if self._lidar is not None:
                ranges = self._lidar.get_ranges()

            for det in detections:
                bearing = self._bbox_to_bearing(det.bbox)
                distance = self._estimate_distance(det.bbox, bearing, ranges)
                result.objects.append(PerceivedObject(
                    class_name=det.class_name,
                    confidence=det.confidence,
                    bbox=det.bbox,
                    estimated_distance=distance,
                    bearing_angle=bearing,
                ))

        # Forward distance from lidar
        if self._lidar is not None:
            ranges = self._lidar.get_ranges()
            if ranges is not None:
                result.min_forward_distance = self._compute_forward_distance(ranges)

        with self._lock:
            self._latest_result = result

        return result

    def _bbox_to_bearing(self, bbox: tuple[int, int, int, int]) -> float:
        x1, y1, x2, y2 = bbox
        centre_x = (x1 + x2) / 2.0
        normalised = (centre_x / self._image_width) - 0.5
        return normalised * self._camera_hfov

    def _estimate_distance(
        self,
        bbox: tuple[int, int, int, int],
        bearing: float,
        ranges: np.ndarray | None,
    ) -> float:
        if ranges is not None and len(ranges) > 0:
            n = len(ranges)
            angle_step = 2 * 3.14159 / n
            idx = int((bearing + 3.14159) / angle_step) % n
            window = 5
            start = max(0, idx - window)
            end = min(n, idx + window + 1)
            sector = ranges[start:end]
            valid = sector[(sector > 0) & np.isfinite(sector)]
            if len(valid) > 0:
                return float(np.min(valid))

        x1, y1, x2, y2 = bbox
        bbox_height = y2 - y1
        if bbox_height > 0:
            return max(0.5, 300.0 / bbox_height)
        return float("inf")

    def _compute_forward_distance(self, ranges: np.ndarray) -> float:
        n = len(ranges)
        if n == 0:
            return float("inf")
        centre = n // 2
        window = max(1, n // 12)
        sector = ranges[centre - window : centre + window + 1]
        valid = sector[(sector > 0) & np.isfinite(sector)]
        if len(valid) == 0:
            return float("inf")
        return float(np.min(valid))
