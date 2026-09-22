"""Automatic Emergency Braking controller.

Uses lidar range data (or object detections) to detect imminent collisions
and commands an emergency stop when obstacles are within the braking distance.
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass
from typing import Any

import numpy as np

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class AEBDecision:
    """Result of one AEB evaluation cycle."""

    should_brake: bool
    min_obstacle_distance: float
    braking_distance: float
    time_to_collision: float


class AEBController:
    """Compute emergency braking decisions from range data.

    The controller checks whether any obstacle in the forward sector is
    closer than the required braking distance at the current speed.
    """

    def __init__(
        self,
        forward_angle: float = math.radians(30),
        reaction_time: float = 0.3,
        deceleration: float = 6.0,
        safety_margin: float = 1.0,
    ) -> None:
        self._forward_angle = forward_angle
        self._reaction_time = reaction_time
        self._deceleration = deceleration
        self._safety_margin = safety_margin

    def compute_braking_distance(self, speed: float) -> float:
        reaction_dist = speed * self._reaction_time
        braking_dist = (speed ** 2) / (2.0 * self._deceleration)
        return reaction_dist + braking_dist + self._safety_margin

    def evaluate_ranges(
        self,
        ranges: np.ndarray,
        angle_min: float,
        angle_step: float,
        current_speed: float,
    ) -> AEBDecision:
        """Evaluate range data and decide whether to brake."""
        n = len(ranges)
        angles = angle_min + np.arange(n) * angle_step

        forward_mask = np.abs(angles) < self._forward_angle
        valid_mask = np.isfinite(ranges) & (ranges > 0)
        combined = forward_mask & valid_mask

        if not np.any(combined):
            return AEBDecision(
                should_brake=False,
                min_obstacle_distance=float("inf"),
                braking_distance=self.compute_braking_distance(current_speed),
                time_to_collision=float("inf"),
            )

        forward_ranges = ranges[combined]
        min_dist = float(np.min(forward_ranges))
        braking_dist = self.compute_braking_distance(current_speed)

        ttc = min_dist / current_speed if current_speed > 0.01 else float("inf")

        return AEBDecision(
            should_brake=min_dist < braking_dist,
            min_obstacle_distance=min_dist,
            braking_distance=braking_dist,
            time_to_collision=ttc,
        )

    def evaluate_detections(
        self,
        detections: list[Any],
        image_width: int,
        current_speed: float,
        depth_estimate_fn: Any = None,
    ) -> AEBDecision:
        """Evaluate object detections (camera-based) for AEB.

        Uses bbox size as a rough distance proxy when no depth is available.
        """
        braking_dist = self.compute_braking_distance(current_speed)

        if not detections:
            return AEBDecision(
                should_brake=False,
                min_obstacle_distance=float("inf"),
                braking_distance=braking_dist,
                time_to_collision=float("inf"),
            )

        min_dist = float("inf")
        centre_x = image_width / 2.0

        for det in detections:
            x1, y1, x2, y2 = det.bbox
            bbox_centre_x = (x1 + x2) / 2.0
            bbox_width_ratio = (x2 - x1) / image_width

            if abs(bbox_centre_x - centre_x) > centre_x * 0.6:
                continue

            if depth_estimate_fn is not None:
                dist = depth_estimate_fn(det)
            else:
                dist = max(0.5, 10.0 * (1.0 - bbox_width_ratio))

            min_dist = min(min_dist, dist)

        ttc = min_dist / current_speed if current_speed > 0.01 else float("inf")

        return AEBDecision(
            should_brake=min_dist < braking_dist,
            min_obstacle_distance=min_dist,
            braking_distance=braking_dist,
            time_to_collision=ttc,
        )
