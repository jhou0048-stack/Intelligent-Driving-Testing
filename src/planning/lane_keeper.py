"""Basic lane-keeping planner using proportional control."""

from __future__ import annotations

import math
from typing import Any

from planning.planner import Planner, Waypoint


class LaneKeeper(Planner):
    """Proportional lane-keeping controller.

    Uses the lateral offset from lane centre (from perception or ground truth)
    to compute a corrective steering waypoint at a fixed lookahead distance.
    """

    def __init__(
        self,
        lane_width: float = 3.0,
        lookahead: float = 5.0,
        kp: float = 1.0,
        default_speed: float = 1.0,
    ) -> None:
        self._lane_width = lane_width
        self._lookahead = lookahead
        self._kp = kp
        self._default_speed = default_speed

    def plan(
        self,
        current_pose: dict[str, float],
        perception_data: dict[str, Any],
    ) -> list[Waypoint]:
        lateral_offset = perception_data.get("lateral_offset", 0.0)
        correction = -self._kp * lateral_offset
        heading = current_pose.get("heading", 0.0) + correction
        x = current_pose.get("x", 0.0) + self._lookahead * math.cos(heading)
        y = current_pose.get("y", 0.0) + self._lookahead * math.sin(heading)
        return [
            Waypoint(
                x=x,
                y=y,
                target_speed=self._default_speed,
                heading=heading,
            )
        ]

    def compute_steering(
        self,
        current_pose: dict[str, float],
        target: Waypoint,
    ) -> tuple[float, float]:
        """Return ``(speed, steering_angle)`` toward the target waypoint."""
        dx = target.x - current_pose.get("x", 0.0)
        dy = target.y - current_pose.get("y", 0.0)
        target_heading = math.atan2(dy, dx)
        current_heading = current_pose.get("heading", 0.0)
        steering_angle = target_heading - current_heading
        steering_angle = math.atan2(
            math.sin(steering_angle), math.cos(steering_angle)
        )
        return target.target_speed, steering_angle

    def reset(self) -> None:
        pass
