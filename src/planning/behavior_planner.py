"""Higher-level behavior planning with a rule-based state machine."""

from __future__ import annotations

from abc import ABC, abstractmethod
from enum import Enum
from typing import Any

from perception.object_detector import Detection


class DrivingBehavior(Enum):
    LANE_KEEPING = "lane_keeping"
    LANE_CHANGE_LEFT = "lane_change_left"
    LANE_CHANGE_RIGHT = "lane_change_right"
    EMERGENCY_STOP = "emergency_stop"
    FOLLOW = "follow"


class BehaviorPlanner(ABC):
    """Abstract interface for deciding driving behaviour."""

    @abstractmethod
    def decide(
        self,
        current_pose: dict[str, float],
        detections: list[Detection],
        current_behavior: DrivingBehavior,
    ) -> DrivingBehavior:
        """Return the next driving behaviour given current state."""


class SimpleBehaviorPlanner(BehaviorPlanner):
    """Rule-based behaviour planner: lane-keep or emergency-stop."""

    def __init__(self, emergency_stop_distance: float = 5.0) -> None:
        self._emergency_stop_distance = emergency_stop_distance

    def decide(
        self,
        current_pose: dict[str, float],
        detections: list[Detection],
        current_behavior: DrivingBehavior,
    ) -> DrivingBehavior:
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            bbox_height = y2 - y1
            if bbox_height > self._emergency_stop_distance:
                return DrivingBehavior.EMERGENCY_STOP
        return DrivingBehavior.LANE_KEEPING
