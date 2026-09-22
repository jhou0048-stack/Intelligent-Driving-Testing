"""Simulator-agnostic path and behavior planning interface."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Waypoint:
    """A target point the vehicle should reach."""

    x: float
    y: float
    target_speed: float
    heading: float = 0.0


class Planner(ABC):
    """Abstract interface for path/behavior planners."""

    @abstractmethod
    def plan(
        self,
        current_pose: dict[str, float],
        perception_data: dict[str, Any],
    ) -> list[Waypoint]:
        """Generate waypoints given current state and perception input."""

    @abstractmethod
    def reset(self) -> None:
        """Clear any internal planning state."""
