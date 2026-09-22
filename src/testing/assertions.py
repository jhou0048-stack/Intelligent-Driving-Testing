"""Reusable pass/fail condition evaluators for scenarios."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any


class ScenarioAssertion(ABC):
    """Abstract evaluator for a single pass/fail condition."""

    @abstractmethod
    def evaluate(self, metrics: dict[str, Any]) -> tuple[bool, str]:
        """Return ``(passed, reason)`` based on collected metrics."""


class PositionWithinAssertion(ScenarioAssertion):
    """Pass if the final position is within a bounding box."""

    def __init__(
        self,
        x_min: float = -float("inf"),
        x_max: float = float("inf"),
        y_min: float = -float("inf"),
        y_max: float = float("inf"),
    ) -> None:
        self._x_min = x_min
        self._x_max = x_max
        self._y_min = y_min
        self._y_max = y_max

    def evaluate(self, metrics: dict[str, Any]) -> tuple[bool, str]:
        x = metrics.get("final_x", 0.0)
        y = metrics.get("final_y", 0.0)
        ok = self._x_min <= x <= self._x_max and self._y_min <= y <= self._y_max
        if ok:
            return True, f"Position ({x:.2f}, {y:.2f}) is within bounds"
        return False, (
            f"Position ({x:.2f}, {y:.2f}) outside bounds "
            f"[{self._x_min}, {self._x_max}] x [{self._y_min}, {self._y_max}]"
        )


class MaxLateralDeviationAssertion(ScenarioAssertion):
    """Pass if lateral deviation never exceeds a threshold."""

    def __init__(self, max_deviation: float = 1.0) -> None:
        self._max_deviation = max_deviation

    def evaluate(self, metrics: dict[str, Any]) -> tuple[bool, str]:
        deviation = metrics.get("max_lateral_deviation", 0.0)
        if deviation <= self._max_deviation:
            return True, f"Lateral deviation {deviation:.3f}m within {self._max_deviation}m"
        return False, (
            f"Lateral deviation {deviation:.3f}m exceeds {self._max_deviation}m"
        )


class NoCollisionAssertion(ScenarioAssertion):
    """Pass if no collision events were recorded."""

    def evaluate(self, metrics: dict[str, Any]) -> tuple[bool, str]:
        collisions = metrics.get("collision_count", 0)
        if collisions == 0:
            return True, "No collisions detected"
        return False, f"{collisions} collision(s) detected"
