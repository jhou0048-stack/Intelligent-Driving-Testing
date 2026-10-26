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
        if "final_x" not in metrics or "final_y" not in metrics:
            return False, "Position metrics are missing"
        x = metrics["final_x"]
        y = metrics["final_y"]
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
        if "max_lateral_deviation" not in metrics:
            return False, "Lateral-deviation metric is missing"
        deviation = metrics["max_lateral_deviation"]
        if deviation <= self._max_deviation:
            return True, f"Lateral deviation {deviation:.3f}m within {self._max_deviation}m"
        return False, (
            f"Lateral deviation {deviation:.3f}m exceeds {self._max_deviation}m"
        )


class NoCollisionAssertion(ScenarioAssertion):
    """Pass if no collision events were recorded."""

    def evaluate(self, metrics: dict[str, Any]) -> tuple[bool, str]:
        if "collision_count" not in metrics:
            return False, "Collision metric is missing"
        collisions = metrics["collision_count"]
        if collisions == 0:
            return True, "No collisions detected"
        return False, f"{collisions} collision(s) detected"


class MinimumDistanceTraveledAssertion(ScenarioAssertion):
    """Pass if the ego vehicle travels at least the required distance."""

    def __init__(self, min_distance: float) -> None:
        self._min_distance = min_distance

    def evaluate(self, metrics: dict[str, Any]) -> tuple[bool, str]:
        if "distance_traveled" not in metrics:
            return False, "Distance-traveled metric is missing"
        distance = metrics["distance_traveled"]
        if distance >= self._min_distance:
            return True, (
                f"Distance {distance:.3f}m meets {self._min_distance}m minimum"
            )
        return False, (
            f"Distance {distance:.3f}m below {self._min_distance}m minimum"
        )


class FinalSpeedBelowAssertion(ScenarioAssertion):
    """Pass if the final vehicle speed is below the configured limit."""

    def __init__(self, max_speed: float) -> None:
        self._max_speed = max_speed

    def evaluate(self, metrics: dict[str, Any]) -> tuple[bool, str]:
        if "final_speed" not in metrics:
            return False, "Final-speed metric is missing"
        speed = metrics["final_speed"]
        if speed <= self._max_speed:
            return True, f"Final speed {speed:.3f}m/s within {self._max_speed}m/s"
        return False, f"Final speed {speed:.3f}m/s exceeds {self._max_speed}m/s"


class AEBTriggeredAssertion(ScenarioAssertion):
    """Pass if the AEB state became active during the scenario."""

    def evaluate(self, metrics: dict[str, Any]) -> tuple[bool, str]:
        if "aeb_triggered" not in metrics:
            return False, "AEB-triggered metric is missing"
        if metrics["aeb_triggered"]:
            return True, "AEB triggered"
        return False, "AEB did not trigger"


class MinimumPerceptionStepsAssertion(ScenarioAssertion):
    """Pass if enough closed-loop perception cycles were executed."""

    def __init__(self, min_steps: int) -> None:
        self._min_steps = min_steps

    def evaluate(self, metrics: dict[str, Any]) -> tuple[bool, str]:
        if "perception_steps" not in metrics:
            return False, "Perception-step metric is missing"
        steps = metrics["perception_steps"]
        if steps >= self._min_steps:
            return True, f"Perception executed {steps} steps"
        return False, f"Perception executed {steps} steps; need {self._min_steps}"


class MinimumFaultActivationsAssertion(ScenarioAssertion):
    """Pass if configured sensor faults affected enough read cycles."""

    def __init__(self, min_activations: int = 1) -> None:
        self._min_activations = min_activations

    def evaluate(self, metrics: dict[str, Any]) -> tuple[bool, str]:
        if "fault_injection_count" not in metrics:
            return False, "Fault-injection metric is missing"
        count = metrics["fault_injection_count"]
        if count >= self._min_activations:
            return True, f"Fault injection affected {count} sensor reads"
        return False, (
            f"Fault injection affected {count} reads; "
            f"need {self._min_activations}"
        )
