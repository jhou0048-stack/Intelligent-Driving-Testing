"""LKA validation runner — wraps the LKA pipeline with metric collection."""

from __future__ import annotations

import logging
import time
from typing import Any

from testing.assertions import ScenarioAssertion
from testing.metrics import LKAMetrics
from testing.report import ScenarioResult

logger = logging.getLogger(__name__)


class LKAValidator:
    """Run an LKA pipeline and evaluate lane-keeping performance.

    Wraps around an :class:`LKAPipeline`, hooking into each step to collect
    metrics, then evaluates pass/fail conditions.
    """

    def __init__(
        self,
        scenario_name: str,
        assertions: list[ScenarioAssertion] | None = None,
        lane_width: float = 3.0,
    ) -> None:
        self._scenario_name = scenario_name
        self._assertions = assertions or []
        self._lane_width = lane_width
        self._metrics = LKAMetrics()
        self._start_time: float = 0.0

    @property
    def metrics(self) -> LKAMetrics:
        return self._metrics

    def on_step(
        self,
        lateral_offset: float,
        steering_angle: float,
        confidence: float,
    ) -> None:
        """Called after each LKA pipeline step to record metrics."""
        elapsed = time.monotonic() - self._start_time if self._start_time else 0.0
        self._metrics.record_step(
            timestamp=elapsed,
            lateral_offset=lateral_offset,
            steering_angle=steering_angle,
            confidence=confidence,
            lane_width=self._lane_width,
        )

    def on_no_detection(self) -> None:
        """Called when the pipeline step had no camera image or detection."""
        self._metrics.record_no_detection()

    def start(self) -> None:
        self._start_time = time.monotonic()
        self._metrics = LKAMetrics()

    def evaluate(self) -> ScenarioResult:
        """Evaluate all assertions against the collected metrics."""
        duration = time.monotonic() - self._start_time if self._start_time else 0.0
        metrics_dict = self._metrics.to_dict()

        # Map LKA-specific keys to what assertions expect
        metrics_dict["max_lateral_deviation"] = self._metrics.lateral_offset.max()

        failures: list[str] = []
        for assertion in self._assertions:
            passed, reason = assertion.evaluate(metrics_dict)
            if not passed:
                failures.append(reason)

        all_passed = len(failures) == 0
        error = "; ".join(failures) if failures else None

        return ScenarioResult(
            scenario_name=self._scenario_name,
            passed=all_passed,
            duration=duration,
            metrics=metrics_dict,
            error=error,
        )


class LKAValidationSuite:
    """Defines and runs a suite of LKA validation scenarios.

    Each scenario is a dict with keys:
        name, world, duration, speed, kp,
        assertions (list of ScenarioAssertion).
    """

    def __init__(self, scenarios: list[dict[str, Any]] | None = None) -> None:
        self._scenarios = scenarios or []
        self._results: list[ScenarioResult] = []

    def add_scenario(self, scenario: dict[str, Any]) -> None:
        self._scenarios.append(scenario)

    @property
    def scenarios(self) -> list[dict[str, Any]]:
        return list(self._scenarios)

    @property
    def results(self) -> list[ScenarioResult]:
        return list(self._results)

    def record_result(self, result: ScenarioResult) -> None:
        self._results.append(result)

    def summary(self) -> dict[str, Any]:
        total = len(self._results)
        passed = sum(1 for r in self._results if r.passed)
        return {
            "total": total,
            "passed": passed,
            "failed": total - passed,
            "pass_rate": passed / total if total > 0 else 0.0,
            "scenarios": [
                {
                    "name": r.scenario_name,
                    "passed": r.passed,
                    "duration": r.duration,
                    "error": r.error,
                    **r.metrics,
                }
                for r in self._results
            ],
        }

    @staticmethod
    def default_suite() -> LKAValidationSuite:
        """Return a pre-configured suite with standard LKA test cases."""
        from testing.assertions import (
            MaxLateralDeviationAssertion,
            NoCollisionAssertion,
        )

        suite = LKAValidationSuite()
        suite.add_scenario({
            "name": "straight_road_basic",
            "world": "simple_road",
            "duration": 10.0,
            "speed": 0.5,
            "kp": 0.5,
            "assertions": [
                MaxLateralDeviationAssertion(max_deviation=0.5),
            ],
        })
        suite.add_scenario({
            "name": "straight_road_fast",
            "world": "simple_road",
            "duration": 10.0,
            "speed": 1.0,
            "kp": 0.8,
            "assertions": [
                MaxLateralDeviationAssertion(max_deviation=0.8),
            ],
        })
        suite.add_scenario({
            "name": "straight_road_no_collision",
            "world": "simple_road",
            "duration": 15.0,
            "speed": 0.5,
            "kp": 0.5,
            "assertions": [
                MaxLateralDeviationAssertion(max_deviation=0.5),
                NoCollisionAssertion(),
            ],
        })
        return suite
