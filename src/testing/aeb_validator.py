"""AEB validation runner — evaluates AEB performance with metric collection."""

from __future__ import annotations

import logging
import time
from typing import Any

from planning.aeb_controller import AEBDecision
from testing.assertions import ScenarioAssertion
from testing.metrics import MetricCollector
from testing.report import ScenarioResult

logger = logging.getLogger(__name__)


class AEBMetrics:
    """Aggregated AEB validation metrics."""

    def __init__(self) -> None:
        self.min_distance = MetricCollector("min_distance")
        self.ttc = MetricCollector("ttc")
        self.braking_distance = MetricCollector("braking_distance")
        self.total_steps: int = 0
        self.brake_activations: int = 0
        self.false_positives: int = 0
        self.true_positives: int = 0
        self.collision_occurred: bool = False

    def record_step(
        self,
        timestamp: float,
        decision: AEBDecision,
        actual_collision: bool = False,
    ) -> None:
        self.total_steps += 1
        self.min_distance.record(timestamp, decision.min_obstacle_distance)
        self.ttc.record(timestamp, min(decision.time_to_collision, 100.0))
        self.braking_distance.record(timestamp, decision.braking_distance)

        if decision.should_brake:
            self.brake_activations += 1
            if decision.min_obstacle_distance < decision.braking_distance * 1.5:
                self.true_positives += 1
            else:
                self.false_positives += 1

        if actual_collision:
            self.collision_occurred = True

    @property
    def false_positive_rate(self) -> float:
        if self.brake_activations == 0:
            return 0.0
        return self.false_positives / self.brake_activations

    @property
    def brake_rate(self) -> float:
        if self.total_steps == 0:
            return 0.0
        return self.brake_activations / self.total_steps

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "total_steps": self.total_steps,
            "brake_activations": self.brake_activations,
            "brake_rate": self.brake_rate,
            "true_positives": self.true_positives,
            "false_positives": self.false_positives,
            "false_positive_rate": self.false_positive_rate,
            "collision_occurred": self.collision_occurred,
            "collision_count": 1 if self.collision_occurred else 0,
        }
        result.update(self.min_distance.summary())
        result.update(self.ttc.summary())
        result.update(self.braking_distance.summary())
        return result


class AEBValidator:
    """Run an AEB pipeline and evaluate braking performance."""

    def __init__(
        self,
        scenario_name: str,
        assertions: list[ScenarioAssertion] | None = None,
    ) -> None:
        self._scenario_name = scenario_name
        self._assertions = assertions or []
        self._metrics = AEBMetrics()
        self._start_time: float = 0.0

    @property
    def metrics(self) -> AEBMetrics:
        return self._metrics

    def start(self) -> None:
        self._start_time = time.monotonic()
        self._metrics = AEBMetrics()

    def on_decision(self, decision: AEBDecision, collision: bool = False) -> None:
        elapsed = time.monotonic() - self._start_time if self._start_time else 0.0
        self._metrics.record_step(elapsed, decision, collision)

    def evaluate(self) -> ScenarioResult:
        duration = time.monotonic() - self._start_time if self._start_time else 0.0
        metrics_dict = self._metrics.to_dict()

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


class AEBValidationSuite:
    """Defines and runs a suite of AEB validation scenarios."""

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
                {"name": r.scenario_name, "passed": r.passed, "error": r.error, **r.metrics}
                for r in self._results
            ],
        }

    @staticmethod
    def default_suite() -> AEBValidationSuite:
        from testing.assertions import NoCollisionAssertion

        suite = AEBValidationSuite()
        suite.add_scenario({
            "name": "obstacle_ahead_stop",
            "world": "follow_scenario",
            "duration": 15.0,
            "speed": 0.5,
            "assertions": [NoCollisionAssertion()],
        })
        suite.add_scenario({
            "name": "pedestrian_crossing_stop",
            "world": "pedestrian_crossing",
            "duration": 15.0,
            "speed": 0.5,
            "assertions": [NoCollisionAssertion()],
        })
        suite.add_scenario({
            "name": "high_speed_braking",
            "world": "follow_scenario",
            "duration": 10.0,
            "speed": 1.0,
            "assertions": [NoCollisionAssertion()],
        })
        return suite
