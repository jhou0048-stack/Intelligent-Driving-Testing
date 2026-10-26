"""Regression comparison for saved ADAS scenario results."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import pandas as pd


@dataclass(frozen=True)
class RegressionOutcome:
    """One baseline expectation evaluated against current results."""

    scenario: str
    check: str
    passed: bool
    actual: Any
    expected: str


class RegressionComparator:
    """Compare a JSON result payload against configured metric limits."""

    def __init__(self, baseline: dict[str, Any]) -> None:
        self._baseline = baseline

    def compare(
        self,
        result_payload: dict[str, Any],
    ) -> tuple[pd.DataFrame, bool]:
        current = {
            result["scenario_name"]: result
            for result in result_payload.get("results", [])
        }
        outcomes: list[RegressionOutcome] = []

        for scenario, expectation in self._baseline.get("scenarios", {}).items():
            result = current.get(scenario)
            if result is None:
                outcomes.append(RegressionOutcome(
                    scenario,
                    "scenario_present",
                    False,
                    "missing",
                    "present",
                ))
                continue

            if expectation.get("required_pass", True):
                outcomes.append(RegressionOutcome(
                    scenario,
                    "scenario_passed",
                    bool(result.get("passed")),
                    bool(result.get("passed")),
                    "True",
                ))

            metrics = result.get("metrics", {})
            for metric_name, limits in expectation.get("metrics", {}).items():
                if metric_name not in metrics:
                    outcomes.append(RegressionOutcome(
                        scenario,
                        metric_name,
                        False,
                        "missing",
                        self._format_limits(limits),
                    ))
                    continue
                actual = metrics[metric_name]
                passed = self._within_limits(actual, limits)
                outcomes.append(RegressionOutcome(
                    scenario,
                    metric_name,
                    passed,
                    actual,
                    self._format_limits(limits),
                ))

        dataframe = pd.DataFrame(
            [outcome.__dict__ for outcome in outcomes],
            columns=["scenario", "check", "passed", "actual", "expected"],
        )
        all_passed = bool(outcomes) and all(item.passed for item in outcomes)
        return dataframe, all_passed

    @staticmethod
    def _within_limits(actual: Any, limits: dict[str, Any]) -> bool:
        if "min" in limits and actual < limits["min"]:
            return False
        if "max" in limits and actual > limits["max"]:
            return False
        if "equals" in limits and actual != limits["equals"]:
            return False
        return True

    @staticmethod
    def _format_limits(limits: dict[str, Any]) -> str:
        return ", ".join(f"{key}={value}" for key, value in limits.items())
