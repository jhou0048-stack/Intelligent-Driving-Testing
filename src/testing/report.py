"""Test report generation."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import pandas as pd


@dataclass
class ScenarioResult:
    """Result of executing a single scenario."""

    scenario_name: str
    passed: bool
    duration: float
    metrics: dict[str, Any] = field(default_factory=dict)
    error: str | None = None


class TestReport:
    """Collects scenario results and generates reports."""

    def __init__(self) -> None:
        self._results: list[ScenarioResult] = []

    def add_result(self, result: ScenarioResult) -> None:
        self._results.append(result)

    @property
    def results(self) -> list[ScenarioResult]:
        return list(self._results)

    def to_dataframe(self) -> pd.DataFrame:
        rows = []
        for r in self._results:
            row = {
                "scenario": r.scenario_name,
                "passed": r.passed,
                "duration": r.duration,
                "error": r.error or "",
            }
            row.update(r.metrics)
            rows.append(row)
        return pd.DataFrame(rows)

    def to_csv(self, path: str) -> None:
        self.to_dataframe().to_csv(path, index=False)

    def summary(self) -> str:
        total = len(self._results)
        passed = sum(1 for r in self._results if r.passed)
        failed = total - passed
        return f"{total} scenarios: {passed} passed, {failed} failed"
