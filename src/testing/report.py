"""Test report generation."""

from __future__ import annotations

import json
from dataclasses import asdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import pandas as pd


@dataclass
class AssertionResult:
    """Outcome of one scenario pass condition."""

    name: str
    passed: bool
    reason: str


@dataclass
class ScenarioResult:
    """Result of executing a single scenario."""

    scenario_name: str
    passed: bool
    duration: float
    metrics: dict[str, Any] = field(default_factory=dict)
    assertions: list[AssertionResult] = field(default_factory=list)
    error: str | None = None


class TestReport:
    """Collects scenario results and generates reports."""

    __test__ = False

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
                "assertions_total": len(r.assertions),
                "assertions_passed": sum(a.passed for a in r.assertions),
                "assertion_details": "; ".join(
                    f"{a.name}: {a.reason}" for a in r.assertions
                ),
            }
            row.update(r.metrics)
            rows.append(row)
        return pd.DataFrame(rows)

    def to_csv(self, path: str) -> None:
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.to_dataframe().to_csv(output_path, index=False)

    def to_json(self, path: str) -> None:
        """Write complete scenario results as structured JSON."""
        output_path = Path(path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "summary": self.summary_dict(),
            "results": [asdict(result) for result in self._results],
        }
        output_path.write_text(
            json.dumps(payload, indent=2, allow_nan=False),
            encoding="utf-8",
        )

    @property
    def all_passed(self) -> bool:
        return bool(self._results) and all(r.passed for r in self._results)

    def summary_dict(self) -> dict[str, int | float]:
        total = len(self._results)
        passed = sum(1 for r in self._results if r.passed)
        failed = total - passed
        return {
            "total": total,
            "passed": passed,
            "failed": failed,
            "pass_rate": passed / total if total else 0.0,
        }

    def summary(self) -> str:
        summary = self.summary_dict()
        return (
            f"{summary['total']} scenarios: {summary['passed']} passed, "
            f"{summary['failed']} failed"
        )
