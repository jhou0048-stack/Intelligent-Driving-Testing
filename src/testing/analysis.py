"""Data analysis and visualization utilities for test results."""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Any

import pandas as pd

from testing.report import ScenarioResult, TestReport


class ResultAnalyzer:
    """Analyze test results and generate summary statistics."""

    def __init__(self, report: TestReport | None = None) -> None:
        self._report = report or TestReport()

    @property
    def report(self) -> TestReport:
        return self._report

    def pass_rate(self) -> float:
        results = self._report.results
        if not results:
            return 0.0
        return sum(1 for r in results if r.passed) / len(results)

    def failed_scenarios(self) -> list[ScenarioResult]:
        return [r for r in self._report.results if not r.passed]

    def metric_summary(self, metric_name: str) -> dict[str, float]:
        """Compute summary stats for a metric across all scenarios."""
        values = []
        for r in self._report.results:
            if metric_name in r.metrics:
                val = r.metrics[metric_name]
                if isinstance(val, (int, float)):
                    values.append(float(val))

        if not values:
            return {"count": 0, "min": 0.0, "max": 0.0, "mean": 0.0}

        return {
            "count": len(values),
            "min": min(values),
            "max": max(values),
            "mean": sum(values) / len(values),
        }

    def comparison_table(self) -> pd.DataFrame:
        """Generate a comparison table across all scenarios."""
        rows = []
        for r in self._report.results:
            row = {
                "scenario": r.scenario_name,
                "passed": r.passed,
                "duration_s": round(r.duration, 2),
            }
            for key in ["lateral_offset_max", "lateral_offset_mean",
                         "detection_rate", "lane_departure_rate",
                         "brake_activations", "min_distance_min"]:
                if key in r.metrics:
                    row[key] = r.metrics[key]
            rows.append(row)
        return pd.DataFrame(rows)

    def to_json(self, path: str | Path) -> None:
        """Export all results as JSON."""
        data = {
            "summary": {
                "total": len(self._report.results),
                "passed": sum(1 for r in self._report.results if r.passed),
                "pass_rate": self.pass_rate(),
            },
            "scenarios": [
                {
                    "name": r.scenario_name,
                    "passed": r.passed,
                    "duration": r.duration,
                    "error": r.error,
                    "metrics": r.metrics,
                }
                for r in self._report.results
            ],
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2, default=str)

    @classmethod
    def from_csv(cls, path: str | Path) -> ResultAnalyzer:
        """Load results from a CSV file."""
        report = TestReport()
        df = pd.read_csv(path)
        for _, row in df.iterrows():
            metrics = {k: v for k, v in row.items()
                       if k not in ("scenario", "passed", "duration", "error")}
            report.add_result(ScenarioResult(
                scenario_name=str(row["scenario"]),
                passed=bool(row["passed"]),
                duration=float(row.get("duration", 0)),
                metrics=metrics,
                error=str(row["error"]) if row.get("error") else None,
            ))
        return cls(report)
