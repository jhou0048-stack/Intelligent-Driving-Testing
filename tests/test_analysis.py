"""Tests for result analysis utilities."""

import json
import os
import tempfile

import pytest

from testing.analysis import ResultAnalyzer
from testing.report import ScenarioResult, TestReport


def _make_report() -> TestReport:
    report = TestReport()
    report.add_result(ScenarioResult(
        scenario_name="lka_basic",
        passed=True,
        duration=10.0,
        metrics={
            "lateral_offset_max": 0.3,
            "lateral_offset_mean": 0.1,
            "detection_rate": 0.95,
            "lane_departure_rate": 0.0,
        },
    ))
    report.add_result(ScenarioResult(
        scenario_name="aeb_stop",
        passed=False,
        duration=8.0,
        metrics={
            "lateral_offset_max": 0.8,
            "brake_activations": 3,
            "min_distance_min": 1.5,
        },
        error="collision detected",
    ))
    return report


def test_pass_rate() -> None:
    analyzer = ResultAnalyzer(_make_report())
    assert analyzer.pass_rate() == pytest.approx(0.5)


def test_pass_rate_empty() -> None:
    analyzer = ResultAnalyzer()
    assert analyzer.pass_rate() == 0.0


def test_failed_scenarios() -> None:
    analyzer = ResultAnalyzer(_make_report())
    failed = analyzer.failed_scenarios()
    assert len(failed) == 1
    assert failed[0].scenario_name == "aeb_stop"


def test_metric_summary() -> None:
    analyzer = ResultAnalyzer(_make_report())
    s = analyzer.metric_summary("lateral_offset_max")
    assert s["count"] == 2
    assert s["min"] == pytest.approx(0.3)
    assert s["max"] == pytest.approx(0.8)
    assert s["mean"] == pytest.approx(0.55)


def test_metric_summary_missing() -> None:
    analyzer = ResultAnalyzer(_make_report())
    s = analyzer.metric_summary("nonexistent")
    assert s["count"] == 0


def test_comparison_table() -> None:
    analyzer = ResultAnalyzer(_make_report())
    df = analyzer.comparison_table()
    assert len(df) == 2
    assert "scenario" in df.columns
    assert "passed" in df.columns


def test_to_json() -> None:
    analyzer = ResultAnalyzer(_make_report())
    fd, path = tempfile.mkstemp(suffix=".json")
    os.close(fd)
    try:
        analyzer.to_json(path)
        with open(path) as f:
            data = json.load(f)
        assert data["summary"]["total"] == 2
        assert data["summary"]["pass_rate"] == pytest.approx(0.5)
        assert len(data["scenarios"]) == 2
    finally:
        os.unlink(path)


def test_from_csv_roundtrip() -> None:
    report = _make_report()
    fd, path = tempfile.mkstemp(suffix=".csv")
    os.close(fd)
    try:
        report.to_csv(path)
        analyzer = ResultAnalyzer.from_csv(path)
        assert len(analyzer.report.results) == 2
        assert analyzer.report.results[0].scenario_name == "lka_basic"
    finally:
        os.unlink(path)
