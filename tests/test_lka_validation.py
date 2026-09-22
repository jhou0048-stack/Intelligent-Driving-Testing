"""Tests for LKA validation framework: metrics, validator, and suite."""

import math
from unittest.mock import MagicMock

import pytest

from testing.assertions import MaxLateralDeviationAssertion, NoCollisionAssertion
from testing.lka_validator import LKAValidationSuite, LKAValidator
from testing.metrics import LKAMetrics, MetricCollector, MetricSample


# --- MetricSample -------------------------------------------------------------


def test_metric_sample_fields() -> None:
    s = MetricSample(timestamp=1.0, value=0.5)
    assert s.timestamp == 1.0
    assert s.value == 0.5


# --- MetricCollector ----------------------------------------------------------


def test_collector_empty() -> None:
    c = MetricCollector("test")
    assert c.count == 0
    assert c.max() == 0.0
    assert c.min() == 0.0
    assert c.mean() == 0.0
    assert c.std() == 0.0
    assert c.rms() == 0.0


def test_collector_single_sample() -> None:
    c = MetricCollector("test")
    c.record(0.0, 3.0)
    assert c.count == 1
    assert c.max() == 3.0
    assert c.min() == 3.0
    assert c.mean() == 3.0
    assert c.std() == 0.0  # need >= 2 for stdev


def test_collector_multiple_samples() -> None:
    c = MetricCollector("x")
    c.record(0.0, 1.0)
    c.record(1.0, 3.0)
    c.record(2.0, 5.0)
    assert c.count == 3
    assert c.max() == 5.0
    assert c.min() == 1.0
    assert c.mean() == pytest.approx(3.0)
    assert c.std() == pytest.approx(2.0)
    assert c.rms() == pytest.approx(math.sqrt((1 + 9 + 25) / 3))


def test_collector_summary_keys() -> None:
    c = MetricCollector("lat")
    c.record(0.0, 1.0)
    s = c.summary()
    assert "lat_count" in s
    assert "lat_max" in s
    assert "lat_min" in s
    assert "lat_mean" in s
    assert "lat_std" in s
    assert "lat_rms" in s


def test_collector_values_property() -> None:
    c = MetricCollector("v")
    c.record(0.0, 10.0)
    c.record(1.0, 20.0)
    assert c.values == [10.0, 20.0]


# --- LKAMetrics ---------------------------------------------------------------


def test_lka_metrics_initial() -> None:
    m = LKAMetrics()
    assert m.total_steps == 0
    assert m.lane_departure_count == 0
    assert m.no_detection_steps == 0
    assert m.detection_rate == 0.0
    assert m.lane_departure_rate == 0.0


def test_lka_metrics_record_step() -> None:
    m = LKAMetrics()
    m.record_step(timestamp=0.0, lateral_offset=0.1, steering_angle=0.05, confidence=1.0)
    assert m.total_steps == 1
    assert m.lateral_offset.count == 1
    assert m.steering_angle.count == 1
    assert m.confidence.count == 1
    assert m.lane_departure_count == 0


def test_lka_metrics_lane_departure() -> None:
    m = LKAMetrics()
    # lateral_offset > 1.0 means abs(offset) * half_lane > half_lane → departure
    m.record_step(timestamp=0.0, lateral_offset=1.5, steering_angle=0.0, confidence=0.5)
    assert m.lane_departure_count == 1
    assert m.lane_departure_rate == 1.0


def test_lka_metrics_no_detection() -> None:
    m = LKAMetrics()
    m.record_no_detection()
    m.record_step(timestamp=1.0, lateral_offset=0.1, steering_angle=0.0, confidence=1.0)
    assert m.total_steps == 2
    assert m.no_detection_steps == 1
    assert m.detection_rate == pytest.approx(0.5)


def test_lka_metrics_to_dict() -> None:
    m = LKAMetrics()
    m.record_step(timestamp=0.0, lateral_offset=0.2, steering_angle=0.1, confidence=0.9)
    d = m.to_dict()
    assert d["total_steps"] == 1
    assert "lateral_offset_max" in d
    assert "steering_angle_mean" in d
    assert "confidence_rms" in d
    assert d["detection_rate"] == 1.0


# --- LKAValidator -------------------------------------------------------------


def test_validator_no_assertions_passes() -> None:
    v = LKAValidator(scenario_name="test")
    v.start()
    v.on_step(lateral_offset=0.1, steering_angle=0.05, confidence=1.0)
    result = v.evaluate()
    assert result.passed is True
    assert result.scenario_name == "test"
    assert result.error is None


def test_validator_assertion_fails() -> None:
    v = LKAValidator(
        scenario_name="fail_test",
        assertions=[MaxLateralDeviationAssertion(max_deviation=0.1)],
    )
    v.start()
    v.on_step(lateral_offset=0.5, steering_angle=0.0, confidence=1.0)
    result = v.evaluate()
    assert result.passed is False
    assert result.error is not None
    assert "exceeds" in result.error


def test_validator_assertion_passes() -> None:
    v = LKAValidator(
        scenario_name="pass_test",
        assertions=[MaxLateralDeviationAssertion(max_deviation=1.0)],
    )
    v.start()
    v.on_step(lateral_offset=0.2, steering_angle=0.0, confidence=1.0)
    result = v.evaluate()
    assert result.passed is True


def test_validator_multiple_assertions() -> None:
    v = LKAValidator(
        scenario_name="multi",
        assertions=[
            MaxLateralDeviationAssertion(max_deviation=0.1),
            NoCollisionAssertion(),
        ],
    )
    v.start()
    v.on_step(lateral_offset=0.5, steering_angle=0.0, confidence=1.0)
    result = v.evaluate()
    assert result.passed is False
    assert "exceeds" in result.error


def test_validator_records_no_detection() -> None:
    v = LKAValidator(scenario_name="det")
    v.start()
    v.on_no_detection()
    v.on_step(lateral_offset=0.0, steering_angle=0.0, confidence=1.0)
    result = v.evaluate()
    assert result.metrics["no_detection_steps"] == 1
    assert result.metrics["detection_rate"] == pytest.approx(0.5)


def test_validator_metrics_in_result() -> None:
    v = LKAValidator(scenario_name="metrics_check")
    v.start()
    for i in range(5):
        v.on_step(lateral_offset=0.1 * i, steering_angle=0.01 * i, confidence=0.9)
    result = v.evaluate()
    assert result.metrics["total_steps"] == 5
    assert result.metrics["lateral_offset_max"] == pytest.approx(0.4)


# --- LKAValidationSuite ------------------------------------------------------


def test_suite_empty() -> None:
    suite = LKAValidationSuite()
    assert len(suite.scenarios) == 0
    assert len(suite.results) == 0
    s = suite.summary()
    assert s["total"] == 0
    assert s["pass_rate"] == 0.0


def test_suite_add_scenario() -> None:
    suite = LKAValidationSuite()
    suite.add_scenario({"name": "test1", "world": "simple_road"})
    assert len(suite.scenarios) == 1


def test_suite_record_result() -> None:
    from testing.report import ScenarioResult

    suite = LKAValidationSuite()
    suite.record_result(ScenarioResult(
        scenario_name="t1", passed=True, duration=5.0, metrics={}
    ))
    suite.record_result(ScenarioResult(
        scenario_name="t2", passed=False, duration=3.0, metrics={}, error="failed"
    ))
    s = suite.summary()
    assert s["total"] == 2
    assert s["passed"] == 1
    assert s["failed"] == 1
    assert s["pass_rate"] == pytest.approx(0.5)


def test_suite_default_has_scenarios() -> None:
    suite = LKAValidationSuite.default_suite()
    assert len(suite.scenarios) >= 3
    for sc in suite.scenarios:
        assert "name" in sc
        assert "world" in sc
        assert "assertions" in sc
