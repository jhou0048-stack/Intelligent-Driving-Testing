"""Tests for AEB controller, pipeline, and validation framework."""

import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from planning.aeb_controller import AEBController, AEBDecision
from planning.aeb_pipeline import AEBPipeline
from testing.aeb_validator import AEBMetrics, AEBValidationSuite, AEBValidator
from testing.assertions import NoCollisionAssertion


# --- AEBDecision --------------------------------------------------------------


def test_aeb_decision_fields() -> None:
    d = AEBDecision(
        should_brake=True,
        min_obstacle_distance=3.0,
        braking_distance=5.0,
        time_to_collision=2.0,
    )
    assert d.should_brake is True
    assert d.min_obstacle_distance == 3.0


# --- AEBController ------------------------------------------------------------


def test_braking_distance_at_zero_speed() -> None:
    aeb = AEBController(safety_margin=1.0)
    assert aeb.compute_braking_distance(0.0) == pytest.approx(1.0)


def test_braking_distance_increases_with_speed() -> None:
    aeb = AEBController()
    d1 = aeb.compute_braking_distance(1.0)
    d2 = aeb.compute_braking_distance(2.0)
    assert d2 > d1


def test_evaluate_ranges_no_obstacle() -> None:
    aeb = AEBController(forward_angle=math.radians(30))
    ranges = np.full(360, 100.0)
    decision = aeb.evaluate_ranges(ranges, -math.pi, 2 * math.pi / 360, 1.0)
    assert decision.should_brake is False
    assert decision.min_obstacle_distance == pytest.approx(100.0, abs=1.0)


def test_evaluate_ranges_obstacle_ahead() -> None:
    aeb = AEBController(forward_angle=math.radians(30), safety_margin=1.0)
    ranges = np.full(360, 100.0)
    # Place obstacle directly ahead (angle ~0)
    ranges[180] = 1.0
    angle_min = -math.pi
    angle_step = 2 * math.pi / 360
    decision = aeb.evaluate_ranges(ranges, angle_min, angle_step, 1.0)
    assert decision.should_brake is True
    assert decision.min_obstacle_distance == pytest.approx(1.0)


def test_evaluate_ranges_obstacle_behind_no_brake() -> None:
    aeb = AEBController(forward_angle=math.radians(30))
    ranges = np.full(360, 100.0)
    ranges[0] = 0.5  # Behind the vehicle (angle = -pi)
    decision = aeb.evaluate_ranges(ranges, -math.pi, 2 * math.pi / 360, 1.0)
    assert decision.should_brake is False


def test_evaluate_ranges_empty_valid() -> None:
    aeb = AEBController()
    ranges = np.full(10, float("inf"))
    decision = aeb.evaluate_ranges(ranges, -0.5, 0.1, 1.0)
    assert decision.should_brake is False
    assert decision.min_obstacle_distance == float("inf")


def test_evaluate_detections_empty() -> None:
    aeb = AEBController()
    decision = aeb.evaluate_detections([], 640, 1.0)
    assert decision.should_brake is False


def test_evaluate_detections_close_object() -> None:
    aeb = AEBController(safety_margin=1.0)
    det = MagicMock()
    det.bbox = (200, 100, 440, 400)  # large bbox = close object
    decision = aeb.evaluate_detections([det], 640, 1.0)
    assert decision.min_obstacle_distance < 10.0


def test_evaluate_detections_off_centre_ignored() -> None:
    aeb = AEBController()
    det = MagicMock()
    det.bbox = (0, 0, 50, 50)  # far left
    decision = aeb.evaluate_detections([det], 640, 1.0)
    assert decision.should_brake is False


def test_ttc_at_zero_speed() -> None:
    aeb = AEBController()
    ranges = np.full(360, 2.0)
    decision = aeb.evaluate_ranges(ranges, -math.pi, 2 * math.pi / 360, 0.0)
    assert decision.time_to_collision == float("inf")


# --- AEBPipeline --------------------------------------------------------------


def test_aeb_pipeline_start_stop() -> None:
    ctrl = MagicMock()
    pipeline = AEBPipeline(controller=ctrl, loop_hz=100.0)
    assert pipeline.is_running is False
    pipeline.start()
    assert pipeline.is_running is True
    pipeline.stop()
    assert pipeline.is_running is False


def test_aeb_pipeline_step_with_lidar() -> None:
    ctrl = MagicMock()
    lidar = MagicMock()
    lidar.get_ranges.return_value = np.full(360, 1.0)
    lidar._angle_min = -math.pi
    lidar._angle_step = 2 * math.pi / 360

    pipeline = AEBPipeline(
        controller=ctrl,
        lidar=lidar,
        current_speed_fn=lambda: 1.0,
    )
    pipeline._step()
    assert pipeline.step_count == 1
    assert pipeline.last_decision is not None
    assert pipeline.last_decision.should_brake is True
    ctrl.stop.assert_called()


def test_aeb_pipeline_no_sensor_skips() -> None:
    ctrl = MagicMock()
    pipeline = AEBPipeline(controller=ctrl)
    pipeline._step()
    assert pipeline.step_count == 0


def test_aeb_pipeline_no_obstacle_no_brake() -> None:
    ctrl = MagicMock()
    lidar = MagicMock()
    lidar.get_ranges.return_value = np.full(360, 100.0)
    lidar._angle_min = -math.pi
    lidar._angle_step = 2 * math.pi / 360

    pipeline = AEBPipeline(
        controller=ctrl, lidar=lidar,
        current_speed_fn=lambda: 0.5,
    )
    pipeline._step()
    assert pipeline.is_braking is False
    ctrl.stop.assert_not_called()


def test_aeb_pipeline_context_manager() -> None:
    ctrl = MagicMock()
    pipeline = AEBPipeline(controller=ctrl, loop_hz=100.0)
    with pipeline:
        assert pipeline.is_running is True
    assert pipeline.is_running is False


# --- AEBMetrics ---------------------------------------------------------------


def test_aeb_metrics_initial() -> None:
    m = AEBMetrics()
    assert m.total_steps == 0
    assert m.brake_activations == 0
    assert m.false_positive_rate == 0.0


def test_aeb_metrics_record_brake() -> None:
    m = AEBMetrics()
    d = AEBDecision(should_brake=True, min_obstacle_distance=2.0,
                     braking_distance=3.0, time_to_collision=1.0)
    m.record_step(0.0, d)
    assert m.brake_activations == 1
    assert m.true_positives == 1


def test_aeb_metrics_false_positive() -> None:
    m = AEBMetrics()
    d = AEBDecision(should_brake=True, min_obstacle_distance=50.0,
                     braking_distance=3.0, time_to_collision=100.0)
    m.record_step(0.0, d)
    assert m.false_positives == 1
    assert m.false_positive_rate == pytest.approx(1.0)


def test_aeb_metrics_collision() -> None:
    m = AEBMetrics()
    d = AEBDecision(should_brake=False, min_obstacle_distance=0.1,
                     braking_distance=3.0, time_to_collision=0.1)
    m.record_step(0.0, d, actual_collision=True)
    assert m.collision_occurred is True


def test_aeb_metrics_to_dict() -> None:
    m = AEBMetrics()
    d = AEBDecision(should_brake=True, min_obstacle_distance=2.0,
                     braking_distance=3.0, time_to_collision=1.5)
    m.record_step(0.0, d)
    result = m.to_dict()
    assert result["brake_activations"] == 1
    assert "min_distance_max" in result
    assert "ttc_mean" in result


# --- AEBValidator -------------------------------------------------------------


def test_aeb_validator_passes() -> None:
    v = AEBValidator("test", assertions=[NoCollisionAssertion()])
    v.start()
    d = AEBDecision(should_brake=True, min_obstacle_distance=2.0,
                     braking_distance=3.0, time_to_collision=1.0)
    v.on_decision(d)
    result = v.evaluate()
    assert result.passed is True


def test_aeb_validator_collision_fails() -> None:
    v = AEBValidator("test", assertions=[NoCollisionAssertion()])
    v.start()
    d = AEBDecision(should_brake=False, min_obstacle_distance=0.0,
                     braking_distance=3.0, time_to_collision=0.0)
    v.on_decision(d, collision=True)
    result = v.evaluate()
    assert result.passed is False
    assert "collision" in result.error.lower()


# --- AEBValidationSuite -------------------------------------------------------


def test_aeb_suite_default() -> None:
    suite = AEBValidationSuite.default_suite()
    assert len(suite.scenarios) >= 3


def test_aeb_suite_summary() -> None:
    from testing.report import ScenarioResult
    suite = AEBValidationSuite()
    suite.record_result(ScenarioResult("s1", True, 5.0, {}))
    suite.record_result(ScenarioResult("s2", False, 3.0, {}, error="fail"))
    s = suite.summary()
    assert s["total"] == 2
    assert s["passed"] == 1
