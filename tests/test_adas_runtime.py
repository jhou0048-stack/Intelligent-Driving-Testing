"""Tests for closed-loop scenario runtime metrics."""

from unittest.mock import MagicMock

import pytest

from perception.perception_pipeline import PerceptionResult
from testing.adas_runtime import GazeboADASRuntime
from testing.scenario import ScenarioSUTConfig


def test_adas_runtime_collects_perception_and_aeb_metrics() -> None:
    runtime = GazeboADASRuntime(
        simulator=MagicMock(),
        controller=MagicMock(),
        world_name="follow_scenario",
        model_name="ego_vehicle",
        config=ScenarioSUTConfig(mode="adas", duration=1.0),
    )
    runtime._pipeline = MagicMock()
    runtime._pipeline.is_aeb_active = True

    runtime._on_step(PerceptionResult(
        timestamp=1.0,
        lateral_offset=0.1,
        lane_confidence=0.8,
        min_forward_distance=1.5,
        left_line_detected=True,
    ))
    metrics = runtime.metrics()

    assert metrics["perception_steps"] == 1
    assert metrics["lane_detection_steps"] == 1
    assert metrics["lane_detection_rate"] == pytest.approx(1.0)
    assert metrics["mean_lane_confidence"] == pytest.approx(0.8)
    assert metrics["aeb_active_steps"] == 1
    assert metrics["aeb_triggered"] is True
    assert metrics["min_forward_distance"] == pytest.approx(1.5)


def test_adas_runtime_omits_infinite_forward_distance() -> None:
    runtime = GazeboADASRuntime(
        simulator=MagicMock(),
        controller=MagicMock(),
        world_name="simple_road",
        model_name="ego_vehicle",
        config=ScenarioSUTConfig(mode="lka", duration=1.0),
    )

    metrics = runtime.metrics()

    assert "min_forward_distance" not in metrics
