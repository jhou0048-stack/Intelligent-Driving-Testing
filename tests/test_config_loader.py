"""Tests for YAML config loader."""

import os
import tempfile

import pytest
import yaml

from testing.config_loader import (
    build_assertions,
    build_scenario,
    load_config,
    load_scenario_suite,
)
from testing.assertions import (
    MaxLateralDeviationAssertion,
    NoCollisionAssertion,
    PositionWithinAssertion,
)


def _write_yaml(data, suffix=".yaml") -> str:
    fd, path = tempfile.mkstemp(suffix=suffix)
    with os.fdopen(fd, "w") as f:
        yaml.dump(data, f)
    return path


# --- load_config --------------------------------------------------------------


def test_load_config() -> None:
    path = _write_yaml({"key": "value"})
    try:
        result = load_config(path)
        assert result == {"key": "value"}
    finally:
        os.unlink(path)


# --- build_scenario -----------------------------------------------------------


def test_build_scenario_minimal() -> None:
    data = {"name": "test", "world": "simple_road"}
    sc = build_scenario(data)
    assert sc.name == "test"
    assert sc.world == "simple_road"
    assert sc.model_name == "ego_vehicle"
    assert sc.actions == []
    assert sc.timeout == 60.0


def test_build_scenario_with_actions() -> None:
    data = {
        "name": "drive_test",
        "world": "simple_road",
        "model": "my_car",
        "timeout": 30.0,
        "actions": [
            {"type": "drive", "params": {"speed": 1.0}, "duration": 5.0},
            {"type": "stop", "duration": 1.0},
        ],
    }
    sc = build_scenario(data)
    assert sc.model_name == "my_car"
    assert len(sc.actions) == 2
    assert sc.actions[0].action_type == "drive"
    assert sc.actions[0].params["speed"] == 1.0
    assert sc.actions[0].duration == 5.0
    assert sc.timeout == 30.0


# --- build_assertions ---------------------------------------------------------


def test_build_assertions_empty() -> None:
    assert build_assertions([]) == []


def test_build_assertions_max_lateral() -> None:
    conditions = [{"type": "max_lateral_deviation", "max_deviation": 0.5}]
    result = build_assertions(conditions)
    assert len(result) == 1
    assert isinstance(result[0], MaxLateralDeviationAssertion)


def test_build_assertions_no_collision() -> None:
    conditions = [{"type": "no_collision"}]
    result = build_assertions(conditions)
    assert len(result) == 1
    assert isinstance(result[0], NoCollisionAssertion)


def test_build_assertions_position_within() -> None:
    conditions = [{"type": "position_within", "x_min": -5, "x_max": 5}]
    result = build_assertions(conditions)
    assert len(result) == 1
    assert isinstance(result[0], PositionWithinAssertion)


def test_build_assertions_unknown_type_rejected() -> None:
    conditions = [{"type": "unknown_thing"}]
    with pytest.raises(ValueError, match="Unknown assertion type"):
        build_assertions(conditions)


def test_build_scenario_with_telemetry() -> None:
    data = {
        "name": "telemetry_test",
        "world": "simple_road",
        "telemetry": {
            "lane_center_y": -1.5,
            "sample_interval": 0.1,
            "startup_timeout": 2.0,
        },
    }
    scenario = build_scenario(data)
    assert scenario.telemetry.lane_center_y == pytest.approx(-1.5)
    assert scenario.telemetry.sample_interval == pytest.approx(0.1)
    assert scenario.telemetry.startup_timeout == pytest.approx(2.0)


def test_build_scenario_with_closed_loop_sut() -> None:
    scenario = build_scenario({
        "name": "aeb_test",
        "world": "follow_scenario",
        "sut": {
            "mode": "aeb",
            "duration": 12.0,
            "speed": 2.0,
            "loop_hz": 20.0,
        },
    })
    assert scenario.sut.mode == "aeb"
    assert scenario.sut.duration == pytest.approx(12.0)
    assert scenario.sut.speed == pytest.approx(2.0)


def test_build_scenario_preserves_fault_configuration() -> None:
    scenario = build_scenario({
        "name": "fault_test",
        "world": "simple_road",
        "sut": {
            "mode": "lka",
            "duration": 2.0,
            "faults": [
                {
                    "target": "camera",
                    "type": "dropout",
                    "start_time": 0.5,
                    "duration": 0.5,
                }
            ],
        },
    })
    assert scenario.sut.faults[0]["type"] == "dropout"


def test_build_scenario_rejects_invalid_fault() -> None:
    with pytest.raises(ValueError, match="Unsupported camera fault"):
        build_scenario({
            "name": "bad_fault",
            "world": "simple_road",
            "sut": {
                "mode": "lka",
                "duration": 2.0,
                "faults": [{"target": "camera", "type": "corruption"}],
            },
        })


def test_build_scenario_rejects_unknown_sut_mode() -> None:
    with pytest.raises(ValueError, match="Unknown SUT mode"):
        build_scenario({
            "name": "bad",
            "world": "simple_road",
            "sut": {"mode": "unknown", "duration": 1.0},
        })


def test_build_scenario_rejects_zero_closed_loop_duration() -> None:
    with pytest.raises(ValueError, match="duration"):
        build_scenario({
            "name": "bad",
            "world": "simple_road",
            "sut": {"mode": "lka", "duration": 0.0},
        })


def test_build_assertions_multiple() -> None:
    conditions = [
        {"type": "no_collision"},
        {"type": "max_lateral_deviation", "max_deviation": 1.0},
    ]
    result = build_assertions(conditions)
    assert len(result) == 2


# --- load_scenario_suite ------------------------------------------------------


def test_load_scenario_suite_list() -> None:
    data = {
        "scenarios": [
            {"name": "s1", "world": "w1"},
            {"name": "s2", "world": "w2"},
        ]
    }
    path = _write_yaml(data)
    try:
        suite = load_scenario_suite(path)
        assert len(suite) == 2
        assert suite[0].name == "s1"
        assert suite[1].name == "s2"
    finally:
        os.unlink(path)


def test_load_scenario_suite_single() -> None:
    data = {"name": "single", "world": "simple_road"}
    path = _write_yaml(data)
    try:
        suite = load_scenario_suite(path)
        assert len(suite) == 1
    finally:
        os.unlink(path)


def test_load_real_config() -> None:
    config_path = os.path.join(
        os.path.dirname(__file__), "..", "config", "scenarios.yaml"
    )
    if os.path.exists(config_path):
        suite = load_scenario_suite(config_path)
        assert len(suite) >= 4
        for sc in suite:
            assert sc.name
            assert sc.world
