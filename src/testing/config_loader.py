"""YAML-based configuration loader for scenarios and pipelines."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from testing.assertions import (
    AEBTriggeredAssertion,
    FinalSpeedBelowAssertion,
    MaxLateralDeviationAssertion,
    MinimumDistanceTraveledAssertion,
    MinimumFaultActivationsAssertion,
    MinimumPerceptionStepsAssertion,
    NoCollisionAssertion,
    PositionWithinAssertion,
    ScenarioAssertion,
)
from testing.scenario import (
    ScenarioAction,
    ScenarioConfig,
    ScenarioSUTConfig,
    ScenarioTelemetryConfig,
)
from testing.fault_injection import FaultSpec

_ASSERTION_REGISTRY: dict[str, type[ScenarioAssertion]] = {
    "aeb_triggered": AEBTriggeredAssertion,
    "final_speed_below": FinalSpeedBelowAssertion,
    "max_lateral_deviation": MaxLateralDeviationAssertion,
    "minimum_distance_traveled": MinimumDistanceTraveledAssertion,
    "minimum_fault_activations": MinimumFaultActivationsAssertion,
    "minimum_perception_steps": MinimumPerceptionStepsAssertion,
    "no_collision": NoCollisionAssertion,
    "position_within": PositionWithinAssertion,
}


def load_config(path: str | Path) -> dict[str, Any]:
    """Load a YAML config file and return the parsed dict."""
    with open(path) as f:
        return yaml.safe_load(f)


def build_scenario(data: dict[str, Any]) -> ScenarioConfig:
    """Build a ScenarioConfig from a parsed YAML dict."""
    actions = []
    for a in data.get("actions", []):
        actions.append(ScenarioAction(
            action_type=a["type"],
            params=a.get("params", {}),
            duration=a.get("duration", 0.0),
        ))

    telemetry_data = data.get("telemetry", {})
    telemetry = ScenarioTelemetryConfig(
        lane_center_y=float(telemetry_data.get("lane_center_y", 0.0)),
        odometry_topic=telemetry_data.get("odometry_topic"),
        contact_topic=telemetry_data.get("contact_topic"),
        sample_interval=float(telemetry_data.get("sample_interval", 0.05)),
        startup_timeout=float(telemetry_data.get("startup_timeout", 5.0)),
    )
    if telemetry.sample_interval <= 0 or telemetry.startup_timeout <= 0:
        raise ValueError(
            "Telemetry sample_interval and startup_timeout must be positive"
        )

    sut_data = data.get("sut", {})
    sut = ScenarioSUTConfig(
        mode=str(sut_data.get("mode", "open_loop")),
        duration=float(sut_data.get("duration", 0.0)),
        speed=float(sut_data.get("speed", 0.5)),
        loop_hz=float(sut_data.get("loop_hz", 10.0)),
        enable_yolo=bool(sut_data.get("enable_yolo", False)),
        yolo_model=str(sut_data.get("yolo_model", "yolov8n.pt")),
        yolo_confidence=float(sut_data.get("yolo_confidence", 0.5)),
        faults=list(sut_data.get("faults", [])),
    )
    valid_modes = {"open_loop", "lka", "aeb", "adas"}
    if sut.mode not in valid_modes:
        raise ValueError(
            f"Unknown SUT mode {sut.mode!r}; expected one of "
            f"{sorted(valid_modes)}"
        )
    if sut.mode != "open_loop" and sut.duration <= 0:
        raise ValueError("Closed-loop SUT duration must be greater than zero")
    if sut.loop_hz <= 0:
        raise ValueError("SUT loop_hz must be greater than zero")
    for fault in sut.faults:
        FaultSpec.from_dict(fault)

    timeout = float(data.get("timeout", 60.0))
    if timeout <= 0:
        raise ValueError("Scenario timeout must be greater than zero")

    return ScenarioConfig(
        name=data["name"],
        world=data["world"],
        model_name=data.get("model", "ego_vehicle"),
        actions=actions,
        pass_conditions=data.get("pass_conditions", []),
        telemetry=telemetry,
        sut=sut,
        timeout=timeout,
    )


def build_assertions(conditions: list[dict[str, Any]]) -> list[ScenarioAssertion]:
    """Build a list of assertions from YAML pass_conditions."""
    assertions: list[ScenarioAssertion] = []
    for cond in conditions:
        cond_type = cond["type"]
        params = {k: v for k, v in cond.items() if k != "type"}
        cls = _ASSERTION_REGISTRY.get(cond_type)
        if cls is None:
            raise ValueError(f"Unknown assertion type: {cond_type}")
        assertions.append(cls(**params))
    return assertions


def load_scenario_suite(path: str | Path) -> list[ScenarioConfig]:
    """Load a YAML file containing a list of scenarios."""
    data = load_config(path)
    scenarios = data if isinstance(data, list) else data.get("scenarios", [data])
    return [build_scenario(s) for s in scenarios]
