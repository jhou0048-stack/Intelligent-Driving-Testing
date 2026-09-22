"""YAML-based configuration loader for scenarios and pipelines."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from testing.assertions import (
    MaxLateralDeviationAssertion,
    NoCollisionAssertion,
    PositionWithinAssertion,
    ScenarioAssertion,
)
from testing.scenario import ScenarioAction, ScenarioConfig

_ASSERTION_REGISTRY: dict[str, type[ScenarioAssertion]] = {
    "max_lateral_deviation": MaxLateralDeviationAssertion,
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

    return ScenarioConfig(
        name=data["name"],
        world=data["world"],
        model_name=data.get("model", "ego_vehicle"),
        actions=actions,
        pass_conditions=data.get("pass_conditions", []),
        timeout=data.get("timeout", 60.0),
    )


def build_assertions(conditions: list[dict[str, Any]]) -> list[ScenarioAssertion]:
    """Build a list of assertions from YAML pass_conditions."""
    assertions: list[ScenarioAssertion] = []
    for cond in conditions:
        cond_type = cond["type"]
        params = {k: v for k, v in cond.items() if k != "type"}
        cls = _ASSERTION_REGISTRY.get(cond_type)
        if cls is not None:
            assertions.append(cls(**params))
    return assertions


def load_scenario_suite(path: str | Path) -> list[ScenarioConfig]:
    """Load a YAML file containing a list of scenarios."""
    data = load_config(path)
    scenarios = data if isinstance(data, list) else data.get("scenarios", [data])
    return [build_scenario(s) for s in scenarios]
