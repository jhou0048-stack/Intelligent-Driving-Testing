"""Scenario definition data structures."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass
class ScenarioAction:
    """One step in a test scenario."""

    action_type: str
    params: dict[str, Any] = field(default_factory=dict)
    duration: float = 0.0


@dataclass
class ScenarioConfig:
    """Complete scenario definition."""

    name: str
    world: str
    model_name: str = "ego_vehicle"
    actions: list[ScenarioAction] = field(default_factory=list)
    pass_conditions: list[dict[str, Any]] = field(default_factory=list)
    timeout: float = 60.0
