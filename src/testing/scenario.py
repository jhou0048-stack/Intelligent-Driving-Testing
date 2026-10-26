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
class ScenarioTelemetryConfig:
    """Telemetry settings used while executing a scenario."""

    lane_center_y: float = 0.0
    odometry_topic: str | None = None
    contact_topic: str | None = None
    sample_interval: float = 0.05
    startup_timeout: float = 5.0


@dataclass
class ScenarioSUTConfig:
    """System-under-test settings for closed-loop ADAS scenarios."""

    mode: str = "open_loop"
    duration: float = 0.0
    speed: float = 0.5
    loop_hz: float = 10.0
    enable_yolo: bool = False
    yolo_model: str = "yolov8n.pt"
    yolo_confidence: float = 0.5
    faults: list[dict[str, Any]] = field(default_factory=list)


@dataclass
class ScenarioConfig:
    """Complete scenario definition."""

    name: str
    world: str
    model_name: str = "ego_vehicle"
    actions: list[ScenarioAction] = field(default_factory=list)
    pass_conditions: list[dict[str, Any]] = field(default_factory=list)
    telemetry: ScenarioTelemetryConfig = field(
        default_factory=ScenarioTelemetryConfig
    )
    sut: ScenarioSUTConfig = field(default_factory=ScenarioSUTConfig)
    timeout: float = 60.0
