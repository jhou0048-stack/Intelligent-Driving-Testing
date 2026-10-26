"""Reusable validation, metrics, and test-support package."""

from testing.aeb_validator import AEBMetrics, AEBValidationSuite, AEBValidator
from testing.analysis import ResultAnalyzer
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
from testing.lka_validator import LKAValidationSuite, LKAValidator
from testing.metrics import LKAMetrics, MetricCollector, MetricSample
from testing.perception_validator import (
    ObjectDetectionValidator,
    bounding_box_iou,
)
from testing.config_loader import build_assertions, build_scenario, load_config, load_scenario_suite
from testing.report import AssertionResult, ScenarioResult, TestReport
from testing.regression import RegressionComparator, RegressionOutcome
from testing.runner import GazeboScenarioRunner, ScenarioRunner
from testing.scenario import (
    ScenarioAction,
    ScenarioConfig,
    ScenarioSUTConfig,
    ScenarioTelemetryConfig,
)

__all__ = [
    "AEBMetrics",
    "AEBTriggeredAssertion",
    "AEBValidationSuite",
    "AEBValidator",
    "AssertionResult",
    "ResultAnalyzer",
    "build_assertions",
    "build_scenario",
    "load_config",
    "load_scenario_suite",
    "GazeboScenarioRunner",
    "FinalSpeedBelowAssertion",
    "LKAMetrics",
    "LKAValidationSuite",
    "LKAValidator",
    "MaxLateralDeviationAssertion",
    "MinimumDistanceTraveledAssertion",
    "MinimumFaultActivationsAssertion",
    "MinimumPerceptionStepsAssertion",
    "MetricCollector",
    "MetricSample",
    "NoCollisionAssertion",
    "ObjectDetectionValidator",
    "PositionWithinAssertion",
    "RegressionComparator",
    "RegressionOutcome",
    "ScenarioAction",
    "ScenarioAssertion",
    "ScenarioConfig",
    "ScenarioSUTConfig",
    "ScenarioTelemetryConfig",
    "ScenarioResult",
    "ScenarioRunner",
    "TestReport",
    "bounding_box_iou",
]
