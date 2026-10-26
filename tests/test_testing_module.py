"""Tests for the testing/orchestration module."""

from unittest.mock import MagicMock, patch

import pytest

from testing import (
    AEBTriggeredAssertion,
    FinalSpeedBelowAssertion,
    GazeboScenarioRunner,
    MaxLateralDeviationAssertion,
    MinimumDistanceTraveledAssertion,
    MinimumPerceptionStepsAssertion,
    NoCollisionAssertion,
    PositionWithinAssertion,
    ScenarioAction,
    ScenarioAssertion,
    ScenarioConfig,
    ScenarioResult,
    ScenarioRunner,
    ScenarioSUTConfig,
    ScenarioTelemetryConfig,
    TestReport,
)


# --- ABC tests ---------------------------------------------------------------


def test_scenario_runner_is_abstract() -> None:
    with pytest.raises(TypeError):
        ScenarioRunner()  # type: ignore[abstract]


def test_scenario_assertion_is_abstract() -> None:
    with pytest.raises(TypeError):
        ScenarioAssertion()  # type: ignore[abstract]


# --- Dataclass tests ----------------------------------------------------------


def test_scenario_action_defaults() -> None:
    action = ScenarioAction(action_type="drive")
    assert action.action_type == "drive"
    assert action.params == {}
    assert action.duration == 0.0


def test_scenario_config_defaults() -> None:
    config = ScenarioConfig(name="test", world="simple_road")
    assert config.name == "test"
    assert config.world == "simple_road"
    assert config.model_name == "ego_vehicle"
    assert config.actions == []
    assert config.pass_conditions == []
    assert config.telemetry == ScenarioTelemetryConfig()
    assert config.sut == ScenarioSUTConfig()
    assert config.timeout == 60.0


def test_scenario_result_fields() -> None:
    result = ScenarioResult(
        scenario_name="test",
        passed=True,
        duration=5.0,
        metrics={"final_x": 10.0},
    )
    assert result.scenario_name == "test"
    assert result.passed is True
    assert result.duration == pytest.approx(5.0)
    assert result.error is None


def test_scenario_result_with_error() -> None:
    result = ScenarioResult(
        scenario_name="test", passed=False, duration=1.0, error="timeout"
    )
    assert result.passed is False
    assert result.error == "timeout"


# --- Assertion tests ----------------------------------------------------------


def test_position_within_pass() -> None:
    a = PositionWithinAssertion(x_min=0, x_max=10, y_min=-5, y_max=5)
    ok, reason = a.evaluate({"final_x": 5.0, "final_y": 2.0})
    assert ok is True
    assert "within" in reason.lower()


def test_position_within_fail() -> None:
    a = PositionWithinAssertion(x_min=0, x_max=10, y_min=-5, y_max=5)
    ok, reason = a.evaluate({"final_x": 15.0, "final_y": 0.0})
    assert ok is False
    assert "outside" in reason.lower()


def test_max_lateral_deviation_pass() -> None:
    a = MaxLateralDeviationAssertion(max_deviation=1.0)
    ok, reason = a.evaluate({"max_lateral_deviation": 0.5})
    assert ok is True


def test_max_lateral_deviation_fail() -> None:
    a = MaxLateralDeviationAssertion(max_deviation=1.0)
    ok, reason = a.evaluate({"max_lateral_deviation": 1.5})
    assert ok is False
    assert "exceeds" in reason.lower()


def test_no_collision_pass() -> None:
    a = NoCollisionAssertion()
    ok, _ = a.evaluate({"collision_count": 0})
    assert ok is True


def test_no_collision_fail() -> None:
    a = NoCollisionAssertion()
    ok, reason = a.evaluate({"collision_count": 3})
    assert ok is False
    assert "3" in reason


@pytest.mark.parametrize(
    ("assertion", "expected"),
    [
        (PositionWithinAssertion(), "position"),
        (MaxLateralDeviationAssertion(), "lateral"),
        (NoCollisionAssertion(), "collision"),
    ],
)
def test_assertions_fail_when_metric_is_missing(
    assertion: ScenarioAssertion, expected: str
) -> None:
    ok, reason = assertion.evaluate({})
    assert ok is False
    assert expected in reason.lower()


@pytest.mark.parametrize(
    ("assertion", "metrics"),
    [
        (MinimumDistanceTraveledAssertion(2.0), {"distance_traveled": 2.1}),
        (FinalSpeedBelowAssertion(0.2), {"final_speed": 0.1}),
        (AEBTriggeredAssertion(), {"aeb_triggered": True}),
        (MinimumPerceptionStepsAssertion(10), {"perception_steps": 10}),
    ],
)
def test_runtime_assertions_pass(
    assertion: ScenarioAssertion, metrics: dict[str, object]
) -> None:
    ok, _reason = assertion.evaluate(metrics)
    assert ok is True


# --- TestReport tests ---------------------------------------------------------


def test_report_empty_summary() -> None:
    report = TestReport()
    assert report.summary() == "0 scenarios: 0 passed, 0 failed"


def test_report_add_results() -> None:
    report = TestReport()
    report.add_result(ScenarioResult("a", True, 1.0))
    report.add_result(ScenarioResult("b", False, 2.0, error="oops"))
    assert len(report.results) == 2
    assert report.summary() == "2 scenarios: 1 passed, 1 failed"


def test_report_to_dataframe() -> None:
    report = TestReport()
    report.add_result(
        ScenarioResult("test1", True, 3.5, metrics={"final_x": 10.0})
    )
    df = report.to_dataframe()
    assert len(df) == 1
    assert "scenario" in df.columns
    assert "passed" in df.columns
    assert "final_x" in df.columns
    assert df.iloc[0]["scenario"] == "test1"


def test_report_to_csv(tmp_path: object) -> None:
    import pathlib

    assert isinstance(tmp_path, pathlib.Path)
    csv_path = str(tmp_path / "report.csv")
    report = TestReport()
    report.add_result(ScenarioResult("s1", True, 1.0))
    report.to_csv(csv_path)
    content = (tmp_path / "report.csv").read_text()
    assert "s1" in content
    assert "True" in content


def test_report_to_json(tmp_path: object) -> None:
    import json
    import pathlib

    assert isinstance(tmp_path, pathlib.Path)
    json_path = tmp_path / "nested" / "report.json"
    report = TestReport()
    report.add_result(ScenarioResult("s1", True, 1.0))
    report.to_json(str(json_path))
    payload = json.loads(json_path.read_text())
    assert payload["summary"]["passed"] == 1
    assert payload["results"][0]["scenario_name"] == "s1"


# --- GazeboScenarioRunner tests (mocked subprocess) --------------------------


def test_gazebo_runner_start_server_subprocess() -> None:
    runner = GazeboScenarioRunner(scenarios_dir="scenarios", headless=True)
    with patch("testing.runner.subprocess.Popen") as mock_popen:
        mock_popen.return_value = MagicMock()
        runner._start_gazebo_server("scenarios/worlds/simple_road.sdf")
    mock_popen.assert_called_once()
    cmd = mock_popen.call_args[0][0]
    assert cmd[0] == "gz"
    assert cmd[1] == "sim"
    assert "-s" in cmd


def test_gazebo_runner_start_gui_subprocess() -> None:
    runner = GazeboScenarioRunner()
    with patch("testing.runner.subprocess.Popen") as mock_popen:
        mock_popen.return_value = MagicMock()
        runner._start_gazebo_gui()
    cmd = mock_popen.call_args[0][0]
    assert "-g" in cmd


def test_gazebo_runner_headless_skips_gui() -> None:
    runner = GazeboScenarioRunner(headless=True)
    config = ScenarioConfig(name="test", world="simple_road")

    with (
        patch("testing.runner.subprocess.Popen") as mock_popen,
        patch.object(runner, "_wait_for_ready", return_value=True),
        patch("testing.runner.GazeboSimulator") as MockSim,
        patch("testing.runner.GazeboAckermannController") as MockCtrl,
        patch("testing.runner.GazeboVehicleTelemetry") as MockTelemetry,
        patch.object(runner, "_unpause_world", return_value=True),
    ):
        mock_proc = MagicMock()
        mock_popen.return_value = mock_proc
        mock_sim = MagicMock()
        MockSim.return_value = mock_sim
        mock_ctrl = MagicMock()
        MockCtrl.return_value = mock_ctrl
        MockTelemetry.return_value.wait_for_odometry.return_value = True

        runner.load_scenario(config)

    assert mock_popen.call_count == 1  # only server, no GUI


def test_gazebo_runner_teardown_terminates_processes() -> None:
    runner = GazeboScenarioRunner()
    server = MagicMock()
    gui = MagicMock()
    runner._server_process = server
    runner._gui_process = gui
    runner._simulator = MagicMock()
    runner._controller = MagicMock()

    runner.teardown()

    server.terminate.assert_called_once()
    gui.terminate.assert_called_once()
    assert runner._server_process is None
    assert runner._gui_process is None
    assert runner._simulator is None
    assert runner._controller is None


def test_gazebo_runner_execute_requires_loaded_scenario() -> None:
    runner = GazeboScenarioRunner()
    with pytest.raises(RuntimeError, match="No scenario loaded"):
        runner.execute()


def test_gazebo_runner_evaluates_assertions() -> None:
    runner = GazeboScenarioRunner()
    runner._config = ScenarioConfig(
        name="test",
        world="simple_road",
        pass_conditions=[
            {"type": "max_lateral_deviation", "max_deviation": 0.5},
            {"type": "no_collision"},
        ],
    )
    runner._controller = MagicMock()
    runner._telemetry = MagicMock()
    runner._telemetry.snapshot.return_value = {
        "max_lateral_deviation": 0.2,
        "collision_count": 0,
    }

    result = runner.execute()

    assert result.passed is True
    assert len(result.assertions) == 2
    assert all(outcome.passed for outcome in result.assertions)


def test_gazebo_runner_fails_failed_assertion() -> None:
    runner = GazeboScenarioRunner()
    runner._config = ScenarioConfig(
        name="test",
        world="simple_road",
        pass_conditions=[{"type": "no_collision"}],
    )
    runner._controller = MagicMock()
    runner._telemetry = MagicMock()
    runner._telemetry.snapshot.return_value = {"collision_count": 1}

    result = runner.execute()

    assert result.passed is False
    assert result.error is not None
    assert "collision" in result.error.lower()


def test_gazebo_runner_context_manager() -> None:
    runner = GazeboScenarioRunner()
    with patch.object(runner, "teardown") as mock_teardown:
        with runner:
            pass
        mock_teardown.assert_called_once()
