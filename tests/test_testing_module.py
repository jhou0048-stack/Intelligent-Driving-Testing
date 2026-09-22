"""Tests for the testing/orchestration module."""

from unittest.mock import MagicMock, patch

import pytest

from testing import (
    GazeboScenarioRunner,
    MaxLateralDeviationAssertion,
    NoCollisionAssertion,
    PositionWithinAssertion,
    ScenarioAction,
    ScenarioAssertion,
    ScenarioConfig,
    ScenarioResult,
    ScenarioRunner,
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
        patch.object(runner, "_unpause_world", return_value=True),
    ):
        mock_proc = MagicMock()
        mock_popen.return_value = mock_proc
        mock_sim = MagicMock()
        MockSim.return_value = mock_sim
        mock_ctrl = MagicMock()
        MockCtrl.return_value = mock_ctrl

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


def test_gazebo_runner_context_manager() -> None:
    runner = GazeboScenarioRunner()
    with patch.object(runner, "teardown") as mock_teardown:
        with runner:
            pass
        mock_teardown.assert_called_once()
