"""Scenario runner interface and Gazebo implementation."""

from __future__ import annotations

import logging
import subprocess
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any

from control import GazeboAckermannController
from simulation import GazeboSimulator, GazeboVehicleTelemetry
from testing.adas_runtime import GazeboADASRuntime
from testing.config_loader import build_assertions
from testing.report import AssertionResult, ScenarioResult
from testing.scenario import ScenarioConfig

logger = logging.getLogger(__name__)

_READY_POLL_INTERVAL = 0.5
_READY_POLL_TIMEOUT = 30.0


class ScenarioRunner(ABC):
    """Abstract interface for running test scenarios."""

    @abstractmethod
    def load_scenario(self, config: ScenarioConfig) -> None:
        """Prepare the simulation environment for a scenario."""

    @abstractmethod
    def execute(self) -> ScenarioResult:
        """Execute the loaded scenario and return results."""

    @abstractmethod
    def teardown(self) -> None:
        """Clean up all simulation resources."""

    def __enter__(self) -> ScenarioRunner:
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.teardown()


class GazeboScenarioRunner(ScenarioRunner):
    """Run scenarios in a Gazebo Harmonic simulation."""

    def __init__(
        self,
        scenarios_dir: str = "scenarios",
        headless: bool = True,
    ) -> None:
        self._scenarios_dir = Path(scenarios_dir)
        self._headless = headless
        self._server_process: subprocess.Popen[bytes] | None = None
        self._gui_process: subprocess.Popen[bytes] | None = None
        self._simulator: GazeboSimulator | None = None
        self._controller: GazeboAckermannController | None = None
        self._telemetry: GazeboVehicleTelemetry | None = None
        self._adas_runtime: GazeboADASRuntime | None = None
        self._config: ScenarioConfig | None = None

    def _start_gazebo_server(self, world_path: str) -> None:
        resource_path = str(self._scenarios_dir / "models")
        env_key = "GZ_SIM_RESOURCE_PATH"
        import os

        env = os.environ.copy()
        existing = env.get(env_key, "")
        env[env_key] = f"{resource_path}:{existing}" if existing else resource_path

        self._server_process = subprocess.Popen(
            ["gz", "sim", "-s", world_path],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            env=env,
        )
        logger.info("Started Gazebo server for %s", world_path)

    def _start_gazebo_gui(self) -> None:
        self._gui_process = subprocess.Popen(
            ["gz", "sim", "-g"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        logger.info("Started Gazebo GUI")

    def _wait_for_ready(self) -> bool:
        assert self._simulator is not None
        deadline = time.monotonic() + _READY_POLL_TIMEOUT
        while time.monotonic() < deadline:
            try:
                topics = self._simulator.get_topic_names()
                if topics:
                    return True
            except RuntimeError:
                pass
            time.sleep(_READY_POLL_INTERVAL)
        return False

    def _unpause_world(self, world_name: str) -> bool:
        assert self._simulator is not None
        import gz.msgs10.world_control_pb2 as wc_pb

        req = wc_pb.WorldControl()
        req.pause = False
        response = self._simulator.request(
            f"/world/{world_name}/control",
            req,
            "WorldControl",
            "Boolean",
            timeout_ms=10000,
        )
        return response is True or (
            hasattr(response, "data") and response.data
        )

    def load_scenario(self, config: ScenarioConfig) -> None:
        self._config = config
        world_path = str(self._scenarios_dir / "worlds" / f"{config.world}.sdf")
        if not Path(world_path).is_file():
            raise FileNotFoundError(f"Scenario world not found: {world_path}")
        self._start_gazebo_server(world_path)

        if not self._headless:
            self._start_gazebo_gui()

        self._simulator = GazeboSimulator()
        self._simulator.connect()

        if not self._wait_for_ready():
            raise RuntimeError("Gazebo server did not become ready in time")

        self._controller = GazeboAckermannController(
            model_name=config.model_name,
            simulator=self._simulator,
            own_simulator=False,
        )
        self._controller.connect()

        self._telemetry = GazeboVehicleTelemetry(
            simulator=self._simulator,
            world_name=config.world,
            model_name=config.model_name,
            lane_center_y=config.telemetry.lane_center_y,
            odometry_topic=config.telemetry.odometry_topic,
            contact_topic=config.telemetry.contact_topic,
        )
        self._telemetry.start()

        if not self._unpause_world(config.world):
            raise RuntimeError(f"Failed to unpause Gazebo world: {config.world}")
        if not self._telemetry.wait_for_odometry(
            config.telemetry.startup_timeout
        ):
            raise RuntimeError(
                "No Gazebo odometry received from "
                f"{self._telemetry.odometry_topic}"
            )

        if config.sut.mode != "open_loop":
            self._adas_runtime = GazeboADASRuntime(
                simulator=self._simulator,
                controller=self._controller,
                world_name=config.world,
                model_name=config.model_name,
                config=config.sut,
            )
            self._adas_runtime.connect()
            if not self._adas_runtime.wait_for_sensors(
                config.telemetry.startup_timeout
            ):
                raise RuntimeError(
                    f"Required sensors did not become ready for {config.sut.mode}"
                )
        logger.info("Scenario '%s' loaded and running", config.name)

    def execute(self) -> ScenarioResult:
        if self._config is None or self._controller is None:
            raise RuntimeError("No scenario loaded. Call load_scenario() first.")

        config = self._config
        start_time = time.monotonic()

        try:
            if config.sut.mode == "open_loop":
                for action in config.actions:
                    self._execute_action(action, start_time)
            else:
                assert self._adas_runtime is not None
                self._adas_runtime.start()
                self._wait_for_action(config.sut.duration, start_time)
                self._adas_runtime.stop()

            self._controller.stop()
            time.sleep(max(config.telemetry.sample_interval, 0.25))

            duration = time.monotonic() - start_time
            metrics = self._snapshot_metrics()
            assertion_results = self._evaluate_assertions(metrics)
            failures = [
                result.reason
                for result in assertion_results
                if not result.passed
            ]
            return ScenarioResult(
                scenario_name=config.name,
                passed=not failures,
                duration=duration,
                metrics=metrics,
                assertions=assertion_results,
                error="; ".join(failures) or None,
            )
        except TimeoutError as exc:
            self._stop_adas_runtime()
            duration = time.monotonic() - start_time
            return ScenarioResult(
                scenario_name=config.name,
                passed=False,
                duration=duration,
                metrics=self._snapshot_metrics(),
                error=str(exc),
            )
        except Exception as exc:
            self._stop_adas_runtime()
            duration = time.monotonic() - start_time
            return ScenarioResult(
                scenario_name=config.name,
                passed=False,
                duration=duration,
                metrics=self._snapshot_metrics(),
                error=str(exc),
            )

    def _execute_action(self, action: Any, start_time: float) -> None:
        assert self._controller is not None
        if action.action_type == "drive":
            speed = action.params.get("speed", 0.0)
            steering = action.params.get("steering_angle", 0.0)
            self._controller.send_command(speed=speed, steering_angle=steering)
            self._wait_for_action(action.duration, start_time)
        elif action.action_type == "stop":
            self._controller.stop()
            self._wait_for_action(action.duration, start_time)
        elif action.action_type == "wait":
            self._wait_for_action(action.duration, start_time)
        else:
            raise ValueError(f"Unknown action type: {action.action_type}")

    def _wait_for_action(self, duration: float, start_time: float) -> None:
        assert self._config is not None
        deadline = time.monotonic() + max(duration, 0.0)
        while time.monotonic() < deadline:
            elapsed = time.monotonic() - start_time
            if elapsed > self._config.timeout:
                raise TimeoutError(
                    f"Scenario timed out after {elapsed:.2f}s"
                )
            remaining = deadline - time.monotonic()
            time.sleep(
                min(self._config.telemetry.sample_interval, remaining)
            )

    def _snapshot_metrics(self) -> dict[str, Any]:
        metrics = self._telemetry.snapshot() if self._telemetry else {}
        if self._adas_runtime is not None:
            metrics.update(self._adas_runtime.metrics())
        return metrics

    def _evaluate_assertions(
        self, metrics: dict[str, Any]
    ) -> list[AssertionResult]:
        assert self._config is not None
        results: list[AssertionResult] = []
        for assertion in build_assertions(self._config.pass_conditions):
            passed, reason = assertion.evaluate(metrics)
            results.append(
                AssertionResult(
                    name=type(assertion).__name__,
                    passed=passed,
                    reason=reason,
                )
            )
        return results

    def teardown(self) -> None:
        self._stop_adas_runtime()
        if self._adas_runtime is not None:
            self._adas_runtime.disconnect()
            self._adas_runtime = None
        if self._controller is not None:
            try:
                self._controller.stop()
            except RuntimeError:
                logger.debug("Controller was already disconnected")
            self._controller.disconnect()
            self._controller = None
        self._telemetry = None
        if self._simulator is not None:
            self._simulator.disconnect()
            self._simulator = None
        if self._server_process is not None:
            self._server_process.terminate()
            try:
                self._server_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self._server_process.kill()
            self._server_process = None
        if self._gui_process is not None:
            self._gui_process.terminate()
            try:
                self._gui_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._gui_process.kill()
            self._gui_process = None
        logger.info("Scenario runner torn down")

    def _stop_adas_runtime(self) -> None:
        if self._adas_runtime is not None:
            self._adas_runtime.stop()
