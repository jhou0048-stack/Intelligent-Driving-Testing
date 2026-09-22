"""Scenario runner interface and Gazebo implementation."""

from __future__ import annotations

import logging
import subprocess
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any

from control import GazeboAckermannController
from simulation import GazeboSimulator
from testing.report import ScenarioResult
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
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
        )
        logger.info("Started Gazebo server for %s", world_path)

    def _start_gazebo_gui(self) -> None:
        self._gui_process = subprocess.Popen(
            ["gz", "sim", "-g"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
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

        self._unpause_world(config.world)
        logger.info("Scenario '%s' loaded and running", config.name)

    def execute(self) -> ScenarioResult:
        if self._config is None or self._controller is None:
            raise RuntimeError("No scenario loaded. Call load_scenario() first.")

        config = self._config
        metrics: dict[str, Any] = {}
        start_time = time.monotonic()

        try:
            for action in config.actions:
                elapsed = time.monotonic() - start_time
                if elapsed > config.timeout:
                    return ScenarioResult(
                        scenario_name=config.name,
                        passed=False,
                        duration=elapsed,
                        metrics=metrics,
                        error="Scenario timed out",
                    )
                self._execute_action(action, metrics)

            duration = time.monotonic() - start_time
            return ScenarioResult(
                scenario_name=config.name,
                passed=True,
                duration=duration,
                metrics=metrics,
            )
        except Exception as exc:
            duration = time.monotonic() - start_time
            return ScenarioResult(
                scenario_name=config.name,
                passed=False,
                duration=duration,
                metrics=metrics,
                error=str(exc),
            )

    def _execute_action(
        self, action: Any, metrics: dict[str, Any]
    ) -> None:
        assert self._controller is not None
        if action.action_type == "drive":
            speed = action.params.get("speed", 0.0)
            steering = action.params.get("steering_angle", 0.0)
            self._controller.send_command(speed=speed, steering_angle=steering)
            if action.duration > 0:
                time.sleep(action.duration)
        elif action.action_type == "stop":
            self._controller.stop()
            if action.duration > 0:
                time.sleep(action.duration)
        elif action.action_type == "wait":
            time.sleep(action.duration)
        else:
            logger.warning("Unknown action type: %s", action.action_type)

    def teardown(self) -> None:
        if self._controller is not None:
            self._controller.disconnect()
            self._controller = None
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
