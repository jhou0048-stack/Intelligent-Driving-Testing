"""Closed-loop ADAS runtime used by Gazebo validation scenarios."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from control import GazeboAckermannController
from perception.camera import GazeboCamera
from perception.lane_detector import LaneDetector
from perception.lidar import GazeboLidar
from perception.object_detector import YOLODetector
from perception.perception_pipeline import PerceptionPipeline, PerceptionResult
from planning.adas_pipeline import ADASPipeline
from planning.aeb_controller import AEBController
from planning.lane_keeper import LaneKeeper
from simulation import GazeboSimulator
from testing.fault_injection import (
    FaultClock,
    FaultInjectedCamera,
    FaultInjectedLidar,
    FaultSpec,
)
from testing.scenario import ScenarioSUTConfig


class GazeboADASRuntime:
    """Own sensors and a single control-arbitrated ADAS pipeline."""

    def __init__(
        self,
        simulator: GazeboSimulator,
        controller: GazeboAckermannController,
        world_name: str,
        model_name: str,
        config: ScenarioSUTConfig,
    ) -> None:
        self._simulator = simulator
        self._controller = controller
        self._world_name = world_name
        self._model_name = model_name
        self._config = config
        self._camera: Any = None
        self._lidar: Any = None
        self._pipeline: ADASPipeline | None = None
        self._fault_clock = FaultClock()
        self._fault_sources: list[Any] = []
        self._faults = [FaultSpec.from_dict(data) for data in config.faults]
        self._lock = threading.Lock()
        self._reset_metrics()

    def _reset_metrics(self) -> None:
        with self._lock:
            self._perception_steps = 0
            self._lane_detection_steps = 0
            self._object_detection_count = 0
            self._aeb_active_steps = 0
            self._min_forward_distance = float("inf")
            self._lane_confidence_total = 0.0

    def connect(self) -> None:
        """Connect only the sensors required by the selected SUT mode."""
        enable_lka = self._config.mode in {"lka", "adas"}
        enable_aeb = self._config.mode in {"aeb", "adas"}
        needs_camera = enable_lka or self._config.enable_yolo

        if needs_camera:
            camera: Any = GazeboCamera(
                model_name=self._model_name,
                world_name=self._world_name,
                simulator=self._simulator,
                own_simulator=False,
            )
            camera_faults = [f for f in self._faults if f.target == "camera"]
            if camera_faults:
                camera = FaultInjectedCamera(
                    camera,
                    camera_faults,
                    self._fault_clock,
                )
                self._fault_sources.append(camera)
            self._camera = camera
            self._camera.connect()

        if enable_aeb:
            lidar: Any = GazeboLidar(
                model_name=self._model_name,
                world_name=self._world_name,
                simulator=self._simulator,
                own_simulator=False,
            )
            lidar_faults = [f for f in self._faults if f.target == "lidar"]
            if lidar_faults:
                lidar = FaultInjectedLidar(
                    lidar,
                    lidar_faults,
                    self._fault_clock,
                )
                self._fault_sources.append(lidar)
            self._lidar = lidar
            self._lidar.connect()

        object_detector: YOLODetector | None = None
        if self._config.enable_yolo:
            object_detector = YOLODetector(
                model_path=self._config.yolo_model,
                confidence_threshold=self._config.yolo_confidence,
            )

        perception = PerceptionPipeline(
            camera=self._camera,
            lidar=self._lidar,
            lane_detector=LaneDetector() if enable_lka else None,
            object_detector=object_detector,
        )
        self._pipeline = ADASPipeline(
            controller=self._controller,
            perception=perception,
            lane_keeper=LaneKeeper(
                kp=0.5,
                default_speed=self._config.speed,
            ),
            aeb_controller=AEBController(),
            default_speed=self._config.speed,
            loop_hz=self._config.loop_hz,
            enable_lka=enable_lka,
            enable_aeb=enable_aeb,
        )
        self._pipeline.add_step_callback(self._on_step)

    def wait_for_sensors(self, timeout: float) -> bool:
        """Wait for every sensor required by this runtime."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            camera_ready = (
                self._camera is None or self._camera.get_image() is not None
            )
            lidar_ready = (
                self._lidar is None or self._lidar.get_ranges() is not None
            )
            if camera_ready and lidar_ready:
                return True
            time.sleep(0.05)
        return False

    def start(self) -> None:
        if self._pipeline is None:
            raise RuntimeError("ADAS runtime is not connected")
        self._reset_metrics()
        self._fault_clock.start()
        self._pipeline.start()

    def stop(self) -> None:
        if self._pipeline is not None and self._pipeline.is_running:
            self._pipeline.stop()

    def disconnect(self) -> None:
        self.stop()
        if self._lidar is not None:
            self._lidar.disconnect()
            self._lidar = None
        if self._camera is not None:
            self._camera.disconnect()
            self._camera = None
        self._pipeline = None

    def _on_step(self, result: PerceptionResult) -> None:
        pipeline = self._pipeline
        with self._lock:
            self._perception_steps += 1
            if result.left_line_detected or result.right_line_detected:
                self._lane_detection_steps += 1
            self._lane_confidence_total += result.lane_confidence
            self._object_detection_count += len(result.objects)
            if pipeline is not None and pipeline.is_aeb_active:
                self._aeb_active_steps += 1
            if math.isfinite(result.min_forward_distance):
                self._min_forward_distance = min(
                    self._min_forward_distance,
                    result.min_forward_distance,
                )

    def metrics(self) -> dict[str, Any]:
        with self._lock:
            result: dict[str, Any] = {
                "perception_steps": self._perception_steps,
                "lane_detection_steps": self._lane_detection_steps,
                "lane_detection_rate": (
                    self._lane_detection_steps / self._perception_steps
                    if self._perception_steps
                    else 0.0
                ),
                "mean_lane_confidence": (
                    self._lane_confidence_total / self._perception_steps
                    if self._perception_steps
                    else 0.0
                ),
                "object_detection_count": self._object_detection_count,
                "aeb_active_steps": self._aeb_active_steps,
                "aeb_triggered": self._aeb_active_steps > 0,
                "faults_configured": len(self._faults),
                "fault_injection_count": sum(
                    source.activation_count for source in self._fault_sources
                ),
            }
            if math.isfinite(self._min_forward_distance):
                result["min_forward_distance"] = self._min_forward_distance
            return result
