"""Integrated ADAS pipeline combining LKA + AEB + perception fusion."""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from control.vehicle_controller import VehicleController
from perception.perception_pipeline import PerceptionPipeline
from planning.aeb_controller import AEBController
from planning.lane_keeper import LaneKeeper

logger = logging.getLogger(__name__)


class ADASPipeline:
    """Unified pipeline: perception fusion → behavior decision → control.

    Runs a single loop that:
    1. Fuses camera + lidar via PerceptionPipeline
    2. Checks AEB (emergency braking overrides everything)
    3. Runs LKA (lane keeping) if no emergency
    """

    def __init__(
        self,
        controller: VehicleController,
        perception: PerceptionPipeline,
        lane_keeper: LaneKeeper | None = None,
        aeb_controller: AEBController | None = None,
        default_speed: float = 0.5,
        loop_hz: float = 10.0,
    ) -> None:
        self._controller = controller
        self._perception = perception
        self._lane_keeper = lane_keeper or LaneKeeper()
        self._aeb = aeb_controller or AEBController()
        self._default_speed = default_speed
        self._loop_period = 1.0 / loop_hz
        self._running = False
        self._thread: threading.Thread | None = None
        self._pose: dict[str, float] = {"x": 0.0, "y": 0.0, "heading": 0.0}
        self._step_count = 0
        self._aeb_active = False
        self._callbacks: list[Any] = []

    def add_step_callback(self, fn: Any) -> None:
        self._callbacks.append(fn)

    def set_pose(self, x: float, y: float, heading: float) -> None:
        self._pose = {"x": x, "y": y, "heading": heading}

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_aeb_active(self) -> bool:
        return self._aeb_active

    @property
    def step_count(self) -> int:
        return self._step_count

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._step_count = 0
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        self._controller.stop()

    def _loop(self) -> None:
        while self._running:
            start = time.monotonic()
            self._step()
            elapsed = time.monotonic() - start
            sleep_time = self._loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _step(self) -> None:
        result = self._perception.process()
        if result is None:
            return

        self._step_count += 1

        # AEB check
        if result.min_forward_distance < self._aeb.compute_braking_distance(
            self._default_speed
        ):
            self._aeb_active = True
            self._controller.stop()
        else:
            self._aeb_active = False
            perception_data = {"lateral_offset": result.lateral_offset}
            waypoints = self._lane_keeper.plan(self._pose, perception_data)
            if waypoints:
                speed, steering = self._lane_keeper.compute_steering(
                    self._pose, waypoints[0]
                )
                self._controller.send_command(
                    speed=speed, steering_angle=steering
                )

        for cb in self._callbacks:
            cb(result)

    def __enter__(self) -> ADASPipeline:
        self.start()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.stop()
