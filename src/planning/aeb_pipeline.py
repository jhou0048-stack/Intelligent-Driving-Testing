"""AEB closed-loop pipeline.

Ties together: Lidar/Camera → AEBController → VehicleController.
Runs alongside the LKA pipeline, overriding with emergency stop when needed.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from control.vehicle_controller import VehicleController
from planning.aeb_controller import AEBController, AEBDecision

logger = logging.getLogger(__name__)


class AEBPipeline:
    """Closed-loop automatic emergency braking.

    Monitors sensor data in a background thread and commands emergency stop
    when an obstacle is within braking distance.
    """

    def __init__(
        self,
        controller: VehicleController,
        aeb_controller: AEBController | None = None,
        lidar: Any = None,
        camera: Any = None,
        object_detector: Any = None,
        current_speed_fn: Any = None,
        loop_hz: float = 20.0,
    ) -> None:
        self._controller = controller
        self._aeb = aeb_controller or AEBController()
        self._lidar = lidar
        self._camera = camera
        self._detector = object_detector
        self._speed_fn = current_speed_fn or (lambda: 0.5)
        self._loop_period = 1.0 / loop_hz
        self._running = False
        self._braking = False
        self._thread: threading.Thread | None = None
        self._last_decision: AEBDecision | None = None
        self._step_count = 0
        self._brake_count = 0

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_braking(self) -> bool:
        return self._braking

    @property
    def step_count(self) -> int:
        return self._step_count

    @property
    def brake_count(self) -> int:
        return self._brake_count

    @property
    def last_decision(self) -> AEBDecision | None:
        return self._last_decision

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._step_count = 0
        self._brake_count = 0
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("AEB pipeline started")

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        logger.info("AEB pipeline stopped after %d steps (%d brakes)",
                     self._step_count, self._brake_count)

    def _loop(self) -> None:
        while self._running:
            start = time.monotonic()
            self._step()
            elapsed = time.monotonic() - start
            sleep_time = self._loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _step(self) -> None:
        speed = self._speed_fn() if callable(self._speed_fn) else 0.5
        decision: AEBDecision | None = None

        if self._lidar is not None:
            ranges = self._lidar.get_ranges()
            if ranges is not None:
                decision = self._aeb.evaluate_ranges(
                    ranges=ranges,
                    angle_min=getattr(self._lidar, '_angle_min', 0.0),
                    angle_step=getattr(self._lidar, '_angle_step', 0.01),
                    current_speed=speed,
                )

        if decision is None and self._camera is not None and self._detector is not None:
            image = self._camera.get_image()
            if image is not None:
                detections = self._detector.detect(image)
                decision = self._aeb.evaluate_detections(
                    detections=detections,
                    image_width=image.shape[1],
                    current_speed=speed,
                )

        if decision is None:
            return

        self._last_decision = decision
        self._step_count += 1

        if decision.should_brake:
            if not self._braking:
                logger.warning("AEB triggered! dist=%.2f braking_dist=%.2f",
                               decision.min_obstacle_distance,
                               decision.braking_distance)
            self._braking = True
            self._brake_count += 1
            self._controller.stop()
        else:
            self._braking = False

    def __enter__(self) -> AEBPipeline:
        self.start()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.stop()
