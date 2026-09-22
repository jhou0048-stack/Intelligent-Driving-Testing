"""Lane Keeping Assist closed-loop pipeline.

Ties together: GazeboCamera → LaneDetector → LaneKeeper → Controller.
Runs at the camera frame rate in a background thread.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any

from control import GazeboAckermannController
from perception.camera import GazeboCamera
from perception.lane_detector import LaneDetector
from planning.lane_keeper import LaneKeeper
from simulation import GazeboSimulator

logger = logging.getLogger(__name__)


class LKAPipeline:
    """Closed-loop lane keeping assist.

    Call :meth:`start` to begin the perception → planning → control loop
    in a background thread. Call :meth:`stop` to end it.
    """

    def __init__(
        self,
        simulator: GazeboSimulator,
        controller: GazeboAckermannController,
        camera: GazeboCamera,
        lane_detector: LaneDetector | None = None,
        lane_keeper: LaneKeeper | None = None,
        loop_hz: float = 10.0,
    ) -> None:
        self._simulator = simulator
        self._controller = controller
        self._camera = camera
        self._lane_detector = lane_detector or LaneDetector()
        self._lane_keeper = lane_keeper or LaneKeeper()
        self._loop_period = 1.0 / loop_hz
        self._running = False
        self._thread: threading.Thread | None = None
        self._pose: dict[str, float] = {"x": 0.0, "y": 0.0, "heading": 0.0}
        self._last_detection: Any = None
        self._step_count = 0

    def set_pose(self, x: float, y: float, heading: float) -> None:
        self._pose = {"x": x, "y": y, "heading": heading}

    def update_pose_from_odometry(self, msg: Any) -> None:
        """Callback for odometry subscription."""
        try:
            pos = msg.pose.position
            orient = msg.pose.orientation
            yaw = math.atan2(
                2.0 * (orient.w * orient.z + orient.x * orient.y),
                1.0 - 2.0 * (orient.y * orient.y + orient.z * orient.z),
            )
            self._pose = {"x": pos.x, "y": pos.y, "heading": yaw}
        except Exception:
            logger.debug("Could not parse odometry message")

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._step_count = 0
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("LKA pipeline started")

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=5.0)
            self._thread = None
        self._controller.stop()
        logger.info("LKA pipeline stopped after %d steps", self._step_count)

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def step_count(self) -> int:
        return self._step_count

    @property
    def last_detection(self) -> Any:
        return self._last_detection

    def _loop(self) -> None:
        while self._running:
            start = time.monotonic()
            self._step()
            elapsed = time.monotonic() - start
            sleep_time = self._loop_period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _step(self) -> None:
        image = self._camera.get_image()
        if image is None:
            return

        detection = self._lane_detector.detect(image)
        self._last_detection = detection

        perception_data = {"lateral_offset": detection.lateral_offset}
        waypoints = self._lane_keeper.plan(self._pose, perception_data)

        if waypoints:
            speed, steering = self._lane_keeper.compute_steering(
                self._pose, waypoints[0]
            )
            self._controller.send_command(
                speed=speed, steering_angle=steering
            )

        self._step_count += 1

    def __enter__(self) -> LKAPipeline:
        self.start()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.stop()
