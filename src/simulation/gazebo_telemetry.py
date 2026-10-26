"""Gazebo telemetry collection for scenario-level validation."""

from __future__ import annotations

import math
import threading
import time
from typing import Any

from simulation.gazebo_client import GazeboSimulator


class GazeboVehicleTelemetry:
    """Collect odometry and chassis-contact metrics from Gazebo topics."""

    def __init__(
        self,
        simulator: GazeboSimulator,
        world_name: str,
        model_name: str,
        lane_center_y: float = 0.0,
        odometry_topic: str | None = None,
        contact_topic: str | None = None,
    ) -> None:
        self._simulator = simulator
        self._world_name = world_name
        self._model_name = model_name
        self._lane_center_y = lane_center_y
        self._odometry_topic = (
            odometry_topic or f"/model/{model_name}/odometry"
        )
        self._contact_topic = (
            contact_topic
            or (
                f"/world/{world_name}/model/{model_name}/link/chassis/"
                "sensor/chassis_contact/contact"
            )
        )
        self._lock = threading.Lock()
        self._odometry_ready = threading.Event()
        self.reset()

    @property
    def odometry_topic(self) -> str:
        return self._odometry_topic

    @property
    def contact_topic(self) -> str:
        return self._contact_topic

    def reset(self) -> None:
        """Clear all measurements while preserving the configured topics."""
        with self._lock:
            self._started_at = time.monotonic()
            self._odometry_samples = 0
            self._contact_samples = 0
            self._final_x: float | None = None
            self._final_y: float | None = None
            self._final_speed: float | None = None
            self._previous_x: float | None = None
            self._previous_y: float | None = None
            self._distance_traveled = 0.0
            self._max_speed = 0.0
            self._speed_total = 0.0
            self._max_lateral_deviation = 0.0
            self._collision_count = 0
            self._last_contact_at: float | None = None
        self._odometry_ready.clear()

    def start(self) -> None:
        """Subscribe to the configured Gazebo telemetry topics."""
        self.reset()
        self._simulator.subscribe(
            self._odometry_topic,
            "Odometry",
            self._on_odometry,
        )
        self._simulator.subscribe(
            self._contact_topic,
            "Contacts",
            self._on_contacts,
        )

    def wait_for_odometry(self, timeout: float) -> bool:
        """Wait until at least one odometry sample has arrived."""
        return self._odometry_ready.wait(timeout=timeout)

    def _on_odometry(self, message: Any) -> None:
        position = message.pose.position
        linear = message.twist.linear
        x = float(position.x)
        y = float(position.y)
        speed = math.sqrt(
            float(linear.x) ** 2
            + float(linear.y) ** 2
            + float(linear.z) ** 2
        )
        deviation = abs(y - self._lane_center_y)

        with self._lock:
            if self._previous_x is not None and self._previous_y is not None:
                self._distance_traveled += math.hypot(
                    x - self._previous_x,
                    y - self._previous_y,
                )
            self._previous_x = x
            self._previous_y = y
            self._odometry_samples += 1
            self._final_x = x
            self._final_y = y
            self._final_speed = speed
            self._max_speed = max(self._max_speed, speed)
            self._speed_total += speed
            self._max_lateral_deviation = max(
                self._max_lateral_deviation,
                deviation,
            )
        self._odometry_ready.set()

    def _on_contacts(self, message: Any) -> None:
        now = time.monotonic()
        contacts = list(getattr(message, "contact", []))
        with self._lock:
            self._contact_samples += 1
            if not contacts:
                return

            # Continuous contact produces many messages. Count it once; a
            # later event is counted after the topic has been quiet.
            if (
                self._last_contact_at is None
                or now - self._last_contact_at > 0.5
            ):
                self._collision_count += 1
            self._last_contact_at = now

    def _topic_is_advertised(self, topic: str) -> bool:
        try:
            return self._simulator.has_topic_publisher(topic)
        except RuntimeError:
            return False

    def snapshot(self) -> dict[str, Any]:
        """Return metrics suitable for scenario assertions and reporting."""
        with self._lock:
            metrics: dict[str, Any] = {
                "telemetry_duration": time.monotonic() - self._started_at,
                "odometry_samples": self._odometry_samples,
                "contact_samples": self._contact_samples,
            }
            if self._odometry_samples:
                metrics.update(
                    {
                        "final_x": self._final_x,
                        "final_y": self._final_y,
                        "final_speed": self._final_speed,
                        "max_speed": self._max_speed,
                        "mean_speed": (
                            self._speed_total / self._odometry_samples
                        ),
                        "distance_traveled": self._distance_traveled,
                        "max_lateral_deviation": (
                            self._max_lateral_deviation
                        ),
                    }
                )

            contact_available = (
                self._contact_samples > 0
                or self._topic_is_advertised(self._contact_topic)
            )
            metrics["contact_topic_available"] = contact_available
            if contact_available:
                metrics["collision_count"] = self._collision_count
            return metrics
