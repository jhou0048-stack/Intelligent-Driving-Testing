"""Gazebo Ackermann-steering vehicle controller."""

from __future__ import annotations

import logging
from collections.abc import Callable
from typing import Any

import gz.msgs10.twist_pb2 as twist_pb

from control.vehicle_controller import VehicleController
from simulation import GazeboSimulator

logger = logging.getLogger(__name__)


def _cmd_vel_topic(model_name: str) -> str:
    return f"/model/{model_name}/cmd_vel"


def _odometry_topic(model_name: str) -> str:
    return f"/model/{model_name}/odometry"


class GazeboAckermannController(VehicleController):
    """Control an Ackermann-steered vehicle in Gazebo.

    Commands are published as ``gz.msgs.Twist`` on
    ``/model/{model_name}/cmd_vel``:

    * ``linear.x`` = target longitudinal speed in m/s.
    * ``angular.z`` = target steering angle in radians.

    This matches the input convention used by Gazebo's
    ``gz-sim-ackermann-steering-system`` plugin.
    """

    def __init__(
        self,
        model_name: str = "ego_vehicle",
        simulator: GazeboSimulator | None = None,
        own_simulator: bool = False,
    ) -> None:
        self._model_name = model_name
        self._simulator = simulator if simulator is not None else GazeboSimulator()
        self._own_simulator = own_simulator or simulator is None
        self._cmd_topic = _cmd_vel_topic(model_name)
        self._odom_topic = _odometry_topic(model_name)

    def connect(self) -> None:
        """Connect the underlying simulator if we own it."""
        if self._own_simulator:
            self._simulator.connect()
        logger.info(
            "Ackermann controller ready for model '%s' on %s",
            self._model_name,
            self._cmd_topic,
        )

    def disconnect(self) -> None:
        """Disconnect the underlying simulator if we own it."""
        if self._own_simulator:
            self._simulator.disconnect()
        logger.info("Ackermann controller disconnected from '%s'", self._model_name)

    def is_connected(self) -> bool:
        """Return ``True`` when the underlying simulator is connected."""
        return self._simulator.is_connected()

    def send_command(self, speed: float, steering_angle: float) -> None:
        """Publish a speed and steering command to the vehicle."""
        msg = twist_pb.Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(steering_angle)
        self._simulator.publish(self._cmd_topic, "Twist", msg)
        logger.debug(
            "Command: speed=%.3f m/s, steering=%.3f rad",
            speed,
            steering_angle,
        )

    def subscribe_odometry(
        self,
        callback: Callable[[Any], None],
    ) -> None:
        """Subscribe to the model's odometry topic.

        Args:
            callback: Callable receiving ``gz.msgs.Odometry`` messages.
        """
        self._simulator.subscribe(self._odom_topic, "Odometry", callback)
        logger.debug("Subscribed to odometry on %s", self._odom_topic)

    @property
    def model_name(self) -> str:
        """Return the controlled model name."""
        return self._model_name

    @property
    def cmd_topic(self) -> str:
        """Return the command topic."""
        return self._cmd_topic

    @property
    def odometry_topic(self) -> str:
        """Return the odometry topic."""
        return self._odom_topic
