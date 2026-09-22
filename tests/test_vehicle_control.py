"""Tests for the simulator-agnostic vehicle-control interface."""

from unittest.mock import MagicMock, patch

import pytest

from control import GazeboAckermannController, VehicleController
from simulation import GazeboSimulator


def test_vehicle_controller_is_abstract() -> None:
    """The base VehicleController class cannot be instantiated directly."""
    with pytest.raises(TypeError):
        VehicleController()  # type: ignore[abstract]


def test_gazebo_controller_default_topics() -> None:
    """Default command and odometry topics use the model name."""
    controller = GazeboAckermannController(model_name="ego_vehicle")
    assert controller.cmd_topic == "/model/ego_vehicle/cmd_vel"
    assert controller.odometry_topic == "/model/ego_vehicle/odometry"
    assert controller.model_name == "ego_vehicle"


def test_gazebo_controller_custom_model_topics() -> None:
    """Topics are generated from the supplied model name."""
    controller = GazeboAckermannController(model_name="test_car")
    assert controller.cmd_topic == "/model/test_car/cmd_vel"
    assert controller.odometry_topic == "/model/test_car/odometry"


def test_gazebo_controller_owns_simulator_by_default() -> None:
    """When no simulator is supplied, the controller creates and owns one."""
    controller = GazeboAckermannController(model_name="ego_vehicle")
    assert isinstance(controller._simulator, GazeboSimulator)
    assert controller._own_simulator is True


def test_gazebo_controller_can_use_external_simulator() -> None:
    """An external simulator can be injected without taking ownership."""
    sim = GazeboSimulator()
    controller = GazeboAckermannController(
        model_name="ego_vehicle", simulator=sim, own_simulator=False
    )
    assert controller._simulator is sim
    assert controller._own_simulator is False


def test_gazebo_controller_connect_owns_simulator() -> None:
    """connect() opens the injected simulator only when owned."""
    controller = GazeboAckermannController(model_name="ego_vehicle")
    with patch.object(controller._simulator, "connect") as mock_connect:
        controller.connect()
    mock_connect.assert_called_once()


def test_gazebo_controller_connect_does_not_touch_external_simulator() -> None:
    """connect() does not connect an externally-owned simulator."""
    sim = MagicMock()
    controller = GazeboAckermannController(
        model_name="ego_vehicle", simulator=sim, own_simulator=False
    )
    controller.connect()
    sim.connect.assert_not_called()


def test_gazebo_controller_disconnect_owns_simulator() -> None:
    """disconnect() closes the simulator only when owned."""
    controller = GazeboAckermannController(model_name="ego_vehicle")
    with patch.object(controller._simulator, "disconnect") as mock_disconnect:
        controller.disconnect()
    mock_disconnect.assert_called_once()


def test_gazebo_controller_disconnect_does_not_touch_external_simulator() -> None:
    """disconnect() does not disconnect an externally-owned simulator."""
    sim = MagicMock()
    controller = GazeboAckermannController(
        model_name="ego_vehicle", simulator=sim, own_simulator=False
    )
    controller.disconnect()
    sim.disconnect.assert_not_called()


def test_gazebo_controller_is_connected_delegates() -> None:
    """is_connected() reports the simulator's connection state."""
    sim = MagicMock()
    sim.is_connected.return_value = True
    controller = GazeboAckermannController(
        model_name="ego_vehicle", simulator=sim, own_simulator=False
    )
    assert controller.is_connected() is True
    sim.is_connected.assert_called_once()


def test_gazebo_controller_send_command_publishes_twist() -> None:
    """send_command() publishes a Twist with linear.x and angular.z set."""
    sim = MagicMock()
    controller = GazeboAckermannController(
        model_name="ego_vehicle", simulator=sim, own_simulator=False
    )
    controller.send_command(speed=0.5, steering_angle=0.3)

    sim.publish.assert_called_once()
    topic, msg_type, msg = sim.publish.call_args[0]
    assert topic == "/model/ego_vehicle/cmd_vel"
    assert msg_type == "Twist"
    assert msg.linear.x == pytest.approx(0.5)
    assert msg.angular.z == pytest.approx(0.3)


def test_gazebo_controller_stop_publishes_zero_command() -> None:
    """stop() publishes a zero-speed, zero-steering command."""
    sim = MagicMock()
    controller = GazeboAckermannController(
        model_name="ego_vehicle", simulator=sim, own_simulator=False
    )
    controller.stop()

    _topic, _msg_type, msg = sim.publish.call_args[0]
    assert msg.linear.x == pytest.approx(0.0)
    assert msg.angular.z == pytest.approx(0.0)


def test_gazebo_controller_subscribe_odometry() -> None:
    """subscribe_odometry() subscribes to the model odometry topic."""
    sim = MagicMock()
    controller = GazeboAckermannController(
        model_name="ego_vehicle", simulator=sim, own_simulator=False
    )

    def callback(_msg: object) -> None:
        pass

    controller.subscribe_odometry(callback)
    sim.subscribe.assert_called_once_with(
        "/model/ego_vehicle/odometry", "Odometry", callback
    )


def test_gazebo_controller_context_manager() -> None:
    """The controller can be used as a context manager."""
    controller = GazeboAckermannController(model_name="ego_vehicle")
    with patch.object(controller, "connect") as mock_connect, patch.object(
        controller, "disconnect"
    ) as mock_disconnect:
        with controller:
            mock_connect.assert_called_once()
        mock_disconnect.assert_called_once()
