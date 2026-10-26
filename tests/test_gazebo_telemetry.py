"""Tests for Gazebo scenario telemetry collection."""

from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from simulation.gazebo_telemetry import GazeboVehicleTelemetry


def _odometry(x: float, y: float, speed: float) -> SimpleNamespace:
    return SimpleNamespace(
        pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)),
        twist=SimpleNamespace(
            linear=SimpleNamespace(x=speed, y=0.0, z=0.0)
        ),
    )


def test_telemetry_subscribes_to_vehicle_topics() -> None:
    simulator = MagicMock()
    telemetry = GazeboVehicleTelemetry(
        simulator=simulator,
        world_name="simple_road",
        model_name="ego_vehicle",
    )

    telemetry.start()

    assert simulator.subscribe.call_count == 2
    topics = [call.args[0] for call in simulator.subscribe.call_args_list]
    assert "/model/ego_vehicle/odometry" in topics
    assert (
        "/world/simple_road/model/ego_vehicle/link/chassis/"
        "sensor/chassis_contact/contact"
    ) in topics


def test_telemetry_collects_odometry_metrics() -> None:
    simulator = MagicMock()
    simulator.has_topic_publisher.return_value = False
    telemetry = GazeboVehicleTelemetry(
        simulator=simulator,
        world_name="simple_road",
        model_name="ego_vehicle",
        lane_center_y=-1.5,
    )

    telemetry._on_odometry(_odometry(3.0, -1.2, 2.0))
    telemetry._on_odometry(_odometry(4.0, -1.0, 1.0))
    metrics = telemetry.snapshot()

    assert metrics["odometry_samples"] == 2
    assert metrics["final_x"] == pytest.approx(4.0)
    assert metrics["final_y"] == pytest.approx(-1.0)
    assert metrics["mean_speed"] == pytest.approx(1.5)
    assert metrics["max_speed"] == pytest.approx(2.0)
    assert metrics["max_lateral_deviation"] == pytest.approx(0.5)


def test_telemetry_counts_contact_event_once() -> None:
    simulator = MagicMock()
    simulator.has_topic_publisher.return_value = True
    telemetry = GazeboVehicleTelemetry(
        simulator=simulator,
        world_name="simple_road",
        model_name="ego_vehicle",
    )
    contact_message = SimpleNamespace(contact=[SimpleNamespace()])

    telemetry._on_contacts(contact_message)
    telemetry._on_contacts(contact_message)
    metrics = telemetry.snapshot()

    assert metrics["collision_count"] == 1
    assert metrics["contact_topic_available"] is True


def test_telemetry_omits_collision_metric_without_topic() -> None:
    simulator = MagicMock()
    simulator.has_topic_publisher.return_value = False
    telemetry = GazeboVehicleTelemetry(
        simulator=simulator,
        world_name="simple_road",
        model_name="ego_vehicle",
    )

    metrics = telemetry.snapshot()

    assert metrics["contact_topic_available"] is False
    assert "collision_count" not in metrics
