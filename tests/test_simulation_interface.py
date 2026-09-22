"""Tests for the simulator-agnostic interface and Gazebo backend."""

from unittest.mock import MagicMock, patch

import pytest

from simulation import GazeboSimulator, Simulator
from simulation.gazebo_client import _resolve_msg_type


def test_simulator_is_abstract() -> None:
    """The base Simulator class cannot be instantiated directly."""
    with pytest.raises(TypeError):
        Simulator()  # type: ignore[abstract]


def test_gazebo_simulator_starts_disconnected() -> None:
    """A fresh GazeboSimulator is not connected."""
    sim = GazeboSimulator()
    assert not sim.is_connected()


def test_gazebo_simulator_requires_connection_for_topic_list() -> None:
    """Operations that need a transport node raise when disconnected."""
    sim = GazeboSimulator()
    with pytest.raises(RuntimeError, match="not connected"):
        sim.get_topic_names()


def test_gazebo_simulator_connect_and_disconnect() -> None:
    """connect()/disconnect() manage the transport node lifecycle."""
    sim = GazeboSimulator()
    sim.connect()
    assert sim.is_connected()
    sim.disconnect()
    assert not sim.is_connected()


def test_gazebo_simulator_context_manager() -> None:
    """The simulator can be used as a context manager."""
    with GazeboSimulator() as sim:
        assert sim.is_connected()
    assert not sim.is_connected()


def test_gazebo_simulator_topic_list_returns_list() -> None:
    """topic_list() returns a Python list of topic names."""
    with GazeboSimulator() as sim:
        topics = sim.get_topic_names()
        assert isinstance(topics, list)


def test_resolve_msg_type_stringmsg() -> None:
    """The short name ``StringMsg`` resolves to the Gazebo protobuf class."""
    msg_cls = _resolve_msg_type("StringMsg")
    assert msg_cls.__name__ == "StringMsg"


def test_resolve_msg_type_camelcase() -> None:
    """CamelCase names map to snake_case protobuf modules."""
    msg_cls = _resolve_msg_type("WorldControl")
    assert msg_cls.__name__ == "WorldControl"


def test_resolve_msg_type_irregular_abbreviation() -> None:
    """Known abbreviation exceptions (e.g. WorldStatistics -> world_stats)."""
    msg_cls = _resolve_msg_type("WorldStatistics")
    assert msg_cls.__name__ == "WorldStatistics"


def test_resolve_msg_type_full_path() -> None:
    """A fully-qualified module path resolves to the protobuf class."""
    msg_cls = _resolve_msg_type("gz.msgs10.stringmsg_pb2.StringMsg")
    assert msg_cls.__name__ == "StringMsg"


def test_resolve_msg_type_unknown_raises() -> None:
    """An unresolvable message type raises a helpful error."""
    with pytest.raises(ModuleNotFoundError):
        _resolve_msg_type("DoesNotExist")


def test_resolve_msg_type_non_string_raises() -> None:
    """Passing a non-string message type raises TypeError."""
    with pytest.raises(TypeError):
        _resolve_msg_type(123)  # type: ignore[arg-type]


def test_gazebo_simulator_publish_requires_connection() -> None:
    """publish() raises when the simulator is not connected."""
    sim = GazeboSimulator()
    with pytest.raises(RuntimeError, match="not connected"):
        sim.publish("/test", "StringMsg", MagicMock())


def test_gazebo_simulator_subscribe_requires_connection() -> None:
    """subscribe() raises when the simulator is not connected."""
    sim = GazeboSimulator()

    def callback(_msg: object) -> None:
        pass

    with pytest.raises(RuntimeError, match="not connected"):
        sim.subscribe("/test", "StringMsg", callback)


def test_gazebo_simulator_publish_uses_advertised_publisher() -> None:
    """publish() advertises a publisher on first use and reuses it."""
    sim = GazeboSimulator()
    fake_node = MagicMock()
    fake_publisher = MagicMock()
    fake_publisher.topic = "/test"
    fake_node.advertise.return_value = fake_publisher

    with patch.object(sim, "_node", fake_node):
        import gz.msgs10.stringmsg_pb2 as stringmsg

        msg = stringmsg.StringMsg()
        msg.data = "hello"
        sim.publish("/test", "StringMsg", msg)
        sim.publish("/test", "StringMsg", msg)

    fake_node.advertise.assert_called_once()
    assert fake_publisher.publish.call_count == 2


def test_gazebo_simulator_request_requires_connection() -> None:
    """request() raises when the simulator is not connected."""
    sim = GazeboSimulator()
    with pytest.raises(RuntimeError, match="not connected"):
        sim.request("/world/default/control", MagicMock(), "WorldControl", "Boolean")


def test_gazebo_simulator_request_calls_node_request() -> None:
    """request() forwards to the transport node and returns the response."""
    sim = GazeboSimulator()
    fake_node = MagicMock()
    fake_response = MagicMock()
    fake_node.request.return_value = (fake_response, True)

    import gz.msgs10.world_control_pb2 as world_control

    request = world_control.WorldControl()
    request.pause = False

    with patch.object(sim, "_node", fake_node):
        response = sim.request(
            "/world/simple_road/control",
            request,
            "WorldControl",
            "Boolean",
            timeout_ms=10000,
        )

    assert response is fake_response
    fake_node.request.assert_called_once()
    call_args = fake_node.request.call_args[0]
    assert call_args[0] == "/world/simple_road/control"
    assert call_args[1] is request


def test_gazebo_simulator_request_returns_none_on_timeout() -> None:
    """request() returns None when the service call times out."""
    sim = GazeboSimulator()
    fake_node = MagicMock()
    fake_node.request.return_value = (None, False)

    import gz.msgs10.world_control_pb2 as world_control

    request = world_control.WorldControl()
    request.pause = False

    with patch.object(sim, "_node", fake_node):
        response = sim.request(
            "/world/simple_road/control",
            request,
            "WorldControl",
            "Boolean",
            timeout_ms=10000,
        )

    assert response is None
