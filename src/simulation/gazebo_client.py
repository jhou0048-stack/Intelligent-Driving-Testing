"""Gazebo Harmonic backend for the simulator-agnostic interface."""

from __future__ import annotations

import importlib
import logging
from collections.abc import Callable
from typing import Any

from simulation.simulator import Simulator

logger = logging.getLogger(__name__)


def _camel_to_snake(name: str) -> str:
    """Convert a CamelCase identifier to snake_case."""
    result = []
    for i, ch in enumerate(name):
        if ch.isupper() and i > 0:
            result.append("_")
        result.append(ch.lower())
    return "".join(result)


# Short message names whose protobuf module does not follow a simple naming rule.
_MSG_MODULE_OVERRIDES: dict[str, str] = {
    "WorldStatistics": "world_stats",
    "StringMsg": "stringmsg",
    "StringMsg_V": "stringmsg_v",
    "LaserScan": "laserscan",
    "CameraInfo": "camerasensor",
}


def _resolve_msg_type(msg_type: str) -> type:
    """Map a message type name to a Gazebo protobuf class.

    Supported forms:

    * ``StringMsg`` -> ``gz.msgs10.stringmsg_pb2.StringMsg``
    * ``WorldControl`` -> ``gz.msgs10.world_control_pb2.WorldControl``
    * ``gz.msgs10.stringmsg_pb2.StringMsg`` -> imported directly
    * ``stringmsg_pb2.StringMsg`` -> imported from ``gz.msgs10``
    """
    if not isinstance(msg_type, str):
        raise TypeError(f"msg_type must be a string, got {type(msg_type)}")

    # Full dotted path already provided.
    if "." in msg_type and msg_type.count(".") >= 2:
        module_name, class_name = msg_type.rsplit(".", 1)
        module = importlib.import_module(module_name)
        return getattr(module, class_name)

    # Partial dotted path like "stringmsg_pb2.StringMsg".
    if "." in msg_type:
        module_name, class_name = msg_type.rsplit(".", 1)
        module_path = f"gz.msgs10.{module_name}"
        module = importlib.import_module(module_path)
        return getattr(module, class_name)

    # Short name like "StringMsg" -> gz.msgs10.stringmsg_pb2.StringMsg.
    class_name = msg_type
    candidates = [
        _MSG_MODULE_OVERRIDES.get(class_name),
        class_name.lower(),
        _camel_to_snake(class_name),
    ]
    last_exc: ModuleNotFoundError | None = None
    for module_name in candidates:
        if module_name is None:
            continue
        module_path = f"gz.msgs10.{module_name}_pb2"
        try:
            module = importlib.import_module(module_path)
        except ModuleNotFoundError as exc:
            last_exc = exc
            continue
        try:
            return getattr(module, class_name)
        except AttributeError as exc:
            last_exc = exc
            continue

    raise ModuleNotFoundError(
        f"Could not resolve message type {msg_type!r}. "
        "Provide a full module path (e.g. 'gz.msgs10.world_control_pb2.WorldControl')."
    ) from last_exc


class GazeboSimulator(Simulator):
    """Gazebo Harmonic backend using ``gz-transport13``.

    The backend connects to the default Gazebo transport partition and exposes
    topic listing, subscription, and publication through the :class:`Simulator`
    interface.
    """

    def __init__(self, partition: str | None = None) -> None:
        self._partition = partition
        self._node: Any | None = None
        self._publishers: dict[str, Any] = {}
        self._subscriptions: list[tuple[str, Callable[[Any], None]]] = []

    def connect(self) -> None:
        """Create a Gazebo transport node."""
        if self._node is not None:
            return
        import gz.transport13 as gz_transport

        options = gz_transport.NodeOptions()
        if self._partition is not None:
            options.partition = self._partition
        self._node = gz_transport.Node(options)
        logger.info("Connected to Gazebo transport")

    def disconnect(self) -> None:
        """Unsubscribe, destroy publishers, and release the node."""
        if self._node is None:
            return
        for topic, _ in self._subscriptions:
            self._node.unsubscribe(topic)
        self._subscriptions.clear()
        self._publishers.clear()
        self._node = None
        logger.info("Disconnected from Gazebo transport")

    def is_connected(self) -> bool:
        """Return ``True`` when a transport node exists."""
        return self._node is not None

    def get_topic_names(self) -> list[str]:
        """Return advertised topics in the current transport partition."""
        self._ensure_connected()
        topics = self._node.topic_list()
        return list(topics) if topics else []

    def subscribe(
        self,
        topic: str,
        msg_type: str,
        callback: Callable[[Any], None],
    ) -> None:
        """Subscribe ``callback`` to ``topic`` with messages of ``msg_type``."""
        self._ensure_connected()
        msg_cls = _resolve_msg_type(msg_type)
        ok = self._node.subscribe(msg_cls, topic, callback)
        if not ok:
            raise RuntimeError(f"Failed to subscribe to {topic}")
        self._subscriptions.append((topic, callback))
        logger.debug("Subscribed to %s", topic)

    def publish(self, topic: str, msg_type: str, message: Any) -> None:
        """Publish ``message`` on ``topic`` as ``msg_type``."""
        self._ensure_connected()
        msg_cls = _resolve_msg_type(msg_type)

        publisher = self._publishers.get(topic)
        if publisher is None:
            publisher = self._node.advertise(topic, msg_cls)
            self._publishers[topic] = publisher

        publisher.publish(message)
        logger.debug("Published to %s", topic)

    def request(
        self,
        service: str,
        request: Any,
        request_type: str,
        response_type: str,
        timeout_ms: int = 5000,
    ) -> Any:
        """Call a Gazebo transport service and return the response.

        Args:
            service: Service name, e.g. ``/world/simple_road/control``.
            request: Populated protobuf request message.
            request_type: Short or full protobuf type name for the request.
            response_type: Short or full protobuf type name for the response.
            timeout_ms: Call timeout in milliseconds.

        Returns:
            The response protobuf message, or ``None`` if the call timed out.
        """
        self._ensure_connected()
        req_cls = _resolve_msg_type(request_type)
        rep_cls = _resolve_msg_type(response_type)
        rep, ok = self._node.request(service, request, req_cls, rep_cls, timeout_ms)
        if not ok:
            logger.warning("Service call to %s timed out", service)
        return rep if ok else None

    def _ensure_connected(self) -> None:
        if self._node is None:
            raise RuntimeError("GazeboSimulator is not connected. Call connect() first.")
