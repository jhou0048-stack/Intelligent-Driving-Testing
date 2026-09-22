"""Simulator-agnostic abstraction for ADAS test environments.

The :class:`Simulator` interface defines the contract that any backend
(Gazebo, CARLA, etc.) must satisfy so that the testing framework remains
independent of simulator internals.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import Any


class Simulator(ABC):
    """Abstract simulator backend used by the ADAS testing framework.

    Implementations are responsible for translating these generic calls into
    simulator-specific operations (e.g. ``gz-transport`` topics for Gazebo).
    """

    @abstractmethod
    def connect(self) -> None:
        """Establish a connection to the simulator backend."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Release all simulator resources and close the connection."""
        ...

    @abstractmethod
    def is_connected(self) -> bool:
        """Return ``True`` when the simulator connection is active."""
        ...

    @abstractmethod
    def get_topic_names(self) -> list[str]:
        """Return a list of currently advertised transport topics."""
        ...

    @abstractmethod
    def subscribe(
        self,
        topic: str,
        msg_type: str,
        callback: Callable[[Any], None],
    ) -> None:
        """Subscribe ``callback`` to messages of ``msg_type`` on ``topic``."""
        ...

    @abstractmethod
    def publish(self, topic: str, msg_type: str, message: Any) -> None:
        """Publish ``message`` of ``msg_type`` on ``topic``."""
        ...

    def __enter__(self) -> Simulator:
        self.connect()
        return self

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> None:
        self.disconnect()
