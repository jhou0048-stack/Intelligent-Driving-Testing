"""Camera sensor interface and Gazebo implementation."""

from __future__ import annotations

import logging
import threading
from abc import abstractmethod
from typing import Any

import numpy as np

from perception.sensor import Sensor, SensorReading
from simulation import GazeboSimulator

logger = logging.getLogger(__name__)

_PIXEL_FORMAT_CHANNELS = {
    0: 1,   # UNKNOWN / L8
    1: 1,   # L_INT8
    2: 1,   # L_INT16
    3: 3,   # R_FLOAT16
    4: 3,   # R_FLOAT32
    5: 3,   # RGB_INT8
    6: 3,   # RGB_INT16
    7: 3,   # RGB_FLOAT16
    8: 3,   # RGB_FLOAT32
}

RGB_INT8 = 5


def _image_topic(world: str, model: str, link: str, sensor: str) -> str:
    return f"/world/{world}/model/{model}/link/{link}/sensor/{sensor}/image"


class Camera(Sensor):
    """Abstract camera sensor that produces image frames."""

    @abstractmethod
    def get_image(self) -> np.ndarray | None:
        """Return the latest frame as an HxWx3 uint8 BGR numpy array."""

    @property
    @abstractmethod
    def width(self) -> int:
        """Image width in pixels."""

    @property
    @abstractmethod
    def height(self) -> int:
        """Image height in pixels."""


class GazeboCamera(Camera):
    """Subscribe to a Gazebo camera sensor and decode images."""

    def __init__(
        self,
        model_name: str = "ego_vehicle",
        link_name: str = "camera_link",
        sensor_name: str = "front_camera",
        world_name: str = "simple_road",
        simulator: GazeboSimulator | None = None,
        own_simulator: bool = False,
        image_width: int = 640,
        image_height: int = 480,
    ) -> None:
        self._model_name = model_name
        self._link_name = link_name
        self._sensor_name = sensor_name
        self._world_name = world_name
        self._simulator = simulator if simulator is not None else GazeboSimulator()
        self._own_simulator = own_simulator or simulator is None
        self._image_width = image_width
        self._image_height = image_height
        self._topic = _image_topic(world_name, model_name, link_name, sensor_name)
        self._lock = threading.Lock()
        self._latest_image: np.ndarray | None = None
        self._latest_timestamp: float = 0.0

    def connect(self) -> None:
        if self._own_simulator:
            self._simulator.connect()
        self._simulator.subscribe(self._topic, "Image", self._on_image)
        logger.info("Camera connected on %s", self._topic)

    def disconnect(self) -> None:
        with self._lock:
            self._latest_image = None
        if self._own_simulator:
            self._simulator.disconnect()
        logger.info("Camera disconnected from %s", self._topic)

    def is_connected(self) -> bool:
        return self._simulator.is_connected()

    def _on_image(self, msg: Any) -> None:
        try:
            w = msg.width
            h = msg.height
            fmt = getattr(msg, "pixel_format_type", RGB_INT8)
            channels = _PIXEL_FORMAT_CHANNELS.get(fmt, 3)
            raw = msg.data
            frame = np.frombuffer(raw, dtype=np.uint8).reshape(h, w, channels)
            if channels == 3:
                bgr = frame[:, :, ::-1].copy()
            else:
                bgr = frame.copy()
            timestamp = 0.0
            if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                stamp = msg.header.stamp
                timestamp = stamp.sec + stamp.nsec * 1e-9
            with self._lock:
                self._latest_image = bgr
                self._latest_timestamp = timestamp
        except Exception:
            logger.exception("Failed to decode camera image")

    def get_image(self) -> np.ndarray | None:
        with self._lock:
            if self._latest_image is None:
                return None
            return self._latest_image.copy()

    def get_latest(self) -> SensorReading | None:
        with self._lock:
            if self._latest_image is None:
                return None
            return SensorReading(
                timestamp=self._latest_timestamp,
                sensor_name=self._sensor_name,
                data=self._latest_image.copy(),
            )

    @property
    def width(self) -> int:
        return self._image_width

    @property
    def height(self) -> int:
        return self._image_height

    @property
    def topic(self) -> str:
        return self._topic
