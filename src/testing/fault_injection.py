"""Deterministic sensor fault injection for ADAS validation scenarios."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class FaultSpec:
    """One time-bounded sensor fault."""

    target: str
    fault_type: str
    start_time: float = 0.0
    duration: float = 1.0
    params: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> FaultSpec:
        target = str(data["target"])
        fault_type = str(data["type"])
        if target not in {"camera", "lidar"}:
            raise ValueError(f"Unsupported fault target: {target}")
        supported = {
            "camera": {"dropout", "frozen_frame", "blur", "noise"},
            "lidar": {"dropout", "frozen_scan", "range_bias", "noise"},
        }
        if fault_type not in supported[target]:
            raise ValueError(
                f"Unsupported {target} fault type: {fault_type}"
            )
        start_time = float(data.get("start_time", 0.0))
        duration = float(data.get("duration", 1.0))
        if start_time < 0 or duration <= 0:
            raise ValueError("Fault start_time must be >= 0 and duration > 0")
        params = {
            key: value
            for key, value in data.items()
            if key not in {"target", "type", "start_time", "duration"}
        }
        return cls(target, fault_type, start_time, duration, params)


class FaultClock:
    """Shared scenario clock used by every injected sensor."""

    def __init__(self) -> None:
        self._started_at: float | None = None

    def start(self) -> None:
        self._started_at = time.monotonic()

    @property
    def elapsed(self) -> float | None:
        if self._started_at is None:
            return None
        return time.monotonic() - self._started_at


class FaultInjectedCamera:
    """Camera decorator supporting dropout, freeze, blur, and noise."""

    def __init__(
        self,
        camera: Any,
        faults: list[FaultSpec],
        clock: FaultClock,
        seed: int = 7,
    ) -> None:
        self._camera = camera
        self._faults = faults
        self._clock = clock
        self._rng = np.random.default_rng(seed)
        self._frozen_frame: np.ndarray | None = None
        self.activation_count = 0

    def connect(self) -> None:
        self._camera.connect()

    def disconnect(self) -> None:
        self._camera.disconnect()

    def is_connected(self) -> bool:
        return self._camera.is_connected()

    def get_image(self) -> np.ndarray | None:
        image = self._camera.get_image()
        active = self._active_faults()
        if not active:
            self._frozen_frame = None
            return image

        for fault in active:
            self.activation_count += 1
            if fault.fault_type == "dropout":
                return None
            if image is None:
                return None
            if fault.fault_type == "frozen_frame":
                if self._frozen_frame is None:
                    self._frozen_frame = image.copy()
                image = self._frozen_frame.copy()
            elif fault.fault_type == "blur":
                kernel = int(fault.params.get("kernel_size", 15))
                kernel = max(3, kernel | 1)
                image = cv2.GaussianBlur(image, (kernel, kernel), 0)
            elif fault.fault_type == "noise":
                sigma = float(fault.params.get("sigma", 20.0))
                noise = self._rng.normal(0.0, sigma, image.shape)
                image = np.clip(image.astype(float) + noise, 0, 255).astype(
                    np.uint8
                )
        return image

    def _active_faults(self) -> list[FaultSpec]:
        elapsed = self._clock.elapsed
        if elapsed is None:
            return []
        return [
            fault
            for fault in self._faults
            if fault.start_time <= elapsed < fault.start_time + fault.duration
        ]

    @property
    def width(self) -> int:
        return self._camera.width

    @property
    def height(self) -> int:
        return self._camera.height


class FaultInjectedLidar:
    """LiDAR decorator supporting dropout, freeze, bias, and noise."""

    def __init__(
        self,
        lidar: Any,
        faults: list[FaultSpec],
        clock: FaultClock,
        seed: int = 11,
    ) -> None:
        self._lidar = lidar
        self._faults = faults
        self._clock = clock
        self._rng = np.random.default_rng(seed)
        self._frozen_scan: np.ndarray | None = None
        self.activation_count = 0

    def connect(self) -> None:
        self._lidar.connect()

    def disconnect(self) -> None:
        self._lidar.disconnect()

    def is_connected(self) -> bool:
        return self._lidar.is_connected()

    def get_ranges(self) -> np.ndarray | None:
        ranges = self._lidar.get_ranges()
        active = self._active_faults()
        if not active:
            self._frozen_scan = None
            return ranges

        for fault in active:
            self.activation_count += 1
            if fault.fault_type == "dropout":
                return None
            if ranges is None:
                return None
            if fault.fault_type == "frozen_scan":
                if self._frozen_scan is None:
                    self._frozen_scan = ranges.copy()
                ranges = self._frozen_scan.copy()
            elif fault.fault_type == "range_bias":
                bias = float(fault.params.get("bias", 0.5))
                ranges = ranges.copy()
                valid = np.isfinite(ranges) & (ranges > 0)
                ranges[valid] += bias
            elif fault.fault_type == "noise":
                sigma = float(fault.params.get("sigma", 0.1))
                ranges = ranges.copy()
                valid = np.isfinite(ranges) & (ranges > 0)
                ranges[valid] += self._rng.normal(0.0, sigma, valid.sum())
                ranges[valid] = np.maximum(ranges[valid], 0.01)
        return ranges

    def get_point_cloud(self) -> np.ndarray | None:
        return self._lidar.get_point_cloud()

    def _active_faults(self) -> list[FaultSpec]:
        elapsed = self._clock.elapsed
        if elapsed is None:
            return []
        return [
            fault
            for fault in self._faults
            if fault.start_time <= elapsed < fault.start_time + fault.duration
        ]

    @property
    def angle_min(self) -> float:
        return self._lidar._angle_min

    @property
    def angle_step(self) -> float:
        return self._lidar._angle_step

    def __getattr__(self, name: str) -> Any:
        return getattr(self._lidar, name)
