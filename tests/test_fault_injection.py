"""Tests for deterministic sensor fault injection."""

import time
from unittest.mock import MagicMock

import numpy as np
import pytest

from testing.fault_injection import (
    FaultClock,
    FaultInjectedCamera,
    FaultInjectedLidar,
    FaultSpec,
)


def _active_clock(elapsed: float = 1.0) -> FaultClock:
    clock = FaultClock()
    clock._started_at = time.monotonic() - elapsed
    return clock


def test_fault_spec_rejects_unknown_type() -> None:
    with pytest.raises(ValueError, match="Unsupported camera fault"):
        FaultSpec.from_dict({"target": "camera", "type": "bad"})


def test_camera_dropout_returns_none_during_window() -> None:
    camera = MagicMock()
    camera.get_image.return_value = np.full((4, 4, 3), 100, dtype=np.uint8)
    injector = FaultInjectedCamera(
        camera,
        [FaultSpec("camera", "dropout", start_time=0.5, duration=2.0)],
        _active_clock(),
    )

    assert injector.get_image() is None
    assert injector.activation_count == 1


def test_camera_blur_preserves_shape() -> None:
    camera = MagicMock()
    camera.get_image.return_value = np.zeros((8, 8, 3), dtype=np.uint8)
    injector = FaultInjectedCamera(
        camera,
        [FaultSpec("camera", "blur", params={"kernel_size": 5})],
        _active_clock(elapsed=0.1),
    )

    result = injector.get_image()

    assert result is not None
    assert result.shape == (8, 8, 3)


def test_lidar_range_bias_changes_only_valid_ranges() -> None:
    lidar = MagicMock()
    lidar.get_ranges.return_value = np.array([1.0, np.inf, 2.0])
    injector = FaultInjectedLidar(
        lidar,
        [FaultSpec("lidar", "range_bias", params={"bias": 0.5})],
        _active_clock(elapsed=0.1),
    )

    result = injector.get_ranges()

    assert result is not None
    assert result[0] == pytest.approx(1.5)
    assert np.isinf(result[1])
    assert result[2] == pytest.approx(2.5)


def test_fault_outside_window_passes_through() -> None:
    camera = MagicMock()
    image = np.full((2, 2, 3), 50, dtype=np.uint8)
    camera.get_image.return_value = image
    injector = FaultInjectedCamera(
        camera,
        [FaultSpec("camera", "dropout", start_time=5.0, duration=1.0)],
        _active_clock(elapsed=1.0),
    )

    assert np.array_equal(injector.get_image(), image)
    assert injector.activation_count == 0


def test_fault_is_inactive_before_scenario_clock_starts() -> None:
    camera = MagicMock()
    image = np.full((2, 2, 3), 50, dtype=np.uint8)
    camera.get_image.return_value = image
    injector = FaultInjectedCamera(
        camera,
        [FaultSpec("camera", "dropout", start_time=0.0, duration=1.0)],
        FaultClock(),
    )

    assert np.array_equal(injector.get_image(), image)
    assert injector.activation_count == 0
