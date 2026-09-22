"""Tests for lane detection and LKA pipeline."""

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from perception.lane_detector import LaneDetectionResult, LaneDetector
from planning.lka_pipeline import LKAPipeline


# --- LaneDetectionResult dataclass --------------------------------------------


def test_lane_detection_result_fields() -> None:
    r = LaneDetectionResult(
        lateral_offset=0.1,
        left_line_detected=True,
        right_line_detected=False,
        confidence=0.5,
    )
    assert r.lateral_offset == pytest.approx(0.1)
    assert r.left_line_detected is True
    assert r.right_line_detected is False


# --- LaneDetector -------------------------------------------------------------


def _make_road_image(
    width: int = 640,
    height: int = 480,
    left_x: int | None = 200,
    right_x: int | None = 440,
) -> np.ndarray:
    """Create a synthetic road image with white lane lines on dark background."""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:, :] = (40, 40, 40)  # dark road
    roi_top = int(height * 0.6)
    if left_x is not None:
        img[roi_top:, left_x - 2 : left_x + 2] = (255, 255, 255)
    if right_x is not None:
        img[roi_top:, right_x - 2 : right_x + 2] = (255, 255, 255)
    return img


def test_detect_both_lines_centred() -> None:
    detector = LaneDetector(white_threshold=200)
    img = _make_road_image(left_x=200, right_x=440)
    result = detector.detect(img)
    assert result.left_line_detected is True
    assert result.right_line_detected is True
    assert result.confidence == pytest.approx(1.0)
    assert abs(result.lateral_offset) < 0.2


def test_detect_both_lines_offset_left() -> None:
    detector = LaneDetector(white_threshold=200)
    img = _make_road_image(left_x=100, right_x=340)
    result = detector.detect(img)
    assert result.lateral_offset > 0  # vehicle is right of lane centre


def test_detect_both_lines_offset_right() -> None:
    detector = LaneDetector(white_threshold=200)
    img = _make_road_image(left_x=300, right_x=540)
    result = detector.detect(img)
    assert result.lateral_offset < 0  # vehicle is left of lane centre


def test_detect_only_left_line() -> None:
    detector = LaneDetector(white_threshold=200)
    img = _make_road_image(left_x=200, right_x=None)
    result = detector.detect(img)
    assert result.left_line_detected is True
    assert result.right_line_detected is False
    assert result.confidence == pytest.approx(0.5)


def test_detect_only_right_line() -> None:
    detector = LaneDetector(white_threshold=200)
    img = _make_road_image(left_x=None, right_x=440)
    result = detector.detect(img)
    assert result.left_line_detected is False
    assert result.right_line_detected is True
    assert result.confidence == pytest.approx(0.5)


def test_detect_no_lines() -> None:
    detector = LaneDetector(white_threshold=200)
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    result = detector.detect(img)
    assert result.left_line_detected is False
    assert result.right_line_detected is False
    assert result.confidence == pytest.approx(0.0)
    assert result.lateral_offset == pytest.approx(0.0)


def test_offset_clamped_to_range() -> None:
    detector = LaneDetector(white_threshold=200)
    img = _make_road_image(left_x=10, right_x=20)
    result = detector.detect(img)
    assert -1.0 <= result.lateral_offset <= 1.0


# --- LKAPipeline (mocked) ----------------------------------------------------


def test_lka_pipeline_start_stop() -> None:
    sim = MagicMock()
    ctrl = MagicMock()
    cam = MagicMock()
    cam.get_image.return_value = None

    pipeline = LKAPipeline(
        simulator=sim, controller=ctrl, camera=cam, loop_hz=100.0,
    )
    assert pipeline.is_running is False

    pipeline.start()
    assert pipeline.is_running is True

    pipeline.stop()
    assert pipeline.is_running is False
    ctrl.stop.assert_called()


def test_lka_pipeline_step_sends_command() -> None:
    sim = MagicMock()
    ctrl = MagicMock()
    cam = MagicMock()
    cam.get_image.return_value = _make_road_image()

    pipeline = LKAPipeline(
        simulator=sim, controller=ctrl, camera=cam,
    )
    pipeline._step()

    assert pipeline.step_count == 1
    ctrl.send_command.assert_called_once()
    call_kwargs = ctrl.send_command.call_args
    speed = call_kwargs[1]["speed"] if "speed" in call_kwargs[1] else call_kwargs[0][0]
    assert speed > 0


def test_lka_pipeline_no_image_skips() -> None:
    sim = MagicMock()
    ctrl = MagicMock()
    cam = MagicMock()
    cam.get_image.return_value = None

    pipeline = LKAPipeline(
        simulator=sim, controller=ctrl, camera=cam,
    )
    pipeline._step()

    assert pipeline.step_count == 0
    ctrl.send_command.assert_not_called()


def test_lka_pipeline_set_pose() -> None:
    sim = MagicMock()
    ctrl = MagicMock()
    cam = MagicMock()

    pipeline = LKAPipeline(
        simulator=sim, controller=ctrl, camera=cam,
    )
    pipeline.set_pose(1.0, 2.0, 0.5)
    assert pipeline._pose["x"] == pytest.approx(1.0)
    assert pipeline._pose["y"] == pytest.approx(2.0)
    assert pipeline._pose["heading"] == pytest.approx(0.5)


def test_lka_pipeline_context_manager() -> None:
    sim = MagicMock()
    ctrl = MagicMock()
    cam = MagicMock()
    cam.get_image.return_value = None

    pipeline = LKAPipeline(
        simulator=sim, controller=ctrl, camera=cam, loop_hz=100.0,
    )
    with pipeline:
        assert pipeline.is_running is True
    assert pipeline.is_running is False
