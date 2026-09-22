"""Tests for the integrated perception pipeline and ADAS pipeline."""

import math
from unittest.mock import MagicMock

import numpy as np
import pytest

from perception.perception_pipeline import (
    PerceivedObject,
    PerceptionPipeline,
    PerceptionResult,
)
from planning.adas_pipeline import ADASPipeline


# --- PerceivedObject ----------------------------------------------------------


def test_perceived_object_fields() -> None:
    obj = PerceivedObject(
        class_name="car", confidence=0.9,
        bbox=(100, 100, 200, 200),
        estimated_distance=5.0, bearing_angle=0.1,
    )
    assert obj.class_name == "car"
    assert obj.estimated_distance == 5.0


# --- PerceptionResult ---------------------------------------------------------


def test_perception_result_defaults() -> None:
    r = PerceptionResult(timestamp=1.0)
    assert r.objects == []
    assert r.lateral_offset == 0.0
    assert r.min_forward_distance == float("inf")


# --- PerceptionPipeline -------------------------------------------------------


def test_pipeline_no_sensors() -> None:
    p = PerceptionPipeline()
    result = p.process()
    assert result is not None
    assert result.lateral_offset == 0.0
    assert len(result.objects) == 0


def test_pipeline_with_lane_detector() -> None:
    camera = MagicMock()
    camera.get_image.return_value = np.zeros((480, 640, 3), dtype=np.uint8)

    lane_det = MagicMock()
    lane_result = MagicMock()
    lane_result.lateral_offset = 0.15
    lane_result.confidence = 1.0
    lane_result.left_line_detected = True
    lane_result.right_line_detected = True
    lane_det.detect.return_value = lane_result

    p = PerceptionPipeline(camera=camera, lane_detector=lane_det)
    result = p.process()
    assert result.lateral_offset == pytest.approx(0.15)
    assert result.lane_confidence == 1.0


def test_pipeline_with_object_detector() -> None:
    camera = MagicMock()
    camera.get_image.return_value = np.zeros((480, 640, 3), dtype=np.uint8)

    det = MagicMock()
    det.class_name = "car"
    det.confidence = 0.95
    det.bbox = (280, 200, 360, 350)
    obj_detector = MagicMock()
    obj_detector.detect.return_value = [det]

    p = PerceptionPipeline(camera=camera, object_detector=obj_detector)
    result = p.process()
    assert len(result.objects) == 1
    assert result.objects[0].class_name == "car"


def test_pipeline_with_lidar() -> None:
    lidar = MagicMock()
    ranges = np.full(360, 10.0)
    ranges[180] = 3.0  # obstacle ahead
    lidar.get_ranges.return_value = ranges

    p = PerceptionPipeline(lidar=lidar)
    result = p.process()
    assert result.min_forward_distance == pytest.approx(3.0)


def test_pipeline_lidar_all_inf() -> None:
    lidar = MagicMock()
    lidar.get_ranges.return_value = np.full(360, float("inf"))

    p = PerceptionPipeline(lidar=lidar)
    result = p.process()
    assert result.min_forward_distance == float("inf")


def test_pipeline_latest_result() -> None:
    p = PerceptionPipeline()
    assert p.latest_result is None
    p.process()
    assert p.latest_result is not None


def test_pipeline_distance_from_lidar_fusion() -> None:
    camera = MagicMock()
    camera.get_image.return_value = np.zeros((480, 640, 3), dtype=np.uint8)

    lidar = MagicMock()
    ranges = np.full(360, 50.0)
    ranges[180] = 5.0
    lidar.get_ranges.return_value = ranges

    det = MagicMock()
    det.class_name = "person"
    det.confidence = 0.8
    det.bbox = (300, 200, 340, 400)  # near centre
    obj_detector = MagicMock()
    obj_detector.detect.return_value = [det]

    p = PerceptionPipeline(
        camera=camera, lidar=lidar, object_detector=obj_detector,
    )
    result = p.process()
    assert len(result.objects) == 1
    assert result.objects[0].estimated_distance < 50.0


# --- ADASPipeline -------------------------------------------------------------


def test_adas_pipeline_start_stop() -> None:
    ctrl = MagicMock()
    perception = MagicMock()
    perception.process.return_value = None

    p = ADASPipeline(controller=ctrl, perception=perception, loop_hz=100.0)
    assert p.is_running is False
    p.start()
    assert p.is_running is True
    p.stop()
    assert p.is_running is False


def test_adas_pipeline_step_no_result() -> None:
    ctrl = MagicMock()
    perception = MagicMock()
    perception.process.return_value = None

    p = ADASPipeline(controller=ctrl, perception=perception)
    p._step()
    assert p.step_count == 0


def test_adas_pipeline_step_lka() -> None:
    ctrl = MagicMock()
    result = PerceptionResult(
        timestamp=1.0,
        lateral_offset=0.1,
        min_forward_distance=50.0,
    )
    perception = MagicMock()
    perception.process.return_value = result

    p = ADASPipeline(controller=ctrl, perception=perception, default_speed=0.5)
    p._step()
    assert p.step_count == 1
    assert p.is_aeb_active is False
    ctrl.send_command.assert_called_once()


def test_adas_pipeline_step_aeb() -> None:
    ctrl = MagicMock()
    result = PerceptionResult(
        timestamp=1.0,
        lateral_offset=0.0,
        min_forward_distance=0.5,  # very close
    )
    perception = MagicMock()
    perception.process.return_value = result

    p = ADASPipeline(controller=ctrl, perception=perception, default_speed=1.0)
    p._step()
    assert p.is_aeb_active is True
    ctrl.stop.assert_called()


def test_adas_pipeline_callback() -> None:
    ctrl = MagicMock()
    result = PerceptionResult(timestamp=1.0, min_forward_distance=50.0)
    perception = MagicMock()
    perception.process.return_value = result

    called = []
    p = ADASPipeline(controller=ctrl, perception=perception)
    p.add_step_callback(lambda r: called.append(r))
    p._step()
    assert len(called) == 1


def test_adas_pipeline_context_manager() -> None:
    ctrl = MagicMock()
    perception = MagicMock()
    perception.process.return_value = None

    p = ADASPipeline(controller=ctrl, perception=perception, loop_hz=100.0)
    with p:
        assert p.is_running is True
    assert p.is_running is False


def test_adas_pipeline_set_pose() -> None:
    ctrl = MagicMock()
    perception = MagicMock()
    p = ADASPipeline(controller=ctrl, perception=perception)
    p.set_pose(1.0, 2.0, 0.5)
    assert p._pose["x"] == pytest.approx(1.0)
