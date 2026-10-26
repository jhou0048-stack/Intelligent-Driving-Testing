"""Tests for offline object-detection validation."""

from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import pytest

from perception.object_detector import Detection
from testing.perception_validator import (
    ObjectDetectionValidator,
    bounding_box_iou,
)


class FakeDetector:
    def __init__(self, detections: list[Detection]) -> None:
        self._detections = detections

    def detect(self, image: np.ndarray) -> list[Detection]:
        return list(self._detections)


def test_bounding_box_iou() -> None:
    assert bounding_box_iou((0, 0, 10, 10), (0, 0, 10, 10)) == 1.0
    assert bounding_box_iou((0, 0, 5, 5), (5, 5, 10, 10)) == 0.0
    assert bounding_box_iou((0, 0, 10, 10), (5, 5, 15, 15)) == pytest.approx(
        25 / 175
    )


def test_object_detection_validator_computes_metrics(tmp_path: Path) -> None:
    image_path = tmp_path / "frame.png"
    cv2.imwrite(str(image_path), np.zeros((32, 32, 3), dtype=np.uint8))
    manifest = pd.DataFrame([{
        "image_path": "frame.png",
        "class_name": "car",
        "x1": 5,
        "y1": 5,
        "x2": 20,
        "y2": 20,
    }])
    detector = FakeDetector([
        Detection("car", 0.9, (5, 5, 20, 20)),
        Detection("person", 0.8, (1, 1, 4, 8)),
    ])

    rows, summary = ObjectDetectionValidator(detector).evaluate(
        manifest,
        tmp_path,
    )

    assert len(rows) == 1
    assert summary["true_positive"] == 1
    assert summary["false_positive"] == 1
    assert summary["false_negative"] == 0
    assert summary["precision"] == pytest.approx(0.5)
    assert summary["recall"] == pytest.approx(1.0)


def test_object_detection_validator_rejects_bad_manifest() -> None:
    validator = ObjectDetectionValidator(FakeDetector([]))
    with pytest.raises(ValueError, match="missing columns"):
        validator.evaluate(pd.DataFrame([{"image_path": "frame.png"}]))
