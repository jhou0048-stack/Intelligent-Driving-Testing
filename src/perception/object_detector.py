"""Object detection interface and YOLO implementation."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Detection:
    """A single detected object in an image."""

    class_name: str
    confidence: float
    bbox: tuple[int, int, int, int]  # x1, y1, x2, y2


class ObjectDetector(ABC):
    """Abstract interface for image-based object detection."""

    @abstractmethod
    def detect(self, image: np.ndarray) -> list[Detection]:
        """Run detection on a BGR image and return a list of detections."""


class YOLODetector(ObjectDetector):
    """Object detector backed by Ultralytics YOLO."""

    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        confidence_threshold: float = 0.5,
    ) -> None:
        from ultralytics import YOLO

        self._model = YOLO(model_path)
        self._confidence_threshold = confidence_threshold

    def detect(self, image: np.ndarray) -> list[Detection]:
        results = self._model(image, verbose=False)
        detections: list[Detection] = []
        for result in results:
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < self._confidence_threshold:
                    continue
                cls_id = int(box.cls[0])
                cls_name = result.names.get(cls_id, str(cls_id))
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])
                detections.append(
                    Detection(
                        class_name=cls_name,
                        confidence=conf,
                        bbox=(x1, y1, x2, y2),
                    )
                )
        return detections
