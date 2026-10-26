"""Dataset-based object-detection validation and metric reporting."""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import pandas as pd

from perception.object_detector import Detection, ObjectDetector


@dataclass(frozen=True)
class GroundTruth:
    """One labelled object in a validation image."""

    class_name: str
    bbox: tuple[int, int, int, int]


def bounding_box_iou(
    first: tuple[int, int, int, int],
    second: tuple[int, int, int, int],
) -> float:
    """Calculate intersection-over-union for two pixel bounding boxes."""
    ax1, ay1, ax2, ay2 = first
    bx1, by1, bx2, by2 = second
    intersection_width = max(0, min(ax2, bx2) - max(ax1, bx1))
    intersection_height = max(0, min(ay2, by2) - max(ay1, by1))
    intersection = intersection_width * intersection_height
    first_area = max(0, ax2 - ax1) * max(0, ay2 - ay1)
    second_area = max(0, bx2 - bx1) * max(0, by2 - by1)
    union = first_area + second_area - intersection
    return intersection / union if union else 0.0


class ObjectDetectionValidator:
    """Validate an object detector against a CSV ground-truth manifest."""

    REQUIRED_COLUMNS = {"image_path", "class_name", "x1", "y1", "x2", "y2"}

    def __init__(
        self,
        detector: ObjectDetector,
        iou_threshold: float = 0.5,
    ) -> None:
        if not 0.0 < iou_threshold <= 1.0:
            raise ValueError("iou_threshold must be in (0, 1]")
        self._detector = detector
        self._iou_threshold = iou_threshold

    def evaluate(
        self,
        manifest: pd.DataFrame,
        image_root: str | Path = ".",
    ) -> tuple[pd.DataFrame, dict[str, Any]]:
        """Run detection over every image and return rows plus summary."""
        missing = self.REQUIRED_COLUMNS - set(manifest.columns)
        if missing:
            raise ValueError(
                f"Manifest is missing columns: {', '.join(sorted(missing))}"
            )

        root = Path(image_root)
        rows: list[dict[str, Any]] = []
        for image_path, group in manifest.groupby("image_path", sort=True):
            full_path = root / str(image_path)
            image = cv2.imread(str(full_path))
            if image is None:
                raise FileNotFoundError(f"Could not read image: {full_path}")
            ground_truth = [
                GroundTruth(
                    class_name=str(row.class_name),
                    bbox=(
                        int(row.x1),
                        int(row.y1),
                        int(row.x2),
                        int(row.y2),
                    ),
                )
                for row in group.itertuples(index=False)
            ]

            started_at = time.perf_counter()
            detections = self._detector.detect(image)
            latency_ms = (time.perf_counter() - started_at) * 1000.0
            true_positive, false_positive, false_negative = self._match(
                detections,
                ground_truth,
            )
            rows.append({
                "image_path": str(image_path),
                "ground_truth_count": len(ground_truth),
                "detection_count": len(detections),
                "true_positive": true_positive,
                "false_positive": false_positive,
                "false_negative": false_negative,
                "latency_ms": latency_ms,
            })

        results = pd.DataFrame(rows)
        summary = self._summarize(results)
        return results, summary

    def _match(
        self,
        detections: list[Detection],
        ground_truth: list[GroundTruth],
    ) -> tuple[int, int, int]:
        matched_truth: set[int] = set()
        true_positive = 0

        for detection in sorted(
            detections,
            key=lambda item: item.confidence,
            reverse=True,
        ):
            best_index: int | None = None
            best_iou = 0.0
            for index, truth in enumerate(ground_truth):
                if index in matched_truth or truth.class_name != detection.class_name:
                    continue
                iou = bounding_box_iou(detection.bbox, truth.bbox)
                if iou > best_iou:
                    best_index = index
                    best_iou = iou
            if best_index is not None and best_iou >= self._iou_threshold:
                matched_truth.add(best_index)
                true_positive += 1

        false_positive = len(detections) - true_positive
        false_negative = len(ground_truth) - true_positive
        return true_positive, false_positive, false_negative

    @staticmethod
    def _summarize(results: pd.DataFrame) -> dict[str, Any]:
        if results.empty:
            return {
                "images": 0,
                "true_positive": 0,
                "false_positive": 0,
                "false_negative": 0,
                "precision": 0.0,
                "recall": 0.0,
                "f1_score": 0.0,
                "mean_latency_ms": 0.0,
            }
        true_positive = int(results["true_positive"].sum())
        false_positive = int(results["false_positive"].sum())
        false_negative = int(results["false_negative"].sum())
        precision_denominator = true_positive + false_positive
        recall_denominator = true_positive + false_negative
        precision = (
            true_positive / precision_denominator
            if precision_denominator
            else 0.0
        )
        recall = (
            true_positive / recall_denominator if recall_denominator else 0.0
        )
        f1_score = (
            2.0 * precision * recall / (precision + recall)
            if precision + recall
            else 0.0
        )
        return {
            "images": len(results),
            "true_positive": true_positive,
            "false_positive": false_positive,
            "false_negative": false_negative,
            "precision": precision,
            "recall": recall,
            "f1_score": f1_score,
            "mean_latency_ms": float(results["latency_ms"].mean()),
        }
