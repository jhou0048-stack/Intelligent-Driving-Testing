"""Lane line detection using OpenCV image processing."""

from __future__ import annotations

import logging
from dataclasses import dataclass

import cv2
import numpy as np

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class LaneDetectionResult:
    """Result of lane line detection on a single frame."""

    lateral_offset: float
    left_line_detected: bool
    right_line_detected: bool
    confidence: float


class LaneDetector:
    """Detect lane markings and compute lateral offset from lane centre.

    Pipeline: crop ROI → grayscale → blur → threshold (white lines on dark
    road) → Hough lines → fit left/right boundaries → compute offset from
    image centre.
    """

    def __init__(
        self,
        roi_top_fraction: float = 0.6,
        white_threshold: int = 200,
        image_width: int = 640,
        image_height: int = 480,
    ) -> None:
        self._roi_top_frac = roi_top_fraction
        self._white_thresh = white_threshold
        self._img_w = image_width
        self._img_h = image_height

    def detect(self, image: np.ndarray) -> LaneDetectionResult:
        """Detect lanes in a BGR image and return the lateral offset.

        A positive offset means the vehicle is left of centre; negative means
        right of centre. The value is in normalised image coordinates
        (fraction of half-image width), so +-1.0 means at the lane edge.
        """
        h, w = image.shape[:2]
        roi_top = int(h * self._roi_top_frac)
        roi = image[roi_top:, :]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(blurred, self._white_thresh, 255, cv2.THRESH_BINARY)

        lines = cv2.HoughLinesP(
            binary,
            rho=1,
            theta=np.pi / 180,
            threshold=30,
            minLineLength=20,
            maxLineGap=10,
        )

        if lines is None:
            return LaneDetectionResult(
                lateral_offset=0.0,
                left_line_detected=False,
                right_line_detected=False,
                confidence=0.0,
            )

        centre_x = w / 2.0
        left_xs: list[float] = []
        right_xs: list[float] = []

        for line in lines:
            coords = line[0] if lines.ndim == 3 else line
            x1, y1, x2, y2 = coords
            mid_x = (x1 + x2) / 2.0
            if mid_x < centre_x:
                left_xs.append(mid_x)
            else:
                right_xs.append(mid_x)

        left_detected = len(left_xs) > 0
        right_detected = len(right_xs) > 0

        if left_detected and right_detected:
            left_avg = np.mean(left_xs)
            right_avg = np.mean(right_xs)
            lane_centre = (left_avg + right_avg) / 2.0
            offset = (centre_x - lane_centre) / (centre_x)
            confidence = 1.0
        elif left_detected:
            left_avg = np.mean(left_xs)
            offset = (centre_x - left_avg) / centre_x - 1.0
            confidence = 0.5
        elif right_detected:
            right_avg = np.mean(right_xs)
            offset = (centre_x - right_avg) / centre_x + 1.0
            confidence = 0.5
        else:
            offset = 0.0
            confidence = 0.0

        offset = max(-1.0, min(1.0, offset))

        return LaneDetectionResult(
            lateral_offset=offset,
            left_line_detected=left_detected,
            right_line_detected=right_detected,
            confidence=confidence,
        )
