"""Time-series metric collection and evaluation for ADAS validation."""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass, field
from typing import Any


@dataclass
class MetricSample:
    """A single timestamped metric observation."""

    timestamp: float
    value: float


class MetricCollector:
    """Collects timestamped samples and computes summary statistics."""

    def __init__(self, name: str) -> None:
        self.name = name
        self._samples: list[MetricSample] = []

    def record(self, timestamp: float, value: float) -> None:
        self._samples.append(MetricSample(timestamp=timestamp, value=value))

    @property
    def samples(self) -> list[MetricSample]:
        return list(self._samples)

    @property
    def count(self) -> int:
        return len(self._samples)

    @property
    def values(self) -> list[float]:
        return [s.value for s in self._samples]

    def max(self) -> float:
        if not self._samples:
            return 0.0
        return max(s.value for s in self._samples)

    def min(self) -> float:
        if not self._samples:
            return 0.0
        return min(s.value for s in self._samples)

    def mean(self) -> float:
        if not self._samples:
            return 0.0
        return statistics.mean(s.value for s in self._samples)

    def std(self) -> float:
        if len(self._samples) < 2:
            return 0.0
        return statistics.stdev(s.value for s in self._samples)

    def rms(self) -> float:
        if not self._samples:
            return 0.0
        return math.sqrt(sum(s.value ** 2 for s in self._samples) / len(self._samples))

    def summary(self) -> dict[str, float]:
        return {
            f"{self.name}_count": float(self.count),
            f"{self.name}_max": self.max(),
            f"{self.name}_min": self.min(),
            f"{self.name}_mean": self.mean(),
            f"{self.name}_std": self.std(),
            f"{self.name}_rms": self.rms(),
        }


@dataclass
class LKAMetrics:
    """Aggregated lane-keeping validation metrics."""

    lateral_offset: MetricCollector = field(
        default_factory=lambda: MetricCollector("lateral_offset")
    )
    steering_angle: MetricCollector = field(
        default_factory=lambda: MetricCollector("steering_angle")
    )
    confidence: MetricCollector = field(
        default_factory=lambda: MetricCollector("confidence")
    )
    lane_departure_count: int = 0
    total_steps: int = 0
    no_detection_steps: int = 0

    def record_step(
        self,
        timestamp: float,
        lateral_offset: float,
        steering_angle: float,
        confidence: float,
        lane_width: float = 3.0,
    ) -> None:
        self.total_steps += 1
        self.lateral_offset.record(timestamp, abs(lateral_offset))
        self.steering_angle.record(timestamp, steering_angle)
        self.confidence.record(timestamp, confidence)

        half_lane = lane_width / 2.0
        if abs(lateral_offset) * half_lane > half_lane:
            self.lane_departure_count += 1

    def record_no_detection(self) -> None:
        self.total_steps += 1
        self.no_detection_steps += 1

    @property
    def lane_departure_rate(self) -> float:
        if self.total_steps == 0:
            return 0.0
        return self.lane_departure_count / self.total_steps

    @property
    def detection_rate(self) -> float:
        if self.total_steps == 0:
            return 0.0
        detected = self.total_steps - self.no_detection_steps
        return detected / self.total_steps

    def to_dict(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "total_steps": self.total_steps,
            "no_detection_steps": self.no_detection_steps,
            "detection_rate": self.detection_rate,
            "lane_departure_count": self.lane_departure_count,
            "lane_departure_rate": self.lane_departure_rate,
        }
        result.update(self.lateral_offset.summary())
        result.update(self.steering_angle.summary())
        result.update(self.confidence.summary())
        return result
