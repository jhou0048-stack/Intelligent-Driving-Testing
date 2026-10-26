#!/usr/bin/env python3
"""Validate an Ultralytics YOLO model against a labelled image manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import pandas as pd

from perception.object_detector import YOLODetector
from testing.perception_validator import ObjectDetectionValidator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate YOLO detections")
    parser.add_argument("--manifest", required=True, help="Ground-truth CSV")
    parser.add_argument("--image-root", default=".", help="Image root folder")
    parser.add_argument("--model", default="yolov8n.pt", help="YOLO weights")
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--iou", type=float, default=0.5)
    parser.add_argument(
        "--output",
        default="reports/yolo_validation.csv",
        help="Per-image CSV output",
    )
    parser.add_argument(
        "--json",
        default="reports/yolo_validation.json",
        help="Summary JSON output",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    manifest = pd.read_csv(args.manifest)
    detector = YOLODetector(
        model_path=args.model,
        confidence_threshold=args.confidence,
    )
    validator = ObjectDetectionValidator(detector, iou_threshold=args.iou)
    results, summary = validator.evaluate(manifest, args.image_root)

    csv_path = Path(args.output)
    json_path = Path(args.json)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    json_path.parent.mkdir(parents=True, exist_ok=True)
    results.to_csv(csv_path, index=False)
    json_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(
        "[YOLO-VAL] "
        f"images={summary['images']} "
        f"precision={summary['precision']:.3f} "
        f"recall={summary['recall']:.3f} "
        f"f1={summary['f1_score']:.3f} "
        f"latency={summary['mean_latency_ms']:.2f}ms"
    )
    print(f"[YOLO-VAL] CSV report written to {csv_path}")
    print(f"[YOLO-VAL] JSON report written to {json_path}")


if __name__ == "__main__":
    main()
