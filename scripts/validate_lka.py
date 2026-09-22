#!/usr/bin/env python3
"""Run the LKA validation suite against a Gazebo simulation.

Prerequisites:
    Terminal 1:  gz sim -s scenarios/worlds/simple_road.sdf
    Terminal 2:  gz sim -g   (optional)
    Terminal 3:  python scripts/validate_lka.py

Runs each scenario in the default validation suite, collects metrics,
evaluates pass/fail conditions, and writes a CSV report.
"""

from __future__ import annotations

import argparse
import json
import math
import time

import gz.msgs10.world_control_pb2 as wc_pb

from control import GazeboAckermannController
from perception.camera import GazeboCamera
from perception.lane_detector import LaneDetector
from planning.lane_keeper import LaneKeeper
from planning.lka_pipeline import LKAPipeline
from simulation import GazeboSimulator
from testing.lka_validator import LKAValidationSuite, LKAValidator
from testing.report import TestReport


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run LKA validation suite")
    parser.add_argument("--world", default="simple_road")
    parser.add_argument("--model", default="ego_vehicle")
    parser.add_argument("--output", default="lka_validation_report.csv",
                        help="CSV output path")
    parser.add_argument("--json", default=None,
                        help="Optional JSON summary output path")
    return parser.parse_args()


def run_scenario(
    sim: GazeboSimulator,
    scenario: dict,
    model: str,
    world: str,
) -> None:
    """Run one scenario from the validation suite."""
    validator: LKAValidator = scenario["_validator"]

    controller = GazeboAckermannController(
        model_name=model, simulator=sim, own_simulator=False,
    )
    controller.connect()

    camera = GazeboCamera(
        model_name=model, world_name=world,
        simulator=sim, own_simulator=False,
    )
    camera.connect()

    lane_detector = LaneDetector()
    lane_keeper = LaneKeeper(
        kp=scenario.get("kp", 0.5),
        default_speed=scenario.get("speed", 0.5),
        lookahead=5.0,
    )

    pipeline = LKAPipeline(
        simulator=sim, controller=controller, camera=camera,
        lane_detector=lane_detector, lane_keeper=lane_keeper,
        loop_hz=10.0,
    )

    # Hook validation into the pipeline
    original_step = pipeline._step

    def instrumented_step() -> None:
        image = camera.get_image()
        if image is None:
            validator.on_no_detection()
            return

        detection = lane_detector.detect(image)
        pipeline._last_detection = detection

        perception_data = {"lateral_offset": detection.lateral_offset}
        waypoints = lane_keeper.plan(pipeline._pose, perception_data)

        steering_angle = 0.0
        if waypoints:
            speed, steering_angle = lane_keeper.compute_steering(
                pipeline._pose, waypoints[0]
            )
            controller.send_command(speed=speed, steering_angle=steering_angle)

        pipeline._step_count += 1

        validator.on_step(
            lateral_offset=detection.lateral_offset,
            steering_angle=steering_angle,
            confidence=detection.confidence,
        )

    pipeline._step = instrumented_step  # type: ignore[assignment]

    # Wait for camera
    for _ in range(50):
        if camera.get_image() is not None:
            break
        time.sleep(0.1)

    duration = scenario.get("duration", 10.0)
    validator.start()
    pipeline.start()
    time.sleep(duration)
    pipeline.stop()

    camera.disconnect()
    controller.disconnect()


def main() -> None:
    args = parse_args()
    suite = LKAValidationSuite.default_suite()
    report = TestReport()

    sim = GazeboSimulator()
    sim.connect()
    print("[LKA-VAL] Connected to Gazebo")

    # Unpause
    req = wc_pb.WorldControl()
    req.pause = False
    sim.request(
        f"/world/{args.world}/control", req,
        "WorldControl", "Boolean", timeout_ms=10000,
    )
    print(f"[LKA-VAL] World '{args.world}' unpaused")

    for scenario in suite.scenarios:
        name = scenario["name"]
        print(f"\n[LKA-VAL] === Running: {name} ===")

        validator = LKAValidator(
            scenario_name=name,
            assertions=scenario.get("assertions", []),
            lane_width=3.0,
        )
        scenario["_validator"] = validator

        run_scenario(sim, scenario, args.model, args.world)
        result = validator.evaluate()
        suite.record_result(result)
        report.add_result(result)

        status = "PASS" if result.passed else "FAIL"
        print(f"  [{status}] {name}")
        if result.error:
            print(f"    Error: {result.error}")
        m = result.metrics
        print(f"    Steps: {m.get('total_steps', 0)}, "
              f"Detection rate: {m.get('detection_rate', 0):.1%}, "
              f"Max lateral offset: {m.get('lateral_offset_max', 0):.4f}, "
              f"Lane departure rate: {m.get('lane_departure_rate', 0):.1%}")

    sim.disconnect()

    report.to_csv(args.output)
    print(f"\n[LKA-VAL] Report written to {args.output}")

    summary = suite.summary()
    print(f"[LKA-VAL] {summary['passed']}/{summary['total']} passed "
          f"({summary['pass_rate']:.0%})")

    if args.json:
        with open(args.json, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"[LKA-VAL] JSON summary written to {args.json}")


if __name__ == "__main__":
    main()
