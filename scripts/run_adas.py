#!/usr/bin/env python3
"""Run the integrated ADAS pipeline (LKA + AEB + Perception) end-to-end.

Prerequisites:
    Terminal 1:  gz sim -s scenarios/worlds/follow_scenario.sdf
    Terminal 2:  gz sim -g   (optional)
    Terminal 3:  python scripts/run_adas.py
"""

from __future__ import annotations

import argparse
import json
import time

import gz.msgs10.world_control_pb2 as wc_pb

from control import GazeboAckermannController
from perception.camera import GazeboCamera
from perception.lane_detector import LaneDetector
from perception.lidar import GazeboLidar
from perception.perception_pipeline import PerceptionPipeline
from planning.adas_pipeline import ADASPipeline
from planning.aeb_controller import AEBController
from planning.lane_keeper import LaneKeeper
from simulation import GazeboSimulator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run integrated ADAS pipeline")
    parser.add_argument("--world", default="follow_scenario")
    parser.add_argument("--model", default="ego_vehicle")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--speed", type=float, default=0.5)
    parser.add_argument("--output", default=None, help="JSON output path")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    sim = GazeboSimulator()
    sim.connect()
    print("[ADAS] Connected to Gazebo")

    req = wc_pb.WorldControl()
    req.pause = False
    sim.request(
        f"/world/{args.world}/control", req,
        "WorldControl", "Boolean", timeout_ms=10000,
    )

    controller = GazeboAckermannController(
        model_name=args.model, simulator=sim, own_simulator=False,
    )
    controller.connect()

    camera = GazeboCamera(
        model_name=args.model, world_name=args.world,
        simulator=sim, own_simulator=False,
    )
    camera.connect()

    lidar = GazeboLidar(
        model_name=args.model, world_name=args.world,
        simulator=sim, own_simulator=False,
    )
    lidar.connect()

    perception = PerceptionPipeline(
        camera=camera,
        lidar=lidar,
        lane_detector=LaneDetector(),
    )

    pipeline = ADASPipeline(
        controller=controller,
        perception=perception,
        lane_keeper=LaneKeeper(kp=0.5, default_speed=args.speed),
        aeb_controller=AEBController(),
        default_speed=args.speed,
        loop_hz=10.0,
    )

    step_log: list[dict] = []

    def log_step(result):
        step_log.append({
            "timestamp": result.timestamp,
            "lateral_offset": result.lateral_offset,
            "min_forward_distance": result.min_forward_distance,
            "objects": len(result.objects),
            "aeb_active": pipeline.is_aeb_active,
        })

    pipeline.add_step_callback(log_step)

    for _ in range(50):
        if camera.get_image() is not None:
            break
        time.sleep(0.1)

    print(f"[ADAS] Running for {args.duration}s (speed={args.speed})")
    pipeline.start()
    time.sleep(args.duration)
    pipeline.stop()

    print(f"\n[ADAS] === Results ===")
    print(f"  Steps: {pipeline.step_count}")
    print(f"  AEB active at end: {pipeline.is_aeb_active}")
    aeb_steps = sum(1 for s in step_log if s.get("aeb_active"))
    print(f"  AEB active steps: {aeb_steps}/{len(step_log)}")

    if args.output:
        with open(args.output, "w") as f:
            json.dump({"steps": step_log, "total": pipeline.step_count}, f, indent=2)
        print(f"  Log written to {args.output}")

    lidar.disconnect()
    camera.disconnect()
    controller.disconnect()
    sim.disconnect()
    print("[ADAS] Done")


if __name__ == "__main__":
    main()
