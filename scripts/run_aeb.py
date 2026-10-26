#!/usr/bin/env python3
"""Run the AEB pipeline end-to-end against a Gazebo simulation.

Prerequisites:
    Terminal 1:  gz sim -s scenarios/worlds/follow_scenario.sdf
    Terminal 2:  gz sim -g   (optional)
    Terminal 3:  python scripts/run_aeb.py
"""

from __future__ import annotations

import argparse
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
    parser = argparse.ArgumentParser(description="Run AEB pipeline")
    parser.add_argument("--world", default="follow_scenario")
    parser.add_argument("--model", default="ego_vehicle")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--speed", type=float, default=0.5)
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    sim = GazeboSimulator()
    sim.connect()
    print("[AEB] Connected to Gazebo")

    req = wc_pb.WorldControl()
    req.pause = False
    sim.request(
        f"/world/{args.world}/control", req,
        "WorldControl", "Boolean", timeout_ms=10000,
    )
    print(f"[AEB] World '{args.world}' unpaused")

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

    # A single pipeline arbitrates commands so AEB always overrides LKA.
    pipeline = ADASPipeline(
        controller=controller,
        perception=perception,
        lane_keeper=LaneKeeper(kp=0.5, default_speed=args.speed),
        aeb_controller=AEBController(),
        default_speed=args.speed,
        loop_hz=20.0,
        enable_lka=True,
        enable_aeb=True,
    )
    aeb_active_steps = 0
    minimum_distance = float("inf")

    def record_step(result: object) -> None:
        nonlocal aeb_active_steps, minimum_distance
        if pipeline.is_aeb_active:
            aeb_active_steps += 1
        distance = getattr(result, "min_forward_distance", float("inf"))
        minimum_distance = min(minimum_distance, distance)

    pipeline.add_step_callback(record_step)

    # Wait for sensors
    for _ in range(50):
        if camera.get_image() is not None:
            break
        time.sleep(0.1)

    print(f"[AEB] Starting LKA + AEB pipelines (speed={args.speed} m/s)")
    pipeline.start()

    time.sleep(args.duration)

    pipeline.stop()

    print("\n[AEB] === Results ===")
    print(f"  Perception/control steps: {pipeline.step_count}")
    print(f"  AEB active steps: {aeb_active_steps}")
    print(f"  Minimum forward distance: {minimum_distance:.2f}m")

    controller.stop()
    lidar.disconnect()
    camera.disconnect()
    controller.disconnect()
    sim.disconnect()
    print("[AEB] Done")


if __name__ == "__main__":
    main()
