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
from planning.aeb_controller import AEBController
from planning.aeb_pipeline import AEBPipeline
from planning.lane_keeper import LaneKeeper
from planning.lka_pipeline import LKAPipeline
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

    # LKA for lane keeping
    lka_pipeline = LKAPipeline(
        simulator=sim, controller=controller, camera=camera,
        lane_detector=LaneDetector(),
        lane_keeper=LaneKeeper(kp=0.5, default_speed=args.speed),
        loop_hz=10.0,
    )

    # AEB overrides LKA when obstacle detected
    aeb_pipeline = AEBPipeline(
        controller=controller,
        aeb_controller=AEBController(),
        lidar=lidar,
        current_speed_fn=lambda: args.speed,
        loop_hz=20.0,
    )

    # Wait for sensors
    for _ in range(50):
        if camera.get_image() is not None:
            break
        time.sleep(0.1)

    print(f"[AEB] Starting LKA + AEB pipelines (speed={args.speed} m/s)")
    lka_pipeline.start()
    aeb_pipeline.start()

    time.sleep(args.duration)

    aeb_pipeline.stop()
    lka_pipeline.stop()

    print(f"\n[AEB] === Results ===")
    print(f"  AEB steps: {aeb_pipeline.step_count}")
    print(f"  Brake activations: {aeb_pipeline.brake_count}")
    d = aeb_pipeline.last_decision
    if d:
        print(f"  Last decision: brake={d.should_brake}, "
              f"dist={d.min_obstacle_distance:.2f}m, "
              f"TTC={d.time_to_collision:.2f}s")
    print(f"  LKA steps: {lka_pipeline.step_count}")

    controller.stop()
    lidar.disconnect()
    camera.disconnect()
    controller.disconnect()
    sim.disconnect()
    print("[AEB] Done")


if __name__ == "__main__":
    main()
