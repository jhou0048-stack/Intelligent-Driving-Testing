#!/usr/bin/env python3
"""Run the LKA pipeline end-to-end against a Gazebo simulation.

Prerequisites:
    Terminal 1:  gz sim -s scenarios/worlds/simple_road.sdf
    Terminal 2:  gz sim -g   (optional, for visualisation)
    Terminal 3:  python scripts/run_lka.py

The script unpauses the world, starts the perception → planning → control
loop, runs for a configurable duration, then reports the result.
"""

from __future__ import annotations

import argparse
import math
import time

import gz.msgs10.world_control_pb2 as wc_pb

from control import GazeboAckermannController
from perception.camera import GazeboCamera
from perception.lane_detector import LaneDetector
from planning.lane_keeper import LaneKeeper
from planning.lka_pipeline import LKAPipeline
from simulation import GazeboSimulator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run LKA pipeline")
    parser.add_argument("--world", default="simple_road")
    parser.add_argument("--model", default="ego_vehicle")
    parser.add_argument("--duration", type=float, default=10.0,
                        help="Run duration in seconds")
    parser.add_argument("--speed", type=float, default=0.5,
                        help="Target speed in m/s")
    parser.add_argument("--kp", type=float, default=0.5,
                        help="Proportional gain for lane keeping")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    sim = GazeboSimulator()
    sim.connect()
    print(f"[LKA] Connected to Gazebo transport")

    # Unpause the simulation
    req = wc_pb.WorldControl()
    req.pause = False
    resp = sim.request(
        f"/world/{args.world}/control", req,
        "WorldControl", "Boolean", timeout_ms=10000,
    )
    if resp is True or (hasattr(resp, "data") and resp.data):
        print(f"[LKA] World '{args.world}' unpaused")
    else:
        print(f"[LKA] Warning: could not unpause world")

    controller = GazeboAckermannController(
        model_name=args.model, simulator=sim, own_simulator=False,
    )
    controller.connect()

    camera = GazeboCamera(
        model_name=args.model,
        world_name=args.world,
        simulator=sim,
        own_simulator=False,
    )
    camera.connect()

    lane_detector = LaneDetector()
    lane_keeper = LaneKeeper(
        kp=args.kp,
        default_speed=args.speed,
        lookahead=5.0,
    )

    pipeline = LKAPipeline(
        simulator=sim,
        controller=controller,
        camera=camera,
        lane_detector=lane_detector,
        lane_keeper=lane_keeper,
        loop_hz=10.0,
    )

    # Subscribe to odometry for pose updates
    controller.subscribe_odometry(pipeline.update_pose_from_odometry)

    print(f"[LKA] Starting LKA pipeline (speed={args.speed} m/s, kp={args.kp})")
    print(f"[LKA] Running for {args.duration}s ...")

    # Wait for first camera frame
    for _ in range(50):
        if camera.get_image() is not None:
            break
        time.sleep(0.1)
    else:
        print("[LKA] Warning: no camera frame received after 5s, starting anyway")

    pipeline.start()
    time.sleep(args.duration)
    pipeline.stop()

    print(f"\n[LKA] === Results ===")
    print(f"  Steps executed: {pipeline.step_count}")
    det = pipeline.last_detection
    if det:
        print(f"  Last lateral offset: {det.lateral_offset:.4f}")
        print(f"  Left line detected:  {det.left_line_detected}")
        print(f"  Right line detected: {det.right_line_detected}")
        print(f"  Confidence:          {det.confidence:.2f}")
    else:
        print("  No detection results (camera may not have produced frames)")

    pose = pipeline._pose
    print(f"  Final pose: x={pose['x']:.3f}  y={pose['y']:.3f}  "
          f"heading={math.degrees(pose['heading']):.1f} deg")

    controller.stop()
    camera.disconnect()
    controller.disconnect()
    sim.disconnect()
    print("[LKA] Done")


if __name__ == "__main__":
    main()
