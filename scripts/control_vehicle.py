"""Manual verification script for vehicle control.

Run this script while the simple_road world server is running:

    Terminal 1: gz sim -s scenarios/worlds/simple_road.sdf
    Terminal 2: python scripts/control_vehicle.py

The script unpauses the simulation, drives the ego vehicle forward, steers
left, then brings it to a stop while printing odometry samples.
"""

from __future__ import annotations

import argparse
import time

import gz.msgs10.world_control_pb2 as world_control_pb

from control import GazeboAckermannController
from simulation import GazeboSimulator


def _unpause_world(sim: GazeboSimulator, world_name: str) -> bool:
    """Resume a paused Gazebo world server."""
    request = world_control_pb.WorldControl()
    request.pause = False
    response = sim.request(
        f"/world/{world_name}/control",
        request,
        "WorldControl",
        "Boolean",
        timeout_ms=10000,
    )
    # gz-transport may return the Boolean response as a native Python bool.
    return response is True or (hasattr(response, "data") and response.data)


def main() -> None:
    parser = argparse.ArgumentParser(description="Manual vehicle-control smoke test")
    parser.add_argument(
        "--world",
        default="simple_road",
        help="Name of the Gazebo world to unpause (default: simple_road)",
    )
    parser.add_argument(
        "--model",
        default="ego_vehicle",
        help="Model name to command (default: ego_vehicle)",
    )
    args = parser.parse_args()

    latest_pose: dict[str, float] = {}

    def on_odometry(msg: object) -> None:
        pose = msg.pose
        latest_pose["x"] = pose.position.x
        latest_pose["y"] = pose.position.y
        latest_pose["z"] = pose.position.z

    with GazeboSimulator() as sim:
        print(f"Unpausing world '{args.world}'...")
        if not _unpause_world(sim, args.world):
            print("Warning: failed to unpause the world; simulation may still be paused")
        else:
            print("World unpaused")

        with GazeboAckermannController(
            model_name=args.model, simulator=sim, own_simulator=False
        ) as controller:
            print(f"Connected to {controller.model_name}")
            print(f"Command topic: {controller.cmd_topic}")
            print(f"Odometry topic: {controller.odometry_topic}")

            controller.subscribe_odometry(on_odometry)
            print("Subscribed to odometry")

            # Wait a moment for physics to settle after unpause.
            time.sleep(0.5)

            # Drive forward slowly.
            print("Driving forward...")
            controller.send_command(speed=0.5, steering_angle=0.0)
            time.sleep(3.0)

            # Gentle left turn while moving.
            print("Turning left...")
            controller.send_command(speed=0.3, steering_angle=0.3)
            time.sleep(3.0)

            # Stop.
            print("Stopping...")
            controller.stop()
            time.sleep(1.0)

            print("Final pose:", latest_pose)


if __name__ == "__main__":
    main()
