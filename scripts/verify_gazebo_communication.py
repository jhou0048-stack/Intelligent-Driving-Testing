"""Manual verification script for Python ↔ Gazebo transport communication.

Run this script while a Gazebo server is active:

    Terminal 1: gz sim -s shapes.sdf
    Terminal 2: python scripts/verify_gazebo_communication.py

The script publishes a ``StringMsg`` on ``/adas/verify`` and prints any
messages it receives back.
"""

from __future__ import annotations

import time

import gz.msgs10.stringmsg_pb2 as stringmsg

from simulation import GazeboSimulator


def main() -> None:
    topic = "/adas/verify"
    msg_type = "StringMsg"

    received: list[str] = []

    def callback(msg: object) -> None:
        received.append(msg.data)
        print(f"Received on {topic}: {msg.data}")

    with GazeboSimulator() as sim:
        print("Connected to Gazebo transport")
        print("Advertised topics:", sim.get_topic_names())

        sim.subscribe(topic, msg_type, callback)
        print(f"Subscribed to {topic}")

        for i in range(5):
            message = stringmsg.StringMsg()
            message.data = f"hello {i}"
            sim.publish(topic, msg_type, message)
            print(f"Published: hello {i}")
            time.sleep(0.2)

        time.sleep(0.5)
        print(f"Received {len(received)} messages back")


if __name__ == "__main__":
    main()
