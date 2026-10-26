#!/usr/bin/env python3
"""Run vehicle, pedestrian, and integrated ADAS braking scenarios."""

from __future__ import annotations

import argparse
import subprocess
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run AEB validation")
    parser.add_argument("--config", default="config/scenarios.yaml")
    parser.add_argument("--output", default="reports/aeb_validation.csv")
    parser.add_argument("--json", default="reports/aeb_validation.json")
    parser.add_argument("--show-gui", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    command = [
        sys.executable,
        "scripts/run_scenarios.py",
        "--config",
        args.config,
        "--output",
        args.output,
        "--json",
        args.json,
        "--scenario",
        "aeb_obstacle_ahead",
        "--scenario",
        "aeb_pedestrian_crossing",
        "--scenario",
        "integrated_adas_obstacle_stop",
    ]
    if args.show_gui:
        command.append("--show-gui")
    raise SystemExit(subprocess.run(command, check=False).returncode)


if __name__ == "__main__":
    main()
