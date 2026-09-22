#!/usr/bin/env python3
"""Batch-run all scenarios from a YAML config file.

Usage:
    python scripts/run_scenarios.py --config config/scenarios.yaml
    python scripts/run_scenarios.py --config config/scenarios.yaml --output results.csv
"""

from __future__ import annotations

import argparse
import json
import sys

from testing.config_loader import build_assertions, load_scenario_suite
from testing.report import TestReport


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Batch scenario runner")
    parser.add_argument("--config", default="config/scenarios.yaml",
                        help="YAML scenario config file")
    parser.add_argument("--output", default="scenario_results.csv",
                        help="CSV output path")
    parser.add_argument("--json", default=None,
                        help="Optional JSON summary output")
    parser.add_argument("--dry-run", action="store_true",
                        help="Parse and validate scenarios without running")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scenarios = load_scenario_suite(args.config)
    report = TestReport()

    print(f"[BATCH] Loaded {len(scenarios)} scenarios from {args.config}")

    for sc in scenarios:
        assertions = build_assertions(sc.pass_conditions)
        print(f"  - {sc.name}: world={sc.world}, "
              f"actions={len(sc.actions)}, "
              f"assertions={len(assertions)}, "
              f"timeout={sc.timeout}s")

    if args.dry_run:
        print("[BATCH] Dry run complete — all scenarios parsed successfully")
        return

    print("\n[BATCH] Note: Full execution requires a running Gazebo instance.")
    print("[BATCH] Use 'gz sim -s scenarios/worlds/<world>.sdf' to start.")
    print(f"[BATCH] Scenarios validated. Output would go to: {args.output}")

    if args.json:
        summary = {
            "total": len(scenarios),
            "scenarios": [
                {
                    "name": sc.name,
                    "world": sc.world,
                    "actions": len(sc.actions),
                    "conditions": len(sc.pass_conditions),
                }
                for sc in scenarios
            ],
        }
        with open(args.json, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"[BATCH] Scenario summary written to {args.json}")


if __name__ == "__main__":
    main()
