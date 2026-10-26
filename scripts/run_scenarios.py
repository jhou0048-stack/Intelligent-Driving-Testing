#!/usr/bin/env python3
"""Batch-run all scenarios from a YAML config file.

Usage:
    python scripts/run_scenarios.py --config config/scenarios.yaml
    python scripts/run_scenarios.py --config config/scenarios.yaml --output results.csv
"""

from __future__ import annotations

import argparse
import sys
import time

from testing.config_loader import build_assertions, load_scenario_suite
from testing.report import ScenarioResult, TestReport
from testing.runner import GazeboScenarioRunner


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Batch scenario runner")
    parser.add_argument("--config", default="config/scenarios.yaml",
                        help="YAML scenario config file")
    parser.add_argument("--output", default="reports/scenario_results.csv",
                        help="CSV output path")
    parser.add_argument("--json", default="reports/scenario_results.json",
                        help="JSON result output path")
    parser.add_argument("--scenarios-dir", default="scenarios",
                        help="Directory containing worlds and models")
    parser.add_argument("--show-gui", action="store_true",
                        help="Launch the Gazebo GUI while scenarios run")
    parser.add_argument("--scenario", action="append", default=[],
                        help="Run only a named scenario (repeatable)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Parse and validate scenarios without running")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scenarios = load_scenario_suite(args.config)
    if args.scenario:
        requested = set(args.scenario)
        scenarios = [sc for sc in scenarios if sc.name in requested]
        found = {sc.name for sc in scenarios}
        missing = requested - found
        if missing:
            print(f"[BATCH] Unknown scenario(s): {', '.join(sorted(missing))}")
            sys.exit(2)
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

    for sc in scenarios:
        print(f"\n[BATCH] Running {sc.name} ...")
        started_at = time.monotonic()
        try:
            with GazeboScenarioRunner(
                scenarios_dir=args.scenarios_dir,
                headless=not args.show_gui,
            ) as runner:
                runner.load_scenario(sc)
                result = runner.execute()
        except Exception as exc:
            result = ScenarioResult(
                scenario_name=sc.name,
                passed=False,
                duration=time.monotonic() - started_at,
                error=str(exc),
            )

        report.add_result(result)
        status = "PASS" if result.passed else "FAIL"
        print(f"[BATCH] {status} {sc.name} ({result.duration:.2f}s)")
        if result.error:
            print(f"        {result.error}")

    report.to_csv(args.output)
    print(f"\n[BATCH] CSV report written to {args.output}")
    if args.json:
        report.to_json(args.json)
        print(f"[BATCH] JSON report written to {args.json}")
    print(f"[BATCH] {report.summary()}")
    if not report.all_passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
