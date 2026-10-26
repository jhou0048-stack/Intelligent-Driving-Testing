#!/usr/bin/env python3
"""Compare ADAS scenario results against the regression baseline."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

from testing.regression import RegressionComparator


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check ADAS regressions")
    parser.add_argument(
        "--results",
        default="reports/scenario_results.json",
        help="Scenario result JSON",
    )
    parser.add_argument(
        "--baseline",
        default="config/regression_baseline.yaml",
        help="Regression baseline YAML",
    )
    parser.add_argument(
        "--output",
        default="reports/regression_comparison.csv",
        help="Comparison CSV",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result_payload = json.loads(Path(args.results).read_text(encoding="utf-8"))
    baseline = yaml.safe_load(Path(args.baseline).read_text(encoding="utf-8"))
    dataframe, passed = RegressionComparator(baseline).compare(result_payload)

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    dataframe.to_csv(output_path, index=False)

    print(dataframe.to_string(index=False))
    status = "PASS" if passed else "FAIL"
    print(f"[REGRESSION] {status} — report written to {output_path}")
    if not passed:
        sys.exit(1)


if __name__ == "__main__":
    main()
