# Simulation-Based ADAS Testing & Validation Framework

A portfolio project for building repeatable, simulation-based validation workflows for advanced driver-assistance systems (ADAS) and autonomous-driving systems.

This initial version contains only the project foundation. It does **not** implement vehicle-control, perception, planning, or other autonomous-driving behavior.

## Goals

- Organize CARLA simulation scenarios and test infrastructure cleanly.
- Keep simulation, perception, planning, control, and validation concerns independent.
- Support repeatable PyTest-based validation and report generation.
- Provide clear extension points for OpenCV, YOLO, and Pandas workflows.

## Project structure

```text
.
├── config/             # Version-controlled configuration
├── data/               # Local input/output datasets (ignored by Git)
├── docs/               # Architecture and project documentation
├── logs/               # Runtime logs (ignored by Git)
├── reports/            # Generated test and analysis reports (ignored by Git)
├── scenarios/          # Scenario definitions and supporting documentation
├── scripts/            # Developer and automation entry points
├── src/
│   ├── control/        # Vehicle-control domain package
│   ├── perception/     # Computer-vision and detection domain package
│   ├── planning/       # Route and behavior-planning domain package
│   ├── simulation/     # CARLA integration and simulation lifecycle package
│   └── testing/        # Shared validation, metrics, and test-support package
└── tests/               # PyTest test suite
```

See [docs/architecture.md](docs/architecture.md) for the intended dependency boundaries.

## Prerequisites

- Python 3.10 or later
- A CARLA Simulator installation compatible with the Python client version in `requirements.txt`

The CARLA server is installed separately. Its server and Python client versions should match.

## Setup

Create and activate a virtual environment:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

## Verify the scaffold

```bash
pytest
```

## Current scope

Only the repository structure, package boundaries, configuration foundation, and a package-import smoke test are present. Scenario execution and ADAS algorithms will be added in later milestones.

