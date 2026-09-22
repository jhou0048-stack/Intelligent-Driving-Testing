# Simulation-Based ADAS Testing & Validation Framework

A portfolio project for building repeatable, simulation-based validation workflows for advanced driver-assistance systems (ADAS) and autonomous-driving systems using Gazebo Harmonic.

## Features

- **Lane Keeping Assist (LKA)** — OpenCV-based lane detection with proportional steering control
- **Automatic Emergency Braking (AEB)** — Lidar/camera-based obstacle detection with emergency stop
- **Integrated ADAS Pipeline** — Fused perception (camera + lidar + YOLO) driving LKA + AEB together
- **Validation Framework** — Automated metric collection, pass/fail assertions, CSV/JSON reporting
- **YAML Scenario Config** — Define and batch-run test scenarios from config files
- **200+ Unit Tests** — Comprehensive coverage of all modules

## Architecture

```text
┌─────────────┐    ┌──────────────┐    ┌──────────────┐
│  Perception  │───▶│   Planning   │───▶│   Control    │
│  Camera      │    │  LKA Pipeline│    │  Ackermann   │
│  Lidar       │    │  AEB Pipeline│    │  Controller  │
│  LaneDetect  │    │  ADAS Pipeline│   └──────┬───────┘
│  YOLO        │    └──────────────┘           │
│  Fusion      │                               ▼
└──────┬───────┘                        ┌──────────────┐
       │                                │  Simulation  │
       └───────────────────────────────▶│  Gazebo      │
                                        └──────────────┘
       ┌──────────────┐
       │   Testing    │  Scenarios, Assertions, Metrics,
       │   Framework  │  Validators, Reports, Config Loader
       └──────────────┘
```

## Project Structure

```text
.
├── config/             # YAML configuration (default.yaml, scenarios.yaml)
├── scenarios/
│   ├── models/         # SDF models (ego_vehicle, obstacle_vehicle, pedestrian)
│   └── worlds/         # SDF worlds (simple_road, follow, pedestrian, lane_change)
├── scripts/            # Entry-point scripts (run_lka, run_aeb, run_adas, etc.)
├── src/
│   ├── control/        # Vehicle control (Ackermann controller)
│   ├── perception/     # Camera, Lidar, LaneDetector, YOLO, PerceptionPipeline
│   ├── planning/       # LKA, AEB, ADAS pipelines, BehaviorPlanner
│   ├── simulation/     # Gazebo Harmonic client
│   └── testing/        # Assertions, Metrics, Validators, Reports, Config loader
└── tests/              # PyTest suite (200+ tests)
```

## Prerequisites

- Python 3.12+
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install_osx) (macOS: `brew install gz-harmonic`)
- OpenCV (`pip install opencv-python`)

## Setup

```bash
python3.12 -m venv .venv312
source .venv312/bin/activate
pip install -r requirements.txt
pip install -e .
```

> **macOS note:** Gazebo Python bindings come from Homebrew at `/opt/homebrew/lib/python3.12/site-packages`. A `.pth` file in the venv makes them available.

## Quick Start

### Run Tests

```bash
pytest tests/ -v
```

### Run LKA Pipeline

```bash
# Terminal 1
gz sim -s scenarios/worlds/simple_road.sdf
# Terminal 2 (optional)
gz sim -g
# Terminal 3
python scripts/run_lka.py --speed 0.5 --duration 10
```

### Run AEB Pipeline

```bash
gz sim -s scenarios/worlds/follow_scenario.sdf
python scripts/run_aeb.py --speed 0.5 --duration 15
```

### Run Integrated ADAS

```bash
gz sim -s scenarios/worlds/follow_scenario.sdf
python scripts/run_adas.py --speed 0.5 --duration 15 --output results.json
```

### Run LKA Validation Suite

```bash
gz sim -s scenarios/worlds/simple_road.sdf
python scripts/validate_lka.py --output report.csv --json summary.json
```

### Batch Run Scenarios

```bash
python scripts/run_scenarios.py --config config/scenarios.yaml --dry-run
```

## Modules

| Package | Key Classes |
|---------|-------------|
| `simulation` | `Simulator` ABC, `GazeboSimulator` |
| `control` | `VehicleController` ABC, `GazeboAckermannController` |
| `perception` | `Camera`, `Lidar`, `LaneDetector`, `ObjectDetector`, `PerceptionPipeline` |
| `planning` | `LaneKeeper`, `AEBController`, `LKAPipeline`, `AEBPipeline`, `ADASPipeline` |
| `testing` | `ScenarioRunner`, `LKAValidator`, `AEBValidator`, `MetricCollector`, `TestReport`, `ResultAnalyzer` |

## Testing

The project includes 200+ unit tests covering all modules:

```bash
# Run all tests with verbose output
PYTHONPATH=src python -m pytest tests/ -v

# Run specific test file
PYTHONPATH=src python -m pytest tests/test_aeb.py -v

# Run with coverage
PYTHONPATH=src python -m pytest tests/ --cov=src --cov-report=term-missing
```

## macOS Notes

- `gz sim` requires separate terminals for server (`gz sim -s`) and GUI (`gz sim -g`)
- Gazebo starts paused; scripts send `WorldControl{pause: false}` automatically
- `gpu_lidar` requires Metal/ogre2; falls back to CPU `lidar` if unavailable
