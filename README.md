# Intelligent Driving Testing

A simulation-based ADAS testing and validation framework built with Python,
Gazebo Harmonic, PyTest, OpenCV, YOLO, and Pandas. The project focuses on
repeatable scenario execution, measurable safety criteria, fault injection,
and regression reporting for ADAS validation roles.

## What is validated

- Lane Keeping Assist (LKA) using Gazebo camera frames and OpenCV lane detection
- Automatic Emergency Braking (AEB) using Gazebo lidar ranges
- Integrated LKA + AEB control with AEB command priority
- Real vehicle odometry, distance, speed, lateral deviation, and collision data
- YAML pass/fail conditions with CSV and JSON reports
- Camera and lidar fault injection with time-bounded fault schedules
- YOLO precision, recall, F1, false-positive, false-negative, and latency metrics
- Regression comparison against version-controlled metric limits

The automated suite contains five Gazebo scenarios: straight-road LKA, vehicle
AEB, pedestrian AEB, integrated ADAS obstacle stopping, and degraded-mode LKA
with camera dropout.

## Architecture

```text
Gazebo worlds and vehicle model
        │
        ├── Camera ── OpenCV / optional YOLO ─┐
        ├── Lidar ── obstacle distance ───────┤
        └── Odometry / contact telemetry ─┐   │
                                          │   ▼
                                     ADAS pipeline
                                          │
                                   command arbitration
                                   AEB > LKA > cruise
                                          │
                                          ▼
                                  Ackermann controller
                                          │
                                          ▼
Scenario Runner ── metrics ── assertions ── PASS / FAIL
                                          │
                                     CSV / JSON
                                          │
                                  regression comparison
```

Detailed boundaries and data flow are documented in
[`docs/architecture.md`](docs/architecture.md).

## Project structure

```text
.
├── config/                 # Scenarios, regression limits, and CI template
├── scenarios/
│   ├── models/             # Ego, obstacle, and pedestrian SDF models
│   └── worlds/             # Road and hazard test worlds
├── scripts/                # CLI entry points
├── src/
│   ├── control/            # Vehicle-command interfaces and Gazebo controller
│   ├── perception/         # Camera, lidar, lanes, YOLO, sensor fusion
│   ├── planning/           # LKA, AEB, and integrated ADAS pipelines
│   ├── simulation/         # Gazebo transport and telemetry adapters
│   └── testing/            # Runner, assertions, faults, reports, regression
└── tests/                  # Deterministic PyTest suite
```

Generated content under `data/`, `logs/`, and `reports/` is ignored by Git.
Model weights such as `*.pt` are also intentionally excluded.

## Prerequisites

- Python 3.12+
- Gazebo Harmonic
- macOS Apple Silicon or Linux with compatible Gazebo Python bindings

On macOS:

```bash
brew tap osrf/simulation
brew install gz-harmonic
```

Gazebo's Homebrew Python bindings are normally installed under
`/opt/homebrew/lib/python3.12/site-packages`. Make that directory visible inside
the project virtual environment if `import gz.transport13` is not found.

## Setup

```bash
python3.12 -m venv .venv312
source .venv312/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install -e .
```

Verify the environment:

```bash
python --version
gz sim --versions
python -c "import gz.transport13; print('Gazebo Python bindings ready')"
pytest -q
```

## Run the complete validation suite

The batch runner starts and stops its own Gazebo server for every scenario:

```bash
python scripts/run_scenarios.py \
  --config config/scenarios.yaml \
  --output reports/scenario_results.csv \
  --json reports/scenario_results.json
```

Use `--show-gui` to display Gazebo, `--dry-run` to validate YAML without running,
or select individual scenarios:

```bash
python scripts/run_scenarios.py \
  --scenario aeb_obstacle_ahead \
  --scenario aeb_pedestrian_crossing
```

The command exits with status `1` if any scenario or assertion fails, making it
suitable for automation.

## Check regressions

After running the complete scenario suite:

```bash
python scripts/check_regression.py \
  --results reports/scenario_results.json \
  --baseline config/regression_baseline.yaml
```

Regression limits are version controlled in
[`config/regression_baseline.yaml`](config/regression_baseline.yaml).

## Fault injection

Faults are declared inside a scenario's `sut` section:

```yaml
sut:
  mode: lka
  duration: 8.0
  speed: 0.5
  faults:
    - target: camera
      type: dropout
      start_time: 3.0
      duration: 2.0
```

Supported faults:

- Camera: `dropout`, `frozen_frame`, `blur`, `noise`
- Lidar: `dropout`, `frozen_scan`, `range_bias`, `noise`

## YOLO validation

YOLO weights are supplied at runtime and are never committed. A ground-truth CSV
must contain:

```text
image_path,class_name,x1,y1,x2,y2
frame_001.png,car,120,180,320,350
```

Run validation with:

```bash
python scripts/validate_yolo.py \
  --manifest data/labels.csv \
  --image-root data/images \
  --model yolov8n.pt
```

To enable YOLO in the live integrated pipeline:

```bash
python scripts/run_adas.py --yolo-model yolov8n.pt --output reports/adas.json
```

## Tests and coverage

```bash
python -m pytest -q
python -m pytest --cov=src --cov-report=term-missing
```

Latest local verification:

```text
247 passed
88% total statement coverage
5/5 Gazebo validation scenarios passed
```

A GitHub Actions template is included at
`config/github-actions-unit-tests.yml`. Copy it to
`.github/workflows/unit-tests.yml` after granting the pushing credential the
GitHub `workflow` permission. Gazebo end-to-end tests remain a local validation
stage because they require the simulator, rendering, and native bindings.

## Scope

This is a portfolio and test-engineering framework, not production vehicle
software. Its LKA and AEB implementations are intentionally understandable
baselines used to demonstrate simulation, validation, measurement, fault
injection, and regression-testing techniques.
