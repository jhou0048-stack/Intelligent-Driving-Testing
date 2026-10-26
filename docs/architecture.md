# Architecture

The framework separates simulator integration, ADAS logic, and validation so
each layer can be tested independently.

## Package boundaries

- `simulation`: simulator interfaces, Gazebo transport, topic discovery, service
  calls, publisher discovery, and vehicle telemetry.
- `perception`: camera and lidar adapters, OpenCV lane detection, YOLO adapter,
  and fused perception results.
- `planning`: LKA, AEB, behavior, and integrated command-arbitration pipelines.
- `control`: simulator-independent vehicle control and Gazebo Ackermann commands.
- `testing`: scenario loading, closed-loop runtime setup, fault injection,
  assertions, result reporting, offline perception validation, and regression.

Gazebo connections and model loading occur only at runtime. Importing a package
does not start the simulator or load YOLO weights.

## Scenario lifecycle

```text
load YAML
   │
   ▼
start Gazebo server and optional GUI
   │
   ▼
connect transport and wait for command subscriber
   │
   ▼
subscribe odometry, contacts, camera, and/or lidar
   │
   ▼
unpause world and verify sensor readiness
   │
   ▼
run open-loop actions or lka/aeb/adas closed loop
   │
   ▼
stop vehicle and collect final metrics
   │
   ▼
evaluate every pass condition
   │
   ▼
write CSV/JSON and return a process exit status
   │
   ▼
disconnect resources and terminate Gazebo
```

Each scenario receives a new Gazebo process. State is therefore isolated between
tests and teardown runs even after failures.

## Command arbitration

The integrated `ADASPipeline` owns the final vehicle command. Emergency braking
has priority over lane keeping; AEB and LKA do not publish competing commands
from separate threads.

```text
perception result
      │
      ├── hazard inside braking distance ──> stop
      │
      └── safe ──> LKA steering or straight cruise
```

## Validation data

`GazeboVehicleTelemetry` subscribes to real simulator topics and reports:

- final and maximum speed
- mean speed and distance travelled
- final position and maximum lateral deviation
- odometry/contact sample counts
- collision-event count

The closed-loop runtime adds perception cycles, lane-detection rate, minimum
forward distance, AEB activation, YOLO object counts, and fault activations.

Assertions fail when required metrics are missing. Missing telemetry is never
interpreted as a successful zero-collision or zero-deviation result.

## Generated content

`data/`, `logs/`, `reports/`, and model weights remain local. Configuration,
source, SDF assets, tests, and regression limits are version controlled.
