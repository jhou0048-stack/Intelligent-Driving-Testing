# Architecture

The framework uses a `src` layout with packages divided by responsibility. This keeps simulator integration separate from algorithms and validation logic, so components can be tested or replaced independently.

## Package boundaries

- `simulation`: CARLA client adapters, world lifecycle, actors, sensors, and simulation timing.
- `perception`: image preprocessing, detection adapters, and normalized perception outputs.
- `planning`: route and behavior-planning interfaces and implementations.
- `control`: vehicle-command interfaces and control implementations.
- `testing`: scenario orchestration, measurements, assertions, metrics, and report preparation.

These packages are intentionally empty except for package documentation in the initial scaffold. Future code should avoid connecting to CARLA or loading models at import time.

## Dependency direction

Domain packages should exchange small typed data structures rather than importing CARLA objects throughout the codebase. Simulator-specific types should remain at the `simulation` boundary.

Tests may import all domain packages. Production packages must not import from `tests/`.

## Runtime content

- `config/` contains safe, version-controlled defaults.
- `scenarios/` contains declarative scenario definitions.
- `data/` contains local datasets and captures.
- `logs/` contains runtime diagnostics.
- `reports/` contains generated validation output.

The last three directories retain placeholders in Git, while their runtime contents remain ignored.

