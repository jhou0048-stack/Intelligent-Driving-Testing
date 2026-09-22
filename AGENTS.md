# AGENTS.md

## Project scope

This repository is a simulation-based ADAS testing and validation portfolio project built with Python, Gazebo Harmonic, PyTest, OpenCV, YOLO, and Pandas.

The current milestone is infrastructure-only. Do not add autonomous-driving behavior, vehicle-control logic, object-detection pipelines, or route-planning algorithms unless a later task explicitly requests them.

## Architecture

- Keep production code under `src/` and tests under `tests/`.
- Preserve the domain boundaries: `simulation`, `perception`, `control`, `planning`, and `testing`.
- Keep Gazebo-specific integration inside `simulation`; other packages should depend on small, explicit simulator interfaces rather than simulator internals.
- Keep shared test orchestration, assertions, metrics, and reporting helpers inside `testing`.
- Store scenario definitions in `scenarios/` and configuration in `config/`.
- Treat `data/`, `logs/`, and `reports/` as generated or local runtime directories.
- Add focused modules instead of accumulating application logic in a single `main.py`.

## Development conventions

- Target Python 3.12 or later and add type hints to new public interfaces.
- Use `pathlib` for filesystem paths and `logging` instead of `print` in library code.
- Keep imports free of simulator connections, model loading, and other side effects.
- Write PyTest coverage for new behavior and keep tests deterministic where possible.
- Document new configuration keys and scenario formats.
- Never commit credentials, local datasets, generated reports, logs, or model weights.

## Validation

Run the test suite from the repository root:

```bash
pytest
```

