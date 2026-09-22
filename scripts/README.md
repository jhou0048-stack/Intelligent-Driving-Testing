# Scripts

This directory contains developer and automation entry points. Reusable logic belongs in the appropriate package under `src/`; scripts only coordinate that logic.

## Available scripts

| Script | Description |
|--------|-------------|
| `verify_gazebo_communication.py` | Check that Python can list Gazebo topics and echo messages |
| `control_vehicle.py` | Manual smoke test for Ackermann vehicle control in Gazebo |
| `run_lka.py` | Run the LKA (Lane Keeping Assist) pipeline end-to-end |
| `validate_lka.py` | Run the LKA validation suite with metrics and CSV/JSON report |
| `run_aeb.py` | Run the AEB (Automatic Emergency Braking) pipeline with lidar |
| `run_adas.py` | Run the integrated ADAS pipeline (LKA + AEB + perception fusion) |
| `run_scenarios.py` | Batch-run all scenarios from a YAML config file |

## Prerequisites

All scripts require a running Gazebo Harmonic simulation:

```bash
# Terminal 1: Start the Gazebo server
gz sim -s scenarios/worlds/simple_road.sdf

# Terminal 2 (optional): Start the GUI
gz sim -g

# Terminal 3: Run a script
python scripts/run_lka.py
```
