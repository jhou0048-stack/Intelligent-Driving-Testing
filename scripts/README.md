# Command-line tools

Reusable behavior belongs under `src/`; these scripts only parse arguments and
coordinate framework components.

| Script | Description |
|---|---|
| `run_scenarios.py` | Start Gazebo per scenario, execute YAML suites, assert, report |
| `check_regression.py` | Compare scenario JSON against regression limits |
| `validate_yolo.py` | Evaluate YOLO against a labelled image manifest |
| `run_lka.py` | Interactive LKA demonstration against a running world |
| `run_aeb.py` | Integrated LKA+AEB demonstration with safe command arbitration |
| `run_adas.py` | Integrated ADAS demonstration with optional YOLO |
| `validate_lka.py` | Run the isolated LKA and camera-fault scenario subset |
| `validate_aeb.py` | Run the vehicle, pedestrian, and integrated AEB subset |
| `control_vehicle.py` | Manual Ackermann control smoke test |
| `verify_gazebo_communication.py` | Gazebo transport connectivity check |

`run_scenarios.py` manages the Gazebo server automatically. Demonstration
scripts expect a world to already be running, for example:

```bash
GZ_SIM_RESOURCE_PATH="$PWD/scenarios/models" \
  gz sim -s -r scenarios/worlds/follow_scenario.sdf
python scripts/run_adas.py --duration 15
```
