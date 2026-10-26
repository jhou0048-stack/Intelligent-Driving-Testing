# Gazebo scenarios

`models/` contains reusable SDF actors and sensors. `worlds/` contains isolated
test environments used by `config/scenarios.yaml`.

## Worlds

| World | Purpose |
|---|---|
| `simple_road.sdf` | Straight-road LKA and camera fault injection |
| `follow_scenario.sdf` | Static vehicle AEB validation |
| `pedestrian_crossing.sdf` | Pedestrian AEB validation |
| `lane_change.sdf` | Integrated LKA + AEB obstacle validation |
| `test_ackermann.sdf` | Manual vehicle-model debugging |

## Scenario configuration

The version-controlled test definitions live in `config/scenarios.yaml`:

```yaml
- name: aeb_obstacle_ahead
  world: follow_scenario
  model: ego_vehicle
  timeout: 25.0
  telemetry:
    lane_center_y: 0.0
  sut:
    mode: aeb
    duration: 12.0
    speed: 2.0
    loop_hz: 20.0
  pass_conditions:
    - type: no_collision
    - type: aeb_triggered
```

Supported SUT modes are `open_loop`, `lka`, `aeb`, and `adas`. Closed-loop modes
own the vehicle command for the configured duration. Open-loop scenarios use an
`actions` list containing `drive`, `stop`, or `wait` operations.

Odometry coordinates are relative to the ego vehicle's initial pose, so a
straight lane normally uses `lane_center_y: 0.0`.
