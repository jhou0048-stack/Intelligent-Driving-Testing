"""Tests for the planning module."""

import math

import pytest

from perception.object_detector import Detection
from planning import (
    BehaviorPlanner,
    DrivingBehavior,
    LaneKeeper,
    Planner,
    SimpleBehaviorPlanner,
    Waypoint,
)


# --- ABC tests ---------------------------------------------------------------


def test_planner_is_abstract() -> None:
    with pytest.raises(TypeError):
        Planner()  # type: ignore[abstract]


def test_behavior_planner_is_abstract() -> None:
    with pytest.raises(TypeError):
        BehaviorPlanner()  # type: ignore[abstract]


# --- Waypoint dataclass -------------------------------------------------------


def test_waypoint_fields() -> None:
    wp = Waypoint(x=1.0, y=2.0, target_speed=0.5, heading=0.1)
    assert wp.x == pytest.approx(1.0)
    assert wp.y == pytest.approx(2.0)
    assert wp.target_speed == pytest.approx(0.5)
    assert wp.heading == pytest.approx(0.1)


def test_waypoint_default_heading() -> None:
    wp = Waypoint(x=0, y=0, target_speed=1.0)
    assert wp.heading == pytest.approx(0.0)


# --- LaneKeeper tests --------------------------------------------------------


def test_lane_keeper_zero_offset_goes_straight() -> None:
    keeper = LaneKeeper(kp=1.0, lookahead=5.0)
    pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
    waypoints = keeper.plan(pose, {"lateral_offset": 0.0})
    assert len(waypoints) == 1
    assert waypoints[0].heading == pytest.approx(0.0)
    assert waypoints[0].x == pytest.approx(5.0, abs=0.1)


def test_lane_keeper_positive_offset_steers_right() -> None:
    keeper = LaneKeeper(kp=1.0, lookahead=5.0)
    pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
    waypoints = keeper.plan(pose, {"lateral_offset": 0.5})
    assert waypoints[0].heading < 0


def test_lane_keeper_negative_offset_steers_left() -> None:
    keeper = LaneKeeper(kp=1.0, lookahead=5.0)
    pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
    waypoints = keeper.plan(pose, {"lateral_offset": -0.5})
    assert waypoints[0].heading > 0


@pytest.mark.parametrize("kp", [0.5, 1.0, 2.0])
def test_lane_keeper_kp_scales_correction(kp: float) -> None:
    keeper = LaneKeeper(kp=kp, lookahead=5.0)
    pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
    waypoints = keeper.plan(pose, {"lateral_offset": 1.0})
    expected_heading = -kp * 1.0
    assert waypoints[0].heading == pytest.approx(expected_heading)


def test_lane_keeper_compute_steering() -> None:
    keeper = LaneKeeper()
    pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
    target = Waypoint(x=5.0, y=0.0, target_speed=1.0)
    speed, angle = keeper.compute_steering(pose, target)
    assert speed == pytest.approx(1.0)
    assert angle == pytest.approx(0.0, abs=0.01)


def test_lane_keeper_compute_steering_left_turn() -> None:
    keeper = LaneKeeper()
    pose = {"x": 0.0, "y": 0.0, "heading": 0.0}
    target = Waypoint(x=0.0, y=5.0, target_speed=1.0)
    speed, angle = keeper.compute_steering(pose, target)
    assert angle == pytest.approx(math.pi / 2, abs=0.01)


def test_lane_keeper_reset_is_noop() -> None:
    keeper = LaneKeeper()
    keeper.reset()  # should not raise


# --- SimpleBehaviorPlanner tests ----------------------------------------------


def test_simple_behavior_lane_keeping_no_obstacles() -> None:
    planner = SimpleBehaviorPlanner(emergency_stop_distance=5.0)
    result = planner.decide(
        {"x": 0, "y": 0}, [], DrivingBehavior.LANE_KEEPING
    )
    assert result == DrivingBehavior.LANE_KEEPING


def test_simple_behavior_emergency_stop() -> None:
    planner = SimpleBehaviorPlanner(emergency_stop_distance=50)
    det = Detection(class_name="car", confidence=0.9, bbox=(100, 100, 200, 200))
    result = planner.decide(
        {"x": 0, "y": 0}, [det], DrivingBehavior.LANE_KEEPING
    )
    assert result == DrivingBehavior.EMERGENCY_STOP


def test_simple_behavior_no_stop_for_small_bbox() -> None:
    planner = SimpleBehaviorPlanner(emergency_stop_distance=200)
    det = Detection(class_name="car", confidence=0.9, bbox=(100, 100, 200, 150))
    result = planner.decide(
        {"x": 0, "y": 0}, [det], DrivingBehavior.LANE_KEEPING
    )
    assert result == DrivingBehavior.LANE_KEEPING
