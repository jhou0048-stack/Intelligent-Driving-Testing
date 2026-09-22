"""Tests for scenario files and model existence."""

from pathlib import Path

import pytest

SCENARIOS_DIR = Path(__file__).resolve().parent.parent / "scenarios"


@pytest.mark.parametrize(
    "world_file",
    [
        "simple_road.sdf",
        "follow_scenario.sdf",
        "pedestrian_crossing.sdf",
        "lane_change.sdf",
    ],
)
def test_world_file_exists(world_file: str) -> None:
    assert (SCENARIOS_DIR / "worlds" / world_file).is_file()


@pytest.mark.parametrize(
    "model_dir",
    ["ego_vehicle", "obstacle_vehicle", "pedestrian"],
)
def test_model_directory_has_sdf(model_dir: str) -> None:
    model_path = SCENARIOS_DIR / "models" / model_dir / "model.sdf"
    assert model_path.is_file()


@pytest.mark.parametrize(
    "model_dir",
    ["ego_vehicle", "obstacle_vehicle", "pedestrian"],
)
def test_model_directory_has_config(model_dir: str) -> None:
    config_path = SCENARIOS_DIR / "models" / model_dir / "model.config"
    assert config_path.is_file()


def test_config_world_is_simple_road() -> None:
    import yaml

    config_path = Path(__file__).resolve().parent.parent / "config" / "default.yaml"
    with open(config_path) as f:
        config = yaml.safe_load(f)
    assert config["simulation"]["world"] == "simple_road"


def test_config_has_perception_section() -> None:
    import yaml

    config_path = Path(__file__).resolve().parent.parent / "config" / "default.yaml"
    with open(config_path) as f:
        config = yaml.safe_load(f)
    assert "perception" in config
    assert "camera" in config["perception"]
    assert "lidar" in config["perception"]


def test_config_has_planning_section() -> None:
    import yaml

    config_path = Path(__file__).resolve().parent.parent / "config" / "default.yaml"
    with open(config_path) as f:
        config = yaml.safe_load(f)
    assert "planning" in config
    assert "lane_keeper" in config["planning"]
