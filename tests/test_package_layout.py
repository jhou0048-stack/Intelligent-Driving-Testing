"""Smoke tests for the initial modular package layout."""

from importlib import import_module

import pytest


@pytest.mark.parametrize(
    "package_name",
    ["simulation", "perception", "control", "planning", "testing"],
)
def test_domain_package_is_importable(package_name: str) -> None:
    """Each architectural domain is exposed as an importable package."""
    assert import_module(package_name) is not None

