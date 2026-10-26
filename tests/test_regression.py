"""Tests for regression baseline comparison."""

from testing.regression import RegressionComparator


def test_regression_comparator_passes_values_within_limits() -> None:
    baseline = {
        "scenarios": {
            "lka": {
                "required_pass": True,
                "metrics": {
                    "deviation": {"max": 0.5},
                    "distance": {"min": 3.0},
                },
            }
        }
    }
    current = {
        "results": [{
            "scenario_name": "lka",
            "passed": True,
            "metrics": {"deviation": 0.2, "distance": 4.0},
        }]
    }

    dataframe, passed = RegressionComparator(baseline).compare(current)

    assert passed is True
    assert dataframe["passed"].all()


def test_regression_comparator_fails_missing_scenario() -> None:
    baseline = {"scenarios": {"aeb": {"required_pass": True}}}

    dataframe, passed = RegressionComparator(baseline).compare({"results": []})

    assert passed is False
    assert dataframe.iloc[0]["check"] == "scenario_present"


def test_regression_comparator_fails_metric_regression() -> None:
    baseline = {
        "scenarios": {
            "aeb": {
                "metrics": {"collision_count": {"equals": 0}},
            }
        }
    }
    current = {
        "results": [{
            "scenario_name": "aeb",
            "passed": False,
            "metrics": {"collision_count": 1},
        }]
    }

    dataframe, passed = RegressionComparator(baseline).compare(current)

    assert passed is False
    assert not dataframe["passed"].all()
