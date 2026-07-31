# Off-target tests for mission validation (pure Python, no `machine`
# imports): `python3 tests/test_mission.py`.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))

import mission


def _raises_value_error(raw):
    try:
        mission.validate(raw)
    except ValueError:
        return True
    return False


def test_empty_mission_is_valid():
    assert mission.validate({"waypoints": []}) == {"waypoints": []}


def test_valid_mission_round_trips_with_floats():
    raw = {"waypoints": [
        {"lat": "51.5", "lon": "-0.12", "alt_m": 100},
        {"lat": 51.501, "lon": -0.121, "alt_m": 120.5},
    ]}
    validated = mission.validate(raw)
    assert validated == {"waypoints": [
        {"lat": 51.5, "lon": -0.12, "alt_m": 100.0},
        {"lat": 51.501, "lon": -0.121, "alt_m": 120.5},
    ]}


def test_rejects_wrong_shapes():
    assert _raises_value_error(None)
    assert _raises_value_error([])
    assert _raises_value_error({"waypoints": "nope"})
    assert _raises_value_error({"waypoints": [42]})


def test_rejects_out_of_range_and_missing_fields():
    assert _raises_value_error({"waypoints": [{"lat": 91, "lon": 0, "alt_m": 100}]})
    assert _raises_value_error({"waypoints": [{"lat": 0, "lon": 181, "alt_m": 100}]})
    assert _raises_value_error({"waypoints": [{"lat": 0, "lon": 0, "alt_m": -5}]})
    assert _raises_value_error({"waypoints": [{"lat": 0, "lon": 0}]})
    assert _raises_value_error({"waypoints": [{"lat": "abc", "lon": 0, "alt_m": 100}]})
    assert _raises_value_error({"waypoints": [{"lat": float("nan"), "lon": 0, "alt_m": 100}]})


def test_rejects_too_many_waypoints():
    waypoint = {"lat": 0.0, "lon": 0.0, "alt_m": 100.0}
    assert mission.validate({"waypoints": [dict(waypoint)] * mission.MAX_WAYPOINTS})
    assert _raises_value_error(
        {"waypoints": [dict(waypoint)] * (mission.MAX_WAYPOINTS + 1)}
    )


def test_error_messages_name_the_waypoint():
    try:
        mission.validate({"waypoints": [
            {"lat": 0, "lon": 0, "alt_m": 100},
            {"lat": 99, "lon": 0, "alt_m": 100},
        ]})
    except ValueError as error:
        assert "waypoint 2" in str(error)
    else:
        raise AssertionError("expected ValueError")


if __name__ == "__main__":
    failures = 0
    for name, test in sorted(globals().items()):
        if name.startswith("test_") and callable(test):
            try:
                test()
                print("ok   " + name)
            except AssertionError:
                failures += 1
                print("FAIL " + name)
    sys.exit(1 if failures else 0)
