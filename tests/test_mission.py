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
    assert mission.validate({"waypoints": []}) == {
        "waypoints": [],
        "end_action": mission.DEFAULT_END_ACTION,
        "alt_frame": mission.DEFAULT_ALT_FRAME,
    }


def test_valid_mission_round_trips_with_floats():
    raw = {"waypoints": [
        {"lat": "51.5", "lon": "-0.12", "alt_m": 100},
        {"lat": 51.501, "lon": -0.121, "alt_m": 120.5},
    ]}
    validated = mission.validate(raw)
    assert validated == {
        "waypoints": [
            {"lat": 51.5, "lon": -0.12, "alt_m": 100.0},
            {"lat": 51.501, "lon": -0.121, "alt_m": 120.5},
        ],
        "end_action": mission.DEFAULT_END_ACTION,
        "alt_frame": mission.DEFAULT_ALT_FRAME,
    }


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


def _two_waypoints():
    return [
        {"lat": 51.5, "lon": -0.12, "alt_m": 100.0},
        {"lat": 51.501, "lon": -0.121, "alt_m": 120.0},
    ]


def test_empty_helper_agrees_with_validate():
    # empty() is what load() hands back when there is no file; if it drifted
    # from validate()'s output shape, nav would read a key that isn't there.
    assert mission.validate(mission.empty()) == mission.empty()


def test_every_end_action_round_trips():
    for action in mission.END_ACTIONS:
        validated = mission.validate(
            {"waypoints": _two_waypoints(), "end_action": action}
        )
        assert validated["end_action"] == action


def test_missing_end_action_defaults_rather_than_rejecting():
    # A mission.json written before the ending was selectable must still
    # load, not brick the plan -- see the comment in mission.validate.
    validated = mission.validate({"waypoints": _two_waypoints()})
    assert validated["end_action"] == mission.DEFAULT_END_ACTION


def test_rejects_unknown_end_action():
    assert _raises_value_error(
        {"waypoints": _two_waypoints(), "end_action": "ditch"})
    assert _raises_value_error(
        {"waypoints": _two_waypoints(), "end_action": ""})
    assert _raises_value_error(
        {"waypoints": _two_waypoints(), "end_action": None})


def test_repeat_needs_at_least_two_waypoints():
    # A one-waypoint repeat is a leg from a point back to itself: no track to
    # follow, and the reached-test would retrigger forever.
    one = [{"lat": 51.5, "lon": -0.12, "alt_m": 100.0}]
    assert _raises_value_error(
        {"waypoints": one, "end_action": mission.END_REPEAT})
    assert _raises_value_error(
        {"waypoints": [], "end_action": mission.END_REPEAT})
    assert mission.validate(
        {"waypoints": _two_waypoints(), "end_action": mission.END_REPEAT})


def test_every_alt_frame_round_trips():
    for frame in mission.ALT_FRAMES:
        validated = mission.validate(
            {"waypoints": _two_waypoints(), "alt_frame": frame}
        )
        assert validated["alt_frame"] == frame


def test_missing_alt_frame_defaults_to_relative():
    # The datum a mission.json written before alt_frame existed gets. It must
    # be the relative one: those plans were drawn in an editor defaulting to
    # 100 m and capped at 500, i.e. as height over the field, even though the
    # old nav code went on to read them as MSL.
    validated = mission.validate({"waypoints": _two_waypoints()})
    assert validated["alt_frame"] == mission.ALT_FRAME_REL
    assert mission.DEFAULT_ALT_FRAME == mission.ALT_FRAME_REL


def test_rejects_unknown_alt_frame():
    for bad in ("agl", "", None, 0):
        assert _raises_value_error(
            {"waypoints": _two_waypoints(), "alt_frame": bad})


def test_altitude_bounds_follow_the_datum():
    # 500 m over the field is already beyond this airframe, but as an
    # absolute it would refuse any field above 500 m -- so the bound has to
    # move with the datum rather than being one fixed range.
    high = [{"lat": 51.5, "lon": -0.12, "alt_m": 1200.0}]
    assert _raises_value_error(
        {"waypoints": high, "alt_frame": mission.ALT_FRAME_REL})
    assert mission.validate(
        {"waypoints": high, "alt_frame": mission.ALT_FRAME_AMSL})

    # Below the reference is meaningless relative to the launch point and
    # ordinary as an absolute (Schiphol, the Dead Sea).
    below = [{"lat": 51.5, "lon": -0.12, "alt_m": -20.0}]
    assert _raises_value_error(
        {"waypoints": below, "alt_frame": mission.ALT_FRAME_REL})
    assert mission.validate(
        {"waypoints": below, "alt_frame": mission.ALT_FRAME_AMSL})


def test_single_waypoint_still_allows_the_other_endings():
    one = [{"lat": 51.5, "lon": -0.12, "alt_m": 100.0}]
    for action in (mission.END_LOITER, mission.END_RTH):
        assert mission.validate({"waypoints": one, "end_action": action})


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
