# Off-target tests for waypoint navigation (nav.py is pure Python):
# `python3 tests/test_nav.py`. Includes a kinematic flight simulation --
# the geometry helpers can all be individually right while the assembled
# guidance loop still fails to actually reach a waypoint, so the mission is
# flown end to end against a simple aircraft model.

import os
import sys
from math import cos, degrees, radians, sin

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))

import config as config_module
import mission as mission_module
import nav

_HOME_LAT = 45.0
_HOME_LON = -122.9
_M_PER_DEG = 111320.0


class FakeGps:
    def __init__(self, lat=_HOME_LAT, lon=_HOME_LON, alt_m=100.0,
                 course_deg=0.0, speed_ms=15.0, sats=10, h_acc=2.0):
        self.lat_deg = lat
        self.lon_deg = lon
        self.alt_m = alt_m
        self.course_deg = course_deg
        self.ground_speed_ms = speed_ms
        self.num_sv = sats
        self.h_acc_m = h_acc
        self.fix_ok = True
        self.last_pvt_ms = 0

    def tick(self, ms=200):
        self.last_pvt_ms += ms


def offset_position(north_m, east_m):
    lat = _HOME_LAT + north_m / _M_PER_DEG
    lon = _HOME_LON + east_m / (_M_PER_DEG * cos(radians(_HOME_LAT)))
    return lat, lon


def make_waypoint(north_m, east_m, alt_m=100.0):
    lat, lon = offset_position(north_m, east_m)
    return {"lat": lat, "lon": lon, "alt_m": alt_m}


def make_navigator(waypoints, end_action=None, **overrides):
    config = config_module.defaults()
    config.update(overrides)
    mission = {"waypoints": waypoints}
    if end_action is not None:
        mission["end_action"] = end_action
    navigator = nav.Navigator(config, mission)
    navigator.set_home(_HOME_LAT, _HOME_LON, 100.0)
    return navigator


# --- geometry ------------------------------------------------------------

def test_local_offsets_round_trip():
    lat, lon = offset_position(500.0, -250.0)
    north, east = nav.local_offsets(_HOME_LAT, _HOME_LON, lat, lon)
    assert abs(north - 500.0) < 0.5
    assert abs(east + 250.0) < 0.5


def test_bearing_cardinals():
    assert abs(nav.bearing_deg(100.0, 0.0) - 0.0) < 1e-6
    assert abs(nav.bearing_deg(0.0, 100.0) - 90.0) < 1e-6
    assert abs(nav.bearing_deg(-100.0, 0.0) - 180.0) < 1e-6
    assert abs(nav.bearing_deg(0.0, -100.0) - 270.0) < 1e-6


def test_wrap180():
    assert nav.wrap180(190.0) == -170.0
    assert nav.wrap180(-190.0) == 170.0
    assert nav.wrap180(0.0) == 0.0


def test_cross_track_sign_and_magnitude():
    # Track runs due north; a point 30 m east of it is right of track.
    offset = nav.cross_track_m(0.0, 0.0, 1000.0, 0.0, 500.0, 30.0)
    assert abs(offset - 30.0) < 1e-6
    left = nav.cross_track_m(0.0, 0.0, 1000.0, 0.0, 500.0, -30.0)
    assert abs(left + 30.0) < 1e-6
    on_track = nav.cross_track_m(0.0, 0.0, 1000.0, 0.0, 500.0, 0.0)
    assert abs(on_track) < 1e-9


def test_passed_waypoint_plane_test():
    # Inbound track south->north through a waypoint at 1000 m north.
    assert not nav.passed_waypoint(1000.0, 0.0, 0.0, 0.0, 900.0, 0.0)
    assert nav.passed_waypoint(1000.0, 0.0, 0.0, 0.0, 1001.0, 0.0)
    # Passing wide still counts once the perpendicular plane is crossed --
    # this is what stops a fixed wing orbiting a waypoint it can't hit.
    assert nav.passed_waypoint(1000.0, 0.0, 0.0, 0.0, 1010.0, 300.0)


# --- engage gates --------------------------------------------------------

def test_engage_requires_mission_and_home():
    navigator = nav.Navigator(config_module.defaults(), {"waypoints": []})
    assert navigator.engage_blocker(FakeGps()) == "no mission stored"
    navigator = nav.Navigator(
        config_module.defaults(), {"waypoints": [make_waypoint(100.0, 0.0)]}
    )
    assert navigator.engage_blocker(FakeGps()) == "no home position"


def test_engage_gates_on_fix_quality_and_speed():
    waypoints = [make_waypoint(100.0, 0.0)]
    gps = FakeGps()
    gps.fix_ok = False
    assert make_navigator(waypoints).engage_blocker(gps) == "no GPS fix"

    assert "satellites" in make_navigator(waypoints).engage_blocker(FakeGps(sats=4))
    assert "accuracy" in make_navigator(waypoints).engage_blocker(FakeGps(h_acc=40.0))
    assert "ground speed" in make_navigator(waypoints).engage_blocker(
        FakeGps(speed_ms=0.5)
    )
    assert make_navigator(waypoints).engage_blocker(FakeGps()) is None


def test_engage_refuses_distant_first_waypoint():
    far = [make_waypoint(5000.0, 0.0)]
    blocker = make_navigator(far, nav_max_safe_distance_m=200.0).engage_blocker(
        FakeGps()
    )
    assert "waypoint 1 is" in blocker
    # 0 disables the check, matching INAV.
    assert make_navigator(far, nav_max_safe_distance_m=0.0).engage_blocker(
        FakeGps()
    ) is None


# --- demands -------------------------------------------------------------

def test_demands_held_between_fixes():
    navigator = make_navigator([make_waypoint(400.0, 0.0)])
    navigator.engage()
    gps = FakeGps(course_deg=90.0)
    gps.tick()
    navigator.update(gps)
    first = navigator.yaw_stick
    # Same PVT timestamp -> no recompute, even with the position changed.
    gps.lat_deg += 0.01
    navigator.update(gps)
    assert navigator.yaw_stick == first


def test_course_error_turns_the_short_way():
    navigator = make_navigator([make_waypoint(400.0, 0.0)])  # due north
    navigator.engage()
    # Heading 350 deg: the waypoint is 10 deg to the right, so a small
    # positive (right) yaw demand -- not a 350 deg left turn.
    gps = FakeGps(course_deg=350.0)
    gps.tick()
    navigator.update(gps)
    assert 0.0 < navigator.yaw_stick < 0.5


def test_yaw_demand_saturates_but_stays_normalized():
    navigator = make_navigator([make_waypoint(400.0, 0.0)])
    navigator.engage()
    gps = FakeGps(course_deg=180.0)  # 180 deg out
    gps.tick()
    navigator.update(gps)
    assert abs(navigator.yaw_stick) <= 1.0
    assert abs(navigator.yaw_stick) > 0.9


def test_altitude_error_respects_asymmetric_limits():
    high = make_waypoint(400.0, 0.0, alt_m=500.0)
    navigator = make_navigator([high], nav_climb_angle_deg=20.0, pitch_angle_max_deg=30.0)
    navigator.engage()
    gps = FakeGps(alt_m=100.0)
    gps.tick()
    navigator.update(gps)
    # 400 m below target saturates the climb limit: 20/30 of full stick.
    assert abs(navigator.pitch_stick - 20.0 / 30.0) < 1e-9

    low = make_waypoint(400.0, 0.0, alt_m=10.0)
    navigator = make_navigator([low], nav_dive_angle_deg=15.0, pitch_angle_max_deg=30.0)
    navigator.engage()
    gps = FakeGps(alt_m=400.0)
    gps.tick()
    navigator.update(gps)
    assert abs(navigator.pitch_stick + 15.0 / 30.0) < 1e-9


def test_throttle_band_and_pitch_coupling():
    navigator = make_navigator(
        [make_waypoint(400.0, 0.0, alt_m=100.0)],
        nav_cruise_throttle_pct=40.0, nav_min_throttle_pct=20.0,
        nav_max_throttle_pct=70.0, nav_pitch_to_throttle=1.0,
    )
    navigator.engage()
    # Level flight at target altitude -> cruise throttle exactly.
    gps = FakeGps(alt_m=100.0)
    gps.tick()
    navigator.update(gps)
    assert abs(navigator.throttle - 0.40) < 1e-9

    # Climbing adds throttle, but never beyond the configured ceiling.
    climb = make_navigator(
        [make_waypoint(400.0, 0.0, alt_m=1000.0)],
        nav_cruise_throttle_pct=40.0, nav_max_throttle_pct=45.0,
    )
    climb.engage()
    gps = FakeGps(alt_m=100.0)
    gps.tick()
    climb.update(gps)
    assert abs(climb.throttle - 0.45) < 1e-9


# --- sequencing ----------------------------------------------------------

def test_advances_on_radius():
    waypoints = [make_waypoint(400.0, 0.0), make_waypoint(400.0, 400.0)]
    navigator = make_navigator(waypoints, nav_wp_radius_m=25.0)
    navigator.engage()
    lat, lon = offset_position(390.0, 0.0)  # 10 m short, inside the radius
    gps = FakeGps(lat=lat, lon=lon)
    gps.tick()
    navigator.update(gps)
    assert navigator.index == 1


def test_advances_on_passing_wide():
    # 60 m wide of a waypoint with a 25 m radius: the radius test misses,
    # the perpendicular-plane test catches it.
    waypoints = [make_waypoint(400.0, 0.0), make_waypoint(800.0, 0.0)]
    navigator = make_navigator(waypoints, nav_wp_radius_m=25.0)
    navigator.engage()
    lat, lon = offset_position(410.0, 60.0)
    gps = FakeGps(lat=lat, lon=lon)
    gps.tick()
    navigator.update(gps)
    assert navigator.index == 1


def test_end_of_mission_loiters_by_default():
    navigator = make_navigator([make_waypoint(400.0, 0.0)])
    navigator.engage()
    lat, lon = offset_position(400.0, 0.0)
    gps = FakeGps(lat=lat, lon=lon)
    gps.tick()
    navigator.update(gps)
    assert navigator.state == nav.STATE_LOITER


def test_end_of_mission_returns_home_when_configured():
    navigator = make_navigator(
        [make_waypoint(400.0, 0.0)], end_action=mission_module.END_RTH
    )
    navigator.engage()
    lat, lon = offset_position(400.0, 0.0)
    gps = FakeGps(lat=lat, lon=lon)
    gps.tick()
    navigator.update(gps)
    assert navigator.state == nav.STATE_RTH
    # Reaching home then switches to orbiting it.
    gps.lat_deg, gps.lon_deg = _HOME_LAT, _HOME_LON
    gps.tick()
    navigator.update(gps)
    assert navigator.state == nav.STATE_LOITER


def test_end_of_mission_repeats_from_the_last_waypoint():
    # The lap wraparound: reaching the final waypoint rewinds to waypoint 1
    # and stays flying, and the leg being tracked must now start at the LAST
    # waypoint rather than home -- otherwise the cross-track term steers
    # toward a line back to the launch point on every lap but the first.
    waypoints = [make_waypoint(400.0, 0.0), make_waypoint(400.0, 400.0)]
    navigator = make_navigator(waypoints, end_action=mission_module.END_REPEAT)
    navigator.engage()
    assert navigator._leg_start() == (_HOME_LAT, _HOME_LON)

    gps = FakeGps(*offset_position(400.0, 0.0))
    gps.tick()
    navigator.update(gps)
    assert navigator.index == 1

    gps.lat_deg, gps.lon_deg = offset_position(400.0, 400.0)
    gps.tick()
    navigator.update(gps)
    assert navigator.index == 0, "did not rewind to waypoint 1"
    assert navigator.state == nav.STATE_RUN, "repeat must keep flying"
    assert navigator.laps == 1
    last = waypoints[-1]
    assert navigator._leg_start() == (last["lat"], last["lon"])


def test_repeat_does_not_instantly_reflow_the_closing_leg():
    # Sitting on the last waypoint at the moment of the rewind, the aircraft
    # must not also read waypoint 1 as already reached -- that would spin the
    # index through the whole plan in a single fix.
    waypoints = [make_waypoint(400.0, 0.0), make_waypoint(400.0, 400.0)]
    navigator = make_navigator(waypoints, end_action=mission_module.END_REPEAT)
    navigator.engage()
    navigator.index = 1
    gps = FakeGps(*offset_position(400.0, 400.0))
    gps.tick()
    navigator.update(gps)
    assert (navigator.index, navigator.laps) == (0, 1)
    gps.tick()
    navigator.update(gps)
    assert (navigator.index, navigator.laps) == (0, 1), "advanced without moving"


def test_disengage_clears_demands():
    navigator = make_navigator([make_waypoint(400.0, 0.0)])
    navigator.engage()
    gps = FakeGps(course_deg=180.0)
    gps.tick()
    navigator.update(gps)
    navigator.disengage()
    assert navigator.state == nav.STATE_IDLE
    assert navigator.throttle == 0.0
    assert navigator.pitch_stick == 0.0
    assert navigator.yaw_stick == 0.0


# --- flight simulation ---------------------------------------------------

class SimulatedAircraft:
    # First-order kinematic model: the nav demands are interpreted the same
    # way main.py's stabilized cascade interprets pilot sticks -- yaw_stick
    # scales to a yaw rate, pitch_stick to a pitch angle -- and the inner
    # loops are assumed to track them (they are tested separately). Enough
    # to prove the guidance converges; not an aerodynamic model.
    def __init__(self, config, lat=_HOME_LAT, lon=_HOME_LON, alt_m=100.0,
                 course_deg=0.0, speed_ms=15.0):
        self.gps = FakeGps(lat=lat, lon=lon, alt_m=alt_m,
                           course_deg=course_deg, speed_ms=speed_ms)
        self.rate_yaw_dps = config["rate_yaw_dps"]
        self.pitch_scale = config["pitch_angle_max_deg"]
        self.speed_ms = speed_ms
        self.track = []

    def step(self, navigator, dt=0.2):
        navigator.update(self.gps)
        gps = self.gps
        gps.course_deg = (
            gps.course_deg + navigator.yaw_stick * self.rate_yaw_dps * dt
        ) % 360.0
        pitch_deg = navigator.pitch_stick * self.pitch_scale
        gps.alt_m += self.speed_ms * sin(radians(pitch_deg)) * dt
        north = self.speed_ms * cos(radians(gps.course_deg)) * dt
        east = self.speed_ms * sin(radians(gps.course_deg)) * dt
        gps.lat_deg += north / _M_PER_DEG
        gps.lon_deg += east / (_M_PER_DEG * cos(radians(gps.lat_deg)))
        gps.tick(int(dt * 1000))
        self.track.append((gps.lat_deg, gps.lon_deg))


def _distance_to(gps, waypoint):
    north, east = nav.local_offsets(
        _HOME_LAT, _HOME_LON, waypoint["lat"], waypoint["lon"]
    )
    pos_n, pos_e = nav.local_offsets(
        _HOME_LAT, _HOME_LON, gps.lat_deg, gps.lon_deg
    )
    return nav.distance_m(north - pos_n, east - pos_e)


def test_simulated_mission_visits_every_waypoint_and_loiters():
    # A box circuit starting 300 m north, flown from a standing start at
    # home heading due north.
    waypoints = [
        make_waypoint(300.0, 0.0, alt_m=120.0),
        make_waypoint(300.0, 300.0, alt_m=120.0),
        make_waypoint(0.0, 300.0, alt_m=100.0),
        make_waypoint(-100.0, 0.0, alt_m=100.0),
    ]
    config = config_module.defaults()
    navigator = make_navigator(waypoints)
    navigator.engage()
    aircraft = SimulatedAircraft(config)

    reached = []
    last_index = 0
    for _ in range(3000):  # 600 s at 5 Hz
        aircraft.step(navigator)
        if navigator.index != last_index:
            reached.append(last_index)
            last_index = navigator.index
        if navigator.state == nav.STATE_LOITER:
            reached.append(last_index)
            break

    assert navigator.state == nav.STATE_LOITER, "mission never completed"
    assert reached == [0, 1, 2, 3], "waypoints out of order: %r" % (reached,)


def test_simulated_repeat_flies_the_circuit_more_than_once():
    # Same box, but repeating: the plan has to come round to waypoint 1 and
    # run again without ever settling into a loiter, and the second lap must
    # visit the waypoints in the same order as the first.
    waypoints = [
        make_waypoint(300.0, 0.0, alt_m=120.0),
        make_waypoint(300.0, 300.0, alt_m=120.0),
        make_waypoint(0.0, 300.0, alt_m=100.0),
        make_waypoint(-100.0, 0.0, alt_m=100.0),
    ]
    config = config_module.defaults()
    navigator = make_navigator(waypoints, end_action=mission_module.END_REPEAT)
    navigator.engage()
    aircraft = SimulatedAircraft(config)

    visited = [0]
    for _ in range(4000):  # 800 s at 5 Hz: comfortably two laps of the box
        aircraft.step(navigator)
        assert navigator.state != nav.STATE_LOITER, "repeat settled into loiter"
        if navigator.index != visited[-1]:
            visited.append(navigator.index)
        if navigator.laps >= 2:
            break

    assert navigator.laps >= 2, "never completed two laps"
    assert visited[:8] == [0, 1, 2, 3, 0, 1, 2, 3], (
        "waypoints out of order: %r" % (visited,))


def test_simulated_flight_holds_the_loiter_circle():
    waypoints = [make_waypoint(200.0, 0.0, alt_m=100.0)]
    config = config_module.defaults()
    navigator = make_navigator(waypoints, nav_loiter_radius_m=75.0)
    navigator.engage()
    aircraft = SimulatedAircraft(config)

    for _ in range(1500):
        aircraft.step(navigator)
        if navigator.state == nav.STATE_LOITER:
            break
    assert navigator.state == nav.STATE_LOITER

    # Once settled, the radius from the loiter centre should stay near the
    # configured 75 m rather than spiralling in or wandering off.
    for _ in range(400):
        aircraft.step(navigator)
    radii = []
    for _ in range(200):
        aircraft.step(navigator)
        radii.append(_distance_to(aircraft.gps, waypoints[0]))
    assert 35.0 < min(radii), "spiralled inside the circle: %.1f m" % min(radii)
    assert max(radii) < 160.0, "drifted outside the circle: %.1f m" % max(radii)


def test_simulated_flight_climbs_toward_waypoint_altitude():
    waypoints = [make_waypoint(600.0, 0.0, alt_m=200.0)]
    config = config_module.defaults()
    navigator = make_navigator(waypoints)
    navigator.engage()
    aircraft = SimulatedAircraft(config, alt_m=100.0)
    for _ in range(400):
        aircraft.step(navigator)
    assert aircraft.gps.alt_m > 150.0, "did not climb: %.1f m" % aircraft.gps.alt_m


def test_simulated_flight_rejoins_track_when_started_off_line():
    # Start 150 m east of the direct line to a waypoint due north: the
    # cross-track term should pull it back onto the leg, not just chase the
    # point (which would leave a permanent bow in the path).
    waypoints = [make_waypoint(800.0, 0.0, alt_m=100.0)]
    config = config_module.defaults()
    navigator = make_navigator(waypoints)
    navigator.engage()
    lat, lon = offset_position(100.0, 150.0)
    aircraft = SimulatedAircraft(config, lat=lat, lon=lon, course_deg=0.0)

    offsets = []
    for _ in range(300):
        aircraft.step(navigator)
        pos_n, pos_e = nav.local_offsets(
            _HOME_LAT, _HOME_LON, aircraft.gps.lat_deg, aircraft.gps.lon_deg
        )
        if pos_n < 700.0:
            offsets.append(abs(nav.cross_track_m(0.0, 0.0, 800.0, 0.0, pos_n, pos_e)))
    assert offsets[-1] < 40.0, "never rejoined track: %.1f m off" % offsets[-1]
    assert offsets[-1] < offsets[0], "drifted further from track"


if __name__ == "__main__":
    failures = 0
    for name, test in sorted(globals().items()):
        if name.startswith("test_") and callable(test):
            try:
                test()
                print("ok   " + name)
            except AssertionError as error:
                failures += 1
                print("FAIL " + name + ": " + str(error))
    sys.exit(1 if failures else 0)
