# Waypoint navigation for autonomous mode. Structure follows INAV's
# fixed-wing navigation (`navigation/navigation.c`, `navigation_fixedwing.c`)
# with one necessary deviation, called out below.
#
# INAV's fixed-wing guidance emits a BANK ANGLE: `navHeadingError` feeds
# `navPidApply2(&posControl.pids.fw_nav, ...)` clamped to
# `nav_fw_bank_angle` (35 deg default), and yaw/rudder is supplementary
# (only when STATE(FW_HEADING_USE_YAW)). This airframe has no roll actuator
# at all -- 2 V-tail servos, no ailerons -- so the final conversion is
# re-derived: course error becomes a YAW RATE demand, riding the same path
# `turn_assist_gain` already uses for the roll stick in stabilized mode.
# Everything upstream of that (virtual target / cross-track tracking, the
# waypoint state machine, the two-part reached test) follows INAV.
#
# The demand interface is deliberately the pilot's: this module produces
# pitch_stick / yaw_stick / throttle in exactly the units main.py's flight
# loop already reads from the RC channels, so autonomous mode is a third
# demand source feeding the *same* stabilized cascade, mixer and outputs.
# Nothing downstream knows the difference.
#
# Guidance recomputes only when a new NAV-PVT arrives (5 Hz), not every
# 250 Hz iteration: position doesn't meaningfully change in 4 ms, and the
# held demands keep the inner loops running at full rate. Same split as
# INAV, whose nav task runs well below the PID loop rate.
#
# Pure Python (no `machine` imports) so it is testable off-target.

from math import atan2, cos, degrees, radians, sin, sqrt

import mission as mission_module

_M_PER_DEG_LAT = 111320.0

STATE_IDLE = "idle"
STATE_RUN = "run"
STATE_RTH = "rth"
STATE_LOITER = "loiter"
# Automatic landing, mission end_action "land". Follows INAV's fixed-wing
# landing sequence (docs/Fixed Wing Landing.md) with its sensor-dependent
# phases removed: upstream measures wind in a 30 s loiter before choosing
# among up to four approach headings (we use the configured one) and adds a
# flare below nav_fw_land_flare_alt, which upstream documents as
# LIDAR/rangefinder-dependent -- without one INAV also ends at the glide.
STATE_LAND_APPROACH = "land_approach"  # fly to the start of the final leg
STATE_LAND_FINAL = "land_final"        # track the leg down the glideslope
STATE_LAND_GLIDE = "land_glide"        # motor off, fixed pitch, hold course
_LANDING_STATES = (STATE_LAND_APPROACH, STATE_LAND_FINAL, STATE_LAND_GLIDE)

# Angle by which the loiter target leads the aircraft around the circle.
# Large enough to keep the nose pulling around the turn, small enough not
# to cut across the middle.
_LOITER_LEAD_DEG = 35.0


def wrap180(degrees_value):
    while degrees_value > 180.0:
        degrees_value -= 360.0
    while degrees_value < -180.0:
        degrees_value += 360.0
    return degrees_value


def local_offsets(lat_ref, lon_ref, lat, lon):
    # Equirectangular projection to metres north/east of the reference.
    # Exact enough far past the range of this airframe, and far cheaper
    # than great-circle math on a Cortex-M0+ with software floats.
    north = (lat - lat_ref) * _M_PER_DEG_LAT
    east = (lon - lon_ref) * _M_PER_DEG_LAT * cos(radians(lat_ref))
    return north, east


def offset_to_latlon(lat_ref, lon_ref, north, east):
    # Inverse of local_offsets, same equirectangular approximation. The
    # landing geometry is naturally expressed in metres from the touchdown
    # point, but _target()/_apply_demands work in lat/lon, so the synthesized
    # approach point has to come back the other way.
    lat = lat_ref + north / _M_PER_DEG_LAT
    lon = lon_ref + east / (_M_PER_DEG_LAT * cos(radians(lat_ref)))
    return lat, lon


def bearing_deg(north, east):
    # Compass bearing (0 = north, 90 = east) of the offset vector.
    return degrees(atan2(east, north)) % 360.0


def distance_m(north, east):
    return sqrt(north * north + east * east)


def cross_track_m(start_n, start_e, end_n, end_e, pos_n, pos_e):
    # Signed perpendicular distance from the start->end track. Positive
    # means the aircraft is right of track, so the correction steers left.
    track_n = end_n - start_n
    track_e = end_e - start_e
    length = sqrt(track_n * track_n + track_e * track_e)
    if length < 1e-6:
        return 0.0
    # Project the position offset onto the track's right-hand normal: a
    # track on bearing t has direction (cos t, sin t) in (north, east), so
    # "right" is bearing t+90 = (-sin t, cos t). Getting this sign backwards
    # makes the correction steer away from the line rather than onto it.
    return ((pos_e - start_e) * track_n - (pos_n - start_n) * track_e) / length


def passed_waypoint(end_n, end_e, prev_n, prev_e, pos_n, pos_e):
    # INAV's second reached-test: a fixed wing will rarely enter a small
    # radius, so the leg also completes once the aircraft crosses the plane
    # through the waypoint perpendicular to the inbound track.
    track_n = end_n - prev_n
    track_e = end_e - prev_e
    if track_n * track_n + track_e * track_e < 1e-12:
        return False
    # Positive dot product = the waypoint is behind us along the track.
    return (pos_n - end_n) * track_n + (pos_e - end_e) * track_e > 0.0


class Navigator:
    def __init__(self, config, mission):
        self.configure(config)
        self.set_mission(mission)
        self.home = None
        self.state = STATE_IDLE
        self.index = 0
        self.blocked_reason = None
        self._last_pvt_ms = None
        # Held demands, in the same units the RC channels provide.
        self.pitch_stick = 0.0
        self.yaw_stick = 0.0
        self.throttle = 0.0

    def configure(self, config):
        self._cruise = config["nav_cruise_throttle_pct"] / 100.0
        self._throttle_min = config["nav_min_throttle_pct"] / 100.0
        self._throttle_max = config["nav_max_throttle_pct"] / 100.0
        self._climb_deg = config["nav_climb_angle_deg"]
        self._dive_deg = config["nav_dive_angle_deg"]
        self._wp_radius = config["nav_wp_radius_m"]
        self._max_safe_distance = config["nav_max_safe_distance_m"]
        self._heading_p = config["nav_heading_p"]
        self._xtrack_p = config["nav_xtrack_p"]
        self._xtrack_limit = config["nav_xtrack_limit_deg"]
        self._alt_p = config["nav_alt_p"]
        self._pitch_to_throttle = config["nav_pitch_to_throttle"]
        self._loiter_radius = config["nav_loiter_radius_m"]
        self._min_ground_speed = config["nav_min_ground_speed_ms"]
        self._min_sats = int(config["nav_min_sats"])
        self._max_h_acc = config["nav_max_h_acc_m"]
        self._land_heading = config["nav_land_heading_deg"]
        self._land_approach_length = config["nav_land_approach_length_m"]
        self._land_approach_alt = config["nav_land_approach_alt_m"]
        self._land_glide_alt = config["nav_land_glide_alt_m"]
        self._land_glide_pitch = config["nav_land_glide_pitch_deg"]
        # The cascade converts pitch_stick via pitch_angle_max_deg, so nav
        # can never command beyond that however the climb/dive limits are
        # set -- clamp here rather than let the demand be silently clipped.
        self._pitch_scale = config["pitch_angle_max_deg"]
        self._rate_yaw = config["rate_yaw_dps"]

    def set_mission(self, mission):
        # The ending and the altitude datum both travel with the plan (see
        # mission.py), not with config.
        self.waypoints = mission.get("waypoints", []) if mission else []
        self.end_action = (
            mission.get("end_action", mission_module.DEFAULT_END_ACTION)
            if mission else mission_module.DEFAULT_END_ACTION
        )
        self.alt_frame = (
            mission.get("alt_frame", mission_module.DEFAULT_ALT_FRAME)
            if mission else mission_module.DEFAULT_ALT_FRAME
        )
        self.laps = 0

    def set_home(self, lat, lon, alt_m):
        self.home = (lat, lon, alt_m)

    def engage_blocker(self, gps):
        # Mirrors INAV's NAV_STATE_WAYPOINT_INITIALIZE gates: a usable
        # position estimate, a usable heading, and a valid mission. Returns
        # a human-readable reason, or None when nav may engage.
        if not self.waypoints:
            return "no mission stored"
        if self.home is None:
            return "no home position"
        if not gps.fix_ok:
            return "no GPS fix"
        if gps.num_sv < self._min_sats:
            return "only %d satellites" % gps.num_sv
        if gps.h_acc_m > self._max_h_acc:
            return "GPS accuracy %.1f m" % gps.h_acc_m
        # Heading comes from GPS course over ground, which is meaningless
        # at low speed -- this stands in for INAV's estHeadingStatus gate.
        if gps.ground_speed_ms < self._min_ground_speed:
            return "ground speed too low for heading"
        blocker = self._mission_distance_blocker()
        if blocker:
            return blocker
        return None

    def _mission_distance_blocker(self):
        # INAV's nav_wp_max_safe_distance: refuse a mission whose first
        # waypoint is far from where we armed, which catches "yesterday's
        # mission, today's field" before it flies away.
        if self._max_safe_distance <= 0:
            return None
        first = self.waypoints[0]
        north, east = local_offsets(
            self.home[0], self.home[1], first["lat"], first["lon"]
        )
        far = distance_m(north, east)
        if far > self._max_safe_distance:
            return "waypoint 1 is %d m away" % int(far)
        return None

    def engage(self):
        self.index = 0
        self.laps = 0
        self.state = STATE_RUN
        self._last_pvt_ms = None

    def disengage(self):
        self.state = STATE_IDLE
        self.pitch_stick = 0.0
        self.yaw_stick = 0.0
        self.throttle = 0.0

    def _leg_start(self):
        # The leg being tracked runs from the previous waypoint to the
        # active one. For waypoint 1 that is normally home -- but on a
        # repeating plan, every lap after the first arrives from the LAST
        # waypoint instead. Getting this wrong would make the cross-track
        # term steer toward a line back to the launch point on every lap.
        if self.index == 0:
            if self.laps and len(self.waypoints) > 1:
                previous = self.waypoints[-1]
                return previous["lat"], previous["lon"]
            return self.home[0], self.home[1]
        previous = self.waypoints[self.index - 1]
        return previous["lat"], previous["lon"]

    def _target(self):
        # Altitude is returned as MSL in every case, because that is the only
        # thing _apply_demands can compare against (gps.alt_m is the GPS's own
        # MSL figure). Resolving the mission's datum here, once, is what keeps
        # every consumer downstream working in a single unit.
        if self.state == STATE_RTH:
            return self.home[0], self.home[1], self.home[2]
        waypoint = self.waypoints[self.index]
        return waypoint["lat"], waypoint["lon"], self.target_alt_msl(waypoint)

    def target_alt_msl(self, waypoint):
        # A relative plan is anchored to the home altitude captured at arm,
        # so the same plan flown at a different field flies at the same
        # height over that field rather than at a fixed sea-level altitude.
        if self.alt_frame == mission_module.ALT_FRAME_AMSL or self.home is None:
            return waypoint["alt_m"]
        return self.home[2] + waypoint["alt_m"]

    def update(self, gps):
        # Recompute only on a fresh NAV-PVT; demands are held between fixes.
        if self.state == STATE_IDLE or gps.last_pvt_ms is None:
            return
        if gps.last_pvt_ms == self._last_pvt_ms:
            return
        self._last_pvt_ms = gps.last_pvt_ms

        if self.state in _LANDING_STATES:
            self._update_landing(gps)
            return

        target_lat, target_lon, target_alt = self._target()
        pos_n, pos_e = local_offsets(
            self.home[0], self.home[1], gps.lat_deg, gps.lon_deg
        )
        target_n, target_e = local_offsets(
            self.home[0], self.home[1], target_lat, target_lon
        )
        to_target_n = target_n - pos_n
        to_target_e = target_e - pos_e
        range_m = distance_m(to_target_n, to_target_e)

        if self.state == STATE_LOITER:
            desired_course = self._loiter_course(pos_n, pos_e, target_n, target_e)
        else:
            if self._advance_if_reached(range_m, pos_n, pos_e, target_n, target_e):
                return  # target changed; next fix flies the new leg
            desired_course = self._track_course(
                pos_n, pos_e, target_n, target_e, to_target_n, to_target_e, range_m
            )

        self._apply_demands(gps, desired_course, target_alt)

    def _advance_if_reached(self, range_m, pos_n, pos_e, target_n, target_e):
        start_lat, start_lon = self._leg_start()
        start_n, start_e = local_offsets(
            self.home[0], self.home[1], start_lat, start_lon
        )
        reached = range_m <= self._wp_radius or passed_waypoint(
            target_n, target_e, start_n, start_e, pos_n, pos_e
        )
        if not reached:
            return False
        if self.state == STATE_RTH:
            self.state = STATE_LOITER
            return True
        if self.index + 1 < len(self.waypoints):
            self.index += 1
            return True
        # Last waypoint reached -- INAV's NAV_STATE_WAYPOINT_FINISHED. What
        # happens next is the mission's own end_action.
        self.laps += 1
        if self.end_action == mission_module.END_REPEAT:
            self.index = 0          # _leg_start now tracks from the last WP
            return True
        if self.end_action == mission_module.END_RTH:
            self.state = STATE_RTH
        elif self.end_action == mission_module.END_LAND:
            self.state = STATE_LAND_APPROACH
        else:
            self.state = STATE_LOITER
        return True

    def _track_course(
        self, pos_n, pos_e, target_n, target_e, to_target_n, to_target_e, range_m,
        start_n=None, start_e=None,
    ):
        # INAV's virtual-target idea in its simplest honest form: chase the
        # bearing to the waypoint, biased to rejoin the leg's track line.
        #
        # start_n/start_e override the leg start for a leg that isn't between
        # two mission waypoints -- the landing final, which runs from a
        # synthesized approach point that was never in the waypoint list.
        direct = bearing_deg(to_target_n, to_target_e)
        if start_n is None:
            start_lat, start_lon = self._leg_start()
            start_n, start_e = local_offsets(
                self.home[0], self.home[1], start_lat, start_lon
            )
        offset = cross_track_m(start_n, start_e, target_n, target_e, pos_n, pos_e)
        correction = -offset * self._xtrack_p
        if correction > self._xtrack_limit:
            correction = self._xtrack_limit
        elif correction < -self._xtrack_limit:
            correction = -self._xtrack_limit
        # Close in, the track line stops being meaningful -- fly the point.
        if range_m < self._wp_radius * 2.0:
            return direct
        track = bearing_deg(target_n - start_n, target_e - start_e)
        return (track + correction) % 360.0

    # --- automatic landing ------------------------------------------------

    def land_entry_offsets(self):
        # Start of the final leg: one approach length back from touchdown
        # along the RECIPROCAL of the landing heading, so that flying entry
        # -> touchdown means flying on the configured heading. Touchdown is
        # home, which is the origin of the local frame, so the entry point
        # is just the reverse-heading vector.
        heading = radians(self._land_heading)
        return (-self._land_approach_length * cos(heading),
                -self._land_approach_length * sin(heading))

    def land_target_alt_rel(self, range_to_touchdown_m):
        # Linear glideslope: approach altitude at the start of the final
        # leg, home altitude at touchdown. Clamped at the top so that being
        # beyond the entry point (a long or overshooting approach) holds the
        # approach altitude rather than commanding a climb above it.
        if self._land_approach_length <= 0.0:
            return 0.0
        fraction = range_to_touchdown_m / self._land_approach_length
        if fraction > 1.0:
            fraction = 1.0
        return self._land_approach_alt * fraction

    def _update_landing(self, gps):
        pos_n, pos_e = local_offsets(
            self.home[0], self.home[1], gps.lat_deg, gps.lon_deg
        )
        entry_n, entry_e = self.land_entry_offsets()

        if self.state == STATE_LAND_APPROACH:
            to_entry_n = entry_n - pos_n
            to_entry_e = entry_e - pos_e
            if distance_m(to_entry_n, to_entry_e) > self._wp_radius:
                # Fly to the entry point at approach altitude, so the
                # glideslope starts from a known height rather than from
                # wherever the mission happened to leave off.
                self._apply_demands(
                    gps,
                    bearing_deg(to_entry_n, to_entry_e),
                    self.home[2] + self._land_approach_alt,
                )
                return
            self.state = STATE_LAND_FINAL

        # Deliberately NOT checked during the approach: down there the
        # aircraft is still being flown up to approach altitude, and a
        # mission that happened to end low would otherwise cut the motor
        # immediately and glide off on the landing heading from wherever it
        # was. INAV likewise only glides once established on final.
        if gps.alt_m - self.home[2] <= self._land_glide_alt:
            self.state = STATE_LAND_GLIDE

        if self.state == STATE_LAND_GLIDE:
            # Motor off, nose fixed slightly down, holding the landing
            # heading. No altitude loop -- there is nothing left to hold
            # altitude with, and chasing a glideslope on GPS altitude this
            # close to the ground would only inject noise into the pitch.
            # Terminal: no path back out re-arms the motor.
            self._steer(gps, self._land_heading)
            self.pitch_stick = (
                -self._land_glide_pitch / self._pitch_scale
                if self._pitch_scale else 0.0
            )
            self.throttle = 0.0
            return

        # Final: track the entry -> touchdown line with the same cross-track
        # correction the mission legs use, descending down the glideslope.
        range_m = distance_m(-pos_n, -pos_e)
        course = self._track_course(
            pos_n, pos_e, 0.0, 0.0, -pos_n, -pos_e, range_m, entry_n, entry_e
        )
        self._apply_demands(
            gps, course, self.home[2] + self.land_target_alt_rel(range_m)
        )

    def _loiter_course(self, pos_n, pos_e, center_n, center_e):
        radial_n = pos_n - center_n
        radial_e = pos_e - center_e
        radius = distance_m(radial_n, radial_e)
        if radius < 1e-3:
            return 0.0
        if radius > self._loiter_radius * 2.0:
            # Far outside the circle: just go to it.
            return bearing_deg(center_n - pos_n, center_e - pos_e)
        angle = atan2(radial_e, radial_n) + radians(_LOITER_LEAD_DEG)
        target_n = center_n + self._loiter_radius * cos(angle)
        target_e = center_e + self._loiter_radius * sin(angle)
        return bearing_deg(target_n - pos_n, target_e - pos_e)

    def _steer(self, gps, desired_course):
        # Course error -> yaw rate. This is the deviation from INAV noted at
        # the top: no roll actuator exists, so the turn demand rides the
        # yaw path instead of a bank angle. Split out from _apply_demands
        # because the glide steers without wanting any of the altitude or
        # throttle logic that follows it.
        course_error = wrap180(desired_course - gps.course_deg)
        yaw_rate = course_error * self._heading_p
        if yaw_rate > self._rate_yaw:
            yaw_rate = self._rate_yaw
        elif yaw_rate < -self._rate_yaw:
            yaw_rate = -self._rate_yaw
        self.yaw_stick = yaw_rate / self._rate_yaw if self._rate_yaw else 0.0

    def _apply_demands(self, gps, desired_course, target_alt):
        self._steer(gps, desired_course)

        # Altitude error -> pitch angle, with INAV's asymmetric climb/dive
        # limits, then normalized into the cascade's stick units.
        pitch_deg = (target_alt - gps.alt_m) * self._alt_p
        climb_limit = min(self._climb_deg, self._pitch_scale)
        dive_limit = min(self._dive_deg, self._pitch_scale)
        if pitch_deg > climb_limit:
            pitch_deg = climb_limit
        elif pitch_deg < -dive_limit:
            pitch_deg = -dive_limit
        self.pitch_stick = pitch_deg / self._pitch_scale if self._pitch_scale else 0.0

        # INAV's fixedWingPitchToThrottleCorrection: hold speed through the
        # climb rather than letting pitch alone bleed it off.
        throttle = self._cruise + pitch_deg * self._pitch_to_throttle / 100.0
        if throttle > self._throttle_max:
            throttle = self._throttle_max
        elif throttle < self._throttle_min:
            throttle = self._throttle_min
        self.throttle = throttle
