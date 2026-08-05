# Parameter schema, validation, and atomic flash persistence.
# Pure Python (no `machine` imports) so it is testable off-target.

import os

CONFIG_FILE = "config.json"
_TMP_FILE = "config.json.tmp"
_BACKUP_FILE = "config.json.bak"

# name -> (default, minimum, maximum, step). All numeric except the two WiFi
# string entries (minimum/maximum/step all None for those). The web form
# renders each numeric entry as a slider using min/max/step directly. Ranges
# are sanity bounds, not flight-tuning advice.
SCHEMA = {
    # PID+FF gains, stabilized mode: pitch is a cascaded angle->rate loop
    # (outer level-P -> inner rate PID+FF), yaw is rate-only (no outer loop,
    # matching INAV's fixed wing -- yaw never gets an angle mode). No roll
    # axis/actuator exists (2 V-tail servos, no ailerons) -- roll stick
    # instead feeds turn_assist_gain below.
    #
    # Raw gain values and defaults are taken directly from INAV `master`
    # (`src/main/fc/settings.yaml`: fw_p/i/d/ff_pitch, fw_p/i/d/ff_yaw,
    # fw_p_level) -- main.py divides each by INAV's own scale constants
    # (`flight/pid.h`: FP_PID_RATE_P/I/D/FF_MULTIPLIER, FP_PID_LEVEL_P_MULTIPLIER)
    # before use, so these numbers and ranges match INAV's real tuning surface
    # instead of being made up for our own unit convention.
    "pid_pitch_p": (5.0, 0.0, 255.0, 1.0),
    "pid_pitch_i": (7.0, 0.0, 255.0, 1.0),
    "pid_pitch_d": (0.0, 0.0, 255.0, 1.0),
    "pid_pitch_ff": (50.0, 0.0, 255.0, 1.0),
    "pid_yaw_p": (6.0, 0.0, 255.0, 1.0),
    "pid_yaw_i": (10.0, 0.0, 255.0, 1.0),
    "pid_yaw_d": (0.0, 0.0, 255.0, 1.0),
    "pid_yaw_ff": (60.0, 0.0, 255.0, 1.0),
    # Outer level loop (pitch only): angle error (deg) * this gain -> a rate
    # target (deg/s), clamped to +/- pitch_rate_limit_dps. INAV default 20.
    "pitch_level_p": (20.0, 0.0, 255.0, 1.0),
    # Ceiling on the level loop's rate demand, degrees/sec (INAV: configured
    # acro rate * 10; we expose the ceiling directly instead).
    "pitch_rate_limit_dps": (200.0, 10.0, 1000.0, 1.0),
    # Angle mode: max commanded pitch angle at full stick, degrees.
    "pitch_angle_max_deg": (30.0, 5.0, 60.0, 1.0),
    # Rate mode: yaw stick scale, degrees per second at full stick.
    "rate_yaw_dps": (90.0, 10.0, 500.0, 1.0),
    # Turn coordination: roll stick adds this much yaw-rate demand (deg/s at
    # full stick), always on in stabilized mode. Linear approximation of
    # INAV's pidTurnAssistant (which needs airspeed we don't have).
    "turn_assist_gain": (60.0, 0.0, 300.0, 1.0),
    # V-tail mixer gains: servo = pitch_term * pitch + yaw_term * yaw
    "mixer_left_pitch": (0.5, -1.0, 1.0, 0.01),
    "mixer_left_yaw": (0.5, -1.0, 1.0, 0.01),
    "mixer_right_pitch": (0.5, -1.0, 1.0, 0.01),
    "mixer_right_yaw": (-0.5, -1.0, 1.0, 0.01),
    # Servo endpoints, microseconds
    "servo_left_min_us": (1000.0, 500.0, 2500.0, 1.0),
    "servo_left_max_us": (2000.0, 500.0, 2500.0, 1.0),
    "servo_right_min_us": (1000.0, 500.0, 2500.0, 1.0),
    "servo_right_max_us": (2000.0, 500.0, 2500.0, 1.0),
    # RC channel assignments (1-indexed CRSF channel numbers)
    "channel_roll": (1.0, 1.0, 16.0, 1.0),
    "channel_pitch": (2.0, 1.0, 16.0, 1.0),
    "channel_throttle": (3.0, 1.0, 16.0, 1.0),
    "channel_yaw": (4.0, 1.0, 16.0, 1.0),
    "channel_arm": (5.0, 1.0, 16.0, 1.0),
    "channel_mode": (6.0, 1.0, 16.0, 1.0),
    # Arming: throttle must be below this (microseconds) to arm
    "arm_max_throttle_us": (1050.0, 950.0, 1200.0, 1.0),
    # Failsafe: link considered lost after this silence (milliseconds)
    "failsafe_link_timeout_ms": (500.0, 100.0, 5000.0, 10.0),
    # --- Autonomous navigation (see nav.py) -------------------------------
    # Defaults track INAV's fixed-wing nav settings where an equivalent
    # exists (`docs/Settings.md`), converted to this build's units: percent
    # rather than PWM microseconds for throttle, metres rather than
    # centimetres for distances.
    #
    # Throttle band: INAV nav_fw_cruise_thr 1400 / min 1200 / max 1700 us
    # over a 1000-2000 us range = 40% / 20% / 70%.
    "nav_cruise_throttle_pct": (40.0, 0.0, 100.0, 1.0),
    "nav_min_throttle_pct": (20.0, 0.0, 100.0, 1.0),
    "nav_max_throttle_pct": (70.0, 0.0, 100.0, 1.0),
    # INAV nav_fw_climb_angle 20 deg, nav_fw_dive_angle 15 deg -- deliberately
    # asymmetric (diving builds speed far faster than climbing sheds it).
    "nav_climb_angle_deg": (20.0, 5.0, 45.0, 1.0),
    "nav_dive_angle_deg": (15.0, 5.0, 45.0, 1.0),
    # INAV's nav_wp_radius default is 100 cm, which a fixed wing essentially
    # never enters -- upstream relies on the "passed the waypoint" test for
    # the real advance. A radius sized to the airframe makes the primary
    # test do useful work too.
    "nav_wp_radius_m": (25.0, 5.0, 200.0, 1.0),
    # INAV nav_wp_max_safe_distance 100 m: refuse a mission whose first
    # waypoint is far from the arming point. 0 disables the check.
    "nav_max_safe_distance_m": (200.0, 0.0, 2000.0, 10.0),
    # Course error (deg) -> yaw rate demand (deg/s). Replaces INAV's
    # heading-error-to-bank-angle PID; this airframe has no roll actuator.
    "nav_heading_p": (1.5, 0.1, 10.0, 0.1),
    # Cross-track: metres off the leg -> degrees of course correction,
    # capped so a large offset never commands a reversal.
    "nav_xtrack_p": (1.0, 0.0, 10.0, 0.1),
    "nav_xtrack_limit_deg": (45.0, 0.0, 90.0, 1.0),
    # Altitude error (m) -> pitch angle (deg), before the climb/dive clamps.
    "nav_alt_p": (0.5, 0.05, 5.0, 0.05),
    # INAV nav_fw_pitch2thr: throttle percent added per degree of pitch.
    "nav_pitch_to_throttle": (1.0, 0.0, 5.0, 0.1),
    # INAV nav_fw_loiter_radius 7500 cm.
    "nav_loiter_radius_m": (75.0, 20.0, 300.0, 5.0),
    # Below this ground speed GPS course-over-ground is meaningless, so nav
    # refuses to engage (stands in for INAV's compass/heading gate).
    "nav_min_ground_speed_ms": (5.0, 1.0, 20.0, 0.5),
    # Fix-quality gates before autonomous mode will engage.
    "nav_min_sats": (7.0, 4.0, 20.0, 1.0),
    "nav_max_h_acc_m": (10.0, 1.0, 50.0, 1.0),
    # --- Automatic landing (mission end_action "land", see nav.py) ---------
    # Structure follows INAV's fixed-wing landing (docs/Fixed Wing
    # Landing.md): fly to a point one approach-length back from the
    # touchdown point along the reciprocal of the landing heading, track the
    # final leg down a glideslope, then cut the motor and glide in.
    #
    # INAV's own sequence has three more phases than this one. It measures
    # wind in a 30 s loiter and picks the headwind direction of up to four;
    # here the direction is this parameter, set for the day's wind before
    # flight. It also has a flare phase, which upstream documents as
    # LIDAR/rangefinder-dependent -- without one INAV likewise ends at the
    # glide, which is exactly what this airframe does.
    #
    # Course flown on final approach, degrees. Set it INTO WIND on the day:
    # nothing here estimates wind, and a downwind approach lands long and
    # fast. 0 = touching down heading north.
    "nav_land_heading_deg": (0.0, 0.0, 359.0, 1.0),
    # Distance from touchdown back to the start of the final leg. INAV's
    # nav_fw_land_approach_length default is 35000 cm.
    "nav_land_approach_length_m": (350.0, 50.0, 1000.0, 10.0),
    # Height above home to fly the approach leg at, before the glideslope
    # starts down.
    "nav_land_approach_alt_m": (50.0, 10.0, 200.0, 5.0),
    # Height above home at which the motor stops and the glide begins.
    # INAV's nav_fw_land_glide_alt default is 200 cm -- which assumes a
    # barometer. This build has GPS altitude only, whose vertical accuracy
    # (gps.v_acc_m) is typically 3-10 m, so a 2 m trigger sits below the
    # noise floor and would fire late or not at all. Defaulted an order of
    # magnitude higher so the decision is made on a figure the sensor can
    # actually resolve; the cost is a longer powerless glide.
    "nav_land_glide_alt_m": (20.0, 5.0, 60.0, 1.0),
    # Nose-down attitude held through the glide, degrees. This sets where
    # the aircraft actually touches down, and it matters far more here than
    # upstream: INAV defaults nav_fw_land_glide_pitch to 0 (hold level and
    # sink), which is fine when the glide only starts 2 m up, but this build
    # starts it at nav_land_glide_alt_m because GPS altitude cannot resolve
    # 2 m -- so the glide is a long float, and its length is set by this.
    #
    # Float from a 20 m glide start at 15 m/s, simulated in test_nav.py:
    #    5 deg -> 1.3 m/s sink, 225 m float, touchdown ~200 m past home
    #   10 deg -> 2.6 m/s sink, 111 m float, touchdown  ~85 m past home
    #   15 deg -> 3.9 m/s sink,  75 m float, touchdown  ~50 m past home
    # Steeper lands nearer the mark and arrives harder. 10 is the
    # compromise; PLAN FOR THE FIELD TO BE CLEAR WELL BEYOND THE TOUCHDOWN
    # POINT, because there is no flare and nothing here shortens the float.
    "nav_land_glide_pitch_deg": (10.0, 0.0, 30.0, 1.0),
    # WiFi AP shown while disarmed
    "wifi_ssid_suffix": ("pico-wing", None, None, None),
    "wifi_password": ("picowing", None, None, None),
}

_STRING_KEYS = frozenset(
    name for name, bounds in SCHEMA.items() if bounds[1] is None
)

# WPA2 requires an 8-63 character passphrase; a shorter one makes
# ap.config() fail at boot with the bad value already persisted -- a
# web-unreachable brick. SSIDs cap at 32 chars and ours carry a
# "pico-wing-" (10 char) prefix.
_WIFI_PASSWORD_MIN = 8
_WIFI_PASSWORD_MAX = 63
_SSID_SUFFIX_MAX = 22

_CHANNEL_KEYS = (
    "channel_roll", "channel_pitch", "channel_throttle",
    "channel_yaw", "channel_arm", "channel_mode",
)
_SERVO_ENDPOINT_PAIRS = (
    ("servo_left_min_us", "servo_left_max_us"),
    ("servo_right_min_us", "servo_right_max_us"),
)


def defaults():
    return {name: bounds[0] for name, bounds in SCHEMA.items()}


def _string_ok(name, text):
    if name == "wifi_password":
        return _WIFI_PASSWORD_MIN <= len(text) <= _WIFI_PASSWORD_MAX
    if name == "wifi_ssid_suffix":
        return 1 <= len(text) <= _SSID_SUFFIX_MAX
    return True


def _apply_cross_field_rules(validated, fallback):
    # Per-field clamping can't catch relationships: inverted servo endpoints
    # pin the surface at one end (min(max(x, lo), hi) with lo > hi is
    # constant), and duplicate channel assignments make two controls read the
    # same stick. Revert the offending group to the fallback (previous
    # config, or schema defaults), which is known-good.
    for min_key, max_key in _SERVO_ENDPOINT_PAIRS:
        if validated[min_key] >= validated[max_key]:
            validated[min_key] = fallback[min_key]
            validated[max_key] = fallback[max_key]
        if validated[min_key] >= validated[max_key]:
            validated[min_key] = SCHEMA[min_key][0]
            validated[max_key] = SCHEMA[max_key][0]
    assigned = [validated[key] for key in _CHANNEL_KEYS]
    if len(set(assigned)) != len(assigned):
        for key in _CHANNEL_KEYS:
            validated[key] = fallback[key]
        assigned = [validated[key] for key in _CHANNEL_KEYS]
        if len(set(assigned)) != len(assigned):
            for key in _CHANNEL_KEYS:
                validated[key] = SCHEMA[key][0]


def validate(raw_values, base=None):
    # base: the config to merge onto. Defaults to schema defaults (boot-time
    # file load); the web form passes the live config so a partial or
    # malformed POST leaves unmentioned/unparsable parameters at their
    # current values instead of silently factory-resetting them.
    fallback = defaults() if base is None else base
    validated = dict(fallback)
    for name, value in raw_values.items():
        if name not in SCHEMA:
            continue
        if name in _STRING_KEYS:
            text = str(value)
            if _string_ok(name, text):
                validated[name] = text
            continue
        _, minimum, maximum, _ = SCHEMA[name]
        try:
            number = float(value)
        except (TypeError, ValueError):
            continue
        validated[name] = min(max(number, minimum), maximum)
    _apply_cross_field_rules(validated, fallback)
    return validated


def load():
    try:
        import json

        with open(CONFIG_FILE) as config_file:
            return validate(json.load(config_file))
    except (OSError, ValueError):
        pass
    try:
        import json

        with open(_BACKUP_FILE) as backup_file:
            return validate(json.load(backup_file))
    except (OSError, ValueError):
        return defaults()


def save(config):
    import json

    validated = validate(config)
    with open(_TMP_FILE, "w") as tmp_file:
        json.dump(validated, tmp_file)
    try:
        os.remove(_BACKUP_FILE)
    except OSError:
        pass
    try:
        os.rename(CONFIG_FILE, _BACKUP_FILE)
    except OSError:
        pass
    os.rename(_TMP_FILE, CONFIG_FILE)
    return validated


def factory_reset():
    for path in (CONFIG_FILE, _TMP_FILE, _BACKUP_FILE):
        try:
            os.remove(path)
        except OSError:
            pass
