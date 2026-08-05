# Waypoint mission storage for the ground-station portal. Pure Python (no
# `machine` imports) so it is testable off-target; same atomic tmp/bak flash
# persistence pattern as config.py.
#
# Validation is strict (raise, don't repair): unlike config, where falling
# back per-field is safe, silently dropping or clamping a waypoint would
# change the flight plan -- a mission either saves exactly as given or the
# save is rejected with a reason the UI can show.
#
# The portal writes this; nav.Navigator reads it. main.py re-reads the file
# at each arm rather than at boot, so a plan saved from the portal flies
# without a reboot -- which is also why validation has to hold the line here
# rather than trusting whatever last reached flash.

import os

MISSION_FILE = "mission.json"
_TMP_FILE = "mission.json.tmp"
_BACKUP_FILE = "mission.json.bak"

MAX_WAYPOINTS = 20

_LAT_RANGE = (-90.0, 90.0)
_LON_RANGE = (-180.0, 180.0)

# Which datum every waypoint altitude in this plan is measured against.
# Stored with the mission, like end_action, because it is a property of how
# the plan was drawn rather than of the airframe.
#
# Without this the number was ambiguous, and nav.py resolved the ambiguity
# by comparing it straight against the GPS MSL altitude -- so "100" meant
# 100 m above sea level, while the editor's default of 100 m and its 0-500
# range both read as height above the field. At a field 200 m up that is a
# commanded dive into terrain. INAV (waypoint p3 bit 0) and ArduPilot
# (MAV_FRAME_GLOBAL vs GLOBAL_RELATIVE_ALT) both make the datum explicit
# per waypoint and both default to home-relative; this is the same idea at
# mission granularity, which is all an airframe with no terrain data needs.
ALT_FRAME_REL = "rel"     # metres above the home altitude captured at arm
ALT_FRAME_AMSL = "amsl"   # metres above mean sea level, as the GPS reports
ALT_FRAMES = (ALT_FRAME_REL, ALT_FRAME_AMSL)
DEFAULT_ALT_FRAME = ALT_FRAME_REL

# Sane bounds differ by datum: 500 m over the field is already far beyond
# this airframe, but as an absolute it would refuse any field above 500 m.
_ALT_RANGE_BY_FRAME = {
    ALT_FRAME_REL: (0.0, 500.0),
    ALT_FRAME_AMSL: (-100.0, 5000.0),
}

# What happens once the last waypoint is reached. Stored with the mission
# rather than in config.SCHEMA because it is a property of this plan, not of
# the airframe -- loading a different mission should bring its own ending.
END_LOITER = "loiter"   # orbit the last waypoint
END_RTH = "rth"         # fly home, then orbit there
END_REPEAT = "repeat"   # fly back to waypoint 1 and run the plan again
END_LAND = "land"       # fly the approach and glide in at home
END_ACTIONS = (END_LOITER, END_RTH, END_REPEAT, END_LAND)
DEFAULT_END_ACTION = END_LOITER


def empty():
    return {
        "waypoints": [],
        "end_action": DEFAULT_END_ACTION,
        "alt_frame": DEFAULT_ALT_FRAME,
    }


def _number(entry, key, index, bounds):
    try:
        value = float(entry[key])
    except (KeyError, TypeError, ValueError):
        raise ValueError("waypoint %d: missing or non-numeric %s" % (index, key))
    low, high = bounds
    if not low <= value <= high:  # NaN fails this comparison too
        raise ValueError(
            "waypoint %d: %s must be between %g and %g" % (index, key, low, high)
        )
    return value


def validate(raw):
    if not isinstance(raw, dict) or not isinstance(raw.get("waypoints"), list):
        raise ValueError('mission must be {"waypoints": [...]}')
    waypoints = raw["waypoints"]
    if len(waypoints) > MAX_WAYPOINTS:
        raise ValueError("too many waypoints (max %d)" % MAX_WAYPOINTS)
    # Absent means a mission file saved before the datum was recorded. Those
    # were interpreted as MSL, but they were *drawn* in an editor that
    # defaults to 100 m and caps at 500 -- i.e. as height over the field.
    # Defaulting to relative agrees with how they were meant, and with what
    # INAV and ArduPilot both default to; a plan genuinely planned in MSL has
    # to be re-saved with the datum set.
    alt_frame = raw.get("alt_frame", DEFAULT_ALT_FRAME)
    if alt_frame not in ALT_FRAMES:
        raise ValueError("alt_frame must be one of %s" % ", ".join(ALT_FRAMES))
    alt_range = _ALT_RANGE_BY_FRAME[alt_frame]
    validated = []
    for index, entry in enumerate(waypoints, 1):
        if not isinstance(entry, dict):
            raise ValueError("waypoint %d: not an object" % index)
        validated.append({
            "lat": _number(entry, "lat", index, _LAT_RANGE),
            "lon": _number(entry, "lon", index, _LON_RANGE),
            "alt_m": _number(entry, "alt_m", index, alt_range),
        })
    # Absent means an older mission file, saved before the ending was
    # selectable: default rather than reject, so an existing plan still loads.
    end_action = raw.get("end_action", DEFAULT_END_ACTION)
    if end_action not in END_ACTIONS:
        raise ValueError(
            "end_action must be one of %s" % ", ".join(END_ACTIONS))
    if end_action == END_REPEAT and len(validated) < 2:
        raise ValueError("repeat needs at least 2 waypoints")
    return {
        "waypoints": validated,
        "end_action": end_action,
        "alt_frame": alt_frame,
    }


def load():
    for path in (MISSION_FILE, _BACKUP_FILE):
        try:
            import json

            with open(path) as mission_file:
                return validate(json.load(mission_file))
        except (OSError, ValueError):
            continue
    return empty()


def save(mission):
    import json

    validated = validate(mission)
    with open(_TMP_FILE, "w") as tmp_file:
        json.dump(validated, tmp_file)
    try:
        os.remove(_BACKUP_FILE)
    except OSError:
        pass
    try:
        os.rename(MISSION_FILE, _BACKUP_FILE)
    except OSError:
        pass
    os.rename(_TMP_FILE, MISSION_FILE)
    return validated
