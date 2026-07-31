# Autonomous navigation design

Type: grilling (HITL)

## Status

resolved 2026-07-30 (first implementation; unflown)

## Question

Define the autonomous mode: waypoint representation, how the nav loop turns GPS position into attitude/throttle demands for the control loops (cross-track / L1-style guidance vs simple bearing chase), turn coordination without a rudder, heading source without a magnetometer (GPS course-over-ground, low-speed behavior), altitude strategy with GPS-only altitude, waypoint sequencing/advance criteria, and GPS-loss behavior (level out + cut throttle, per owner). Blocked by "Research: BZ-251 UBX configuration" and "Control system design" (demand interfaces).

## Assumptions

- **Transpile from INAV** (surveyed, see "Research: INAV transpile survey"): waypoint execution and guidance from `navigation/navigation.c` (~5466 lines — port the fixed-wing WP subset only), fixed-wing guidance from `navigation/navigation_fixedwing.c` (~939 lines), geo math from `navigation_geo.c`, `sqrt_controller.c`. Skip pos_estimator AGL/flow, geozone, and launch code.
- **Bespoke:** BZ-251 UBX driver (~100 lines, per "Research: BZ-251 UBX configuration") and mission entry (INAV's MSP upload is out of scope — no configurator).
- BZ-251 provides position/velocity/course; no baro. **Correction (2026-07-29):** the BZ-251 module does carry an onboard QMC5883 compass — the earlier "no magnetometer" assumption here and in "Research: MPU6050 attitude estimation" was wrong (both corrected). Whether this part uses GPS-course-only heading (as originally scoped) or adds a real compass-based heading reference is now an open question for the grilling session that resumes this part, not a foregone "no magnetometer exists" constraint.
- Mission entry method is fog (see map "Not yet specified") — this part may graduate it.

## Decision

**Superseded 2026-07-30.** Originally deferred (MVP was manual + stabilized only). Resumed and implemented as `firmware/nav.py`, verified against INAV `master` source rather than from memory.

### The one deviation from INAV, and why

INAV's fixed-wing guidance emits a **bank angle**: `navHeadingError = wrap_18000(virtualTargetBearing - cog)` feeds `navPidApply2(&posControl.pids.fw_nav, …)` clamped to `nav_fw_bank_angle` (35° default), and rudder is supplementary only, gated behind `STATE(FW_HEADING_USE_YAW)`. **This airframe has no roll actuator** (2 V-tail servos, no ailerons — see [Control system design](./control-system-design.md)), so the final conversion is re-derived: course error → **yaw rate demand**, riding the same path `turn_assist_gain` already uses for the roll stick. Everything upstream — virtual-target/cross-track tracking, the waypoint state machine, the two-part reached test — follows INAV.

### What was built

- **Demand interface:** `nav.Navigator` emits `pitch_stick` / `yaw_stick` / `throttle` in exactly the units the RC channels provide, so autonomous is a third demand source into the *same* stabilized cascade, mixer and outputs (`main.py`). Nothing downstream distinguishes it from a pilot.
- **Guidance rate:** recomputed only on a fresh NAV-PVT (5 Hz), held between fixes — the 250 Hz inner loops run on held demands. Mirrors INAV's nav task running below the PID loop rate.
- **Reached test (both, per INAV):** inside `nav_wp_radius_m` (25 m — INAV's 100 cm default is one a fixed wing essentially never enters) **or** past the perpendicular plane through the waypoint. The second test is what actually advances a plane and stops it orbiting a point it cannot hit.
- **Cross-track:** signed perpendicular offset from the leg → bounded course correction (`nav_xtrack_p`, `nav_xtrack_limit_deg`), so it rejoins the track line rather than bow-chasing the point.
- **Altitude:** error → pitch angle with INAV's asymmetric `nav_fw_climb_angle` 20° / `nav_fw_dive_angle` 15° limits, plus `nav_fw_pitch2thr`-style throttle coupling over a cruise/min/max band (40/20/70 %, converted from INAV's 1400/1200/1700 µs).
- **Engage gates** (INAV's `NAV_STATE_WAYPOINT_INITIALIZE` equivalents): fix OK, satellite count, horizontal accuracy, **minimum ground speed** (standing in for `estHeadingStatus`, since course-over-ground is meaningless when slow), a stored mission, a home position, and INAV's `nav_wp_max_safe_distance` check that waypoint 1 is near the arming point. A blocked engage **degrades to stabilized under pilot control** — the switch position never leaves the aircraft uncommanded — and prints the reason.
- **Home:** first good fix after arming (INAV's `GPS_FIX_HOME`), set once per armed session.
- **End of mission:** `nav_end_action` — orbit the last waypoint (default) or fly home and orbit. Chosen over INAV's per-waypoint action types (`WAYPOINT`/`RTH`/`LAND`/`JUMP`/…), which need a richer mission schema than the tap-to-place editor produces; revisit if missions get more complex.
- **GPS loss:** `_NAV_GPS_TIMEOUT_MS` = 5 s, matching INAV's `pos_failure_timeout`, then the owner's recorded procedure — level wings, cut throttle.

### Resolved fog

Mission entry is the web portal's tap-to-place editor (`mission.py` + `/mission`), so that fog item is closed. Heading is **GPS course-over-ground**, gated on ground speed — the QMC5883 compass is now driven (`compass.py`) and shown on the status page, but is **not yet fused into heading**; INAV requires a compass for WP mode, so this remains the most valuable next upgrade. GPS-only altitude stands (no baro — INAV requires one; expect worse altitude hold than these ported gains assume).

### Open / carried forward

- **Unflown.** Validated by 22 off-target tests including a kinematic simulation flying a 4-waypoint box, holding a loiter circle, climbing, and rejoining track from 150 m off-line. A simulation is not an airframe.
- **GPS-loss procedure** deviates from INAV, which flies a controlled descent at `emerg_descent_rate` rather than cutting throttle. Owner decision stands; worth revisiting now that the mode actually exists.
- Compass fusion (tilt compensation, hard/soft-iron calibration) for low-speed and ground heading.
- No airspeed sensor, so INAV's `pidTurnAssistant` physics and `nav_min_ground_speed` throttle boost remain approximated.
