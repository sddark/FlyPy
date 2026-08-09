# Entry point and asyncio main loop.
#
# State machine:
#   DISARMED -> config portal (WiFi AP + web server) runs; flight outputs idle.
#   ARMED    -> portal torn down, 250 Hz flight loop runs: RC in, attitude
#               estimate, mixer, servos + Oneshot125 ESC out. Failsafe: RC
#               link lost -> level wings (stabilized loop, 0 deg pitch
#               target) + cut throttle, in every mode; if the link stays
#               dead past _LINK_LOST_DISARM_MS the loop gives up and
#               returns to the portal (throttle has long been cut, and a
#               dead RX must not require a power cycle to reach config).
# Arming: fresh RC frame (within the failsafe timeout) AND TX arm switch AND
# throttle below arm_max_throttle_us (owner rule). Disarming returns to the
# config portal. Any exception inside the armed loop still forces outputs
# safe (try/finally) -- PWM is hardware, so without that a crash would leave
# the motor spinning at its last commanded throttle.
#
# Mode handling: manual = raw passthrough. stabilized = INAV-style fixed-wing
# cascade: pitch is an outer angle (level) loop feeding an inner rate loop,
# yaw is rate-only (INAV never gives yaw an angle mode either). Roll stick
# has no axis of its own (no roll actuator -- see control-system-design.md)
# and instead feeds the yaw rate target for turn coordination.
#
# autonomous = the nav.Navigator synthesizes the same pitch/yaw/throttle
# demands the pilot's sticks would, feeding the identical stabilized
# cascade -- it is a third demand source, not a separate control path. It
# engages only through nav.py's gates (fix quality, heading validity, a
# valid mission near home); a blocked engage degrades to stabilized under
# pilot control rather than leaving the aircraft uncommanded. GPS lost
# while navigating levels the wings and cuts throttle, per the owner's
# recorded failsafe decision.
#
# Gain scaling matches INAV `master` exactly (`flight/pid.c`, `flight/pid.h`):
# raw config gains are INAV's own settings.yaml numbers, divided by INAV's
# scale constants below before use, summed (P+I+D+FF) and clamped to
# INAV's +/-500 pidSumLimit, then normalized to our own [-1, 1] mixer
# convention. See control-system-design.md for full citations.

import asyncio
import gc
import time
from math import degrees

# server first, deliberately. It pulls in microdot -- ~60 KB of source, by
# far the largest thing MicroPython has to compile here -- and compiling it
# needs a big contiguous working block. Importing it before the rest of the
# firmware carves up the heap is the difference between booting and dying
# with "MemoryError: allocating 3112 bytes" inside microdot's own import.
# Same principle as claiming the UART ring buffers first in main().
import server  # isort:skip

import auxpins
import config as config_module
import flightlog
import logic as logic_module
import mission as mission_module
import nav as nav_module
import pins
from attitude import MahonyFilter
from compass import QMC5883, flat_field_angle_deg
from esc import LOOP_FREQUENCY_HZ, EscOutput
from gps import UbxGps
from imu import MPU6050
from led import StatusLed
from mixer import VTailMixer
from nav import Navigator
from pid import PID
from rc import CrsfReceiver
from servos import ServoOutput

# Single source: the Oneshot125 slice trick requires the ESC PWM period to
# equal the flight-loop period, so the rate is defined once, in esc.py.
FLIGHT_LOOP_HZ = LOOP_FREQUENCY_HZ
_LOOP_PERIOD_S = 1.0 / FLIGHT_LOOP_HZ
_LOOP_PERIOD_US = 1_000_000 // FLIGHT_LOOP_HZ
# dt fed to the estimator/PIDs is measured, not assumed (the loop sleeps a
# fixed remainder, so real period = body time + sleep); clamp insane gaps
# (debugger pause, one-off stall) back to nominal instead of integrating
# across them.
_MAX_DT_S = 0.05
_DISARMED_POLL_MS = 100
_GYRO_CALIBRATION_SAMPLES = 200
# The GPS emits NAV-PVT at 5 Hz, so polling it every flight-loop iteration
# read an empty UART almost every time -- and that turned out to cost 1942 us
# a call, measured on the board, because draining the UART and running the
# framing scan is not free just because it finds nothing.
#
# 100 ms, which is the interval gps.py already sizes its 1 KB rxbuf for, and
# still twice the 5 Hz fix rate. It has to be longer than an iteration takes
# or it throttles nothing: at the ~18 Hz this loop currently achieves an
# iteration is ~55 ms, so a 50 ms interval would fire every single pass and
# save exactly nothing. Revisit if the loop ever gets fast enough that this
# starts costing fix latency.
_GPS_POLL_INTERVAL_MS = 100
# gc.mem_free() walks the entire heap: 6843 us on this board. It was being
# called at the 25 Hz logic rate to fill one status field, which is ~17% of
# all CPU spent on a number nothing controls on. Sample it once a second and
# hold the value between samples.
_MEM_FREE_INTERVAL_MS = 1000
# Iterations between scheduled collections. The loop body allocates ~1.8 KB
# per pass -- measured on the board, and unavoidable because this build boxes
# every float at 32 bytes (attitude.update alone accounts for 848 of it). At
# ~53 Hz against the ~38 KB free at arm, the heap fills roughly every 21
# iterations, so a collection fires several times a second WHEREVER the
# allocation that triggered it happens to be. The flight log caught the
# result: a 38-45 ms worst-case iteration against an 18 ms average.
#
# Collecting on a fixed cadence, immediately AFTER the servo and ESC writes,
# does not reduce the total time spent collecting -- the scan is proportional
# to heap size, not to garbage. What it does is move the stall out of the
# control path, so it lands between control updates instead of delaying one.
# That is the part felt as jitter.
#
# 16 keeps a comfortable margin under the ~21 the heap actually allows, so
# the scheduled collection almost always wins the race against an unplanned
# one, without collecting so often that the fixed scan cost dominates.
_GC_EVERY_ITERATIONS = 16
# Armed with the link dead this long -> throttle has been cut the whole
# time; return to the portal rather than staying wedged in a loop only a
# live RC frame could exit.
_LINK_LOST_DISARM_MS = 30_000
# INAV's pos_failure_timeout: how long a stale/absent position estimate is
# tolerated in autonomous before the failsafe procedure runs.
_NAV_GPS_TIMEOUT_MS = 5000

_PORTAL_RESTART_DELAY_MS = 250
_PORTAL_MAX_FAILURES = 5

# Hardware watchdog, started at the first arm (see _Watchdog for why not at
# boot). The armed loop feeds it every iteration (250 Hz) and the disarmed
# loop every poll (10 Hz), so it fires only if the asyncio scheduler itself
# stops advancing -- the one failure the try/finally below cannot cover,
# because a wedged loop never reaches a finally at all and PWM is hardware:
# the motor would hold its last commanded throttle indefinitely. A reset lands
# in the portal with outputs safe and, because the arm switch is still ON and
# arm_switch_seen_off starts False, deliberately does NOT re-arm. In flight
# that means a dead-stick glide -- worse than ArduPilot, which preserves state
# across a watchdog reset and keeps flying, and better than a motor stuck at
# cruise. INAV carries no watchdog at all.
#
# 4 s: comfortably above any plausible microdot chunked send, while still
# bounding a lockup to well under the time it takes to lose sight of a plane.
# The AP-start path is bounded at 5 s but only runs while disarmed, where on
# the first pass the watchdog does not yet exist and on later passes the poll
# loop it returns to is already feeding it.
_WATCHDOG_TIMEOUT_MS = 4000

# After the armed loop dies of an exception the arm switch is usually still
# ON. Re-arming straight into the same fault would spin crash -> portal ->
# crash as fast as the IMU can fail, so recovery demands the switch be
# cycled OFF first -- the same gate rc.py already applies at boot.
_FLIGHT_LOOP_RESTART_DELAY_MS = 500

# INAV flight/pid.h scale constants (fixed-wing PIFF controller).
_PID_P_MULTIPLIER = 31.0
_PID_I_MULTIPLIER = 4.0
_PID_D_MULTIPLIER = 1905.0
_PID_FF_MULTIPLIER = 31.0
_LEVEL_P_MULTIPLIER = 1.0 / 6.56
# INAV's pidSumLimit default (flight/pid.c getPidSumLimit()); the summed
# P+I+D+FF clamps to this before being normalized to our own [-1, 1] scale.
_PID_SUM_LIMIT = 500.0


def _pid_gains(config, prefix):
    return (
        config["pid_" + prefix + "_p"] / _PID_P_MULTIPLIER,
        config["pid_" + prefix + "_i"] / _PID_I_MULTIPLIER,
        config["pid_" + prefix + "_d"] / _PID_D_MULTIPLIER,
        config["pid_" + prefix + "_ff"] / _PID_FF_MULTIPLIER,
    )


def _scaled_pid(config, prefix, output_limit=_PID_SUM_LIMIT):
    kp, ki, kd, kff = _pid_gains(config, prefix)
    return PID(kp, ki, kd, kff=kff, output_limit=output_limit)


def _loop_gains(config):
    # The handful of values the 250 Hz path reads directly. Re-read only when
    # user logic changes something, so the fast path keeps using locals.
    return (
        config["failsafe_link_timeout_ms"],
        config["pitch_angle_max_deg"],
        config["pitch_level_p"],
        config["pitch_rate_limit_dps"],
        config["rate_yaw_dps"],
        config["turn_assist_gain"],
    )


# [value, sampled_at_ms]. A list rather than globals so the flight loop pays
# one subscript instead of a global store; None means never sampled.
_mem_free_sample = [0, None]


def _sampled_mem_free(now_ms):
    # See _MEM_FREE_INTERVAL_MS: the call itself is the expensive part, so
    # the point is to make it rarely, not to make it cheaper.
    if (_mem_free_sample[1] is None
            or time.ticks_diff(now_ms, _mem_free_sample[1]) >= _MEM_FREE_INTERVAL_MS):
        _mem_free_sample[0] = gc.mem_free()
        _mem_free_sample[1] = now_ms
    return _mem_free_sample[0]


def _fill_logic_namespace(ns, rc, gps, navigator, attitude, accel_g, gyro_rad_s,
                          flight_mode, aux, compass_reading, now_ms, armed_ms,
                          dt_s, loop_hz):
    # Reused dict, updated in place: at 25 Hz the cost is negligible either
    # way, but this keeps the flight loop free of per-pass allocation.
    channels = rc.channels
    ns["throttle"] = channels["throttle"]
    ns["pitch"] = channels["pitch"]
    ns["roll"] = channels["roll"]
    ns["yaw"] = channels["yaw"]
    ns["throttle_us"] = rc.throttle_us
    ns["arm_switch"] = rc.arm_switch_on
    ns["mode"] = rc.mode
    ns["flight_mode"] = flight_mode
    ns["armed"] = True
    ns["link_ok"] = rc.link_alive
    ns["link_age_ms"] = time.ticks_diff(now_ms, rc.last_frame_ms)
    raw = rc.raw_channels
    for index in range(16):
        ns["ch%d" % (index + 1)] = raw[index]

    ns["roll_deg"] = attitude.roll_deg
    ns["pitch_deg"] = attitude.pitch_deg
    ns["roll_rate"] = degrees(gyro_rad_s[0])
    ns["pitch_rate"] = degrees(gyro_rad_s[1])
    ns["yaw_rate"] = degrees(gyro_rad_s[2])
    ax, ay, az = accel_g
    ns["accel_x"] = ax
    ns["accel_y"] = ay
    ns["accel_z"] = az
    ns["accel_mag"] = (ax * ax + ay * ay + az * az) ** 0.5

    ns["compass_ok"] = compass_reading is not None
    if compass_reading is None:
        ns["heading_deg"] = 0.0
        ns["mag_x"] = ns["mag_y"] = ns["mag_z"] = 0.0
    else:
        mx, my, mz, heading = compass_reading
        ns["mag_x"] = mx
        ns["mag_y"] = my
        ns["mag_z"] = mz
        ns["heading_deg"] = heading

    ns["gps_fix"] = gps.fix_ok
    ns["fix_type"] = gps.fix_type
    ns["sats"] = gps.num_sv
    ns["lat"] = gps.lat_deg
    ns["lon"] = gps.lon_deg
    ns["gps_alt_m"] = gps.alt_m
    ns["ground_speed_ms"] = gps.ground_speed_ms
    ns["course_deg"] = gps.course_deg
    ns["h_acc_m"] = gps.h_acc_m
    ns["v_acc_m"] = gps.v_acc_m
    ns["course_acc_deg"] = gps.course_acc_deg
    ns["gps_age_ms"] = (
        -1 if gps.last_pvt_ms is None
        else time.ticks_diff(now_ms, gps.last_pvt_ms)
    )

    ns["nav_state"] = navigator.state
    # 1-based to match the numbering the Mission page shows.
    ns["wp_index"] = navigator.index + 1
    ns["wp_total"] = len(navigator.waypoints)
    _fill_nav_geometry(ns, navigator, gps)

    ns["armed_ms"] = armed_ms
    ns["uptime_ms"] = now_ms
    ns["loop_hz"] = loop_hz
    ns["dt_s"] = dt_s
    ns["mem_free"] = _sampled_mem_free(now_ms)

    for number in auxpins.ADC_PINS:
        ns["adc%d" % number] = aux.read_adc(number)


def _fill_nav_geometry(ns, navigator, gps):
    home = navigator.home
    if home is None or not gps.fix_ok:
        ns["home_lat"] = ns["home_lon"] = ns["home_alt_m"] = 0.0
        ns["home_distance_m"] = ns["home_bearing_deg"] = 0.0
        ns["alt_above_home_m"] = 0.0
        ns["wp_distance_m"] = ns["wp_bearing_deg"] = ns["xtrack_m"] = 0.0
        return
    ns["home_lat"] = home[0]
    ns["home_lon"] = home[1]
    ns["home_alt_m"] = home[2]
    ns["alt_above_home_m"] = gps.alt_m - home[2]
    north, east = nav_module.local_offsets(home[0], home[1], gps.lat_deg, gps.lon_deg)
    ns["home_distance_m"] = nav_module.distance_m(north, east)
    ns["home_bearing_deg"] = nav_module.bearing_deg(-north, -east)
    if not navigator.waypoints or navigator.index >= len(navigator.waypoints):
        ns["wp_distance_m"] = ns["wp_bearing_deg"] = ns["xtrack_m"] = 0.0
        return
    waypoint = navigator.waypoints[navigator.index]
    wp_n, wp_e = nav_module.local_offsets(
        home[0], home[1], waypoint["lat"], waypoint["lon"]
    )
    ns["wp_distance_m"] = nav_module.distance_m(wp_n - north, wp_e - east)
    ns["wp_bearing_deg"] = nav_module.bearing_deg(wp_n - north, wp_e - east)
    start_lat, start_lon = navigator._leg_start()
    start_n, start_e = nav_module.local_offsets(home[0], home[1], start_lat, start_lon)
    ns["xtrack_m"] = nav_module.cross_track_m(
        start_n, start_e, wp_n, wp_e, north, east
    )


class FlightOutputs:
    def __init__(self, config):
        self.servo_left = ServoOutput(
            pins.SERVO_LEFT_PIN, config["servo_left_min_us"], config["servo_left_max_us"]
        )
        self.servo_right = ServoOutput(
            pins.SERVO_RIGHT_PIN,
            config["servo_right_min_us"],
            config["servo_right_max_us"],
        )
        self.esc = EscOutput(pins.ESC_PIN)
        self.go_safe()

    def apply_endpoints(self, config):
        self.servo_left.set_endpoints(
            config["servo_left_min_us"], config["servo_left_max_us"]
        )
        self.servo_right.set_endpoints(
            config["servo_right_min_us"], config["servo_right_max_us"]
        )

    def go_safe(self):
        self.servo_left.center()
        self.servo_right.center()
        self.esc.stop()

    def release(self):
        self.esc.release()
        self.servo_left.release()
        self.servo_right.release()


def _link_fresh(rc, config, now_ms):
    if not rc.link_alive:
        return False
    silence_ms = time.ticks_diff(now_ms, rc.last_frame_ms)
    return silence_ms <= config["failsafe_link_timeout_ms"]


def _arm_requested(rc, config, now_ms):
    # A live link is required to arm: without the freshness check, a frame
    # latched before the link died (switch on, throttle low) would arm the
    # plane off stale data.
    return (
        _link_fresh(rc, config, now_ms)
        and rc.arm_switch_seen_off  # never arm on a switch latched since boot
        and rc.arm_switch_on
        and rc.throttle_us <= config["arm_max_throttle_us"]
    )


class _Watchdog:
    # Thin wrapper so a build without machine.WDT (or a timeout the port
    # refuses) degrades to a no-op with a printed reason rather than taking
    # the firmware down -- the portal has to come up regardless.
    #
    # Started on the FIRST ARM, not at boot, and the distinction is not
    # cosmetic. An RP2040 watchdog cannot be stopped once started, so arming
    # it at boot meant every serial session inherited it: mpremote interrupts
    # main.py to do its work, nothing is left feeding the timer, and 4 s
    # later the board resets mid-operation and re-enumerates its USB. That
    # broke deploys and every bench script in this repo, which is a tax paid
    # on the ground for a guarantee only needed in the air.
    #
    # Nothing is lost by waiting. The hazard the watchdog exists for is a
    # wedged flight loop holding the last commanded throttle, which cannot
    # happen before the first arm. While disarmed the outputs are already
    # safe and _Portal.check() has its own reset-to-recover path. From the
    # first arm onward the timer stays up for the rest of the session, fed by
    # the flight loop at 250 Hz and the disarmed poll at 10 Hz.
    def __init__(self, timeout_ms):
        self._timeout_ms = timeout_ms
        self._wdt = None

    def start(self):
        if self._wdt is not None:
            return
        try:
            from machine import WDT

            self._wdt = WDT(timeout=self._timeout_ms)
            print("watchdog armed, %d ms" % self._timeout_ms)
        except (ImportError, ValueError, OSError) as error:
            print("watchdog unavailable:", repr(error))

    def feed(self):
        if self._wdt is not None:
            self._wdt.feed()


class _Portal:
    # Owns the web-server task. An exception inside a bare
    # create_task(app.start_server(...)) sits silently in the task object
    # until the next await -- a portal that never comes back after disarm,
    # with no error printed anywhere. The disarmed poll loop calls check():
    # a dead server task gets its error printed and the server restarted;
    # if it keeps dying, hard-reset into the fresh-boot path, which is
    # known-good (config lives in flash; no RAM state matters while
    # disarmed).
    def __init__(self, make_app):
        self._make_app = make_app
        self._failures = 0
        self._start()

    def _start(self):
        self._app = self._make_app()
        self._task = asyncio.create_task(self._app.start_server(port=80))

    async def _collect(self, verb):
        try:
            await self._task
        except asyncio.CancelledError:
            pass
        except Exception as error:
            print("portal web server", verb, "error:", repr(error))
            return True
        return False

    async def check(self):
        if not self._task.done():
            return
        # Only shutdown() ends start_server cleanly, and stop() is the only
        # caller of shutdown() -- a finished task here means the server died.
        await self._collect("died with")
        self._failures += 1
        if self._failures >= _PORTAL_MAX_FAILURES:
            print("portal keeps failing; resetting to recover")
            import machine

            machine.reset()
        await asyncio.sleep_ms(_PORTAL_RESTART_DELAY_MS)
        print("restarting portal web server")
        self._start()

    async def stop(self):
        if self._task.done():
            pass  # already dead; just collect the error below
        elif self._app.server is not None:
            self._app.shutdown()
        else:
            # start_server hasn't gotten far enough to have a server to
            # close (shutdown() would AttributeError on None); cancel it.
            self._task.cancel()
        await self._collect("shutdown")


async def _wait_for_arm_request(rc, gps, current_config, led, portal, watchdog):
    led.set_mode("blink_slow")
    while True:
        watchdog.feed()
        now_ms = _now_ms()
        rc.update(now_ms)
        # Polled while disarmed so the portal can show fix status and the
        # driver can re-send its boot config as needed. The armed loop polls
        # it too -- that is where nav actually consumes the fixes.
        gps.update(now_ms)
        led.tick(now_ms)
        await portal.check()
        if _arm_requested(rc, current_config(), now_ms):
            return
        await asyncio.sleep_ms(_DISARMED_POLL_MS)


def _gps_lost(gps, now_ms):
    # INAV's pos_failure_timeout (5 s) before it gives up on the position
    # estimate and switches to its emergency procedure.
    if gps.last_pvt_ms is None:
        return True
    if time.ticks_diff(now_ms, gps.last_pvt_ms) > _NAV_GPS_TIMEOUT_MS:
        return True
    return not gps.fix_ok


def _autonomous_demands(navigator, gps, rc, now_ms):
    # Returns (mode, throttle, pitch_stick, yaw_stick).
    #
    # A blocked engage degrades to stabilized under pilot control rather
    # than refusing to fly -- the switch position must never leave the
    # aircraft without a controller. Matches the MVP fallback behaviour,
    # except it now reports why.
    if navigator.state == "idle":
        blocker = navigator.engage_blocker(gps)
        if blocker:
            if blocker != navigator.blocked_reason:
                print("autonomous unavailable:", blocker)
                navigator.blocked_reason = blocker
            return (
                "stabilized",
                rc.channels["throttle"],
                rc.channels["pitch"],
                rc.channels["yaw"],
            )
        navigator.blocked_reason = None
        navigator.engage()
        print("autonomous engaged: %d waypoints" % len(navigator.waypoints))

    if _gps_lost(gps, now_ms):
        # Owner's recorded decision (flight-modes-arming-failsafe.md): GPS
        # lost in autonomous -> level out and cut throttle. INAV instead
        # flies a controlled descent at emerg_descent_rate; that difference
        # is deliberate and still worth revisiting on a real airframe.
        if navigator.state != "idle":
            print("GPS lost in autonomous: levelling and cutting throttle")
            navigator.disengage()
        return "nav_lost", 0.0, 0.0, 0.0

    navigator.update(gps)
    return "autonomous", navigator.throttle, navigator.pitch_stick, navigator.yaw_stick


def _load_logic(logic_engine):
    # Stored source is re-validated at boot rather than trusted: a file
    # hand-edited over USB, or written by an older schema, must not activate
    # rules that no longer compile.
    source = logic_module.load()
    if not source.strip():
        return
    rules, errors = logic_engine.load_source(source)
    if errors:
        print("logic disabled,", len(errors), "error(s); first:", errors[0])
        logic_engine.clear()
        logic_engine.source = source
        logic_engine.errors = [str(e) for e in errors]
        return
    logic_engine.activate(source, rules)
    print("logic: %d rule%s active" % (len(rules), "" if len(rules) == 1 else "s"))


def _capture_home(navigator, gps, config):
    # Home is the first good fix after arming -- INAV's GPS_FIX_HOME. Set
    # once per armed session so a mid-flight fix wobble can't move it.
    if navigator.home is not None or not gps.fix_ok:
        return False
    if gps.num_sv < int(config["nav_min_sats"]):
        return False
    if gps.h_acc_m > config["nav_max_h_acc_m"]:
        return False
    navigator.set_home(gps.lat_deg, gps.lon_deg, gps.alt_m)
    print("home set: %.6f, %.6f, %.1f m" % (gps.lat_deg, gps.lon_deg, gps.alt_m))
    return True


async def _run_flight_loop(rc, gps, config, outputs, led, logic_engine, aux, watchdog):
    # Defense in depth: nothing should be driving outputs while disarmed
    # (the portal's bench-test pages that once did are gone), but IMU init +
    # gyro calibration below take the better part of a second, so start from
    # a known-safe output state unconditionally.
    outputs.go_safe()
    # First arm of the session brings the watchdog up; see _Watchdog. Before
    # the IMU work below, which is the longest blocking stretch in the
    # firmware and would otherwise be the first thing to trip it.
    watchdog.start()
    watchdog.feed()
    try:
        imu = MPU6050()
        imu.calibrate_gyro(sample_count=_GYRO_CALIBRATION_SAMPLES)  # plane at rest, each arm
    except OSError as error:
        # IMU missing/unwired: refuse to arm rather than crash the whole
        # firmware. The config portal must stay reachable regardless of
        # what's wired up yet, so this only blocks entering the flight loop,
        # not boot. Exits on disarm OR link loss -- with a dead RX the arm
        # switch can never read off, and outputs are already safe.
        print("IMU init failed, refusing to arm:", error)
        led.set_mode("blink_fast")
        while rc.arm_switch_on:
            watchdog.feed()
            now_ms = _now_ms()
            rc.update(now_ms)
            led.tick(now_ms)
            if not _link_fresh(rc, config, now_ms):
                break
            await asyncio.sleep_ms(_DISARMED_POLL_MS)
        return
    # Calibration blocks long enough for ELRS to overrun the UART buffer;
    # the backlog is truncated garbage, so drop it rather than resync
    # through it on the first iterations.
    rc.flush()

    attitude = MahonyFilter()
    pitch_pid = _scaled_pid(config, "pitch")
    yaw_pid = _scaled_pid(config, "yaw")
    mixer = VTailMixer(config)
    # Working copy: user logic overrides land here, leaving the saved config
    # untouched so a rule can never write itself into flash.
    effective = dict(config)
    (link_timeout_ms, pitch_angle_max_deg, pitch_level_p,
     pitch_rate_limit_dps, rate_yaw_dps, turn_assist_gain) = _loop_gains(effective)
    active_mode = None
    # Mission is re-read at each arm, so a plan saved from the portal is
    # picked up without a reboot.
    navigator = Navigator(config, mission_module.load())
    logic_namespace = {}
    compass_reading = None
    compass_sensor = None
    if logic_engine.enabled:
        try:
            compass_sensor = QMC5883()
        except OSError:
            compass_sensor = None
    led.set_mode("solid")

    iterations = 0
    overruns = 0
    started_ms = _now_ms()
    last_us = time.ticks_us()
    # Forces a poll (and a home-capture attempt) on the very first pass
    # rather than _GPS_POLL_INTERVAL_MS into the flight.
    last_gps_ms = time.ticks_add(started_ms, -_GPS_POLL_INTERVAL_MS)
    # Starts False so that arming with an already-dead link logs the entry
    # rather than treating failsafe as the unremarkable normal state.
    was_link_lost = False
    lost_since_ms = started_ms
    failsafe_events = 0
    # Records to RAM now, writes to flash once on disarm -- so a bench test
    # can be run with no USB attached at all. See flightlog.py.
    recorder = flightlog.Recorder(started_ms, gc.mem_free())
    exit_reason = "disarm"
    try:
        while rc.arm_switch_on:
            watchdog.feed()
            now_ms = _now_ms()
            rc.update(now_ms)
            # Polled at _GPS_POLL_INTERVAL_MS, not every iteration. This was
            # "cheap at a 5 Hz fix rate: most iterations read an empty UART"
            # -- an assumption the board disagreed with, at 1942 us a call.
            # Home capture rides along because it can only change on new
            # position data anyway.
            if time.ticks_diff(now_ms, last_gps_ms) >= _GPS_POLL_INTERVAL_MS:
                last_gps_ms = now_ms
                gps.update(now_ms)
                _capture_home(navigator, gps, config)
            led.tick(now_ms)

            # Measured dt: the loop sleeps a fixed remainder, so the real
            # period is body time + sleep, not the nominal 4 ms -- feeding
            # the nominal value would mis-scale gyro integration and the
            # PID I/D terms by the overrun ratio.
            now_us = time.ticks_us()
            dt_s = time.ticks_diff(now_us, last_us) / 1_000_000
            last_us = now_us
            if dt_s <= 0.0 or dt_s > _MAX_DT_S:
                dt_s = _LOOP_PERIOD_S

            # Standard body-axis mounting: gyro x/y/z = roll/pitch/yaw rate.
            accel_g, gyro_rad_s = imu.read()
            attitude.update(gyro_rad_s, accel_g, dt_s)
            measured_pitch_rate_dps = degrees(gyro_rad_s[1])
            measured_yaw_rate_dps = degrees(gyro_rad_s[2])

            silence_ms = time.ticks_diff(now_ms, rc.last_frame_ms)
            link_lost = not rc.link_alive or silence_ms > link_timeout_ms
            if link_lost and silence_ms > _LINK_LOST_DISARM_MS:
                exit_reason = "link_timeout"
                break

            # Failsafe behaves as a mode of its own: stabilized flight
            # toward level (0 deg pitch target, zero yaw/roll demand) with
            # throttle cut -- in manual too, since centered surfaces are
            # not "level wings". Treating it as a mode also resets the PIDs
            # on both entry and exit, so no stale integrator on regain.
            # Failsafe was entirely silent, which made "the motor cuts out at
            # part throttle" indistinguishable from an ESC or supply fault:
            # the link drops, throttle is cut, the link returns, and nothing
            # anywhere records that it happened. Logged on transition only,
            # so a flapping link cannot flood the loop it is reporting on.
            if link_lost != was_link_lost:
                was_link_lost = link_lost
                if link_lost:
                    lost_since_ms = now_ms
                    failsafe_events += 1
                    recorder.note_failsafe(now_ms, silence_ms)
                    print("FAILSAFE: link lost after %d ms silence,"
                          " cutting throttle" % silence_ms)
                else:
                    recorder.note_failsafe_cleared(now_ms, time.ticks_diff)
                    print("failsafe cleared: link back after %d ms"
                          % time.ticks_diff(now_ms, lost_since_ms))

            if link_lost:
                led.set_mode("blink_fast")
                mode = "failsafe"
                throttle = 0.0
                pitch_stick = yaw_stick = roll_stick = 0.0
                if navigator.state != "idle":
                    navigator.disengage()
            else:
                led.set_mode("solid")
                mode = rc.mode
                roll_stick = 0.0
                if mode == "autonomous":
                    mode, throttle, pitch_stick, yaw_stick = _autonomous_demands(
                        navigator, gps, rc, now_ms
                    )
                else:
                    if navigator.state != "idle":
                        navigator.disengage()
                    throttle = rc.channels["throttle"]
                    pitch_stick = rc.channels["pitch"]
                    yaw_stick = rc.channels["yaw"]
                    roll_stick = rc.channels["roll"]

            # User logic runs HERE -- after the demands are settled (so it
            # sees the real mode) and before the cascade consumes them, so a
            # parameter it changes takes effect this iteration rather than
            # the next.
            if logic_engine.due(now_ms):
                if compass_sensor is not None:
                    try:
                        mx, my, mz = compass_sensor.read()
                        compass_reading = (mx, my, mz, flat_field_angle_deg(mx, my))
                    except OSError:
                        compass_reading = None
                _fill_logic_namespace(
                    logic_namespace, rc, gps, navigator, attitude, accel_g,
                    gyro_rad_s, mode, aux, compass_reading, now_ms,
                    time.ticks_diff(now_ms, started_ms), dt_s,
                    iterations * 1000 // max(time.ticks_diff(now_ms, started_ms), 1),
                )
                logic_engine.evaluate(logic_namespace, now_ms)
                if logic_engine.overrides:
                    effective.update(logic_engine.overrides)
                    (link_timeout_ms, pitch_angle_max_deg, pitch_level_p,
                     pitch_rate_limit_dps, rate_yaw_dps,
                     turn_assist_gain) = _loop_gains(effective)
                    pitch_pid.set_gains(*_pid_gains(effective, "pitch"))
                    yaw_pid.set_gains(*_pid_gains(effective, "yaw"))
                    mixer.set_gains(effective)
                    navigator.configure(effective)
                    outputs.apply_endpoints(effective)
                if logic_engine.pin_states:
                    aux.apply(logic_engine.pin_states)

            # Demand overrides, applied every iteration from the values the
            # last evaluation held (the same hold-between-updates the nav
            # loop already relies on at 5 Hz). When they are refused the
            # rules still evaluate, so their values stay visible on the
            # status page -- only the application is skipped. The rule for
            # when they may apply lives in logic.demands_allowed().
            # "nav_lost" is set by _autonomous_demands when the position
            # estimate goes stale, and it has already zeroed throttle: this
            # is what stops a rule handing that throttle straight back.
            if logic_engine.demands and logic_module.demands_allowed(
                link_lost, mode == "nav_lost"
            ):
                throttle = logic_engine.demands.get("throttle_demand", throttle)
                pitch_stick = logic_engine.demands.get("pitch_demand", pitch_stick)
                yaw_stick = logic_engine.demands.get("yaw_demand", yaw_stick)
                roll_stick = logic_engine.demands.get("roll_demand", roll_stick)

            if mode != active_mode:
                pitch_pid.reset()
                yaw_pid.reset()
                active_mode = mode

            if mode == "manual":
                pitch_cmd = pitch_stick
                yaw_cmd = yaw_stick
            else:
                # Outer level loop (pitch only -- INAV never gives yaw an
                # angle mode): angle error -> a rate target, ceiling-clamped.
                target_pitch_deg = pitch_stick * pitch_angle_max_deg
                angle_error_deg = target_pitch_deg - attitude.pitch_deg
                pitch_rate_target_dps = angle_error_deg * (pitch_level_p * _LEVEL_P_MULTIPLIER)
                pitch_rate_target_dps = min(
                    max(pitch_rate_target_dps, -pitch_rate_limit_dps), pitch_rate_limit_dps
                )

                # Inner rate loop, pitch: P+I+D+FF on (rate target - measured).
                pitch_rate_error_dps = pitch_rate_target_dps - measured_pitch_rate_dps
                pitch_sum = pitch_pid.update(
                    pitch_rate_error_dps, dt_s,
                    feedforward_input=pitch_rate_target_dps,
                )
                pitch_cmd = pitch_sum / _PID_SUM_LIMIT

                # Yaw: rate-only, stick + turn-coordination folded into one
                # target rate -- no outer loop.
                yaw_rate_target_dps = (
                    yaw_stick * rate_yaw_dps
                    + roll_stick * turn_assist_gain
                )
                yaw_rate_error_dps = yaw_rate_target_dps - measured_yaw_rate_dps
                yaw_sum = yaw_pid.update(
                    yaw_rate_error_dps, dt_s,
                    feedforward_input=yaw_rate_target_dps,
                )
                yaw_cmd = yaw_sum / _PID_SUM_LIMIT

            left, right = mixer.mix(pitch_cmd, yaw_cmd)
            outputs.servo_left.set_normalized(left)
            outputs.servo_right.set_normalized(right)
            outputs.esc.set_throttle(throttle)

            # Deliberately HERE: the surfaces and throttle for this pass are
            # already latched into the PWM hardware, so a collection now
            # delays nothing that matters. Left to fire on its own it lands
            # wherever the allocation that filled the heap happened to be --
            # frequently mid-cascade, between reading the gyro and writing
            # the servos. See _GC_EVERY_ITERATIONS.
            if iterations % _GC_EVERY_ITERATIONS == 0:
                gc.collect()

            iterations += 1
            body_us = time.ticks_diff(time.ticks_us(), now_us)
            recorder.note_iteration(body_us)
            recorder.note_silence(silence_ms)
            # Cached at 1 Hz (see _MEM_FREE_INTERVAL_MS) -- the real call
            # costs 6.8 ms and must never be made at loop rate.
            recorder.note_free(_sampled_mem_free(now_ms))
            remaining_us = _LOOP_PERIOD_US - body_us
            if remaining_us <= 0:
                overruns += 1
                recorder.note_overrun()
                await asyncio.sleep_ms(0)
            else:
                await asyncio.sleep_ms(remaining_us // 1000)
    except Exception as error:
        # Recorded before re-raising so the crash reaches the flight log.
        # main() catches this and returns to the portal; without it landing
        # in the record, a crash during a no-USB bench run leaves no trace
        # at all -- which is the whole reason the log exists.
        exit_reason = "crash: %r" % (error,)
        raise
    finally:
        # Runs on disarm, link-loss timeout, AND any exception (e.g. a
        # transient I2C error in imu.read()): PWM is hardware, so without
        # this a crash would leave the motor at its last commanded throttle.
        outputs.go_safe()
        # Logic-driven pins are armed-only: they fall to a known state on
        # disarm rather than holding whatever the last rule commanded while
        # someone is handling the aircraft.
        aux.all_off()
        led.set_mode("blink_slow")
        total_ms = time.ticks_diff(_now_ms(), started_ms)
        if iterations and total_ms > 0:
            print(
                "flight loop: %d iterations, ~%d Hz achieved (target %d),"
                " %d overruns, %d failsafe event(s)"
                % (iterations, iterations * 1000 // total_ms, FLIGHT_LOOP_HZ,
                   overruns, failsafe_events)
            )
        # After the outputs are safe and nothing is flying: the one flash
        # write of the session. Failure to write must not mask whatever is
        # already propagating out of the try block.
        if iterations:
            if flightlog.append(recorder.record(total_ms, exit_reason)):
                print("flight log written (%d session(s) kept)"
                      % len(flightlog.load()))
            else:
                print("flight log could not be written")


async def main():
    # Allocation ORDER matters here, not just total free memory. Importing
    # this module pulls in server + microdot (~60 KB) and leaves the heap
    # fragmented; MicroPython's collector never compacts, so the UART ring
    # buffers -- by far the largest contiguous requests the firmware makes --
    # must be claimed before anything else carves up what's left. Boot used
    # to die here with "MemoryError: allocating 4097 bytes" while ~97 KB was
    # still free overall. Collect first, take the big buffers, then build the
    # small stuff.
    gc.collect()
    rc = CrsfReceiver(config_module.defaults())
    gps = UbxGps()
    gc.collect()
    watchdog = _Watchdog(_WATCHDOG_TIMEOUT_MS)
    active_config = config_module.load()
    rc.set_channel_map(active_config)
    outputs = FlightOutputs(active_config)
    led = StatusLed()
    aux = auxpins.AuxPins()
    free_pins = auxpins.free_pins()
    logic_engine = logic_module.LogicEngine(config_module.SCHEMA, free_pins)
    _load_logic(logic_engine)
    print("boot: hardware ready, %d bytes free" % gc.mem_free())

    def current_config():
        return active_config

    def persist(updated_config):
        nonlocal active_config
        active_config = updated_config
        config_module.save(updated_config)
        rc.set_channel_map(updated_config)
        # Endpoints are otherwise baked into the ServoOutputs at boot; apply
        # them live so a save is reflected in the bench pages and the next
        # arm without a reboot.
        outputs.apply_endpoints(updated_config)

    def make_app():
        return server.create_app(current_config, persist, rc, gps,
                                 logic_engine, free_pins)

    while True:
        # Collect before building the portal, not just after tearing it down:
        # create_app() builds a whole microdot app and its route table, and
        # on every cycle after the first it is competing with the debris of
        # the previous one.
        gc.collect()
        print("portal up, %d bytes free" % gc.mem_free())
        access_point = server.start_access_point(active_config, watchdog.feed)
        portal = _Portal(make_app)
        try:
            await _wait_for_arm_request(rc, gps, current_config, led, portal,
                                        watchdog)
        finally:
            await portal.stop()
            server.stop_access_point(access_point)
        # Drop the references BEFORE collecting, or the collect cannot
        # actually free any of it: portal holds the microdot app, its routes
        # and its handler closures, which together are the largest thing this
        # firmware ever allocates after the UART buffers. Without this the
        # heap accumulated a portal's worth of debris per arm cycle until
        # rc.py's uart.read() could no longer find a contiguous 2 KB and the
        # second arm of a session died with MemoryError.
        portal = None
        access_point = None
        gc.collect()
        print("armed, %d bytes free" % gc.mem_free())
        try:
            await _run_flight_loop(rc, gps, active_config, outputs, led,
                                   logic_engine, aux, watchdog)
        except Exception as error:
            # The loop's own try/finally has already forced the outputs safe;
            # what it cannot do is get us back somewhere useful. Without this
            # the exception unwound out of main(), asyncio.run() returned, and
            # the board sat dead -- no portal, no way to re-arm, no way to
            # read the error off a plane in a field. A single transient I2C
            # NAK from the IMU was enough to do it.
            print("flight loop crashed:", repr(error))
            _print_traceback(error)
            # Demand the arm switch be cycled OFF before re-arming, so a
            # persistent fault cannot spin crash -> portal -> crash.
            rc.arm_switch_seen_off = False
            watchdog.feed()
            await asyncio.sleep_ms(_FLIGHT_LOOP_RESTART_DELAY_MS)


def _print_traceback(error):
    # MicroPython puts this on sys, CPython on traceback; neither is worth
    # failing the recovery path over if the print itself goes wrong.
    try:
        import sys

        sys.print_exception(error)
    except (ImportError, AttributeError):
        pass


def _now_ms():
    return time.ticks_ms()


if __name__ == "__main__":
    asyncio.run(main())
