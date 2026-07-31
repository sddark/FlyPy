# User logic: a small Python script of one-line assignments, re-evaluated
# while armed. Each line is `name = <expression>`; the expression is compiled
# with compile(src, "<logic>", "eval") -- expression mode, which the Python
# compiler itself rejects `while`, `for`, `import` and every other statement
# for. That is the containment: a rule structurally cannot loop, and there is
# no way to preempt a runaway one on MicroPython (unlike ArduPilot's Lua,
# which can bound a script by instruction count).
#
# It is guard rails against mistakes, not a security sandbox: MicroPython
# resolves builtins even when eval() is handed its own namespace, so an
# expression can still reach `open` or `__import__`. The portal is AP-only,
# password protected, single user -- the threat model is the owner's typos.
# What actually protects the aircraft:
#   * every result is clamped to that parameter's existing config.SCHEMA
#     bounds, so bad logic can produce a wrong value but never an unsafe one;
#   * each rule is compiled AND timed at save, so a slow or throwing rule is
#     rejected on the ground with its line number;
#   * a rule that throws in flight is disabled and reported, and the rest
#     keep running.
#
# Pure Python (no `machine` imports) so it is testable off-target: the engine
# computes desired pin states, and auxpins.py drives the hardware.

import os
import time

_ticks_ms = getattr(time, "ticks_ms", lambda: int(time.time() * 1000))
_ticks_diff = getattr(time, "ticks_diff", lambda a, b: a - b)
_ticks_us = getattr(time, "ticks_us", lambda: int(time.time() * 1000000))

LOGIC_FILE = "logic.json"
_TMP_FILE = "logic.json.tmp"
_BACKUP_FILE = "logic.json.bak"

MAX_LINES = 60
# A rule slower than this is rejected at save. Measured on the RP2040 a
# typical expression evaluates in ~78 us; anything an order of magnitude
# beyond that is a mistake (a comprehension over range(), usually).
MAX_RULE_US = 800
# Nav-rate, not loop-rate: these are parameter adjustments, not inner-loop
# control math. At 25 Hz the whole pass is well under 1% of the flight loop.
EVAL_INTERVAL_MS = 40

# Remapping which stick is throttle mid-flight is never wanted, and the WiFi
# strings are meaningless while armed.
EXCLUDED_TARGETS = (
    "channel_roll", "channel_pitch", "channel_throttle",
    "channel_yaw", "channel_arm", "channel_mode",
    "wifi_ssid_suffix", "wifi_password",
)

PWM_MIN_US = 500
PWM_MAX_US = 2500

KIND_PARAM = "param"
KIND_GPIO = "gpio"
KIND_PWM = "pwm"
KIND_DEMAND = "demand"

# Flight demands a rule may drive, in the same units the RC sticks provide.
# These feed the control path exactly where a stick would, so the cascade,
# its clamps, the mixer and the arming gate all still apply. Failsafe is the
# only thing that refuses them -- see demands_allowed(). In manual mode
# pitch/yaw/throttle apply (manual is a stick passthrough); roll_demand is
# inert there, for the same reason the roll stick is: manual has no turn
# coordination to feed.
DEMAND_TARGETS = {
    "throttle_demand": (0.0, 1.0),
    "pitch_demand": (-1.0, 1.0),
    "yaw_demand": (-1.0, 1.0),
    "roll_demand": (-1.0, 1.0),
}


# The readable names, defined once: the editor's Inputs column and the
# save-time smoke test are both derived from this, so the list the page shows
# can never drift from the names a rule is actually allowed to use. Values
# here are representative, used only to exercise a rule before accepting it.
INPUT_CATALOGUE = (
    ("Pilot / RC", (
        ("throttle", 0.45, "0..1"),
        ("pitch", 0.0, "stick, -1..1"),
        ("roll", 0.0, "stick, -1..1"),
        ("yaw", 0.0, "stick, -1..1"),
        ("throttle_us", 1450.0, "raw pulse width"),
        ("arm_switch", True, "bool"),
        ("mode", "stabilized", "manual / stabilized / autonomous"),
        ("flight_mode", "stabilized", "incl. failsafe, nav_lost"),
        ("armed", True, "bool"),
        ("link_ok", True, "bool"),
        ("link_age_ms", 8, "since last RC frame"),
    )),
    ("Attitude", (
        ("roll_deg", 3.2, "estimator output"),
        ("pitch_deg", -1.4, "estimator output"),
        ("roll_rate", 0.8, "deg/s, gyro X"),
        ("pitch_rate", -0.3, "deg/s, gyro Y"),
        ("yaw_rate", 2.1, "deg/s, gyro Z"),
    )),
    ("IMU", (
        ("accel_x", 0.02, "g"), ("accel_y", -0.01, "g"),
        ("accel_z", 0.99, "g"), ("accel_mag", 1.0, "g, vector magnitude"),
    )),
    ("Compass", (
        ("compass_ok", True, "bool"),
        ("heading_deg", 56.0, "flat-plane, uncalibrated"),
        ("mag_x", 1.53, "gauss"), ("mag_y", 2.26, "gauss"),
        ("mag_z", 0.23, "gauss"),
    )),
    ("GPS", (
        ("gps_fix", True, "bool, 3D fix"),
        ("fix_type", 3, "0 none .. 3 3D"),
        ("sats", 11, "satellites used"),
        ("lat", 45.0012, "degrees"), ("lon", -122.9031, "degrees"),
        ("gps_alt_m", 128.0, "MSL, no baro"),
        ("ground_speed_ms", 16.0, "m/s"),
        ("course_deg", 92.0, "course over ground"),
        ("h_acc_m", 1.8, "horizontal accuracy"),
        ("v_acc_m", 3.1, "vertical accuracy"),
        ("course_acc_deg", 2.5, "degrees"),
        ("gps_age_ms", 140, "since last fix, -1 if none"),
    )),
    ("Navigation", (
        ("nav_state", "run", "idle / run / rth / loiter"),
        ("wp_index", 4, "active waypoint, 1-based"),
        ("wp_total", 5, "waypoints in mission"),
        ("wp_distance_m", 240.0, "to active waypoint"),
        ("wp_bearing_deg", 84.0, "to active waypoint"),
        ("xtrack_m", -12.0, "off track, + is right"),
        ("home_lat", 45.0003, "degrees"), ("home_lon", -122.9005, "degrees"),
        ("home_alt_m", 118.0, "altitude when armed"),
        ("home_distance_m", 310.0, "metres"),
        ("home_bearing_deg", 214.0, "degrees"),
        ("alt_above_home_m", 10.0, "gps_alt_m - home_alt_m"),
    )),
    ("System", (
        ("armed_ms", 42000, "since arming"),
        ("uptime_ms", 305000, "since boot"),
        ("loop_hz", 214, "achieved flight-loop rate"),
        ("dt_s", 0.0047, "last loop period"),
        ("mem_free", 78000, "bytes"),
    )),
)


def sample_namespace(free_pins):
    ns = {}
    for _group, entries in INPUT_CATALOGUE:
        for name, value, _note in entries:
            ns[name] = value
    for index in range(16):
        ns["ch%d" % (index + 1)] = 992
    for number in free_pins:
        ns["gpio%d" % number] = 0
        ns["pwm%d" % number] = 0
        if number >= 26:
            ns["adc%d" % number] = 1.65
    return ns


class RuleError(Exception):
    # Deliberately does not chain to Exception.__init__: MicroPython has no
    # such attribute on the built-in type, and calling it raises
    # AttributeError on the board while working fine under CPython -- the
    # kind of divergence off-target tests cannot see.
    def __init__(self, line_number, message):
        self.line_number = line_number
        self.message = message

    def __str__(self):
        return "line %d: %s" % (self.line_number, self.message)


class Rule:
    def __init__(self, line_number, target, kind, source, code, pin=None,
                 low=None, high=None):
        self.line_number = line_number
        self.target = target
        self.kind = kind
        self.source = source
        self.code = code
        self.pin = pin
        self.low = low
        self.high = high
        self.value = None
        self.raw_value = None
        self.clamped = False
        self.error = None
        # Precomputed so the evaluation loop needs neither a branch nor a
        # string comparison to decide how to handle this rule.
        self.is_gpio = kind == KIND_GPIO
        self.sink = None  # the dict this rule writes into; set by activate()


def demands_allowed(link_lost):
    # The single definition of when a rule may drive the flight demands.
    # Lives here, not inline in the flight loop, so the rule that matters
    # most can be unit-tested off-target.
    #
    # FAILSAFE OUTRANKS EVERYTHING, and it is the only thing that does.
    # Keyed on link_lost rather than on a mode name so that adding or
    # renaming a mode later cannot quietly re-open the path that lets a rule
    # restore throttle after failsafe cut it.
    #
    # Manual is deliberately NOT excluded. It was, briefly, on the theory
    # that it should stay a guaranteed stick-only escape -- but failsafe
    # already is that escape, reachable at any moment by killing the TX, and
    # it cannot be overridden. Manual means "no PID cascade", a control-law
    # distinction; overloading it to also mean "no logic authority" borrowed
    # a switch position to duplicate a guarantee that already existed. A
    # rule that wants its own kill switch can read any spare RC channel:
    #     throttle_demand = 0.4 if ch7 > 1500 else throttle
    return not link_lost


def assignable_targets(schema):
    return [name for name in schema if name not in EXCLUDED_TARGETS]


def _split_assignment(line):
    # Find the assignment '=', rejecting comparisons: ==, !=, <=, >=.
    index = line.find("=")
    while index >= 0:
        before = line[index - 1] if index else ""
        after = line[index + 1] if index + 1 < len(line) else ""
        if before not in "=!<>" and after != "=":
            return index
        index = line.find("=", index + 1)
    return -1


def compile_source(source, schema, free_pins):
    # Returns (rules, errors). Errors carry line numbers so the editor can
    # point at the offending line; a source with any error is never applied.
    rules = []
    errors = []
    lines = source.split("\n")
    if len(lines) > MAX_LINES:
        errors.append(RuleError(MAX_LINES + 1, "too many lines (max %d)" % MAX_LINES))
        return rules, errors
    seen = {}
    for offset, raw_line in enumerate(lines):
        number = offset + 1
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        index = _split_assignment(raw_line)
        if index < 0:
            errors.append(RuleError(number, "expected  name = expression"))
            continue
        target = raw_line[:index].strip()
        expression = raw_line[index + 1:].strip()
        if not expression:
            errors.append(RuleError(number, "missing expression"))
            continue
        kind, pin, low, high = _classify(target, schema, free_pins)
        if kind is None:
            errors.append(RuleError(
                number, "'%s' is not an assignable output" % target))
            continue
        if target in seen:
            errors.append(RuleError(
                number, "'%s' already set on line %d" % (target, seen[target])))
            continue
        try:
            code = compile(expression, "<logic>", "eval")
        except SyntaxError:
            # MicroPython's SyntaxError carries no detail beyond "invalid
            # syntax", so the common mistakes get named here instead.
            # Python's conditional is a three-part expression -- unlike most
            # languages' `if`, the `else` is not optional -- and leaving it
            # off is the mistake almost everyone makes first.
            message = "invalid syntax"
            if " if " in expression and " else " not in expression:
                message = ("a conditional needs both halves:"
                           " A if condition else B")
            elif expression.count("(") != expression.count(")"):
                message = "unbalanced brackets"
            errors.append(RuleError(number, message))
            continue
        seen[target] = number
        rules.append(Rule(number, target, kind, expression, code, pin, low, high))
    return rules, errors


def _classify(target, schema, free_pins):
    if target in DEMAND_TARGETS:
        low, high = DEMAND_TARGETS[target]
        return KIND_DEMAND, None, low, high
    if target in schema and target not in EXCLUDED_TARGETS:
        _default, low, high, _step = schema[target]
        if low is None:
            return None, None, None, None  # string parameter
        return KIND_PARAM, None, low, high
    for prefix, kind, low, high in (
        ("gpio", KIND_GPIO, 0, 1), ("pwm", KIND_PWM, PWM_MIN_US, PWM_MAX_US)
    ):
        if target.startswith(prefix):
            digits = target[len(prefix):]
            if digits.isdigit() and int(digits) in free_pins:
                return kind, int(digits), low, high
    return None, None, None, None


def smoke_test(rules, sample_namespace):
    # Compile-time acceptance is not enough: an expression can be valid and
    # still reference a name that does not exist, or take far too long. Run
    # each rule once against representative values and time it, so both are
    # caught on the ground.
    errors = []
    # One copy for the whole run, not one per rule: copying an 85-name dict
    # per rule dominated the measurement and made every rule look slow.
    namespace = dict(sample_namespace)
    for rule in rules:
        start = _ticks_us()
        try:
            result = eval(rule.code, namespace)
        except NameError as error:
            errors.append(RuleError(rule.line_number, _name_error_text(error)))
            continue
        except Exception as error:
            errors.append(RuleError(
                rule.line_number, "%s: %s" % (type(error).__name__, error)))
            continue
        elapsed = _ticks_diff(_ticks_us(), start)
        if not isinstance(result, (int, float, bool)):
            errors.append(RuleError(rule.line_number, _wrong_type_help(rule, result)))
            continue
        if elapsed > MAX_RULE_US:
            errors.append(RuleError(
                rule.line_number, "too slow (%d us, limit %d)" % (elapsed, MAX_RULE_US)))
    return errors


def _wrong_type_help(rule, result):
    # Say what to write instead, not just what is wrong. Words are the
    # instinctive thing to reach for with a pin, and they are also the
    # dangerous thing -- every non-empty string is truthy, so 'off' would
    # otherwise hold the pin HIGH.
    kind = type(result).__name__
    if rule.kind == KIND_GPIO:
        # Lead with the value form, since a word is what people reach for
        # first and 1/0 is the direct substitution for it.
        return ("%s takes 1 or 0 (or any condition, which is already"
                " true/false) -- got a %s" % (rule.target, kind))
    if rule.kind == KIND_PWM:
        return ("%s takes a pulse width in microseconds, 500-2500"
                " (e.g. 1500) -- got a %s" % (rule.target, kind))
    if rule.kind == KIND_DEMAND:
        return ("%s takes a stick position, %g to %g -- got a %s"
                % (rule.target, rule.low, rule.high, kind))
    return ("%s takes a number -- got a %s" % (rule.target, kind))


def _name_error_text(error):
    text = str(error)
    return text if "name" in text else "name is not defined: %s" % text


class LogicEngine:
    def __init__(self, schema, free_pins):
        self._schema = schema
        self._free_pins = free_pins
        self.rules = []
        self.source = ""
        self.errors = []
        self.enabled = False
        self.overrides = {}
        self.pin_states = {}
        self.demands = {}
        self._last_eval_ms = None

    def load_source(self, source):
        # Compiles without applying; returns errors. Callers apply only a
        # clean source.
        rules, errors = compile_source(source, self._schema, self._free_pins)
        return rules, errors

    def activate(self, source, rules):
        self.source = source
        self.rules = rules
        self.errors = []
        self.enabled = bool(rules)
        self.overrides = {}
        self.pin_states = {}
        self.demands = {}
        self._last_eval_ms = None
        sinks = {KIND_PARAM: self.overrides, KIND_DEMAND: self.demands}
        for rule in rules:
            rule.value = None
            rule.error = None
            rule.sink = sinks.get(rule.kind, self.pin_states)

    def clear(self):
        self.activate("", [])

    def due(self, now_ms):
        if not self.enabled:
            return False
        if self._last_eval_ms is None:
            return True
        return _ticks_diff(now_ms, self._last_eval_ms) >= EVAL_INTERVAL_MS

    def evaluate(self, namespace, now_ms):
        # Outputs are readable as well as writable: each pass sees what the
        # previous pass commanded, which is what lets a rule latch itself on
        # (gpio16 = gpio16 or <condition>).
        #
        # This body is deliberately flat and branch-light -- it runs inside
        # the flight loop's budget. Measured on the RP2040, the obvious
        # version cost 2100 us for five rules against 280 us of actual
        # eval(); the overhead was almost entirely
        # `isinstance(raw, (int, float, bool))`, at ~175 us per call, plus a
        # method call for clamping. Type checking is gone (a non-numeric
        # result now raises inside the try, which is where it belongs -- and
        # save-time smoke_test already rejected such a rule on the ground),
        # clamping is inlined, and each rule carries a direct reference to
        # the dict it writes so there is no per-rule branch or attribute
        # lookup to choose one.
        self._last_eval_ms = now_ms
        for rule in self.rules:
            try:
                raw = eval(rule.code, namespace)
                if rule.is_gpio:
                    # `raw > 0` rather than plain truthiness, for two
                    # reasons. It raises TypeError on a string, so a rule
                    # like `'on' if x else 'off'` is caught and reported
                    # instead of driving the pin permanently HIGH (every
                    # non-empty string is truthy, including 'off'). And it
                    # costs 8.9 us against isinstance()'s 149 us, measured
                    # on this board.
                    value = raw > 0
                    clamped = False
                else:
                    value = raw
                    if value < rule.low:
                        value = rule.low
                        clamped = True
                    elif value > rule.high:
                        value = rule.high
                        clamped = True
                    else:
                        clamped = False
                rule.sink[rule.target] = value
                namespace[rule.target] = value
            except Exception as error:
                # One bad rule is disabled and reported; the rest keep running
                # and the aircraft keeps the last good value for this target.
                rule.error = "%s: %s" % (type(error).__name__, error)
                rule.value = None
                continue
            rule.error = None
            rule.raw_value = raw
            rule.clamped = clamped
            rule.value = value
        return True

    def summary(self):
        # Compact live state for the status page.
        out = []
        for rule in self.rules:
            out.append({
                "line": rule.line_number,
                "target": rule.target,
                "kind": rule.kind,
                "value": rule.value,
                "clamped": rule.clamped,
                "error": rule.error,
            })
        return out


def load():
    for path in (LOGIC_FILE, _BACKUP_FILE):
        try:
            import json

            with open(path) as handle:
                stored = json.load(handle)
            if isinstance(stored, dict) and isinstance(stored.get("source"), str):
                return stored["source"]
        except (OSError, ValueError):
            continue
    return ""


def save(source):
    import json

    with open(_TMP_FILE, "w") as handle:
        json.dump({"source": source}, handle)
    try:
        os.remove(_BACKUP_FILE)
    except OSError:
        pass
    try:
        os.rename(LOGIC_FILE, _BACKUP_FILE)
    except OSError:
        pass
    os.rename(_TMP_FILE, LOGIC_FILE)
    return source
