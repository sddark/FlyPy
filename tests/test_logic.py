# Off-target tests for the user-logic engine (logic.py is pure Python):
# `python3 tests/test_logic.py`.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))

import config as config_module
import logic

FREE_PINS = (7, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 26, 27, 28)
SCHEMA = config_module.SCHEMA

SAMPLE = {
    "throttle": 0.45, "pitch": 0.0, "roll": 0.0, "yaw": 0.0,
    "alt_above_home_m": 40.0, "wp_index": 4, "wp_distance_m": 15.0,
    "ground_speed_ms": 16.0, "ch7": 1700, "adc26": 2.1, "gpio16": 0,
    "gpio7": 0, "sats": 11, "armed": True, "accel_x": 0.02,
}


def compile_source(source):
    return logic.compile_source(source, SCHEMA, FREE_PINS)


def engine_with(source):
    engine = logic.LogicEngine(SCHEMA, FREE_PINS)
    rules, errors = engine.load_source(source)
    assert not errors, "unexpected errors: %s" % [str(e) for e in errors]
    engine.activate(source, rules)
    return engine


def run(engine, namespace=None):
    ns = dict(SAMPLE if namespace is None else namespace)
    engine.evaluate(ns, 0)
    return ns


# --- parsing -------------------------------------------------------------

def test_comments_and_blanks_ignored():
    rules, errors = compile_source("# note\n\n   \n# another")
    assert rules == [] and errors == []


def test_simple_rule_parses():
    rules, errors = compile_source("rate_yaw_dps = 90")
    assert not errors
    assert len(rules) == 1
    assert rules[0].target == "rate_yaw_dps"
    assert rules[0].kind == logic.KIND_PARAM
    assert rules[0].line_number == 1


def test_rule_error_stringifies_with_line_number():
    # str() is what the page shows and what main.py prints, so it must work
    # without relying on Exception.__init__ (absent in MicroPython).
    error = logic.RuleError(7, "invalid syntax")
    assert str(error) == "line 7: invalid syntax"
    assert error.line_number == 7


def test_missing_equals_reports_line():
    rules, errors = compile_source("rate_yaw_dps 90")
    assert len(errors) == 1 and errors[0].line_number == 1
    assert "name = expression" in errors[0].message


def test_comparison_is_not_an_assignment():
    _rules, errors = compile_source("rate_yaw_dps == 90")
    assert errors and "name = expression" in errors[0].message


def test_comparison_inside_expression_is_fine():
    rules, errors = compile_source("gpio15 = wp_index == 4 and wp_distance_m < 20")
    assert not errors and len(rules) == 1
    assert rules[0].kind == logic.KIND_GPIO and rules[0].pin == 15


def test_conditional_without_else_is_named():
    # Python's conditional expression requires all three parts; leaving off
    # the else is the first mistake most people make, and MicroPython's own
    # SyntaxError says only "invalid syntax".
    _rules, errors = compile_source("gpio7 = 'on' if accel_x > 0.5")
    assert len(errors) == 1
    assert "else" in errors[0].message


def test_unbalanced_brackets_named():
    _rules, errors = compile_source("rate_yaw_dps = min(90, 45")
    assert errors and "bracket" in errors[0].message


def test_string_result_rejected_for_pin():
    # Every non-empty string is truthy, so 'off' would drive a pin HIGH.
    rules, errors = compile_source("gpio7 = 'on' if accel_x > 0.5 else 'off'")
    assert not errors, "syntax is valid; the type is the problem"
    problems = logic.smoke_test(rules, SAMPLE)
    assert problems, "a string result must be rejected"
    # The message has to name the value to use, not just what is wrong.
    message = problems[0].message
    assert "1 or 0" in message and "str" in message


def test_wrong_type_help_is_specific_per_output_kind():
    for source, expect in (
        ("gpio7 = 'on'", "1 or 0"),
        ("pwm7 = 'wide'", "microseconds"),
        ("rate_yaw_dps = 'fast'", "takes a number"),
    ):
        rules, errors = compile_source(source)
        assert not errors, source
        problems = logic.smoke_test(rules, SAMPLE)
        assert problems, source
        assert expect in problems[0].message, (source, problems[0].message)


def test_pin_rejects_string_at_runtime_too():
    engine = engine_with("gpio7 = 'on'")
    engine.evaluate(dict(SAMPLE), 0)
    assert engine.rules[0].error is not None
    assert "gpio7" not in engine.pin_states


def test_pin_uses_positive_not_truthiness():
    engine = engine_with("gpio7 = accel_x")
    ns = dict(SAMPLE)
    ns["accel_x"] = -1.0
    engine.evaluate(ns, 0)
    assert engine.pin_states["gpio7"] is False
    ns["accel_x"] = 0.9
    engine.evaluate(ns, 1)
    assert engine.pin_states["gpio7"] is True


def test_demand_targets_parse_and_clamp():
    engine = engine_with("throttle_demand = 2.5")
    run(engine)
    assert engine.demands["throttle_demand"] == 1.0    # clamped to 0..1
    assert engine.rules[0].kind == logic.KIND_DEMAND
    assert "throttle_demand" not in engine.overrides   # separate sink

    engine = engine_with("pitch_demand = -9")
    run(engine)
    assert engine.demands["pitch_demand"] == -1.0


def test_all_four_demands_available():
    source = "\n".join((
        "throttle_demand = 0.4",
        "pitch_demand = -0.2",
        "yaw_demand = 0.1",
        "roll_demand = 0.3",
    ))
    engine = engine_with(source)
    run(engine)
    assert engine.demands == {
        "throttle_demand": 0.4, "pitch_demand": -0.2,
        "yaw_demand": 0.1, "roll_demand": 0.3,
    }
    assert engine.overrides == {} and engine.pin_states == {}


def test_failsafe_outranks_demand_overrides():
    # The requirement: nothing a rule does may survive failsafe. It keys on
    # conditions rather than on mode names so a future mode cannot re-open
    # the path.
    assert not logic.demands_allowed(True)
    assert not logic.demands_allowed(True, False)


def test_gps_failsafe_also_outranks_demand_overrides():
    # The second failsafe: losing the position estimate in autonomous levels
    # the wings and cuts throttle, and a rule must not be able to put that
    # throttle back. The RC link is fine in this case, which is exactly why
    # checking link_lost alone was not enough.
    assert not logic.demands_allowed(False, True)
    assert not logic.demands_allowed(True, True)


def test_demands_allowed_in_every_live_mode():
    # Including manual: failsafe is the escape hatch, so manual does not
    # need to duplicate it. A rule wanting its own kill switch reads a
    # spare RC channel (see test_demand_can_fall_back_to_the_stick).
    assert logic.demands_allowed(False)
    assert logic.demands_allowed(False, False)


def test_failsafe_timeout_is_not_assignable():
    # A rule that could widen the link timeout could postpone the failsafe
    # that revokes its own demand authority.
    assert "failsafe_link_timeout_ms" not in logic.assignable_targets(
        config_module.SCHEMA)
    rules, errors = logic.compile_source(
        "failsafe_link_timeout_ms = 5000", config_module.SCHEMA, FREE_PINS)
    assert not rules and len(errors) == 1


def test_demand_can_fall_back_to_the_stick():
    # The user-space kill switch: an aux channel selects between the rule's
    # value and the pilot's stick, which is readable as an input.
    engine = engine_with("throttle_demand = 0.4 if ch7 > 1500 else throttle")
    ns = dict(SAMPLE)
    ns["ch7"], ns["throttle"] = 1800, 0.25
    engine.evaluate(ns, 0)
    assert engine.demands["throttle_demand"] == 0.4
    ns["ch7"] = 900
    engine.evaluate(ns, 1)
    assert engine.demands["throttle_demand"] == 0.25


def test_demand_wrong_type_message():
    rules, errors = compile_source("throttle_demand = 'fast'")
    assert not errors
    problems = logic.smoke_test(rules, SAMPLE)
    assert problems and "stick position" in problems[0].message


def test_unknown_target_rejected():
    _rules, errors = compile_source("nonsense = 1")
    assert errors and "not an assignable output" in errors[0].message


def test_channel_map_and_wifi_not_assignable():
    for name in ("channel_throttle", "wifi_password"):
        _rules, errors = compile_source(name + " = 1")
        assert errors, name + " should be rejected"


def test_claimed_pin_rejected():
    # GP2 is the left servo, GP8 is I2C -- neither is in FREE_PINS.
    for name in ("gpio2", "gpio8", "gpio99"):
        _rules, errors = compile_source(name + " = True")
        assert errors, name + " should be rejected"


def test_duplicate_target_rejected():
    _rules, errors = compile_source("rate_yaw_dps = 90\nrate_yaw_dps = 45")
    assert errors and "already set on line 1" in errors[0].message


def test_syntax_error_reports_line_number():
    _rules, errors = compile_source("# ok\nrate_yaw_dps = 90 +* 2")
    assert len(errors) == 1 and errors[0].line_number == 2
    assert "invalid syntax" in errors[0].message


def test_statements_rejected_by_eval_mode():
    # This is the containment: expression mode cannot express a loop or an
    # import, so the compiler refuses them outright.
    for bad in ("while True: pass", "import os", "x = 1"):
        _rules, errors = compile_source("rate_yaw_dps = " + bad)
        assert errors, bad + " should not compile"


def test_line_limit():
    source = "\n".join("rate_yaw_dps = %d" % n for n in range(logic.MAX_LINES + 2))
    _rules, errors = compile_source(source)
    assert errors and "too many lines" in errors[0].message


# --- smoke test ----------------------------------------------------------

def test_smoke_test_catches_undefined_name():
    rules, errors = compile_source("rate_yaw_dps = gust * 2")
    assert not errors
    problems = logic.smoke_test(rules, SAMPLE)
    assert len(problems) == 1 and problems[0].line_number == 1


def test_smoke_test_catches_non_numeric():
    rules, _ = compile_source("rate_yaw_dps = 'fast'")
    problems = logic.smoke_test(rules, SAMPLE)
    assert problems and "takes a number" in problems[0].message


def test_smoke_test_passes_good_rules():
    rules, _ = compile_source(
        "rate_yaw_dps = 90 if throttle > 0.3 else 45\ngpio15 = adc26 < 1.8")
    assert logic.smoke_test(rules, SAMPLE) == []


# --- evaluation ----------------------------------------------------------

def test_conditional_expression():
    engine = engine_with("rate_yaw_dps = 90 if throttle > 0.3 else 45")
    run(engine)
    assert engine.overrides["rate_yaw_dps"] == 90


def test_chained_conditionals():
    engine = engine_with(
        "rate_yaw_dps = 45 if throttle < 0.2 else 90 if throttle < 0.6 else 120")
    run(engine)
    assert engine.overrides["rate_yaw_dps"] == 90


def test_compound_condition_sets_pin():
    engine = engine_with("gpio15 = wp_index == 4 and wp_distance_m < 20")
    run(engine)
    assert engine.pin_states["gpio15"] is True


def test_compound_condition_false():
    engine = engine_with("gpio15 = wp_index == 4 and wp_distance_m < 20")
    ns = dict(SAMPLE)
    ns["wp_distance_m"] = 250.0
    engine.evaluate(ns, 0)
    assert engine.pin_states["gpio15"] is False


def test_results_clamped_to_schema_bounds():
    engine = engine_with("rate_yaw_dps = 99999")
    run(engine)
    assert engine.overrides["rate_yaw_dps"] == SCHEMA["rate_yaw_dps"][2]
    assert engine.rules[0].clamped

    engine = engine_with("pitch_angle_max_deg = -50")
    run(engine)
    assert engine.overrides["pitch_angle_max_deg"] == SCHEMA["pitch_angle_max_deg"][1]


def test_pwm_clamped_to_pulse_range():
    engine = engine_with("pwm14 = 99999")
    run(engine)
    assert engine.pin_states["pwm14"] == logic.PWM_MAX_US


def test_outputs_readable_for_latching():
    engine = engine_with("gpio16 = gpio16 or (wp_index == 4 and wp_distance_m < 20)")
    ns = dict(SAMPLE)
    ns["wp_distance_m"] = 500.0
    engine.evaluate(ns, 0)
    assert engine.pin_states["gpio16"] is False
    # Pass through the radius once...
    ns["wp_distance_m"] = 10.0
    engine.evaluate(ns, 1)
    assert engine.pin_states["gpio16"] is True
    # ...and it stays set after leaving it.
    ns["wp_distance_m"] = 500.0
    engine.evaluate(ns, 2)
    assert engine.pin_states["gpio16"] is True


def test_failing_rule_is_isolated():
    engine = engine_with("rate_yaw_dps = 120\npitch_angle_max_deg = 1 / 0")
    run(engine)
    assert engine.overrides["rate_yaw_dps"] == 120
    assert engine.rules[1].error is not None
    assert engine.rules[1].value is None
    assert "pitch_angle_max_deg" not in engine.overrides


def test_summary_reports_state():
    engine = engine_with("rate_yaw_dps = 99999\ngpio15 = True")
    run(engine)
    summary = engine.summary()
    assert summary[0]["target"] == "rate_yaw_dps"
    assert summary[0]["clamped"] is True
    assert summary[1]["value"] is True
    assert all(item["error"] is None for item in summary)


# --- scheduling ----------------------------------------------------------

def test_due_respects_interval():
    engine = engine_with("rate_yaw_dps = 90")
    assert engine.due(1000)
    engine.evaluate(dict(SAMPLE), 1000)
    assert not engine.due(1000 + logic.EVAL_INTERVAL_MS - 1)
    assert engine.due(1000 + logic.EVAL_INTERVAL_MS)


def test_disabled_engine_is_never_due():
    engine = logic.LogicEngine(SCHEMA, FREE_PINS)
    assert not engine.due(10_000)


def test_clear_disables():
    engine = engine_with("rate_yaw_dps = 90")
    engine.clear()
    assert not engine.enabled and engine.rules == []


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
