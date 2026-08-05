# Off-target tests for config validation (pure Python, no `machine`
# imports): `python3 tests/test_config.py`.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))

import config as config_module

_SERVER_SOURCE = os.path.join(
    os.path.dirname(__file__), "..", "firmware", "server.py")


def _config_group_names():
    # server.py imports `network`, so it cannot be imported off-target --
    # read _CONFIG_GROUPS out of the source instead. Worth the awkwardness:
    # the config page renders only what those groups name, so a parameter
    # added to SCHEMA and forgotten here is invisible in the portal and
    # silently untunable, with nothing anywhere to say so.
    import ast

    with open(_SERVER_SOURCE) as handle:
        tree = ast.parse(handle.read())
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if not any(getattr(t, "id", None) == "_CONFIG_GROUPS" for t in node.targets):
            continue
        names = set()
        for _label, entries in ast.literal_eval(node.value):
            names.update(entries)
        return names
    raise AssertionError("_CONFIG_GROUPS not found in server.py")


def test_every_schema_parameter_is_on_the_config_page():
    missing = sorted(set(config_module.SCHEMA) - _config_group_names())
    assert not missing, "not shown on the config page: %s" % ", ".join(missing)


def test_config_page_names_no_parameter_that_does_not_exist():
    unknown = sorted(_config_group_names() - set(config_module.SCHEMA))
    assert not unknown, "config page names unknown parameters: %s" % ", ".join(unknown)


def test_validate_without_base_uses_defaults():
    assert config_module.validate({}) == config_module.defaults()


def test_validate_merges_onto_base():
    # A partial POST (only some fields present) must keep the base's values
    # for everything it doesn't mention -- not factory-reset them.
    base = config_module.defaults()
    base["pid_pitch_p"] = 9.0
    updated = config_module.validate({"pid_yaw_p": "12"}, base)
    assert updated["pid_pitch_p"] == 9.0
    assert updated["pid_yaw_p"] == 12.0


def test_validate_unparsable_number_keeps_base_value():
    base = config_module.defaults()
    base["pid_pitch_p"] = 9.0
    updated = config_module.validate({"pid_pitch_p": "bogus"}, base)
    assert updated["pid_pitch_p"] == 9.0


def test_numeric_values_clamped_to_schema_bounds():
    updated = config_module.validate({"pid_pitch_p": "9999"})
    assert updated["pid_pitch_p"] == 255.0


def test_short_wifi_password_rejected():
    # WPA2 needs >= 8 chars; a shorter persisted password would fail
    # ap.config() at boot and brick the portal.
    base = config_module.defaults()
    base["wifi_password"] = "longenough"
    updated = config_module.validate({"wifi_password": "abc"}, base)
    assert updated["wifi_password"] == "longenough"


def test_empty_and_oversize_ssid_suffix_rejected():
    default_suffix = config_module.SCHEMA["wifi_ssid_suffix"][0]
    assert config_module.validate({"wifi_ssid_suffix": ""})["wifi_ssid_suffix"] == default_suffix
    too_long = "x" * 23  # "pico-wing-" prefix + 23 exceeds the 32-char SSID cap
    assert config_module.validate({"wifi_ssid_suffix": too_long})["wifi_ssid_suffix"] == default_suffix


def test_inverted_servo_endpoints_revert_to_base_pair():
    # min >= max would pin the servo at one endpoint (the clamp becomes a
    # constant), so the pair reverts together.
    base = config_module.defaults()
    base["servo_left_min_us"] = 1100.0
    base["servo_left_max_us"] = 1900.0
    updated = config_module.validate(
        {"servo_left_min_us": "2200", "servo_left_max_us": "800"}, base
    )
    assert updated["servo_left_min_us"] == 1100.0
    assert updated["servo_left_max_us"] == 1900.0
    assert updated["servo_right_min_us"] == 1000.0  # untouched pair unaffected


def test_duplicate_channel_assignments_revert():
    # channel_arm=3 collides with channel_throttle's default of 3 -- the whole
    # channel map reverts rather than letting two controls read one stick.
    updated = config_module.validate({"channel_arm": "3"})
    assert updated["channel_arm"] == 5.0
    assert updated["channel_throttle"] == 3.0


def test_valid_channel_reassignment_still_applies():
    updated = config_module.validate({"channel_arm": "7"})
    assert updated["channel_arm"] == 7.0


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
