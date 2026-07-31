# Off-target tests for the Mahony attitude filter (pure Python, no
# `machine` imports): `python3 tests/test_attitude.py`. Gravity vectors for
# a given pitch/roll are derived from the same quaternion-to-Euler formulas
# the filter itself uses, so these check the whole update() loop converges
# to the correct steady state, not just the algebra in isolation.

import os
import sys
from math import cos, radians, sin

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))

from attitude import MahonyFilter

_DT = 1.0 / 250
_SETTLE_ITERATIONS = 2500


def _settle(filt, accel_g, iterations=_SETTLE_ITERATIONS):
    for _ in range(iterations):
        filt.update((0.0, 0.0, 0.0), accel_g, _DT)


def test_level_stays_level():
    filt = MahonyFilter()
    _settle(filt, accel_g=(0.0, 0.0, 1.0))
    assert abs(filt.roll_deg) < 0.5
    assert abs(filt.pitch_deg) < 0.5


def test_converges_to_pitch_angle():
    theta = radians(30.0)
    filt = MahonyFilter()
    _settle(filt, accel_g=(-sin(theta), 0.0, cos(theta)))
    assert abs(filt.pitch_deg - 30.0) < 1.0
    assert abs(filt.roll_deg) < 1.0


def test_converges_to_roll_angle():
    phi = radians(20.0)
    filt = MahonyFilter()
    _settle(filt, accel_g=(0.0, sin(phi), cos(phi)))
    assert abs(filt.roll_deg - 20.0) < 1.0
    assert abs(filt.pitch_deg) < 1.0


def test_accel_far_from_one_g_is_ignored():
    # ~4.2 g (hard maneuver/vibration): not a usable gravity reference, so
    # with zero gyro the estimate must not move at all. Same for free fall.
    filt = MahonyFilter()
    for accel_g in ((0.0, 3.0, 3.0), (0.0, 0.0, 0.0)):
        _settle(filt, accel_g=accel_g, iterations=500)
        assert filt.roll_deg == 0.0
        assert filt.pitch_deg == 0.0


def test_reset_returns_to_level():
    filt = MahonyFilter()
    _settle(filt, accel_g=(0.0, sin(radians(20.0)), cos(radians(20.0))))
    filt.reset()
    assert filt.roll_deg == 0.0
    assert filt.pitch_deg == 0.0


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
