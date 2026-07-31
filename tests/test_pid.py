# Off-target tests for the PID controller (pure Python, no `machine`
# imports): `python3 tests/test_pid.py`.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))

from pid import PID


def test_proportional_only():
    controller = PID(kp=0.5, ki=0.0, kd=0.0)
    output = controller.update(error=1.0, dt=0.1)
    assert abs(output - 0.5) < 1e-9


def test_output_clamped_to_limit():
    controller = PID(kp=10.0, ki=0.0, kd=0.0, output_limit=1.0)
    assert controller.update(error=1.0, dt=0.1) == 1.0
    assert controller.update(error=-1.0, dt=0.1) == -1.0


def test_integral_accumulates_with_constant_error():
    controller = PID(kp=0.0, ki=1.0, kd=0.0, output_limit=100.0, integral_limit=100.0)
    output = 0.0
    for _ in range(5):
        output = controller.update(error=2.0, dt=0.1)
    # integral after 5 steps of error=2.0, dt=0.1 -> 1.0; ki=1.0 -> output 1.0
    assert abs(output - 1.0) < 1e-9


def test_integral_saturation_bounded_by_integral_limit():
    controller = PID(kp=0.0, ki=1.0, kd=0.0, output_limit=100.0, integral_limit=2.0)
    output = 0.0
    for _ in range(1000):
        output = controller.update(error=5.0, dt=0.1)
    assert abs(output - 2.0) < 1e-9


def test_derivative_responds_to_error_change():
    controller = PID(kp=0.0, ki=0.0, kd=1.0, output_limit=100.0)
    controller.update(error=0.0, dt=0.1)
    output = controller.update(error=1.0, dt=0.1)
    assert abs(output - 10.0) < 1e-9  # (1.0 - 0.0) / 0.1


def test_first_call_has_no_derivative_kick():
    controller = PID(kp=0.0, ki=0.0, kd=1.0)
    output = controller.update(error=5.0, dt=0.1)
    assert output == 0.0


def test_reset_clears_integral_and_derivative_state():
    controller = PID(kp=0.0, ki=1.0, kd=1.0, output_limit=100.0, integral_limit=100.0)
    controller.update(error=1.0, dt=0.1)
    controller.reset()
    output = controller.update(error=0.0, dt=0.1)
    assert output == 0.0


def test_feedforward_adds_directly_to_output():
    controller = PID(kp=0.0, ki=0.0, kd=0.0, kff=0.5, output_limit=100.0)
    output = controller.update(error=0.0, dt=0.1, feedforward_input=10.0)
    assert abs(output - 5.0) < 1e-9


def test_feedforward_defaults_to_zero_contribution():
    controller = PID(kp=1.0, ki=0.0, kd=0.0, kff=0.5, output_limit=100.0)
    # feedforward_input defaults to 0.0 when the caller doesn't pass it.
    output = controller.update(error=2.0, dt=0.1)
    assert abs(output - 2.0) < 1e-9


def test_feedforward_and_pid_sum_share_the_output_clamp():
    controller = PID(kp=1.0, ki=0.0, kd=0.0, kff=1.0, output_limit=1.0)
    output = controller.update(error=1.0, dt=0.1, feedforward_input=1.0)
    assert output == 1.0  # 1.0 + 1.0 clamped to the shared limit, not summed post-clamp


def test_feedforward_excluded_from_integrator():
    controller = PID(kp=0.0, ki=1.0, kd=0.0, kff=100.0, output_limit=1000.0, integral_limit=1000.0)
    # A large feedforward with zero error must not feed the integrator --
    # only error*dt accumulates.
    controller.update(error=0.0, dt=0.1, feedforward_input=50.0)
    output_after_ff_removed = controller.update(error=0.0, dt=0.1, feedforward_input=0.0)
    assert output_after_ff_removed == 0.0


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
