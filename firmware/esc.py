# Oneshot125 ESC output via the PWM-slice trick (research: the slice period is
# the control-loop period and the duty carries the 125-250 us throttle pulse;
# hardware generates the pulse, so Python jitter does not matter).

from machine import Pin, PWM

# Public: the slice trick ties the ESC PWM period to the flight-loop period,
# so main.py imports this instead of defining its own copy of 250.
LOOP_FREQUENCY_HZ = 250
_LOOP_PERIOD_US = 1_000_000 // LOOP_FREQUENCY_HZ
_DUTY_FULL_SCALE = 65_535

_MIN_THROTTLE_US = 125
_MAX_THROTTLE_US = 250


class EscOutput:
    def __init__(self, pin_number):
        self._pwm = PWM(Pin(pin_number))
        self._pwm.freq(LOOP_FREQUENCY_HZ)
        self.stop()

    def set_throttle(self, throttle):
        clamped = min(max(throttle, 0.0), 1.0)
        pulse_us = _MIN_THROTTLE_US + clamped * (_MAX_THROTTLE_US - _MIN_THROTTLE_US)
        self._pwm.duty_u16(int(pulse_us * _DUTY_FULL_SCALE / _LOOP_PERIOD_US))

    def stop(self):
        self._pwm.duty_u16(0)

    def release(self):
        self.stop()
        self._pwm.deinit()
