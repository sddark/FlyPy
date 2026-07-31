# Standard 50 Hz hobby servo output on a hardware PWM channel.

from machine import Pin, PWM

_SERVO_FREQUENCY_HZ = 50
_SERVO_PERIOD_US = 20_000
_DUTY_FULL_SCALE = 65_535


class ServoOutput:
    def __init__(self, pin_number, minimum_pulse_us=1000, maximum_pulse_us=2000):
        self._pwm = PWM(Pin(pin_number))
        self._pwm.freq(_SERVO_FREQUENCY_HZ)
        self._minimum_pulse_us = minimum_pulse_us
        self._maximum_pulse_us = maximum_pulse_us
        self.center()

    def set_endpoints(self, minimum_pulse_us, maximum_pulse_us):
        self._minimum_pulse_us = minimum_pulse_us
        self._maximum_pulse_us = maximum_pulse_us

    def set_pulse_us(self, pulse_us):
        clamped = min(max(pulse_us, self._minimum_pulse_us), self._maximum_pulse_us)
        duty = int(clamped * _DUTY_FULL_SCALE / _SERVO_PERIOD_US)
        self._pwm.duty_u16(duty)

    def set_normalized(self, position):
        midpoint = (self._minimum_pulse_us + self._maximum_pulse_us) / 2
        half_span = (self._maximum_pulse_us - self._minimum_pulse_us) / 2
        clamped = min(max(position, -1.0), 1.0)
        self.set_pulse_us(midpoint + clamped * half_span)

    def center(self):
        self.set_normalized(0.0)

    def release(self):
        self._pwm.deinit()
