# Hardware side of user logic: drives the free GPIOs that logic.py asks for.
# Kept separate so logic.py stays pure Python and testable off-target, the
# same split as esc.py/servos.py against the rest of the flight code.
#
# Pins are created lazily, on first assignment, so a pin nothing references
# is never configured. A pin is either digital or PWM depending on which name
# the script used (gpioN vs pwmN); asking for both is refused, because they
# cannot share the peripheral.

from machine import ADC, Pin, PWM

import pins as pin_map

ADC_PINS = pin_map.ADC_PINS
_ADC_FULL_SCALE = 65535
_ADC_REFERENCE_V = 3.3

_PWM_FREQUENCY_HZ = 50
_PWM_PERIOD_US = 20_000
_DUTY_FULL_SCALE = 65_535

def free_pins():
    return pin_map.FREE_PINS


class AuxPins:
    def __init__(self):
        self._digital = {}
        self._pwm = {}
        self._adc = {}

    def read_adc(self, number):
        # Volts at the pin. Created on first read so an unused ADC channel
        # never claims its pin.
        channel = self._adc.get(number)
        if channel is None:
            try:
                channel = ADC(Pin(number))
            except (ValueError, OSError):
                return 0.0
            self._adc[number] = channel
        return channel.read_u16() * _ADC_REFERENCE_V / _ADC_FULL_SCALE

    def apply(self, pin_states):
        # pin_states: {"gpio15": True, "pwm14": 1500.0, ...}
        for name, value in pin_states.items():
            if name.startswith("gpio"):
                self._set_digital(int(name[4:]), value)
            elif name.startswith("pwm"):
                self._set_pwm(int(name[3:]), value)

    def _set_digital(self, number, value):
        pin = self._digital.get(number)
        if pin is None:
            if number in self._pwm:
                return  # already a PWM output; refuse to fight over it
            pin = Pin(number, Pin.OUT)
            self._digital[number] = pin
        pin.value(1 if value else 0)

    def _set_pwm(self, number, microseconds):
        channel = self._pwm.get(number)
        if channel is None:
            if number in self._digital:
                return
            channel = PWM(Pin(number))
            channel.freq(_PWM_FREQUENCY_HZ)
            self._pwm[number] = channel
        duty = int(microseconds * _DUTY_FULL_SCALE / _PWM_PERIOD_US)
        channel.duty_u16(duty)

    def all_off(self):
        # Called on disarm and whenever logic stops: outputs fall to a known
        # state rather than holding whatever they were last commanded.
        for pin in self._digital.values():
            pin.value(0)
        for channel in self._pwm.values():
            channel.duty_u16(0)

    def release(self):
        self.all_off()
        for channel in self._pwm.values():
            channel.deinit()
        self._pwm = {}
        self._digital = {}
