# Status LED via the Pico W's onboard LED (wired to the wireless chip as
# "LED", not a plain GPIO -- MicroPython's board definition handles that
# transparently). Non-blocking: `tick()` is called once per flight-loop
# iteration and toggles based on elapsed time, so blinking never blocks the
# control loop. Hardware-only (like esc.py/servos.py) -- not covered by the
# off-target test suite.

import time
from machine import Pin

_BLINK_SLOW_MS = 1000  # portal up, disarmed
_BLINK_FAST_MS = 150  # failsafe (RC link lost while armed)


class StatusLed:
    def __init__(self, pin=None):
        if pin is None:
            pin = Pin("LED", Pin.OUT)
        self._pin = pin
        self._mode = None
        self._value = False
        self._last_toggle_ms = 0
        self.set_mode("off")

    def set_mode(self, mode):
        if mode == self._mode:
            return
        self._mode = mode
        if mode == "solid":
            self._value = True
            self._pin.value(1)
        elif mode == "off":
            self._value = False
            self._pin.value(0)
        # blink_slow / blink_fast pick up their phase on the next tick().

    def tick(self, now_ms):
        if self._mode == "blink_slow":
            period_ms = _BLINK_SLOW_MS
        elif self._mode == "blink_fast":
            period_ms = _BLINK_FAST_MS
        else:
            return
        if time.ticks_diff(now_ms, self._last_toggle_ms) >= period_ms:
            self._value = not self._value
            self._pin.value(1 if self._value else 0)
            self._last_toggle_ms = now_ms
