# Research: MicroPython platform feasibility

Type: research (AFK)

## Status

open

## Question

What can MicroPython on the Pico W actually deliver for this FC, and with which building blocks? Specifically: (a) generating Oneshot125 (125–250 µs pulses) and standard 50 Hz servo PWM from `machine.PWM` — achievable frequencies/resolution and jitter; (b) running a 50–100 Hz control loop reliably alongside WiFi (scheduler options: `asyncio` vs `_thread` — note WiFi/MicroPython threading caveats on Pico W); (c) hosting an AP-mode web server (which MicroPython web frameworks work on Pico W — e.g. microdot — and memory footprint); (d) writing/reading a config file in flash safely; (e) dual-UART + I²C usage. Output: a findings doc with cited MicroPython/Pico W docs, and a recommended architecture (single-core asyncio vs threads), plus any hard blockers.

## Assumptions

- MicroPython (not CircuitPython), latest stable for Pico W.
- WiFi is only active while disarmed, so the web server never shares CPU with active stabilization — this relaxes (b) considerably.
- Fixed-wing loop rates of 50–100 Hz are the target, per owner.

## Decision
