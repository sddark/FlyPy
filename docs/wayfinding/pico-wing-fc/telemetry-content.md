# Telemetry content spec

Type: grilling (HITL)

## Status

open

## Question

Decide exactly which CRSF telemetry the FC sends to the TX: which frames (GPS position, attitude, flight mode string, link/status), at what rates, and what the pilot actually wants on the radio during flight. Blocked by "Research: CRSF protocol under MicroPython" (frame formats and polling mechanics).

## Assumptions

- No battery voltage sensing exists, so battery telemetry is either absent or stubbed.
- Telemetry is nice-to-have relative to control; it must never starve the control loop.
- **Bespoke content, transpiled encoding** (per "Research: INAV transpile survey"): the frame/field selection is decided here fresh (INAV sends everything); the frame encoders port from INAV `telemetry/crsf.c` (~830 lines).

## Decision
