# Research: BZ-251 UBX configuration

Type: research (AFK)

## Status

resolved 2026-07-28

## Question

What does the spec need to say about driving the BZGNSS BZ-251 (u-blox M10) from MicroPython? Cover: (a) default baud/protocol out of the box and how to configure it via UBX (UBX-CFG-VALSET on M10, not legacy CFG-MSG) — target message set for navigation (NAV-PVT at what rate), turning off NMEA spam; (b) NAV-PVT fields the FC needs (fix type, lat/lon, speed, course-over-ground, altitude, sats, hAcc/vAcc) and binary parsing in MicroPython; (c) realistic fix rates on M10 and any dynamic platform model (airborne) setting; (d) existing MicroPython u-blox libraries/reference code (cite + license); (e) power-on time-to-first-fix expectations. Output: findings doc with the exact UBX config sequence and a NAV-PVT parse sketch.

## Assumptions

- GPS connects to the second Pico W UART.
- No magnetometer — course-over-ground is the only heading source (noted on the map's fog list).

## Decision

M10 configured via UBX-CFG-VALSET (RAM layer, every boot): 5 Hz nav rate, airborne <4g dynamic model, NAV-PVT on, all NMEA off. Default baud 115 200. NAV-PVT field map documented; bespoke ~100-line driver recommended over porting a library. TTFF ~27 s cold. Full findings: [docs/research/bz251-ubx.md](../../research/bz251-ubx.md)
