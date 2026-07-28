# Web config & parameter persistence

Type: grilling (HITL)

## Status

open

## Question

Define the configuration part of the spec: the AP-mode network behavior (SSID, security, when the radio comes up/down), the parameter schema (every tunable from control/modes/failsafe/mixer — names, types, ranges, defaults), persistence format in flash (file format, atomicity, factory reset), validation rules, and the simplest-possible web interface (HTML forms; framework choice informed by research). Also decide whether config changes require reboot/disarm to take effect. Blocked by "Research: MicroPython platform feasibility" (web framework + flash findings).

## Assumptions

- Pico W hosts its own AP, WiFi only while disarmed, simplest interface (owner's words: "whatever is easiest").
- All parameters — PIDs, rates, mixing, failsafe, modes — are configurable here.
- **Bespoke** (per "Research: INAV transpile survey"): nothing ports from INAV — its CLI/MSP/EEPROM config is desktop-oriented. Web server (microdot), JSON-to-flash persistence, and validation are built fresh.

## Decision
