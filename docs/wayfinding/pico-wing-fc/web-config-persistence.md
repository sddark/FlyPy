# Web config & parameter persistence

Type: grilling (HITL)

## Status

resolved 2026-07-29

## Question

Define the configuration part of the spec: the AP-mode network behavior (SSID, security, when the radio comes up/down), the parameter schema (every tunable from control/modes/failsafe/mixer — names, types, ranges, defaults), persistence format in flash (file format, atomicity, factory reset), validation rules, and the simplest-possible web interface (HTML forms; framework choice informed by research). Also decide whether config changes require reboot/disarm to take effect. Blocked by "Research: MicroPython platform feasibility" (web framework + flash findings).

## Assumptions

- Pico W hosts its own AP, WiFi only while disarmed, simplest interface (owner's words: "whatever is easiest").
- All parameters — PIDs, rates, mixing, failsafe, modes — are configurable here.
- **Bespoke** (per "Research: INAV transpile survey"): nothing ports from INAV — its CLI/MSP/EEPROM config is desktop-oriented. Web server (microdot), JSON-to-flash persistence, and validation are built fresh.

## Decision

- **AP/network:** Pico W hosts a WPA2 AP, SSID = `"pico-wing-" + wifi_ssid_suffix`, password = `wifi_password` — both are config values with shipped defaults, changeable via the portal itself. No pairing flow, QR code, or per-unit randomized password for the MVP — fixed and owner-changeable is enough for a bench-flying hobby project. Radio comes up only while disarmed and is torn down before arming (already implemented in `firmware/server.py`/`main.py`).
- **Parameter schema:** adopt `firmware/config.py`'s `SCHEMA` (name → default/min/max, numeric or string) as canonical; the web form renders one field per entry, one page for config plus dedicated bench-test pages (servo positioning, motor/ESC throttle, live RC-channel view, live IMU accel/gyro/attitude for wiring verification).
- **Persistence:** JSON file on flash (`config.json`), atomic write via tmp-file + rename, one backup file (`config.json.bak`) restored automatically if the primary is missing/corrupt; factory reset deletes all three files and reverts to schema defaults.
- **Validation:** every incoming value is clamped to its schema's [min, max] (or coerced to string); unknown keys are ignored; missing keys fall back to defaults.
- **Apply timing:** config changes **apply live — no reboot or re-arm cycle needed**. Saving updates the in-memory active config and dependent modules (e.g. the RC channel map) immediately, matching the `persist()` callback plumbing in `firmware/main.py`.
- **Framework:** microdot + plain HTML forms, no client-side framework — confirmed as "simplest possible" per the owner. Nothing ports from INAV here (its CLI/MSP/EEPROM tooling is desktop-oriented).
