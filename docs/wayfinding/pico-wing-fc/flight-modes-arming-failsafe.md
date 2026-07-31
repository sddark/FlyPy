# Flight modes, arming & failsafe

Type: grilling (HITL)

## Status

resolved 2026-07-29

## Question

Turn the owner's decided behavior into a precise state-machine spec: armed/disarmed states, the arming TX-switch logic with the zero-throttle pre-arm check, mode selection channel ranges (manual / stabilized / autonomous), transition rules between modes, and exact failsafe behavior — RC link lost → level wings + cut throttle; GPS lost in autonomous → level out + cut throttle — including detection timeouts and how CRSF link loss is detected. Blocked by "Research: CRSF protocol under MicroPython" (failsafe signalling details).

## Assumptions

- **Transpile from INAV** (surveyed, see "Research: INAV transpile survey"): arming/runtime state from `fc/fc_core.c` + `fc/runtime_config.c`, mode-activation channel ranges from `fc/rc_modes.c`, failsafe detection/procedure structure from `flight/failsafe.c` (~633 lines) — all simplified to this build's three modes and level+cut-throttle procedures.
- Owner's decisions (map Notes) are fixed: TX-switch arming, zero-throttle only pre-arm, level+cut-throttle on both RC and GPS loss.
- WiFi config only while disarmed — the state machine must encode that.

## Decision

- **Arm:** TX arm-switch ON **and** throttle ≤ `arm_max_throttle_us` (default 1050 µs) → armed. Disarm: switch OFF, any throttle → disarmed, WiFi config portal comes back up.
- **Mode select:** one 3-position switch channel, split into equal thirds of CRSF travel: low = manual, mid = stabilized, high = autonomous. Since autonomous nav is deferred for the MVP, **the high third falls back to stabilized behavior** — there is no dead or undefined switch position. Mode changes take effect immediately, freely, at any time while armed (no lockout or confirmation).
- **RC-link-loss failsafe:** link considered lost after `failsafe_link_timeout_ms` (default 500 ms) of silence (no CRSF 0x16 frames), since ELRS signals loss by simply stopping transmission rather than sending an explicit failsafe frame. On loss, in every mode (including the autonomous-falls-back-to-stabilized case): zero roll/pitch/yaw demand into the mixer (wings level) + throttle cut to 0.
- **GPS-loss failsafe (autonomous):** not applicable for the MVP — no GPS/autonomous mode yet. Revisit alongside [Autonomous navigation design](./autonomous-navigation.md) (deferred).
- WiFi portal runs only while disarmed and is torn down before the flight loop starts on arm — matches `firmware/main.py`'s state machine.
