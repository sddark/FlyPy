# Flight modes, arming & failsafe

Type: grilling (HITL)

## Status

open

## Question

Turn the owner's decided behavior into a precise state-machine spec: armed/disarmed states, the arming TX-switch logic with the zero-throttle pre-arm check, mode selection channel ranges (manual / stabilized / autonomous), transition rules between modes, and exact failsafe behavior — RC link lost → level wings + cut throttle; GPS lost in autonomous → level out + cut throttle — including detection timeouts and how CRSF link loss is detected. Blocked by "Research: CRSF protocol under MicroPython" (failsafe signalling details).

## Assumptions

- Owner's decisions (map Notes) are fixed: TX-switch arming, zero-throttle only pre-arm, level+cut-throttle on both RC and GPS loss.
- WiFi config only while disarmed — the state machine must encode that.

## Decision
