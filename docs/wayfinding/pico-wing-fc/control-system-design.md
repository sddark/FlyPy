# Control system design

Type: grilling (HITL)

## Status

open

## Question

Define the control core of the spec: loop rate(s), PID structure for pitch/roll (and yaw handling for a V-tail), rate-vs-attitude loop nesting per flight mode, the V-tail mixer (elevator/aileron → left/right surface equations, throws, directions), throttle path (manual vs stabilized vs autonomous), channel mapping from CRSF, and the full list of tunable parameters with ranges and defaults — everything the web config must expose. Blocked by "Research: MPU6050 attitude estimation" (estimator choice shapes the loops) and "Research: MicroPython platform feasibility" (achievable rates).

## Assumptions

- **Transpile from INAV**: loops and V-tail mixer port from INAV `flight/pid.c` + `mixer.c` (per owner directive); this part becomes "adapt INAV's fixed-wing controller" not "design a controller".
- Modes are manual / stabilized / autonomous (autonomous feeds its own demands into these loops).
- V-tail only; no rudder channel, no coordinated-turn logic.
- All loop parameters are web-configurable, per owner.

## Decision
