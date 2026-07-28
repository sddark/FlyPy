# Autonomous navigation design

Type: grilling (HITL)

## Status

open

## Question

Define the autonomous mode: waypoint representation, how the nav loop turns GPS position into attitude/throttle demands for the control loops (cross-track / L1-style guidance vs simple bearing chase), turn coordination without a rudder, heading source without a magnetometer (GPS course-over-ground, low-speed behavior), altitude strategy with GPS-only altitude, waypoint sequencing/advance criteria, and GPS-loss behavior (level out + cut throttle, per owner). Blocked by "Research: BZ-251 UBX configuration" and "Control system design" (demand interfaces).

## Assumptions

- BZ-251 provides position/velocity/course; no magnetometer, no baro.
- Mission entry method is fog (see map "Not yet specified") — this part may graduate it.

## Decision
