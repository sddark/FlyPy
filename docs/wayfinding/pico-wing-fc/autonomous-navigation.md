# Autonomous navigation design

Type: grilling (HITL)

## Status

open

## Question

Define the autonomous mode: waypoint representation, how the nav loop turns GPS position into attitude/throttle demands for the control loops (cross-track / L1-style guidance vs simple bearing chase), turn coordination without a rudder, heading source without a magnetometer (GPS course-over-ground, low-speed behavior), altitude strategy with GPS-only altitude, waypoint sequencing/advance criteria, and GPS-loss behavior (level out + cut throttle, per owner). Blocked by "Research: BZ-251 UBX configuration" and "Control system design" (demand interfaces).

## Assumptions

- **Transpile from INAV** (surveyed, see "Research: INAV transpile survey"): waypoint execution and guidance from `navigation/navigation.c` (~5466 lines — port the fixed-wing WP subset only), fixed-wing guidance from `navigation/navigation_fixedwing.c` (~939 lines), geo math from `navigation_geo.c`, `sqrt_controller.c`. Skip pos_estimator AGL/flow, geozone, and launch code.
- **Bespoke:** BZ-251 UBX driver (~100 lines, per "Research: BZ-251 UBX configuration") and mission entry (INAV's MSP upload is out of scope — no configurator).
- BZ-251 provides position/velocity/course; no magnetometer, no baro.
- Mission entry method is fog (see map "Not yet specified") — this part may graduate it.

## Decision
