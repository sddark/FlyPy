# Research: INAV transpile survey

Type: research (AFK)

## Status

open

## Question

Which INAV modules should this project transpile to MicroPython, and what does each one actually look like in the source? For each candidate module, point at the exact files/functions in the INAV repo (with permalinks) and summarize size, dependencies, and MicroPython-porting difficulty: (a) fixed-wing PID controller and V-tail mixer (`src/main/flight/pid.c`, `mixer.c`); (b) navigation core — waypoint mission execution, L1/cross-track guidance, GPS-loss handling (`src/main/navigation/`); (c) arming/failsafe state machine (`src/main/fc/fc_core.c`, `runtime_config`, failsafe.c); (d) CRSF rx/telemetry (`src/main/rx/crsf.c`, `telemetry/crsf.c`); (e) attitude estimation (what INAV uses for gyro+accel — IMU Madgwick/Mahony variant in `src/main/flight/imu.c`). Also note INAV's license (GPLv3) and what that implies for a transpiled derivative. Output: findings doc mapping each spec part to its INAV source-of-truth, so the grilling sessions design *against real code*, not memory.

## Assumptions

- Owner directive: transpiling INAV parts is preferable — a proven base, less hallucination. INAV's approach is the default wherever it conflicts with other research (its imu.c estimator vs the Mahony recommendation is exactly such a case — likely agreement, verify).
- Target INAV: latest stable release branch, fixed-wing platform only; multirotor-specific code ignored.

## Decision
