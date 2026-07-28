# Research: INAV transpile survey

Type: research (AFK)

## Status

resolved 2026-08-04

## Question

Which INAV modules should this project transpile to MicroPython, and what does each one actually look like in the source? For each candidate module, point at the exact files/functions in the INAV repo (with permalinks) and summarize size, dependencies, and MicroPython-porting difficulty: (a) fixed-wing PID controller and V-tail mixer (`src/main/flight/pid.c`, `mixer.c`); (b) navigation core — waypoint mission execution, L1/cross-track guidance, GPS-loss handling (`src/main/navigation/`); (c) arming/failsafe state machine (`src/main/fc/fc_core.c`, `runtime_config`, failsafe.c); (d) CRSF rx/telemetry (`src/main/rx/crsf.c`, `telemetry/crsf.c`); (e) attitude estimation (what INAV uses for gyro+accel — IMU Madgwick/Mahony variant in `src/main/flight/imu.c`). Also note INAV's license (GPLv3) and what that implies for a transpiled derivative. Output: findings doc mapping each spec part to its INAV source-of-truth, so the grilling sessions design *against real code*, not memory.

## Assumptions

- Owner directive: transpiling INAV parts is preferable — a proven base, less hallucination. INAV's approach is the default wherever it conflicts with other research (its imu.c estimator vs the Mahony recommendation is exactly such a case — likely agreement, verify).
- Target INAV: latest stable release branch, fixed-wing platform only; multirotor-specific code ignored.

## Decision

Verified against INAV `master` (fixed-wing relevant files only; line counts are whole-file, ported subset is smaller).

### Transpile from INAV (port module-by-module)

| Module | INAV source (all under `src/main/`) | Size | Porting difficulty |
|---|---|---|---|
| Fixed-wing PID controller | `flight/pid.c` | ~1510 lines | Medium — port the fixed-wing PIFF/PID path only; drop multirotor, anti-windup extras as needed |
| V-tail mixer | `flight/mixer.c` | ~739 lines | Easy — mixer table + `mixTable()` math; V-tail row only |
| Attitude estimation (gyro+accel) | `flight/imu.c` | ~986 lines | Medium — Mahony-style complementary filter; confirms the Mahony recommendation from "Research: MPU6050 attitude estimation" |
| Navigation core (WP execution, guidance, GPS-loss) | `navigation/navigation.c` (~5466 lines), `navigation/navigation_fixedwing.c` (~939), `navigation_geo.c`, `sqrt_controller.c` | Large | Hard — port the fixed-wing waypoint/RTH subset only; skip pos_estimator AGL/flow/geozone/launch |
| Arming / runtime state | `fc/fc_core.c`, `fc/runtime_config.c`, `fc/rc_modes.c` | ~1100 lines | Medium — port the arming-state and mode-activation logic, simplified to 3 modes |
| Failsafe state machine | `flight/failsafe.c` | ~633 lines | Medium — port detection/procedure structure; procedures simplified to level+cut-throttle |
| CRSF RX frame handling | `rx/crsf.c` | ~368 lines | Easy — frame decode maps directly onto the layouts in "Research: CRSF protocol" |
| CRSF telemetry encoding | `telemetry/crsf.c` | ~830 lines | Easy — port frame encoders only; content selection is bespoke (see below) |

### Bespoke (no INAV counterpart — build fresh)

- **MicroPython hardware drivers:** MPU6050 (I²C), BZ-251 UBX (research part already recommends a bespoke ~100-line driver), servo/ESC PWM + Oneshot125 output on RP2040 slices.
- **Web config + persistence:** Pico W AP, microdot HTML forms, JSON-to-flash parameter store. INAV's equivalents (CLI/MSP/EEPROM) are desktop-oriented; nothing to port.
- **Scheduler/runtime:** asyncio loop replacing INAV's `fc_tasks.c` scheduler; only the task-rate *budget* is borrowed.
- **Telemetry content spec:** which frames/fields to send is a bespoke decision (INAV sends everything); encoding ported from `telemetry/crsf.c`.
- **Mission entry:** fog item — INAV's MSP mission upload is out of scope (no configurator), so entry is bespoke (web UI / file).

### System diagram (transpile vs bespoke)

Candidate for the system-level block diagram the "Spec structure & format" part asks for:

```mermaid
flowchart LR
    subgraph inputs[Inputs]
        crsf_rx["CRSF RX decode<br/>(transpiled: rx/crsf.c)"]
        gps_drv["BZ-251 UBX driver<br/>(bespoke)"]
        imu_drv["MPU6050 I²C driver<br/>(bespoke)"]
    end

    subgraph core[Flight core — asyncio scheduler (bespoke)]
        imu["Attitude estimation<br/>(transpiled: flight/imu.c)"]
        pid["Fixed-wing PID<br/>(transpiled: flight/pid.c)"]
        mixer["V-tail mixer<br/>(transpiled: flight/mixer.c)"]
        nav["Waypoint nav / guidance<br/>(transpiled: navigation/ fixed-wing subset)"]
        modes["Arming, modes & failsafe<br/>(transpiled: fc_core.c, rc_modes.c, failsafe.c)"]
    end

    subgraph outputs[Outputs]
        pwm["Servo/ESC PWM + Oneshot125<br/>(bespoke)"]
        telem["CRSF telemetry encoders<br/>(transpiled: telemetry/crsf.c;<br/>content bespoke)"]
    end

    subgraph ground[Ground — disarmed only]
        webcfg["Web config + flash persistence<br/>(bespoke)"]
        mission["Mission entry<br/>(bespoke — fog item)"]
    end

    crsf_rx --> modes
    crsf_rx --> pid
    imu_drv --> imu
    gps_drv --> nav
    imu --> pid
    nav --> pid
    modes --> pid
    modes --> nav
    pid --> mixer --> pwm
    nav --> telem
    webcfg -.configures.-> core
    mission -.loads waypoints.-> nav
```

### License

INAV is **GPL-3.0** (`LICENSE`, repo-reported SPDX GPL-3.0). A transpiled derivative is a derivative work: the MicroPython firmware must be GPLv3, with source available. Record this in the spec; it also means all ported files keep attribution headers.

Confirmed the assumption in "Research: MPU6050 attitude estimation": INAV's `imu.c` uses a Mahony-family estimator for gyro+accel without mag — the two research parts agree, no deviation to justify.
