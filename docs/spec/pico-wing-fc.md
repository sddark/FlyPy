# Pico W Fixed-Wing Flight Controller — Spec (MVP)

Status: **MVP** (manual + stabilized modes). Telemetry and autonomous navigation are deferred — see [Deferred / Post-MVP](#deferred--post-mvp).

This is the build deliverable: what to implement, not why. For the reasoning, assumptions, and research behind each decision, see the wayfinding trail at [docs/wayfinding/pico-wing-fc.md](../wayfinding/pico-wing-fc.md) and its per-part docs — each section below links back to the part that resolved it.

## System overview

MicroPython flight controller on a Raspberry Pi Pico W (RP2040) for a V-tail fixed-wing drone.

**Fixed hardware:**

| Component | Interface | Notes |
|---|---|---|
| Raspberry Pi Pico W | — | RP2040, MicroPython |
| MPU6050 (gyro + accel) | I²C | No magnetometer on this chip |
| BZGNSS BZ-251 GPS | UART (u-blox M10, UBX) + onboard QMC5883 compass (I²C) | Wired, unused for MVP (autonomous nav deferred) — the compass's role, if any, is undecided |
| ELRS receiver | UART (CRSF, 420 000 baud) | RX only for MVP (telemetry deferred) |
| 1× ESC + BEC | PWM (Oneshot125) | Powers the Pico W via VSYS; no battery voltage sensing |
| 2× V-tail servos | PWM (50 Hz) | Digital, standard PWM |

System-level block diagram (inputs/outputs, transpiled-vs-bespoke): [docs/wayfinding/pico-wing-fc/system-diagrams.md](../wayfinding/pico-wing-fc/system-diagrams.md).

**Implementation strategy:** transpile from INAV where a module exists (PID, mixer, attitude estimator, arming/failsafe structure); build bespoke where it doesn't (hardware drivers, web config, scheduler). See [Research: INAV transpile survey](../wayfinding/pico-wing-fc/research-inav-survey.md) for the full module map and license note (INAV is GPLv3 — this derivative inherits it).

**Reference implementation:** `firmware/` in this repo implements this spec's MVP scope. File paths below point at the module for each part:

| Module | Role |
|---|---|
| `firmware/main.py` | Entry point, arm/disarm state machine, 250 Hz flight loop |
| `firmware/rc.py` | CRSF frame parsing + channel normalization |
| `firmware/imu.py` | MPU6050 I²C driver (raw registers, gyro bias calibration) |
| `firmware/attitude.py` | Mahony attitude estimator (roll/pitch from gyro+accel) |
| `firmware/pid.py` | Generic PID controller (pitch angle loop, yaw rate loop) |
| `firmware/mixer.py` | V-tail linear mixer |
| `firmware/servos.py` / `firmware/esc.py` | Servo PWM / Oneshot125 ESC output |
| `firmware/led.py` | Status LED (portal/armed/failsafe) |
| `firmware/config.py` | Parameter schema, validation, flash persistence |
| `firmware/server.py` | Web config portal + bench-test pages |
| `firmware/pins.py` | Pin map |
| `tests/` | Off-target unit tests (`python3 tests/test_*.py`) for every pure-Python module above |

---

## Pin map & power architecture

Resolved by: [pin-map-power.md](../wayfinding/pico-wing-fc/pin-map-power.md)

**Inputs:** CRSF UART RX, GPS UART RX (wired, unused for MVP), MPU6050 I²C.
**Outputs:** 2× servo PWM, 1× ESC PWM (Oneshot125), status LED (onboard, WL-chip).

| Function | Peripheral | Pins |
|---|---|---|
| CRSF (ELRS RX) | UART0 | GP0 TX (telemetry wire, unused for MVP), GP1 RX |
| GPS (BZ-251 UBX) | UART1 | GP4 TX, GP5 RX (unused for MVP) |
| MPU6050 | I2C0 | GP8 SDA, GP9 SCL |
| Left V-tail servo | PWM slice 1 ch A, 50 Hz | GP2 |
| Right V-tail servo | PWM slice 1 ch B, 50 Hz | GP3 |
| ESC (Oneshot125) | PWM slice 3 ch A | GP6 |
| Status LED | WL_GPIO0 (wireless chip) | no dedicated GPIO |

**Power:** ESC BEC → Pico W **VSYS**. USB is never connected while the battery/ESC is connected — config happens over the WiFi portal, not USB-serial, so the two power sources are never combined. This is an operating rule, not a hardware safeguard (no diode-ORing circuit).

**Failure behavior:** none (passive pin/power assignment; failures surface in the parts that use these pins).

Reference: `firmware/pins.py`.

---

## Control system

Resolved by: [control-system-design.md](../wayfinding/pico-wing-fc/control-system-design.md)

**Inputs:** CRSF channels (roll, pitch, yaw, throttle — normalized), IMU attitude estimate (stabilized mode only).
**Outputs:** left/right servo commands [-1, 1], ESC throttle [0, 1].
**Interfaces:** consumes RC channels from [Flight modes, arming & failsafe](#flight-modes-arming--failsafe); attitude estimate from the MPU6050 driver (Mahony @ 200 Hz, per [research-mpu6050-attitude.md](../wayfinding/pico-wing-fc/research-mpu6050-attitude.md)).

**Loop rate:** single 250 Hz flight loop — all axes, attitude estimate, mixer, and outputs run together; no separate outer/inner loop rates for the MVP.

**No roll axis/actuator:** the airframe has 2 V-tail servos and no ailerons. Roll stick does not drive a roll PID — it feeds turn coordination instead (below), grounded in how INAV itself handles airframes with no aileron output (verified against `pidController()`/`servoMixer()` in INAV `master`: INAV always computes a roll-axis PID, but it only reaches a servo if the mixer table routes it there — a no-aileron airframe's table simply doesn't, so INAV couples roll into yaw instead via its switchable "Turn Assistant" feature).

**Per-mode behavior:**
- **Manual:** raw stick passthrough — pitch/yaw feed the mixer directly, no PID. Throttle passthrough. Roll stick unused.
- **Stabilized — INAV's real cascade and gain pipeline, not an invented approximation:**
  - *Pitch:* outer level loop → inner rate loop. `angleRateTarget_dps = clamp(angleError_deg * (pitch_level_p / 6.56), ±pitch_rate_limit_dps)` (INAV `pidLevel()`), then the inner rate loop (below) drives toward that target using the Mahony pitch-rate estimate.
  - *Yaw:* rate-only, no outer loop (INAV never gives yaw an angle mode). Target rate = `rate_yaw_dps · yaw_stick + turn_assist_gain · roll_stick` (turn coordination folded straight into the yaw-rate target, always on, no separate switch) — a linear approximation of INAV's `pidTurnAssistant()` bank→yaw-rate coupling, whose real formula needs airspeed (`GRAVITY · tan(bank) / airspeed`) that this build doesn't have.
  - *Inner rate loop (pitch and yaw, identical shape):* `output = clamp(P·rateError + I·∫rateError + D·Δrate/dt + FF·rateTarget, ±500)`, then **÷500** before the mixer — INAV's own `pidSumLimit` and gain-scale constants (`kP=raw/31, kI=raw/4, kD=raw/1905, kFF=raw/31`) applied unchanged, with the raw `pid_*_p/i/d/ff` values taken directly from INAV's `settings.yaml` defaults (pitch P/I/D/FF = 5/7/0/50, yaw = 6/10/0/60). D defaults to 0 on every fixed-wing axis in INAV, not just yaw — feedforward carries most of the response instead.
  - Throttle passthrough (no altitude/speed hold).
- **Autonomous:** deferred — see [Deferred / Post-MVP](#deferred--post-mvp). The mode-switch position that would select it falls back to stabilized behavior.
- Switching modes resets both PID controllers' integrators (avoids stale windup carrying across a manual↔stabilized transition).

**V-tail mixer:** linear mix, clamped to [-1, 1], consuming the already-normalized (÷500) pitch/yaw commands from the cascade above:
```
left  = mixer_left_pitch  * pitch + mixer_left_yaw  * yaw
right = mixer_right_pitch * pitch + mixer_right_yaw * yaw
```
No differential-throw or exponential curves for the MVP.

**Failure behavior:** none independently — control outputs are overridden by the failsafe procedure in [Flight modes, arming & failsafe](#flight-modes-arming--failsafe) when the RC link is lost.

Reference: `firmware/mixer.py`, `firmware/main.py`.

---

## Flight modes, arming & failsafe

Resolved by: [flight-modes-arming-failsafe.md](../wayfinding/pico-wing-fc/flight-modes-arming-failsafe.md)

**Inputs:** CRSF arm-switch channel, mode-switch channel, throttle channel, link-alive/frame-timestamp state.
**Outputs:** armed/disarmed state, active flight mode, failsafe override of control outputs.
**Interfaces:** gates the WiFi config portal ([Web config & parameter persistence](#web-config--parameter-persistence)) and feeds mode into [Control system](#control-system).

**Arming:** TX arm-switch ON **and** throttle ≤ `arm_max_throttle_us` (default 1050 µs) → armed. Disarm: switch OFF, any throttle → disarmed, WiFi config portal comes back up.

**Mode select:** one 3-position switch channel, split into equal thirds of CRSF travel: low = manual, mid = stabilized, high = autonomous. Since autonomous nav is deferred, **the high third falls back to stabilized behavior** — no dead/undefined switch position. Mode changes take effect immediately, at any time while armed (no lockout or confirmation).

**RC-link-loss failsafe:** link considered lost after `failsafe_link_timeout_ms` (default 500 ms) of silence (no CRSF 0x16 frames) — ELRS signals loss by simply stopping transmission. On loss, in every mode: wings level (zero roll/pitch/yaw demand into the mixer) + throttle cut to 0.

**GPS-loss failsafe (autonomous):** not applicable for the MVP — see [Deferred / Post-MVP](#deferred--post-mvp).

**WiFi portal:** runs only while disarmed; torn down before the flight loop starts on arm.

Reference: `firmware/main.py`.

---

## Web config & parameter persistence

Resolved by: [web-config-persistence.md](../wayfinding/pico-wing-fc/web-config-persistence.md)

**Inputs:** HTTP form submissions (config, servo test, motor test, IMU test).
**Outputs:** persisted config (flash), live parameter updates to the flight loop, HTML pages.
**Interfaces:** gated by armed/disarmed state ([Flight modes, arming & failsafe](#flight-modes-arming--failsafe)); parameters feed [Control system](#control-system) and [Flight modes, arming & failsafe](#flight-modes-arming--failsafe).

**AP/network:** Pico W hosts a WPA2 AP, SSID = `"pico-wing-" + wifi_ssid_suffix`, password = `wifi_password` — both config values with shipped defaults, changeable via the portal. No pairing flow or per-unit randomized password for the MVP. Radio is up only while disarmed.

**Persistence:** JSON file on flash (`config.json`), atomic write via tmp-file + rename, one backup file (`config.json.bak`) restored automatically if the primary is missing/corrupt. `config.factory_reset()` deletes all three files and reverts to schema defaults, but is not exposed in the web UI (owner call) — invoke it manually (e.g. over `mpremote exec`) if needed.

**Validation:** every incoming value is clamped to its schema's [min, max] (or coerced to string); unknown keys ignored; missing keys fall back to defaults.

**Apply timing:** changes apply **live — no reboot or re-arm cycle needed**.

**Framework:** microdot + plain HTML forms, no client-side framework.

**Failure behavior:** corrupt/unreadable primary config falls back to the backup file, then to schema defaults — never a crash on boot.

Reference: `firmware/config.py`, `firmware/server.py`.

### Parameter reference

PID/FF gains below are stored as INAV's own raw `settings.yaml`-style numbers (not pre-scaled) — `firmware/main.py` divides each by INAV's real scale constants before use (`P/31, I/4, D/1905, FF/31`; level-P by `6.56`), sums P+I+D+FF, and clamps to INAV's `±500` before normalizing to this project's own `[-1, 1]` mixer convention. See [Control system](#control-system) for the exact formulas and citations.

| Parameter | Default | Range | Notes |
|---|---|---|---|
| `pid_pitch_p` / `_i` / `_d` / `_ff` | 5 / 7 / 0 / 50 | 0–255 (all) | Pitch inner rate loop — INAV `fw_p/i/d/ff_pitch` defaults exactly |
| `pid_yaw_p` / `_i` / `_d` / `_ff` | 6 / 10 / 0 / 60 | 0–255 (all) | Yaw rate loop — INAV `fw_p/i/d/ff_yaw` defaults exactly |
| `pitch_level_p` | 20 | 0–255 | Outer level loop gain (angle error → rate target) — INAV `fw_p_level` |
| `pitch_rate_limit_dps` | 200 | 10–1000 | Ceiling on the level loop's rate demand |
| `pitch_angle_max_deg` | 30 | 5–60 | Stick-to-target-angle scale: max commanded pitch angle at full stick |
| `rate_yaw_dps` | 90 | 10–500 | Yaw stick-to-rate scale |
| `turn_assist_gain` | 60 | 0–300 | Roll-stick→yaw-rate scale (turn coordination; no roll axis exists) |
| `mixer_left_pitch` / `mixer_left_yaw` | 0.5 / 0.5 | -1–1 | V-tail mixer gains, left surface |
| `mixer_right_pitch` / `mixer_right_yaw` | 0.5 / -0.5 | -1–1 | V-tail mixer gains, right surface |
| `servo_left_min_us` / `_max_us` | 1000 / 2000 | 500–2500 | Left servo pulse endpoints |
| `servo_right_min_us` / `_max_us` | 1000 / 2000 | 500–2500 | Right servo pulse endpoints |
| `channel_roll/pitch/throttle/yaw/arm/mode` | 1/2/3/4/5/6 | 1–16 | CRSF channel assignment |
| `arm_max_throttle_us` | 1050 | 950–1200 | Pre-arm throttle ceiling |
| `failsafe_link_timeout_ms` | 500 | 100–5000 | RC-loss detection window |
| `wifi_ssid_suffix` | `pico-wing` | string | AP SSID suffix |
| `wifi_password` | `picowing` | string | AP password |

(Canonical source: `firmware/config.py`'s `SCHEMA`.)

---

## Deferred / Post-MVP

These are descoped from the MVP, not dropped. Each keeps its own open wayfinding doc with assumptions already on file for when it resumes.

- **Telemetry (FC→TX):** no CRSF telemetry frames sent to the transmitter — RX (channels in) only. See [telemetry-content.md](../wayfinding/pico-wing-fc/telemetry-content.md).
- **Autonomous navigation:** no GPS driver or waypoint nav loop; the BZ-251 (and its onboard QMC5883 compass) is wired but unused. Mode-switch autonomous position falls back to stabilized in the meantime. See [autonomous-navigation.md](../wayfinding/pico-wing-fc/autonomous-navigation.md). Also unresolved until this resumes: waypoint mission entry method, heading reference (GPS course vs. the QMC5883 compass vs. both — corrected 2026-07-29, a magnetometer does exist on this hardware), altitude reference strategy (GPS-only, no baro) — tracked on the [wayfinding map](../wayfinding/pico-wing-fc.md#not-yet-specified).
