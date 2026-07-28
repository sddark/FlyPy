# Research findings: MPU6050 attitude estimation

Resolves wayfinder part `research-mpu6050-attitude`.

## Reading the MPU6050 from MicroPython

- I²C at 400 kHz; register-level driver is ~50 lines (WHO_AM_I check, wake from sleep, set gyro/accel full scale, set DLPF, burst-read 14 bytes from 0x3B).
- Existing references: `OneMadGypsy/upy-motion` (MicroPython MPU6050 driver with auto-calibration), various minimal drivers (e.g. martinfitzpatrick.com tutorial code). None needed as a dependency — bespoke is trivial.
- **DMP (on-chip fusion): avoid.** Requires large proprietary firmware blobs, poorly supported in MicroPython, and we need raw gyro for the rate loops anyway. Use raw gyro + accel.
- **Calibration:** at boot (disarmed, craft still), average ~500–1000 gyro samples → gyro offsets; accel offsets from a level-rest assumption or stored from a web-config calibration routine. Store offsets in the config file; re-zero gyro every boot (cheap insurance).

## Estimator choice (gyro + accel, no magnetometer)

- Candidates: complementary filter, Mahony, Madgwick.
- **Recommendation: Mahony (explicit complementary / PI-feedback quaternion estimator).**
  - Mahony ≈ Madgwick accuracy for the gyro+accel-only case (Madgwick's gradient step buys little without a magnetometer), but Mahony is cheaper: one quaternion integrate + PI correction per step, no normalizing gradient descent — friendlier to pure-Python RP2040.
  - A plain complementary filter is cheapest but drifts more in sustained maneuvers and has no principled gyro-bias estimation; Mahony's integral term estimates gyro bias in flight — valuable on a £4 IMU.
  - Reference implementations: `ahrs` Python package docs (https://ahrs.readthedocs.io/en/latest/filters/mahony.html — clear algorithm statement), madflight's AHRS (https://madflight.com/AHRS/ — fixed-wing-oriented discussion), numerous small MicroPython Mahony ports searchable under github.com/topics/madgwick.
- Expected cost: a Mahony update in pure MicroPython ≈ 1–2 ms worst case → fits a 100 Hz loop with huge margin (or use `micropython.native`/`viper` emitter to cut it further).
- **Yaw:** without a magnetometer, yaw from the IMU alone drifts (deg/min). Acceptable: stabilized mode needs only pitch/roll; autonomous heading comes from GPS course-over-ground (see nav part). Spec must state this explicitly.
- **Rate mode / manual:** raw gyro rates feed PIDs directly; estimator not in the path.

## Rates & filtering chain

- MPU6050 gyro output: 1 kHz max with DLPF. Target: sample at **200 Hz**, DLPF = **98 Hz** as the starting point (common drone starting point; drop to **42 Hz** if prop vibration couples through — latency vs noise trade, both values configurable).
- Chain: raw sample → (optional 2nd-order PT1 biquad on gyro, configurable cutoff) → Mahony at 200 Hz → attitude + rates to control loops at 100 Hz (decimate by 2).
- Loop rates: estimator 200 Hz, PID/control 100 Hz, outputs at 100 Hz (Oneshot125) / 50 Hz (servos). All within MicroPython budget per the platform research.

## Vibration

- Soft-mount the IMU (foam/grommets) — spec should call this out in the hardware section.
- DLPF 98→42 Hz fallback + PT1 filter cutoffs as web-config parameters covers the rest.

## Blockers / risks

- None blocking. Recommendation: **Mahony @ 200 Hz, DLPF 98 Hz, gyro zeroed each boot, accel calibration via web config.**
