# Research: MPU6050 attitude estimation

Type: research (AFK)

## Status

resolved 2026-07-28

## Question

What attitude-estimation approach should the spec choose for the MPU6050 on MicroPython? Cover: (a) reading the MPU6050 over I²C in MicroPython (existing drivers, sample rates, DMP-vs-raw on this chip, calibration/offset routines); (b) filter choices for fixed-wing attitude from gyro+accel (no magnetometer): complementary filter vs Madgwick/Mahony — computational cost on RP2040 MicroPython, code size, tuning surface; (c) gyro-only rate mode vs attitude mode requirements; (d) vibration/noise mitigation relevant to a prop airframe; (e) known MicroPython implementations to reference (cite + license). Output: findings doc recommending one estimator with justification and expected achievable loop rate.

## Assumptions

- MPU6050 on I²C; no magnetometer ever joins this build (yaw will be gyro-integrated + GPS course, handled by the navigation part).
- Control loop target 50–100 Hz.

## Decision

Recommended: **Mahony** quaternion estimator at 200 Hz (cheaper than Madgwick, estimates gyro bias in flight), raw I²C driver (no DMP), DLPF 98 Hz start with 42 Hz fallback, gyro zeroed each boot, accel calibration via web config. Yaw drifts without magnetometer — pitch/roll from IMU, heading from GPS course. Full findings: [docs/research/mpu6050-attitude.md](../../research/mpu6050-attitude.md)
