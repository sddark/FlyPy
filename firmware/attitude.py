# Mahony attitude estimator (gyro + accelerometer, no magnetometer -- yaw is
# not observable and isn't estimated; heading comes from GPS course when
# autonomous nav resumes). Reference: Mahony/Madgwick's open-source AHRS
# algorithm (x-io Technologies, MIT-licensed), per "Research: MPU6050
# attitude estimation" (Mahony recommended over Madgwick: cheaper, estimates
# gyro bias in flight via the integral feedback term below).
#
# Pure Python (no `machine` imports) so it is testable off-target with
# synthetic gyro/accel samples -- only the IMU driver touches I2C.
#
# This was briefly restructured and compiled with micropython.viper, then
# reverted deliberately. The whole update costs ~1.5 ms and the servos are
# driven at 50 Hz (servos.py), so the loop already computes several surface
# positions for every one the hardware can emit -- the optimisation bought
# headroom that is not needed, in exchange for a duplicated float-maths
# implementation that had to be kept in step by hand. Measurements are in
# the git history if the budget ever does get tight.

from math import asin, atan2, degrees, sqrt

_DEFAULT_KP = 2.0  # proportional gain on the gravity-vector error
_DEFAULT_KI = 0.05  # integral gain -- corrects gyro bias over time

# Only trust the accelerometer as a gravity reference when its magnitude is
# near 1 g. Outside this window (coordinated turns, throttle changes,
# vibration spikes) the measured vector is gravity + real acceleration and
# would drag the attitude toward a false "down"; coast on the gyro instead.
_ACCEL_NORM_MIN_G = 0.5
_ACCEL_NORM_MAX_G = 1.5


class MahonyFilter:
    def __init__(self, kp=_DEFAULT_KP, ki=_DEFAULT_KI):
        self._kp = kp
        self._ki = ki
        self._q0, self._q1, self._q2, self._q3 = 1.0, 0.0, 0.0, 0.0
        self._integral_fb = [0.0, 0.0, 0.0]

    def reset(self):
        self._q0, self._q1, self._q2, self._q3 = 1.0, 0.0, 0.0, 0.0
        self._integral_fb = [0.0, 0.0, 0.0]

    def update(self, gyro_rad_s, accel_g, dt):
        gx, gy, gz = gyro_rad_s
        ax, ay, az = accel_g
        q0, q1, q2, q3 = self._q0, self._q1, self._q2, self._q3

        accel_norm = sqrt(ax * ax + ay * ay + az * az)
        if _ACCEL_NORM_MIN_G < accel_norm < _ACCEL_NORM_MAX_G:
            ax, ay, az = ax / accel_norm, ay / accel_norm, az / accel_norm

            # Estimated direction of gravity from the current quaternion.
            half_vx = q1 * q3 - q0 * q2
            half_vy = q0 * q1 + q2 * q3
            half_vz = q0 * q0 - 0.5 + q3 * q3

            # Error = cross product between measured and estimated gravity.
            half_ex = ay * half_vz - az * half_vy
            half_ey = az * half_vx - ax * half_vz
            half_ez = ax * half_vy - ay * half_vx

            if self._ki > 0.0:
                self._integral_fb[0] += self._ki * half_ex * dt
                self._integral_fb[1] += self._ki * half_ey * dt
                self._integral_fb[2] += self._ki * half_ez * dt
                gx += self._integral_fb[0]
                gy += self._integral_fb[1]
                gz += self._integral_fb[2]

            gx += self._kp * half_ex
            gy += self._kp * half_ey
            gz += self._kp * half_ez

        # Integrate the quaternion rate.
        gx *= 0.5 * dt
        gy *= 0.5 * dt
        gz *= 0.5 * dt
        qa, qb, qc = q0, q1, q2
        q0 += -qb * gx - qc * gy - q3 * gz
        q1 += qa * gx + qc * gz - q3 * gy
        q2 += qa * gy - qb * gz + q3 * gx
        q3 += qa * gz + qb * gy - qc * gx

        norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self._q0, self._q1, self._q2, self._q3 = (
            q0 / norm,
            q1 / norm,
            q2 / norm,
            q3 / norm,
        )

    @property
    def roll_deg(self):
        q0, q1, q2, q3 = self._q0, self._q1, self._q2, self._q3
        return degrees(atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)))

    @property
    def pitch_deg(self):
        q0, q1, q2, q3 = self._q0, self._q1, self._q2, self._q3
        sin_pitch = min(max(2.0 * (q0 * q2 - q3 * q1), -1.0), 1.0)
        return degrees(asin(sin_pitch))
