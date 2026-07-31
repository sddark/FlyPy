# Generic PID+FF controller for the stabilized-mode pitch and yaw rate loops.
# Pure Python (no `machine` imports) so it is testable off-target -- only
# the callers feed it real sensor/dt values.
#
# Feedforward: matches INAV's fixed-wing controller (`pidApplyFixedWingRateController`
# in flight/pid.c) -- a term proportional to the rate *target* (not the error),
# summed into the same clamp as P/I/D, and excluded from the integrator. It's
# what INAV uses instead of a yaw/pitch/roll D-term for most of the response
# (all three fixed-wing D gains default to 0 upstream); D handles residual
# correction, FF drives the bulk of the response to a stick input directly.
#
# Integrator anti-windup: the integral term is clamped directly (not the
# integrator's accumulation), and only accumulates while the output isn't
# already saturated in the same direction as the new error -- simple
# conditional-integration clamping.


class PID:
    def __init__(self, kp, ki, kd, kff=0.0, output_limit=1.0, integral_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff = kff
        self.output_limit = output_limit
        self.integral_limit = (
            integral_limit if integral_limit is not None else output_limit
        )
        self._integral = 0.0
        self._previous_error = 0.0
        self._has_previous = False

    def set_gains(self, kp, ki, kd, kff):
        # Live retune (user logic can drive the gains). Deliberately does NOT
        # reset the integrator: a gain change mid-flight should not dump the
        # accumulated trim the aircraft is currently relying on.
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kff = kff

    def reset(self):
        self._integral = 0.0
        self._previous_error = 0.0
        self._has_previous = False

    def update(self, error, dt, feedforward_input=0.0):
        derivative = 0.0
        if self._has_previous and dt > 0:
            derivative = (error - self._previous_error) / dt
        self._previous_error = error
        self._has_previous = True

        candidate_integral = self._integral + error * dt
        candidate_integral = min(
            max(candidate_integral, -self.integral_limit), self.integral_limit
        )
        unclamped = (
            self.kp * error
            + self.ki * candidate_integral
            + self.kd * derivative
            + self.kff * feedforward_input
        )
        output = min(max(unclamped, -self.output_limit), self.output_limit)

        # Only commit the integrator step if doing so didn't just get clamped
        # away, or if it's un-winding (error and integral disagree in sign).
        if output == unclamped or error * self._integral <= 0:
            self._integral = candidate_integral

        return output
