# V-tail mixer: pitch and yaw commands in [-1, 1] become left/right servo
# positions in [-1, 1]. Commands come from manual passthrough or the
# stabilized cascade in main.py; the failsafe rule (stabilized level wings +
# cut throttle) applies in every mode.


class VTailMixer:
    def __init__(self, config):
        self.set_gains(config)

    def set_gains(self, config):
        self._left_pitch = config["mixer_left_pitch"]
        self._left_yaw = config["mixer_left_yaw"]
        self._right_pitch = config["mixer_right_pitch"]
        self._right_yaw = config["mixer_right_yaw"]

    def mix(self, pitch, yaw):
        left = self._left_pitch * pitch + self._left_yaw * yaw
        right = self._right_pitch * pitch + self._right_yaw * yaw
        return _clamp(left), _clamp(right)


def _clamp(value):
    return min(max(value, -1.0), 1.0)
