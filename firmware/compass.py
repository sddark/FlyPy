# QMC5883 magnetometer -- the compass on the BZ-251 GPS module's I2C pads
# (see the 2026-07-29 correction in "Research: BZ-251 UBX configuration":
# the earlier no-magnetometer assumption was wrong). Shares I2C0 with the
# MPU6050 (fixed address 0x0D vs the IMU's 0x68).
#
# Driver only: reports the raw field vector in gauss, plus a flat-plane
# angle helper. How (and whether) this feeds the heading estimate --
# tilt compensation via the attitude quaternion, hard/soft-iron
# calibration, declination -- is the deferred autonomous-navigation part's
# question, not this driver's.
#
# Register decode is pure Python, testable off-target with a fabricated
# I2C read; only QMC5883 touches the bus (injectable i2c object, same
# pattern as imu.MPU6050).

from math import atan2, degrees

_DEFAULT_ADDRESS = 0x0D

_REG_DATA = 0x00        # x lsb/msb, y lsb/msb, z lsb/msb -- little-endian
_REG_STATUS = 0x06
_REG_CONTROL_1 = 0x09
_REG_SET_RESET = 0x0B

_STATUS_DATA_READY = 0x01

# INAV/Betaflight configuration: OSR=512, range +/-8 G, 200 Hz ODR,
# continuous mode -- 3000 LSB/gauss at this range.
_CONTROL_1_CONFIG = 0x1D
_SET_RESET_RECOMMENDED = 0x01  # datasheet-mandated value

_LSB_PER_GAUSS = 3000.0

SAMPLE_LENGTH = 6


def decode_sample(data):
    # data: 6 bytes from _REG_DATA. Returns (x, y, z) in gauss, sensor
    # frame, no calibration applied.
    values = []
    for axis in range(3):
        raw = data[2 * axis] | (data[2 * axis + 1] << 8)
        if raw & 0x8000:
            raw -= 65536
        values.append(raw / _LSB_PER_GAUSS)
    return tuple(values)


def flat_field_angle_deg(x, y):
    # Planar angle of the field in the sensor frame, degrees [0, 360).
    # Only a true magnetic heading when the board is level and the sensor
    # axes align with the airframe; mounting, tilt compensation, and
    # declination are integration-time concerns (autonomous-navigation
    # part). Useful on the bench as a "compass responds and rotates" check.
    return degrees(atan2(y, x)) % 360.0


class QMC5883:
    def __init__(self, i2c=None, address=_DEFAULT_ADDRESS):
        if i2c is None:
            i2c = _open_i2c()
        self._i2c = i2c
        self._address = address
        self._configure()

    def _write(self, register, value):
        self._i2c.writeto_mem(self._address, register, bytes([value]))

    def _configure(self):
        self._write(_REG_SET_RESET, _SET_RESET_RECOMMENDED)
        self._write(_REG_CONTROL_1, _CONTROL_1_CONFIG)

    def data_ready(self):
        status = self._i2c.readfrom_mem(self._address, _REG_STATUS, 1)[0]
        return bool(status & _STATUS_DATA_READY)

    def read(self):
        # Returns (x, y, z) gauss. At 200 Hz ODR a stale-by-5-ms sample is
        # fine for any caller here, so no data_ready gate on the read path.
        data = self._i2c.readfrom_mem(self._address, _REG_DATA, SAMPLE_LENGTH)
        return decode_sample(data)


def _open_i2c():
    import pins
    from machine import I2C, Pin

    # Same bus as the IMU -- the QMC5883 sits on the BZ-251's I2C pads,
    # wired to I2C0 alongside the MPU6050.
    return I2C(pins.IMU_I2C_ID, sda=Pin(pins.IMU_SDA_PIN), scl=Pin(pins.IMU_SCL_PIN))
