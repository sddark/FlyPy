# MPU6050 driver (I2C, raw registers -- no DMP), per "Research: MPU6050
# attitude estimation": ~250 Hz sample rate, DLPF ~94 Hz, gyro zeroed at
# boot, no magnetometer. Bespoke (no INAV counterpart to transpile).
#
# Register decode/bias math is pure Python so it's testable off-target with
# a fabricated I2C burst; only MPU6050.read()/calibrate() touch the bus,
# via an injectable i2c object (readfrom_mem/writeto_mem), same pattern as
# rc.CrsfReceiver's injectable UART.

import time
from math import radians

# MicroPython's `time` has sleep_ms; CPython's doesn't. Falling back to a
# no-op under CPython keeps calibrate_gyro() testable off-target with a fake
# I2C bus -- on real hardware the real delay is used.
_sleep_ms = getattr(time, "sleep_ms", lambda _ms: None)

_DEFAULT_ADDRESS = 0x68

_REG_PWR_MGMT_1 = 0x6B
_REG_SMPLRT_DIV = 0x19
_REG_CONFIG = 0x1A
_REG_GYRO_CONFIG = 0x1B
_REG_ACCEL_CONFIG = 0x1C
_REG_ACCEL_XOUT_H = 0x3B

_BURST_LENGTH = 14  # accel (6) + temp (2) + gyro (6)

_SAMPLE_RATE_HZ = 250
_SMPLRT_DIV = 1000 // _SAMPLE_RATE_HZ - 1  # internal rate is 1 kHz with DLPF on
_DLPF_CFG_94HZ = 2
_GYRO_FS_SEL_500DPS = 0x08  # FS_SEL=1 -> 65.5 LSB/(deg/s)
_ACCEL_AFS_SEL_4G = 0x08  # AFS_SEL=1 -> 8192 LSB/g

_GYRO_LSB_PER_DPS = 65.5
_ACCEL_LSB_PER_G = 8192.0


def _signed16(high_byte, low_byte):
    value = (high_byte << 8) | low_byte
    return value - 65536 if value & 0x8000 else value


def decode_burst(data):
    # data: 14 bytes from ACCEL_XOUT_H. Returns (accel_g, gyro_dps), each a
    # 3-tuple, in physical units -- no bias correction applied here.
    ax = _signed16(data[0], data[1]) / _ACCEL_LSB_PER_G
    ay = _signed16(data[2], data[3]) / _ACCEL_LSB_PER_G
    az = _signed16(data[4], data[5]) / _ACCEL_LSB_PER_G
    # bytes 6-7 are temperature; skipped.
    gx = _signed16(data[8], data[9]) / _GYRO_LSB_PER_DPS
    gy = _signed16(data[10], data[11]) / _GYRO_LSB_PER_DPS
    gz = _signed16(data[12], data[13]) / _GYRO_LSB_PER_DPS
    return (ax, ay, az), (gx, gy, gz)


def average_axes(samples):
    # samples: list of 3-tuples -> per-axis mean 3-tuple. Used for gyro bias
    # calibration (averaging over the plane at rest at boot).
    count = len(samples)
    sums = [0.0, 0.0, 0.0]
    for sample in samples:
        for axis in range(3):
            sums[axis] += sample[axis]
    return tuple(total / count for total in sums)


class MPU6050:
    def __init__(self, i2c=None, address=_DEFAULT_ADDRESS):
        if i2c is None:
            i2c = _open_i2c()
        self._i2c = i2c
        self._address = address
        self._gyro_bias_dps = (0.0, 0.0, 0.0)
        self._configure()

    def _write(self, register, value):
        self._i2c.writeto_mem(self._address, register, bytes([value]))

    def _configure(self):
        self._write(_REG_PWR_MGMT_1, 0x01)  # wake, PLL with X-gyro reference
        self._write(_REG_SMPLRT_DIV, _SMPLRT_DIV)
        self._write(_REG_CONFIG, _DLPF_CFG_94HZ)
        self._write(_REG_GYRO_CONFIG, _GYRO_FS_SEL_500DPS)
        self._write(_REG_ACCEL_CONFIG, _ACCEL_AFS_SEL_4G)

    def _read_burst(self):
        data = self._i2c.readfrom_mem(self._address, _REG_ACCEL_XOUT_H, _BURST_LENGTH)
        return decode_burst(data)

    def calibrate_gyro(self, sample_count=200, delay_ms=2):
        # Plane must be stationary. Called once at boot (see main.py); no
        # accelerometer calibration for the MVP (per research: web-config
        # calibration is a later addition).
        samples = []
        for _ in range(sample_count):
            _, gyro_dps = self._read_burst()
            samples.append(gyro_dps)
            _sleep_ms(delay_ms)
        self._gyro_bias_dps = average_axes(samples)

    def read(self):
        # Returns (accel_g, gyro_rad_s) -- gyro bias-corrected and converted
        # to rad/s, ready for attitude.MahonyFilter.update(). Unrolled (no
        # generator/comprehension) -- this runs every flight-loop iteration
        # and the generator machinery is avoidable allocation on RP2040.
        accel_g, gyro_dps = self._read_burst()
        bias = self._gyro_bias_dps
        gyro_rad_s = (
            radians(gyro_dps[0] - bias[0]),
            radians(gyro_dps[1] - bias[1]),
            radians(gyro_dps[2] - bias[2]),
        )
        return accel_g, gyro_rad_s


def _open_i2c():
    import pins
    from machine import I2C, Pin

    return I2C(pins.IMU_I2C_ID, sda=Pin(pins.IMU_SDA_PIN), scl=Pin(pins.IMU_SCL_PIN))
