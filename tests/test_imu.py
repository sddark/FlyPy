# Off-target tests for the MPU6050 driver (register decode/bias math is
# pure Python; hardware access goes through an injectable I2C object, same
# pattern as rc.CrsfReceiver's injectable UART): `python3 tests/test_imu.py`.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))

import imu


def pack_burst(ax_raw, ay_raw, az_raw, temp_raw, gx_raw, gy_raw, gz_raw):
    data = bytearray()
    for value in (ax_raw, ay_raw, az_raw, temp_raw, gx_raw, gy_raw, gz_raw):
        value &= 0xFFFF
        data.append((value >> 8) & 0xFF)
        data.append(value & 0xFF)
    return bytes(data)


class FakeI2C:
    def __init__(self, burst=None):
        self.writes = []
        self._burst = burst or pack_burst(0, 0, 0, 0, 0, 0, 0)

    def set_burst(self, burst):
        self._burst = burst

    def writeto_mem(self, address, register, data):
        self.writes.append((address, register, bytes(data)))

    def readfrom_mem(self, address, register, nbytes):
        assert register == imu._REG_ACCEL_XOUT_H
        assert nbytes == imu._BURST_LENGTH
        return self._burst


def test_decode_burst_scales_signed_values():
    burst = pack_burst(8192, 0, 4096, 0, 655, -655, 131)
    accel_g, gyro_dps = imu.decode_burst(burst)
    assert accel_g == (1.0, 0.0, 0.5)
    assert gyro_dps == (10.0, -10.0, 2.0)


def test_average_axes():
    result = imu.average_axes([(1.0, 2.0, 3.0), (3.0, 4.0, 5.0)])
    assert result == (2.0, 3.0, 4.0)


def test_configure_writes_expected_registers():
    fake = FakeI2C()
    imu.MPU6050(i2c=fake, address=0x68)
    assert fake.writes == [
        (0x68, imu._REG_PWR_MGMT_1, bytes([0x01])),
        (0x68, imu._REG_SMPLRT_DIV, bytes([imu._SMPLRT_DIV])),
        (0x68, imu._REG_CONFIG, bytes([imu._DLPF_CFG_94HZ])),
        (0x68, imu._REG_GYRO_CONFIG, bytes([imu._GYRO_FS_SEL_500DPS])),
        (0x68, imu._REG_ACCEL_CONFIG, bytes([imu._ACCEL_AFS_SEL_4G])),
    ]


def test_calibrate_gyro_removes_constant_bias():
    fake = FakeI2C(pack_burst(0, 0, 8192, 0, 655, 0, 0))  # level, gx bias = 10 dps
    sensor = imu.MPU6050(i2c=fake)
    sensor.calibrate_gyro(sample_count=5, delay_ms=0)

    accel_g, gyro_rad_s = sensor.read()
    assert accel_g == (0.0, 0.0, 1.0)
    assert all(abs(component) < 1e-9 for component in gyro_rad_s)


if __name__ == "__main__":
    failures = 0
    for name, test in sorted(globals().items()):
        if name.startswith("test_") and callable(test):
            try:
                test()
                print("ok   " + name)
            except AssertionError:
                failures += 1
                print("FAIL " + name)
    sys.exit(1 if failures else 0)
