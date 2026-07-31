# Off-target tests for the QMC5883 compass driver (register decode is pure
# Python; bus access goes through an injectable I2C object, same pattern as
# tests/test_imu.py): `python3 tests/test_compass.py`.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))

import compass


def pack_sample(x_raw, y_raw, z_raw):
    data = bytearray()
    for value in (x_raw, y_raw, z_raw):
        value &= 0xFFFF
        data.append(value & 0xFF)         # little-endian, unlike the MPU6050
        data.append((value >> 8) & 0xFF)
    return bytes(data)


class FakeI2C:
    def __init__(self, sample=None, status=0x01):
        self.writes = []
        self._sample = sample or pack_sample(0, 0, 0)
        self._status = status

    def writeto_mem(self, address, register, data):
        self.writes.append((address, register, bytes(data)))

    def readfrom_mem(self, address, register, nbytes):
        if register == compass._REG_STATUS:
            return bytes([self._status])
        assert register == compass._REG_DATA
        assert nbytes == compass.SAMPLE_LENGTH
        return self._sample


def test_decode_sample_scales_signed_little_endian():
    x, y, z = compass.decode_sample(pack_sample(3000, -3000, 1500))
    assert abs(x - 1.0) < 1e-9
    assert abs(y + 1.0) < 1e-9
    assert abs(z - 0.5) < 1e-9


def test_configure_writes_setreset_then_mode():
    fake = FakeI2C()
    compass.QMC5883(i2c=fake, address=0x0D)
    assert fake.writes == [
        (0x0D, compass._REG_SET_RESET, bytes([compass._SET_RESET_RECOMMENDED])),
        (0x0D, compass._REG_CONTROL_1, bytes([compass._CONTROL_1_CONFIG])),
    ]


def test_read_returns_gauss_tuple():
    fake = FakeI2C(pack_sample(1500, 0, -3000))
    sensor = compass.QMC5883(i2c=fake)
    assert sensor.read() == (0.5, 0.0, -1.0)


def test_data_ready_reads_status_bit():
    assert compass.QMC5883(i2c=FakeI2C(status=0x01)).data_ready()
    assert not compass.QMC5883(i2c=FakeI2C(status=0x04)).data_ready()


def test_flat_field_angle_quadrants():
    assert abs(compass.flat_field_angle_deg(1.0, 0.0) - 0.0) < 1e-9
    assert abs(compass.flat_field_angle_deg(0.0, 1.0) - 90.0) < 1e-9
    assert abs(compass.flat_field_angle_deg(-1.0, 0.0) - 180.0) < 1e-9
    assert abs(compass.flat_field_angle_deg(0.0, -1.0) - 270.0) < 1e-9


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
