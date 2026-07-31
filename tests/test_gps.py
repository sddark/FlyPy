# Off-target tests for the u-blox M10 UBX driver (framing/parsing is pure
# Python): `python3 tests/test_gps.py`. NAV-PVT payloads are fabricated with
# struct.pack_into at the documented offsets so the parse checks the real
# field map, not a round trip through the same code.

import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))

import gps


class FakeUart:
    def __init__(self):
        self._pending = b""
        self.written = []

    def queue(self, data):
        self._pending += data

    def read(self):
        data, self._pending = self._pending, b""
        return data or None

    def write(self, data):
        self.written.append(bytes(data))


def make_pvt_payload(
    lat_deg=51.5, lon_deg=-0.12, alt_m=123.0, speed_ms=15.0, course_deg=270.0,
    fix_type=3, fix_ok=True, num_sv=9, h_acc_m=2.5,
):
    payload = bytearray(gps.NAV_PVT_PAYLOAD_LEN)
    struct.pack_into("<I", payload, 0, 123456)              # iTOW
    payload[20] = fix_type
    payload[21] = 0x01 if fix_ok else 0x00                  # gnssFixOK
    payload[23] = num_sv
    struct.pack_into("<i", payload, 24, round(lon_deg * 1e7))
    struct.pack_into("<i", payload, 28, round(lat_deg * 1e7))
    struct.pack_into("<i", payload, 36, round(alt_m * 1000))  # hMSL mm
    struct.pack_into("<I", payload, 40, round(h_acc_m * 1000))
    struct.pack_into("<I", payload, 44, 3000)               # vAcc mm
    struct.pack_into("<i", payload, 60, round(speed_ms * 1000))
    struct.pack_into("<i", payload, 64, round(course_deg * 1e5))
    struct.pack_into("<I", payload, 72, 250000)             # headAcc 2.5 deg
    return bytes(payload)


def pvt_frame(**kwargs):
    return gps.build_frame(0x01, 0x07, make_pvt_payload(**kwargs))


def test_fletcher_checksum_reference():
    # UBX-ACK-ACK for CFG-VALSET: class 05 01, len 2, payload 06 8A.
    ck_a, ck_b = gps.fletcher_checksum(b"\x05\x01\x02\x00\x06\x8a")
    frame = gps.build_frame(0x05, 0x01, b"\x06\x8a")
    assert frame[-2] == ck_a and frame[-1] == ck_b
    assert frame[:2] == b"\xb5\x62"
    assert frame[2:6] == b"\x05\x01\x02\x00"


def test_parse_nav_pvt_field_map():
    pvt = gps.parse_nav_pvt(make_pvt_payload())
    assert pvt["fix_ok"]
    assert pvt["fix_type"] == 3
    assert pvt["num_sv"] == 9
    assert abs(pvt["lat_deg"] - 51.5) < 1e-6
    assert abs(pvt["lon_deg"] + 0.12) < 1e-6
    assert abs(pvt["alt_m"] - 123.0) < 1e-9
    assert abs(pvt["ground_speed_ms"] - 15.0) < 1e-9
    assert abs(pvt["course_deg"] - 270.0) < 1e-9
    assert abs(pvt["h_acc_m"] - 2.5) < 1e-9
    assert abs(pvt["course_acc_deg"] - 2.5) < 1e-9


def test_fix_ok_requires_flag_and_3d():
    assert not gps.parse_nav_pvt(make_pvt_payload(fix_ok=False))["fix_ok"]
    assert not gps.parse_nav_pvt(make_pvt_payload(fix_type=2))["fix_ok"]


def test_parser_resyncs_through_garbage_and_splits():
    parser = gps.UbxParser()
    good = pvt_frame()
    corrupt = bytearray(pvt_frame())
    corrupt[-1] ^= 0xFF

    assert parser.feed(b"\x00\xb5\x99" + good[:20]) == []
    frames = parser.feed(good[20:] + bytes(corrupt) + good)
    assert len(frames) == 2
    for msg_class, msg_id, payload in frames:
        assert (msg_class, msg_id) == (0x01, 0x07)
        assert len(payload) == gps.NAV_PVT_PAYLOAD_LEN


def test_valset_frame_layout():
    frame = gps.build_valset(((0x30210001, 200), (0x20110021, 8)))
    assert frame[:2] == b"\xb5\x62"
    assert frame[2:4] == b"\x06\x8a"
    payload_len = frame[4] | (frame[5] << 8)
    payload = frame[6 : 6 + payload_len]
    # version 0, RAM layer, reserved
    assert payload[:4] == b"\x00\x01\x00\x00"
    # key 1: u2-sized (0x3 nibble) -> 4 key bytes + 2 value bytes
    assert payload[4:8] == struct.pack("<I", 0x30210001)
    assert payload[8:10] == struct.pack("<H", 200)
    # key 2: u1-sized (0x2 nibble) -> 4 key bytes + 1 value byte
    assert payload[10:14] == struct.pack("<I", 0x20110021)
    assert payload[14] == 8
    assert payload_len == 15
    # frame passes its own parser
    parser = gps.UbxParser()
    assert len(parser.feed(frame)) == 1


def test_receiver_sends_config_then_applies_pvt():
    uart = FakeUart()
    receiver = gps.UbxGps(uart=uart)
    receiver.update(now_ms=0)
    assert len(uart.written) == 1  # boot VALSET sent on first update
    assert uart.written[0] == gps.build_valset(gps.BOOT_CONFIG)

    uart.queue(gps.build_frame(0x05, 0x01, b"\x06\x8a"))  # ACK-ACK for VALSET
    uart.queue(pvt_frame())
    receiver.update(now_ms=100)
    assert receiver.config_acknowledged
    assert receiver.fix_ok
    assert receiver.num_sv == 9
    assert abs(receiver.lat_deg - 51.5) < 1e-6
    assert receiver.last_pvt_ms == 100


def test_receiver_resends_config_when_pvt_goes_silent():
    uart = FakeUart()
    receiver = gps.UbxGps(uart=uart)
    receiver.update(now_ms=0)
    receiver.update(now_ms=1000)
    assert len(uart.written) == 1  # rate-limited: no resend inside the window

    receiver.update(now_ms=4000)
    assert len(uart.written) == 2  # still no PVT -> resent

    uart.queue(pvt_frame())
    receiver.update(now_ms=4100)
    receiver.update(now_ms=6000)
    assert len(uart.written) == 2  # PVT flowing -> no further resends


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
