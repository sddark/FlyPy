# Off-target tests for the CRSF driver (rc.py parses in pure Python, so this
# runs under CPython: `python3 tests/test_rc.py`). Frames are built with an
# independent bit-packer so the round trip actually checks the unpacking.

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "firmware"))

import config as config_module
import rc

_ADDRESS_FLIGHT_CONTROLLER = 0xC8


def pack_channels(values):
    packed = 0
    for index, value in enumerate(values):
        packed |= (value & 0x7FF) << (11 * index)
    return packed.to_bytes(22, "little")


def build_frame(frame_type, payload):
    body = bytes([frame_type]) + payload
    return (
        bytes([_ADDRESS_FLIGHT_CONTROLLER, len(body) + 1]) + body + bytes([rc.crc8(body)])
    )


def channels_frame(values):
    return build_frame(rc.FRAME_TYPE_RC_CHANNELS, pack_channels(values))


class FakeUart:
    def __init__(self):
        self._pending = b""

    def queue(self, data):
        self._pending += data

    def read(self):
        data, self._pending = self._pending, b""
        return data or None


def test_crc8_check_value():
    # Standard CRC-8/DVB-S2 check value.
    assert rc.crc8(b"123456789") == 0xBC


def test_unpack_round_trip():
    values = [172, 992, 1811, 0, 2047, 1024, 500, 1500] + list(range(100, 900, 100))
    assert rc.unpack_channels(pack_channels(values)) == values


def test_oversized_read_drops_stale_frames_and_keeps_the_newest():
    # The allocation that failed on the board was extend()ing a read
    # approaching the UART's 2 KB rxbuf onto the buffer: one contiguous
    # block that a fragmented heap could not supply, with 40 KB still free
    # overall. The read is now truncated to its tail first.
    #
    # Asserted through behaviour rather than by measuring the buffer,
    # because the buffer is small again by the time feed() returns either
    # way -- the resync scan consumes unmatched garbage. What actually
    # differs is WHICH frames survive: a frame thousands of bytes back is
    # hundreds of milliseconds stale and must not be reported as current.
    parser = rc.CrsfParser()
    stale = channels_frame([rc._CRSF_LOW] * 16)
    fresh = channels_frame([rc._CRSF_HIGH] * 16)
    frames = parser.feed(stale + b"\x11" * 4096 + fresh)
    assert len(frames) == 1, (
        "expected only the newest frame, got %d" % len(frames))
    assert rc.unpack_channels(frames[0][1])[0] == rc._CRSF_HIGH


def test_accumulated_buffer_is_capped():
    # The other growth path: many reads that each leave an incomplete frame
    # behind, so the scan cannot consume them and the buffer accumulates.
    # A truncated frame header claiming a length that never arrives is
    # exactly what a stalled loop leaves at the tail of every read.
    parser = rc.CrsfParser()
    partial = bytes([rc.SYNC_ADDRESS, 60]) + b"\x11" * 10
    for _ in range(60):
        parser.feed(partial)
    assert len(parser._buffer) <= rc._MAX_BUFFER_BYTES, (
        "buffer accumulated to %d" % len(parser._buffer))


def test_parser_handles_garbage_split_and_corruption():
    parser = rc.CrsfParser()
    good = channels_frame([992] * 16)
    corrupt = bytearray(channels_frame([1500] * 16))
    corrupt[-1] ^= 0xFF

    assert parser.feed(b"\xff\x00\x9a" + good[:10]) == []
    frames = parser.feed(good[10:] + bytes(corrupt) + good)
    assert len(frames) == 2
    for frame_type, payload in frames:
        assert frame_type == rc.FRAME_TYPE_RC_CHANNELS
        assert rc.unpack_channels(payload) == [992] * 16


def test_receiver_normalizes_channels():
    uart = FakeUart()
    receiver = rc.CrsfReceiver(config_module.defaults(), uart=uart)
    assert not receiver.link_alive

    # roll low, pitch center, throttle high, yaw center, arm high, mode center
    values = [172, 992, 1811, 992, 1811, 992] + [992] * 10
    uart.queue(channels_frame(values))
    receiver.update(now_ms=1234)

    assert receiver.link_alive
    assert receiver.last_frame_ms == 1234
    assert receiver.raw_channels == values
    assert abs(receiver.channels["roll"] + 1.0) < 1e-6
    assert abs(receiver.channels["pitch"]) < 1e-6
    assert abs(receiver.channels["yaw"]) < 1e-6
    assert abs(receiver.channels["throttle"] - 1.0) < 1e-6
    assert receiver.arm_switch_on
    assert receiver.mode == "stabilized"


def test_receiver_mode_thirds_and_disarm():
    uart = FakeUart()
    receiver = rc.CrsfReceiver(config_module.defaults(), uart=uart)
    for mode_value, expected in ((172, "manual"), (992, "stabilized"), (1811, "autonomous")):
        uart.queue(channels_frame([992, 992, 172, 992, 172, mode_value] + [992] * 10))
        receiver.update(now_ms=0)
        assert receiver.mode == expected
        assert not receiver.arm_switch_on
        assert abs(receiver.channels["throttle"]) < 1e-6


def test_channel_to_us_standard_mapping():
    # 0.625 us per tick, center 992 <-> 1500 us.
    assert abs(rc.channel_to_us(992) - 1500.0) < 1e-9
    assert abs(rc.channel_to_us(172) - 987.5) < 1e-9
    assert abs(rc.channel_to_us(1811) - 2011.875) < 1e-9


def test_receiver_reports_throttle_us():
    uart = FakeUart()
    receiver = rc.CrsfReceiver(config_module.defaults(), uart=uart)
    # throttle raw 272 -> 1500 + (272 - 992) * 0.625 = 1050 us
    uart.queue(channels_frame([992, 992, 272, 992, 172, 992] + [992] * 10))
    receiver.update(now_ms=0)
    assert abs(receiver.throttle_us - 1050.0) < 1e-9


def test_flush_discards_backlog_and_partial_frames():
    uart = FakeUart()
    receiver = rc.CrsfReceiver(config_module.defaults(), uart=uart)
    frame = channels_frame([992] * 16)
    uart.queue(frame[:10])
    receiver.update(now_ms=0)  # partial frame left buffered in the parser
    uart.queue(frame[10:] + frame)

    receiver.flush()
    receiver.update(now_ms=5)
    assert not receiver.link_alive  # backlog was dropped, nothing parsed

    uart.queue(frame)
    receiver.update(now_ms=6)
    assert receiver.link_alive  # fresh frames still parse after a flush


def test_arm_switch_must_be_seen_off_before_counting():
    uart = FakeUart()
    receiver = rc.CrsfReceiver(config_module.defaults(), uart=uart)
    assert not receiver.arm_switch_seen_off

    # Switch latched ON from the very first frame (e.g. left on from a
    # previous session): reported on, but the seen-off gate stays closed.
    uart.queue(channels_frame([992, 992, 172, 992, 1811, 992] + [992] * 10))
    receiver.update(now_ms=0)
    assert receiver.arm_switch_on
    assert not receiver.arm_switch_seen_off

    uart.queue(channels_frame([992, 992, 172, 992, 172, 992] + [992] * 10))
    receiver.update(now_ms=1)
    assert not receiver.arm_switch_on
    assert receiver.arm_switch_seen_off

    # Gate stays open once satisfied.
    uart.queue(channels_frame([992, 992, 172, 992, 1811, 992] + [992] * 10))
    receiver.update(now_ms=2)
    assert receiver.arm_switch_on
    assert receiver.arm_switch_seen_off


def test_receiver_ignores_empty_uart():
    uart = FakeUart()
    receiver = rc.CrsfReceiver(config_module.defaults(), uart=uart)
    receiver.update(now_ms=99)
    assert not receiver.link_alive
    assert receiver.last_frame_ms == 0


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
