# CRSF (ELRS) receiver driver. Transport: UART at 420 000 baud, 8N1; frames
# are [addr][len][type][payload...][crc8] where len covers type+payload+crc
# and the CRC is CRC-8/DVB-S2 over type+payload. RC channels arrive as type
# 0x16 frames: 16 x 11-bit values, little-endian bit-packed. ELRS failsafe is
# "no pulses" -- on link loss the receiver simply stops sending 0x16 frames --
# so the driver only stamps last_frame_ms and the main loop applies its
# timeout. Telemetry (FC -> TX) lands with the telemetry part.
#
# Frame parsing is pure Python (no `machine` imports) so it is testable
# off-target; only CrsfReceiver touches the UART. Channels are normalized the
# way the rest of the firmware expects: roll/pitch/yaw in [-1, 1], throttle
# in [0, 1].

CRSF_BAUDRATE = 420_000

SYNC_ADDRESS = 0xC8  # frames from the receiver to the FC carry this sync byte
FRAME_TYPE_RC_CHANNELS = 0x16
_MIN_FRAME_BYTES = 4    # address + length + type + crc, empty payload
_MAX_LENGTH_BYTE = 62   # spec maximum for the length byte
_RC_PAYLOAD_BYTES = 22  # 16 channels x 11 bits

_CHANNEL_COUNT = 16
_CHANNEL_MASK = 0x7FF

# Ceiling on the parser's accumulation buffer. ELRS streams ~3.1 KB/s here
# (120 frames/s of ~26 bytes), so a 100 ms disarmed poll brings ~310 bytes
# and an armed pass far less -- 1 KB is never reached in normal operation.
# It is reached when the loop stalls, and then bytearray.extend() has to
# reallocate a buffer approaching the UART's full 2 KB rxbuf. On a heap this
# fragmented that fails: MemoryError allocating 1864 bytes with 40 KB still
# free overall, because free memory and the largest contiguous block are not
# the same number and MicroPython's GC never compacts.
#
# Discarding the oldest bytes is right rather than merely expedient. A
# backlog this size means nothing has read the link for hundreds of
# milliseconds, and stale stick positions have no value -- only the newest
# frame does. It is the same reasoning flush() already uses after gyro
# calibration.
_MAX_BUFFER_BYTES = 1024

# CRSF channel values (protocol units, not microseconds).
_CRSF_LOW = 172
_CRSF_CENTER = 992
_CRSF_HIGH = 1811
_FULL_SPAN = _CRSF_HIGH - _CRSF_LOW
_HALF_SPAN = _FULL_SPAN / 2

# CRSF ticks -> microseconds: 0.625 us per tick, center 992 <-> 1500 us
# (the standard ELRS/Betaflight mapping). Used for the arming throttle
# threshold, which the config exposes in microseconds.
_US_PER_TICK = 5 / 8
_US_CENTER = 1500.0

# Mode switch thirds: low = manual, middle = stabilized, high = autonomous.
_MODE_LOW_MAX = _CRSF_LOW + _FULL_SPAN // 3
_MODE_MID_MAX = _CRSF_LOW + 2 * _FULL_SPAN // 3

_MAPPED_CHANNELS = ("roll", "pitch", "yaw", "throttle", "arm", "mode")


def _build_crc8_table():
    table = bytearray(256)
    for byte in range(256):
        crc = byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5 if crc & 0x80 else crc << 1) & 0xFF
        table[byte] = crc
    return bytes(table)


_CRC8_TABLE = _build_crc8_table()


def crc8(data):
    crc = 0
    for byte in data:
        crc = _CRC8_TABLE[crc ^ byte]
    return crc


def channel_to_us(raw_value):
    return _US_CENTER + (raw_value - _CRSF_CENTER) * _US_PER_TICK


def unpack_channels(payload):
    # Byte-wise shift register instead of int.from_bytes: the packed value
    # never exceeds 18 bits, so this stays in small-int territory --
    # int.from_bytes over 22 bytes makes a 176-bit bignum and 16 bignum
    # shifts per frame, real allocation/CPU cost at 250 frames/s on RP2040.
    channels = []
    bits = 0
    bit_count = 0
    for byte in payload:
        bits |= byte << bit_count
        bit_count += 8
        while bit_count >= 11:
            channels.append(bits & _CHANNEL_MASK)
            bits >>= 11
            bit_count -= 11
    return channels


class CrsfParser:
    # Accumulates raw UART bytes and yields complete CRC-valid frames.
    # Resynchronizes after garbage by discarding one byte at a time until the
    # 0xC8 sync byte, a plausible length, and a valid CRC line up.

    def __init__(self):
        self._buffer = bytearray()

    def _trim_for(self, incoming):
        # Make room so buffer + incoming never exceeds _MAX_BUFFER_BYTES,
        # dropping from the FRONT because the newest bytes are the ones worth
        # parsing. Dropping mid-frame is fine: the scan in feed() resyncs on
        # the next sync byte, which is what it does after any garbage.
        buffer = self._buffer
        overflow = len(buffer) + incoming - _MAX_BUFFER_BYTES
        if overflow <= 0:
            return
        if overflow >= len(buffer):
            # A single read big enough to blow the budget on its own; keep
            # nothing, and feed() will take the tail of the new data.
            self._buffer = bytearray()
        else:
            self._buffer = bytearray(memoryview(buffer)[overflow:])

    def feed(self, data):
        # Index-based scan, O(n): no `del buffer[...]` (MicroPython bytearray
        # doesn't support deletion) and no per-iteration slice copies (a
        # `buffer = buffer[1:]` resync loop is O(n^2) -- at ELRS's 250
        # frames/s that visibly starved the asyncio web server on the
        # RP2040). memoryview slices are zero-copy views; the only copies
        # made are each frame's payload and the final sub-frame tail.
        if data:
            if len(data) > _MAX_BUFFER_BYTES:
                # One read larger than the whole budget -- the UART's 2 KB
                # rxbuf drained after a long stall. Everything but the tail
                # is stale by definition, so nothing kept is worth keeping.
                data = memoryview(data)[-_MAX_BUFFER_BYTES:]
                self._buffer = bytearray()
            else:
                self._trim_for(len(data))
            self._buffer.extend(data)
        buffer = self._buffer
        view = memoryview(buffer)
        end = len(buffer)
        start = 0
        frames = []
        while end - start >= _MIN_FRAME_BYTES:
            if buffer[start] != SYNC_ADDRESS:
                start += 1
                continue
            length = buffer[start + 1]
            if length < 2 or length > _MAX_LENGTH_BYTE:
                start += 1
                continue
            frame_end = start + 2 + length
            if end < frame_end:
                break
            if crc8(view[start + 2 : frame_end - 1]) != buffer[frame_end - 1]:
                start += 1
                continue
            frames.append((buffer[start + 2], bytes(view[start + 3 : frame_end - 1])))
            start = frame_end
        if start:
            self._buffer = bytearray(view[start:])
        return frames


class CrsfReceiver:
    def __init__(self, config, uart=None):
        if uart is None:
            uart = _open_uart()
        self._uart = uart
        self._parser = CrsfParser()
        self._channel_index = {}
        self.set_channel_map(config)
        self.channels = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "throttle": 0.0}
        self.raw_channels = [0] * _CHANNEL_COUNT
        self.arm_switch_on = False
        self.mode = "manual"
        self.link_alive = False
        self.last_frame_ms = 0
        self.throttle_us = 0.0
        # Arming gate: the switch must be seen OFF at least once before an
        # ON counts (INAV-style). A switch left latched from a previous
        # session must not arm the plane the instant it boots -- that tears
        # the portal down before it ever appears.
        self.arm_switch_seen_off = False

    def set_channel_map(self, config):
        self._channel_index = {
            name: int(config["channel_" + name]) - 1 for name in _MAPPED_CHANNELS
        }

    def flush(self):
        # Discard buffered UART bytes and any partial frame. Used after a
        # blocking pause (gyro calibration) during which ELRS at ~6.5 KB/s
        # overran the rx buffer -- the backlog is truncated/corrupt anyway,
        # so start the flight loop clean instead of resyncing through it.
        self._uart.read()
        self._parser = CrsfParser()

    def update(self, now_ms):
        data = self._uart.read()
        if not data:
            return
        for frame_type, payload in self._parser.feed(data):
            is_channels = (
                frame_type == FRAME_TYPE_RC_CHANNELS
                and len(payload) == _RC_PAYLOAD_BYTES
            )
            if is_channels:
                self._apply_channels(unpack_channels(payload), now_ms)

    def _apply_channels(self, raw_channels, now_ms):
        self.raw_channels = raw_channels
        index = self._channel_index
        self.channels["roll"] = _bipolar(raw_channels[index["roll"]])
        self.channels["pitch"] = _bipolar(raw_channels[index["pitch"]])
        self.channels["yaw"] = _bipolar(raw_channels[index["yaw"]])
        self.channels["throttle"] = _unipolar(raw_channels[index["throttle"]])
        self.throttle_us = channel_to_us(raw_channels[index["throttle"]])
        self.arm_switch_on = raw_channels[index["arm"]] > _CRSF_CENTER
        if not self.arm_switch_on:
            self.arm_switch_seen_off = True
        self.mode = _mode_name(raw_channels[index["mode"]])
        self.link_alive = True
        self.last_frame_ms = now_ms


def _bipolar(raw_value):
    return min(max((raw_value - _CRSF_CENTER) / _HALF_SPAN, -1.0), 1.0)


def _unipolar(raw_value):
    return min(max((raw_value - _CRSF_LOW) / _FULL_SPAN, 0.0), 1.0)


def _mode_name(raw_value):
    if raw_value <= _MODE_LOW_MAX:
        return "manual"
    if raw_value <= _MODE_MID_MAX:
        return "stabilized"
    return "autonomous"


def _open_uart():
    import pins
    from machine import Pin, UART

    # rxbuf: the default (~256 B) overflows between polls -- ELRS at 250 Hz
    # streams ~650 B per 100 ms disarmed-loop interval, so most bytes were
    # dropped and every read was corrupt at the truncation boundaries,
    # forcing the parser into constant byte-at-a-time resync.
    #
    # 2 KB, not 4: the disarmed loop polls every 100 ms (~650 B), so this is
    # still 3x the worst interval, and the armed loop polls every 4 ms. The
    # original 4 KB was sized to ride out the blocking gyro calibration --
    # which main.py now follows with rc.flush(), because that backlog is
    # truncated garbage anyway. Halving it matters because this is the
    # largest single contiguous allocation the firmware makes, and it was
    # failing at boot (MemoryError) once the nav/GPS modules grew the heap
    # footprint -- MicroPython's GC doesn't compact, so a late large request
    # can fail with plenty of total free memory.
    return UART(
        pins.CRSF_UART_ID,
        baudrate=CRSF_BAUDRATE,
        tx=Pin(pins.CRSF_TX_PIN),
        rx=Pin(pins.CRSF_RX_PIN),
        rxbuf=2048,
    )
