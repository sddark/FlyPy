# BZGNSS BZ-251 (u-blox M10) GPS driver, per "Research: BZ-251 UBX
# configuration": UBX binary only (all NMEA off), configured via
# UBX-CFG-VALSET on the RAM layer at every boot -- 5 Hz nav rate, airborne
# <4g dynamic model, NAV-PVT the single enabled output. Bespoke driver by
# decision (no MicroPython UBX library worth porting for two messages).
#
# Framing/parsing is pure Python (no `machine` imports) so it is testable
# off-target; only UbxGps touches the UART (injectable, same pattern as
# rc.CrsfReceiver). The RAM-layer config vanishes if the module reboots
# (brownout, late power-up), so UbxGps re-sends the VALSET whenever NAV-PVT
# has been silent for a few seconds -- the module doesn't need to be alive,
# or even plugged in, when the Pico boots.

import struct
import time

_ticks_diff = getattr(time, "ticks_diff", lambda end, start: end - start)

GPS_BAUDRATE = 115_200

UBX_SYNC_1 = 0xB5
UBX_SYNC_2 = 0x62

_CLASS_NAV = 0x01
_ID_NAV_PVT = 0x07
_CLASS_ACK = 0x05
_ID_ACK_ACK = 0x01
_CLASS_CFG = 0x06
_ID_VALSET = 0x8A

NAV_PVT_PAYLOAD_LEN = 92
_MAX_PAYLOAD_LEN = 512  # resync sanity bound; NAV-PVT is 92
_MIN_FRAME_LEN = 8      # sync(2) + class + id + len(2) + checksum(2)

_VALSET_LAYER_RAM = 0x01

# UBX-CFG-VALSET key IDs (u-blox M10 interface description). Bits 30..28 of
# the key encode the value width: 0x2 -> 1 byte, 0x3 -> 2 bytes, 0x4 -> 4.
_CFG_RATE_MEAS = 0x30210001
_CFG_NAVSPG_DYNMODEL = 0x20110021
_CFG_MSGOUT_NAV_PVT_UART1 = 0x20910007
_CFG_INFMSG_NMEA_UART1 = 0x20920002  # kills TXT info messages
_CFG_MSGOUT_NMEA_UART1 = (
    0x209100BB,  # GGA
    0x209100CA,  # GLL
    0x209100C0,  # GSA
    0x209100C5,  # GSV
    0x209100AC,  # RMC
    0x209100B1,  # VTG
)

_DYNMODEL_AIRBORNE_4G = 8
_NAV_RATE_MS = 200  # 5 Hz

BOOT_CONFIG = (
    (_CFG_RATE_MEAS, _NAV_RATE_MS),
    (_CFG_NAVSPG_DYNMODEL, _DYNMODEL_AIRBORNE_4G),
    (_CFG_MSGOUT_NAV_PVT_UART1, 1),
    (_CFG_INFMSG_NMEA_UART1, 0),
) + tuple((key, 0) for key in _CFG_MSGOUT_NMEA_UART1)

# No PVT for this long -> assume the module (re)booted without its RAM
# config and send the VALSET again, rate-limited to the same interval.
_CONFIG_RETRY_MS = 3000


def fletcher_checksum(data):
    ck_a = 0
    ck_b = 0
    for byte in data:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def build_frame(msg_class, msg_id, payload):
    header = bytes([msg_class, msg_id, len(payload) & 0xFF, len(payload) >> 8])
    ck_a, ck_b = fletcher_checksum(header + payload)
    return bytes([UBX_SYNC_1, UBX_SYNC_2]) + header + payload + bytes([ck_a, ck_b])


def _key_width(key):
    size = (key >> 28) & 0x7
    if size in (1, 2):
        return 1
    if size == 3:
        return 2
    if size == 4:
        return 4
    return 8


def build_valset(key_values):
    payload = bytearray((0x00, _VALSET_LAYER_RAM, 0x00, 0x00))
    for key, value in key_values:
        payload += struct.pack("<I", key)
        # Values are little-endian; truncating a packed u4 keeps the low
        # bytes, which is exactly the 1/2-byte encoding. Config values here
        # are all non-negative.
        payload += struct.pack("<I", value)[: _key_width(key)]
    return build_frame(_CLASS_CFG, _ID_VALSET, bytes(payload))


def parse_nav_pvt(payload):
    # Offsets per the research findings table; scales to working units:
    # degrees, meters, meters/second.
    fix_type = payload[20]
    flags = payload[21]
    return {
        "itow_ms": struct.unpack_from("<I", payload, 0)[0],
        "fix_type": fix_type,
        "fix_ok": bool(flags & 0x01) and fix_type >= 3,
        "num_sv": payload[23],
        "lon_deg": struct.unpack_from("<i", payload, 24)[0] * 1e-7,
        "lat_deg": struct.unpack_from("<i", payload, 28)[0] * 1e-7,
        "alt_m": struct.unpack_from("<i", payload, 36)[0] / 1000.0,
        "h_acc_m": struct.unpack_from("<I", payload, 40)[0] / 1000.0,
        "v_acc_m": struct.unpack_from("<I", payload, 44)[0] / 1000.0,
        "ground_speed_ms": struct.unpack_from("<i", payload, 60)[0] / 1000.0,
        "course_deg": struct.unpack_from("<i", payload, 64)[0] * 1e-5,
        "course_acc_deg": struct.unpack_from("<I", payload, 72)[0] * 1e-5,
    }


class UbxParser:
    # Accumulates raw UART bytes and yields complete checksum-valid frames
    # as (class, id, payload). Index-based resync scan, same O(n) pattern
    # (and for the same reasons) as rc.CrsfParser.

    def __init__(self):
        self._buffer = bytearray()

    def feed(self, data):
        if data:
            self._buffer.extend(data)
        buffer = self._buffer
        view = memoryview(buffer)
        end = len(buffer)
        start = 0
        frames = []
        while end - start >= _MIN_FRAME_LEN:
            if buffer[start] != UBX_SYNC_1 or buffer[start + 1] != UBX_SYNC_2:
                start += 1
                continue
            length = buffer[start + 4] | (buffer[start + 5] << 8)
            if length > _MAX_PAYLOAD_LEN:
                start += 1
                continue
            frame_end = start + _MIN_FRAME_LEN + length
            if end < frame_end:
                break
            ck_a, ck_b = fletcher_checksum(view[start + 2 : frame_end - 2])
            if ck_a != buffer[frame_end - 2] or ck_b != buffer[frame_end - 1]:
                start += 1
                continue
            frames.append(
                (buffer[start + 2], buffer[start + 3], bytes(view[start + 6 : frame_end - 2]))
            )
            start = frame_end
        if start:
            self._buffer = bytearray(view[start:])
        return frames


class UbxGps:
    def __init__(self, uart=None):
        if uart is None:
            uart = _open_uart()
        self._uart = uart
        self._parser = UbxParser()
        self.fix_ok = False
        self.fix_type = 0
        self.num_sv = 0
        self.lat_deg = 0.0
        self.lon_deg = 0.0
        self.alt_m = 0.0
        self.ground_speed_ms = 0.0
        self.course_deg = 0.0
        self.h_acc_m = 0.0
        self.course_acc_deg = 0.0
        self.last_pvt_ms = None
        self.config_acknowledged = False
        self._last_config_ms = None

    def update(self, now_ms):
        data = self._uart.read()
        if data:
            for msg_class, msg_id, payload in self._parser.feed(data):
                is_pvt = (
                    msg_class == _CLASS_NAV
                    and msg_id == _ID_NAV_PVT
                    and len(payload) == NAV_PVT_PAYLOAD_LEN
                )
                if is_pvt:
                    self._apply_pvt(parse_nav_pvt(payload), now_ms)
                elif (
                    msg_class == _CLASS_ACK
                    and msg_id == _ID_ACK_ACK
                    and len(payload) == 2
                    and payload[0] == _CLASS_CFG
                    and payload[1] == _ID_VALSET
                ):
                    self.config_acknowledged = True
        self._maybe_send_config(now_ms)

    def _apply_pvt(self, pvt, now_ms):
        self.fix_ok = pvt["fix_ok"]
        self.fix_type = pvt["fix_type"]
        self.num_sv = pvt["num_sv"]
        self.lat_deg = pvt["lat_deg"]
        self.lon_deg = pvt["lon_deg"]
        self.alt_m = pvt["alt_m"]
        self.ground_speed_ms = pvt["ground_speed_ms"]
        self.course_deg = pvt["course_deg"]
        self.h_acc_m = pvt["h_acc_m"]
        self.course_acc_deg = pvt["course_acc_deg"]
        self.last_pvt_ms = now_ms

    def _maybe_send_config(self, now_ms):
        pvt_silent = (
            self.last_pvt_ms is None
            or _ticks_diff(now_ms, self.last_pvt_ms) > _CONFIG_RETRY_MS
        )
        recently_sent = (
            self._last_config_ms is not None
            and _ticks_diff(now_ms, self._last_config_ms) <= _CONFIG_RETRY_MS
        )
        if pvt_silent and not recently_sent:
            self._uart.write(build_valset(BOOT_CONFIG))
            self._last_config_ms = now_ms
            self.config_acknowledged = False


def _open_uart():
    import pins
    from machine import Pin, UART

    # 5 Hz NAV-PVT is ~100 B per fix, ~500 B/s, and both the armed and
    # disarmed loops drain it at least every 100 ms -- 1 KB is ~2 s of
    # margin. Kept deliberately small: see rc.py's rxbuf note, large
    # contiguous allocations are the scarce resource on this board.
    return UART(
        pins.GPS_UART_ID,
        baudrate=GPS_BAUDRATE,
        tx=Pin(pins.GPS_TX_PIN),
        rx=Pin(pins.GPS_RX_PIN),
        rxbuf=1024,
    )
