# Research findings: CRSF protocol under MicroPython

Resolves wayfinder part `research-crsf-protocol`.

## Transport

- UART, **420 000 baud**, 8N1 (ELRS default; 115200 fallback exists on some setups). Pico W UARTs support arbitrary bauds — no issue.
- General frame: `[address/sync] [length] [type] [payload…] [crc8]`; length covers type+payload+crc; CRC is CRC-8/DVB-S2 over type+payload. Reference: https://github.com/crsf-wg/crsf/wiki and https://cdn.hackaday.io/files/1889418083651744/crsf_protocol.h

## RC channels (type 0x16)

- Payload = **16 channels × 11 bits**, little-endian bit-packed (22 bytes). Values 0–1984; ~992 center, 172/1811 typical end points (CRSF values, not µs).
- Unpacking in MicroPython: `int.from_bytes(payload, 'little')` then shift/mask 16 times — fast, pure-Python fine at 100+ Hz frame rates.
- ELRS sends channels frames at the packet rate (e.g. 50–500 Hz depending on link mode); treat every valid 0x16 frame as the freshest demand.

## Telemetry (FC → RX → TX)

- Telemetry is sent **on the same UART wire, interleaved** (the RX polls / the FC responds in gaps; with ELRS the FC may send telemetry frames opportunistically at a modest rate — standard practice: send a telemetry frame after every N channel frames).
- Relevant frames:
  - **0x02 GPS**: lat (int32, deg×1e7), lon (int32), groundspeed (uint16, km/h×10), heading (uint16, deg×100), altitude (uint16, m + 1000 offset), sats (uint8). Direct feed from NAV-PVT.
  - **0x1E Attitude**: pitch, roll, yaw — int16 each, radians ×10000. Feed from the estimator.
  - **0x21 Flight mode**: null-terminated ASCII string (e.g. "MAN", "STAB", "AUTO", "FS!"), plus arm status conventionally embedded in the string ("*" suffix when disarmed in BF convention).
  - **0x08 Battery**: voltage (uint16, 0.1 V), current, capacity used, percent — **stub: no voltage sensing in this build**; either omit or send zeros. Recommend omitting.
- Parsing/sending reference implementations: https://github.com/AlfredoSystems/AlfredoCRSF (Arduino, MIT — best compact reference), `AlessioMorale/crsf_parser` (Python-oriented, ExpressLRS-focused; check repo license before reuse).

## Failsafe / link-loss detection

- ELRS receiver tracks RF link health itself. On link loss (LQ → 0, or ~1 s without valid packets) the receiver's configured failsafe applies: default **"no pulses"** — the receiver simply **stops sending channel frames**.
- **Therefore FC-side detection: a timeout on 0x16 frames** (e.g. no valid channels frame for 300–500 ms ⇒ link lost). Do not rely on a flag inside the stream.
- ELRS also supports failsafe "set position" per channel — recommend keeping RX on "no pulses" and letting the FC own failsafe behavior (consistent with owner decision: level + cut throttle).

## ELRS quirks

- Channel count is 16 but ELRS maps only 4 full-res + switch channels depending on switch mode; a 6-channel need (4 controls + arm + mode) is fine in standard "Hybrid" switch mode. Arm/mode switches should be on 2-pos channels (AUX1 is the arming convention, ch5).
- Baud: ELRS requires the FC baud to match the RX's configured serial baud (default 420 000).
- No config/MSP equivalent over CRSF is needed here — configuration happens over WiFi, not the radio link.

## Implementation sketch

- RX path: UART IRQ or `uart.any()` polling in the asyncio loop; accumulate bytes, scan for frame boundaries by length/type, CRC-check, dispatch 0x16.
- TX path: a small queue flushed every loop iteration at a capped rate (e.g. 20–30 Hz total telemetry, cycling frame types round-robin).
- CPU cost is trivial; pure-Python parse at these rates is fine.

## Blockers / risks

- None blocking. The half-duplex interleave timing on the telemetry wire is the one detail to validate on the bench; AlfredoCRSF + the crsf-wg wiki give working timing to copy.
