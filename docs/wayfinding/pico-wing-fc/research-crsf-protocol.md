# Research: CRSF protocol under MicroPython

Type: research (AFK)

## Status

resolved 2026-07-28

## Question

What does implementing CRSF (TBS Crossfire / ELRS) on a Pico W UART in MicroPython require? Cover: (a) the protocol itself — baud rate (420 000), frame format, RC-channels frame (0x16) packing/unpacking of 16×11-bit channels; (b) telemetry frames relevant to a fixed-wing FC — GPS (0x02), attitude (0x1E), flight mode (0x21), battery (0x08) — format and how telemetry is polled/sent (half-duplex vs separate wire, ping/capabilities); (c) failsafe signalling in CRSF (how link loss is detectable); (d) any existing MicroPython CRSF libraries or reference code (cite source + license); (e) ELRS-specific quirks. Output: findings doc with enough frame-level detail that the spec's CRSF part can be written without re-reading the protocol docs.

## Assumptions

- Receiver is ELRS, connected to one Pico W UART (RX for channels; telemetry return wire assumed available — pin map part will confirm).
- Channel count needed: ≥6 (4 controls + arm switch + mode switch).

## Decision

420 000 baud, `[addr][len][type][payload][crc8]` frames; channels = type 0x16, 16×11-bit little-endian packed (trivial pure-Python unpack). Telemetry 0x02/0x1E/0x21 documented with byte layouts; battery frame omitted (no sensing). Link loss = timeout on 0x16 frames (ELRS default "no pulses" failsafe). Reference impls: AlfredoCRSF (MIT), crsf-wg wiki. Full findings: [docs/research/crsf-protocol.md](../../research/crsf-protocol.md)
