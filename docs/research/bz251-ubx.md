# Research findings: BZ-251 (u-blox M10) UBX configuration

Resolves wayfinder part `research-bz251-ubx`.

## Module

- BZGNSS BZ-251: u-blox **M10** generation (MAX-M10 class), UART interface, **default baud commonly 115 200** (configurable 9 600–460 800). Ships speaking NMEA + UBX. Manual: https://manuals.plus/bzgnss/bz251-gps-module-manual
- Note: some BZ-251 variants bundle a QMC5883 compass on an I²C pad — **irrelevant here** (no magnetometer in this build), but worth checking the exact unit; if present it changes nothing electrically since we only wire UART.

## M10 configuration — UBX-CFG-VALSET (not legacy CFG-MSG)

- M10 uses the **key-value configuration interface**: `UBX-CFG-VALSET` with 32-bit key IDs (e.g. `CFG-MSGOUT-UBX_NAV_PVT_UART1 = 0x20910007`, `CFG-MSGOUT-NMEA_ID_GGA_UART1`, `CFG-RATE-MEAS`, `CFG-NAVSPG-DYNMODEL`). Legacy `CFG-MSG`/`CFG-RATE` packets are deprecated on M10.
- Reference (authoritative): u-blox M10 SPG 5.x Interface Description — https://content.u-blox.com/sites/default/files/documents/u-blox-M10-SPG-5.20_InterfaceDescription_UBXDOC-304424225-20128.pdf
- **Boot config sequence for this FC (RAM layer, so it's re-applied every boot — no flash wear):**
  1. Set `CFG-RATE-MEAS` to 200 ms (5 Hz nav rate; 10 Hz possible, 5 Hz is plenty for fixed-wing nav and halves UART load).
  2. Set `CFG-NAVSPG-DYNMODEL` = airborne <4g (value 8) — best filter tuning for a fixed wing.
  3. Enable `CFG-MSGOUT-UBX_NAV_PVT_UART1` = 1.
  4. Set all `CFG-MSGOUT-NMEA_ID_*_UART1` = 0 (GGA, GLL, GSA, GSV, RMC, VTG, TXT) — kills the NMEA spam.
  5. Optionally raise baud (`CFG-UART1-BAUDRATE`) to 230 400 if 115 200 ever congests at 5 Hz — expected unnecessary.
- VALSET framing: header `B5 62 06 8A`, payload `[version(0)][layers(1=RAM)][0][0][keyID LE u32][value]…`, 16-bit Fletcher checksum. Straightforward to emit from MicroPython.

## NAV-PVT (0x01 0x07, 92 bytes) — fields the FC needs

| Field | Offset | Type | Use |
|---|---|---|---|
| iTOW | 0 | u4 | frame freshness |
| fixType | 20 | u1 | ≥3 = 3D fix; arming/nav gate |
| flags/gnssFixOK | 21 | u1 | fix validity |
| numSV | 23 | u1 | sats (telemetry) |
| lon/lat | 24/28 | i4 | deg×1e-7 (telemetry 0x02 direct, nav) |
| hMSL | 36 | i4 | mm — only altitude source (no baro) |
| gSpeed | 60 | i4 | mm/s |
| headMot | 64 | i4 | deg×1e-5 — **heading source** (course over ground) |
| hAcc/vAcc | 40/44 | u4 | mm — fix-quality gate |
| headAcc | 72 | u4 | course accuracy |

Parsing in MicroPython: `struct.unpack_from('<I…i4…')` on the payload — one line, fast.

## Performance

- Cold-start TTFF ≈ **27–28 s** open sky; hot start a few seconds. Fix rates: M10 supports up to 25 Hz single-GNSS; 5 Hz multi-GNSS is comfortable.
- Existing code: no dominant MicroPython UBX lib; closest references are `mayeranalytics/pyUBX` (CPython, MIT — good message-layout reference) and `Korving-F/ublox`. Given NAV-PVT + VALSET are the only messages needed, a ~100-line bespoke driver is simpler than porting a library. **Recommend bespoke.**

## Blockers / risks

- None blocking. GPS-only altitude (hMSL noise ±1–3 m) feeds the map's altitude fog. Course-over-ground is meaningless below ~1–2 m/s — confirms the magnetometer fog entry.
