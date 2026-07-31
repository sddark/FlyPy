# Pin map & power architecture

Type: grilling (HITL)

## Status

resolved 2026-07-29

## Question

Define the Pico W pin map and power section of the spec: UART0/UART1 assignment to CRSF and GPS, I²C bus and pins for the MPU6050, GPIOs for the ESC (Oneshot125) and the two V-tail servos, the CRSF telemetry wire, LED/status pins, and how the ESC BEC powers the Pico W (VBUS/VSYS choice, what happens when USB is plugged in for config while the ESC is connected — back-powering safety). Blocked by "Research: MicroPython platform feasibility" (PWM/peripheral constraints may force pin choices).

## Assumptions

- Owner asked for a proposed pin map (they have not wired yet).
- ESC BEC powers the Pico W; no battery voltage sensing.
- Pico W has exactly 2 UARTs: CRSF and GPS take one each.

## Decision

Adopt the pin map already prototyped in `firmware/pins.py`:

| Function | UART/I²C/PWM | Pins |
|---|---|---|
| CRSF (ELRS RX) | UART0 | GP0 TX (telemetry wire, unused for MVP), GP1 RX |
| GPS (BZ-251 UBX) | UART1 | GP4 TX, GP5 RX |
| MPU6050 | I2C0 | GP8 SDA, GP9 SCL |
| Left V-tail servo | PWM slice 1 ch A, 50 Hz | GP2 |
| Right V-tail servo | PWM slice 1 ch B, 50 Hz | GP3 |
| ESC (Oneshot125) | PWM slice 3 ch A | GP6 |

- **Status/LED:** the Pico W's onboard LED is wired to the wireless chip (`WL_GPIO0`), not a general-purpose pin — no dedicated GPIO needed. It signals portal-up / armed / failsafe; exact blink patterns are an implementation detail, not a pin-map concern.
- **Power:** the ESC's BEC feeds the Pico W's **VSYS**. USB is never connected at the same time as the battery/ESC — configuration happens over the WiFi portal, not USB-serial, so the two power sources are never combined. No back-powering protection circuit (diode-ORing) is in scope; this is an operating rule ("don't plug in USB with the battery connected"), not a hardware safeguard.
- CRSF telemetry wire (GP0) is wired but unused for the MVP — see [Telemetry content spec](./telemetry-content.md) (deferred).
