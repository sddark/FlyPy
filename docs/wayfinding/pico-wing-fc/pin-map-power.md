# Pin map & power architecture

Type: grilling (HITL)

## Status

open

## Question

Define the Pico W pin map and power section of the spec: UART0/UART1 assignment to CRSF and GPS, I²C bus and pins for the MPU6050, GPIOs for the ESC (Oneshot125) and the two V-tail servos, the CRSF telemetry wire, LED/status pins, and how the ESC BEC powers the Pico W (VBUS/VSYS choice, what happens when USB is plugged in for config while the ESC is connected — back-powering safety). Blocked by "Research: MicroPython platform feasibility" (PWM/peripheral constraints may force pin choices).

## Assumptions

- Owner asked for a proposed pin map (they have not wired yet).
- ESC BEC powers the Pico W; no battery voltage sensing.
- Pico W has exactly 2 UARTs: CRSF and GPS take one each.

## Decision
