# Research findings: MicroPython platform feasibility (Pico W)

Resolves wayfinder part `research-micropython-platform`.

## PWM outputs — Oneshot125 and servos

- RP2040 `machine.PWM` is hardware PWM driven from the 125 MHz system clock; frequency and duty resolution trade off (fewer duty steps at higher frequency). Docs: https://docs.micropython.org/en/latest/library/machine.PWM.html
- **Servos (50 Hz, 1–2 ms pulses):** trivial for `machine.PWM` — at 50 Hz the hardware gives ample duty resolution (effectively >14 bits). No concern.
- **Oneshot125 (125–250 µs pulse widths):** NOT a fixed-frequency protocol — the pulse is sent at the control-loop rate and its *width* carries the throttle. Two viable approaches:
  1. **Bit-bang / manual GPIO toggling:** at 125 MHz MicroPython can't toggle precisely enough in pure Python; jitter would be tens of µs. Marginal.
  2. **PWM peripheral one-shot trick:** configure the PWM slice period ≈ loop period and set duty = pulse width each loop iteration; the hardware generates the pulse without CPU timing sensitivity. Recommended. Effective resolution at a 4 kHz-capable slice is fine for 125–250 µs (hundreds of steps).
  3. (Fallback if (2) proves awkward: drop to standard 50–490 Hz PWM ESC protocol — many ESCs that do Oneshot125 also accept PWM; decision recorded as a risk, not a requirement.)
- Note: RP2040 has 8 PWM slices / 16 channels; ESC + 2 servos use 2–3 channels. Plenty.

## Control loop timing

- A 50–100 Hz loop is comfortably achievable in a single cooperative `asyncio` loop on RP2040 at 125 MHz (default). `asyncio.sleep_ms(10)` jitter is sub-millisecond when no task blocks.
- **`_thread` caveat:** MicroPython threading on RP2040 exists but is fragile — and critically, **the WiFi/CYW43 driver work happens on core 0 IRQ context; running the control loop on core 1 while WiFi is active is a known source of crashes/hangs** (see micropython discussions/issues). 
- **Key relaxation:** WiFi is only active while disarmed (owner decision). Therefore: **single-core asyncio architecture**, no threads. While armed, the loop handles CRSF parse → estimator → PID → outputs → telemetry; while disarmed+WiFi, the control loop is idle and the web server task runs. WiFi is torn down before arming.

## AP-mode web server

- AP mode: `network.WLAN(network.AP_IF)`, set SSID/password, `active(True)`, then serve on port 80. Well-trodden path.
- **microdot** (https://github.com/miguelgrinberg/microdot) runs on Pico W and is the lightest full-featured option (~small RAM footprint, async variant available). Raw-socket servers also work and are common in tutorials, but microdot gives routing/forms parsing nearly free. Recommend **microdot (async version only if needed; the sync version is fine since WiFi never runs concurrently with flight)**.
- Memory: Pico W has 264 KB SRAM; MicroPython leaves ~150–190 KB free. microdot + simple HTML forms fits comfortably. Avoid serving large assets; inline minimal HTML.

## Config persistence in flash

- Standard pattern: a JSON file on the littlefs filesystem (`open('config.json')`). RP2040 flash wear is not a concern at config-write rates (human edits only, while disarmed).
- Safe-write pattern: write to `config.json.tmp`, then `os.rename()` (atomic on littlefs), keep a `config.json.bak` fallback, validate with a schema + defaults on load, factory-reset = delete file.
- Filesystem is shared with the firmware source files; corruption risk is minimal but the .bak guard is cheap.

## Peripherals

- RP2040 has **2 independent UARTs** and **2 I²C peripherals**, all usable simultaneously in MicroPython (`machine.UART(0/1)`, `machine.I2C(0/1)` with free pin choice from each peripheral's pin mux table). CRSF (UART, 420 000 baud) + GPS (UART, 115 200 baud) + MPU6050 (I²C, 400 kHz) fit with zero contention — all are hardware peripherals fed by IRQ/DMA, no bit-banging.

## Recommendation / blockers

- **Architecture:** single-core `asyncio`, WiFi/webserver only while disarmed, microdot for the web UI, JSON config with tmp+rename safe writes, PWM-slice one-shot trick for Oneshot125, 50 Hz servo PWM.
- **Hard blockers:** none found.
- **Risks:** Oneshot125 pulse-width generation is the only item that needs a bench check early (risk, not blocker).
