# System diagrams

Hierarchical mermaid documentation of the Pico W fixed-wing FC, from system context down to per-part subcomponents. Each level cites the part doc whose decision it reflects; open parts are marked so the diagram updates as they resolve.

Source legend: **[INAV]** = transpiled from INAV (see [Research: INAV transpile survey](./research-inav-survey.md)) · **[bespoke]** = built fresh.

---

## L0 — System context

Everything outside the firmware: airframe hardware, pilot, ground config.

```mermaid
flowchart LR
    tx["Pilot TX (ELRS)"] <-- "CRSF @ 420 000 baud<br/>channels 0x16 down, telemetry up" --> fc
    gps["BZ-251 GPS<br/>(u-blox M10)"] -- "UBX NAV-PVT @ 5 Hz<br/>UART 115 200" --> fc
    imu["MPU6050<br/>gyro+accel"] -- "I²C, raw (no DMP)<br/>DLPF 98 Hz" --> fc
    fc[["Pico W flight controller<br/>(MicroPython, single-core asyncio)"]]
    fc -- "Oneshot125" --> esc["ESC + BEC"]
    fc -- "50 Hz PWM ×2" --> srv["V-tail servos L/R"]
    esc -- "BEC power" --> fc
    phone["Phone/laptop browser"] <-. "WiFi AP, disarmed only<br/>microdot HTML forms" .-> fc
```

Decided by: hardware list (map Notes), [CRSF research](./research-crsf-protocol.md), [BZ-251 research](./research-bz251-ubx.md), [MPU6050 research](./research-mpu6050-attitude.md), [platform research](./research-micropython-platform.md). Wire-level pin assignment is open — [Pin map & power](./pin-map-power.md).

---

## L1 — Firmware block diagram (transpile vs bespoke)

```mermaid
flowchart LR
    subgraph inputs[Input drivers]
        crsf_rx["CRSF RX decode<br/>[INAV rx/crsf.c]"]
        gps_drv["BZ-251 UBX driver<br/>[bespoke ~100 lines]"]
        imu_drv["MPU6050 I²C driver<br/>[bespoke]"]
    end

    subgraph core[Flight core — asyncio scheduler, bespoke]
        est["Attitude estimation (Mahony, 200 Hz)<br/>[INAV flight/imu.c]"]
        pid["Fixed-wing PID<br/>[INAV flight/pid.c]"]
        mixer["V-tail mixer<br/>[INAV flight/mixer.c]"]
        nav["Waypoint nav & guidance<br/>[INAV navigation/ FW subset]"]
        fsm["Arming / modes / failsafe<br/>[INAV fc_core.c, rc_modes.c, failsafe.c]"]
    end

    subgraph outputs[Output drivers]
        pwm["Servo/ESC PWM + Oneshot125<br/>[bespoke]"]
        telem["CRSF telemetry encoders<br/>[INAV telemetry/crsf.c — content bespoke]"]
    end

    subgraph ground[Ground — disarmed only]
        webcfg["Web config + JSON-in-flash<br/>[bespoke]"]
        mission["Mission entry<br/>[bespoke — fog item]"]
    end

    crsf_rx --> fsm & pid
    imu_drv --> est --> pid
    gps_drv --> nav --> pid
    fsm --> pid & nav
    pid --> mixer --> pwm
    est & nav & fsm --> telem
    webcfg & mission -.-> core
```

Decided by: [INAV transpile survey](./research-inav-survey.md) (module split, GPLv3).

---

## L2 — Subcomponents by part

### Scheduler & loop rates

From [platform research](./research-micropython-platform.md) (single-core asyncio) and [MPU6050 research](./research-mpu6050-attitude.md) (200 Hz estimator).

```mermaid
flowchart TB
    sched["asyncio scheduler (bespoke)<br/>replaces INAV fc_tasks.c"]
    sched --> t1["IMU sample + Mahony — 200 Hz"]
    sched --> t2["CRSF RX parse — event-driven on UART"]
    sched --> t3["Control loop (PID + mixer + PWM out) — 50–100 Hz"]
    sched --> t4["GPS NAV-PVT parse + nav update — 5 Hz"]
    sched --> t5["Telemetry send — scheduled per frame type"]
    sched --> t6["Web server (microdot) — disarmed only"]
    t1 --> t3
    t2 --> t3
    t4 --> t3
```

Exact rates/nesting open — [Control system design](./control-system-design.md).

### Control pipeline

```mermaid
flowchart LR
    rc["RC channels<br/>(CRSF 0x16, 16×11-bit)"] --> demands
    navd["Nav demands<br/>(autonomous mode)"] --> demands
    demands["Roll/pitch/yaw + throttle demands<br/>per active mode"] --> pid["PID (fixed-wing)<br/>[INAV pid.c]<br/>tunables web-configured"]
    att["Attitude from Mahony<br/>[INAV imu.c]"] --> pid
    pid --> mixer["V-tail mixer<br/>[INAV mixer.c]<br/>pitch+roll → L/R surface"]
    mixer --> out["2× servo PWM 50 Hz +<br/>ESC Oneshot125"]
```

Open: loop rates, rate-vs-attitude nesting per mode, throttle path per mode, parameter list/ranges/defaults — [Control system design](./control-system-design.md).

### Arming, modes & failsafe state machine

Decided behavior (map Notes): TX-switch arming, zero-throttle pre-arm only, 3 modes on a channel, RC loss → level + cut throttle, GPS loss in autonomous → same, WiFi only disarmed.

```mermaid
stateDiagram-v2
    [*] --> Disarmed
    Disarmed --> Disarmed : WiFi AP up — web config available
    Disarmed --> Armed : arm switch ON<br/>AND throttle == 0 (only pre-arm check)
    Armed --> Disarmed : arm switch OFF
    Armed --> Manual : mode ch = MANUAL
    Armed --> Stabilized : mode ch = STAB
    Armed --> Autonomous : mode ch = AUTO<br/>(requires GPS fix)
    Manual --> Stabilized
    Stabilized --> Autonomous
    Autonomous --> Stabilized
    Stabilized --> Manual
    Autonomous --> Manual
    Manual --> Failsafe : RC link lost (0x16 frame timeout)
    Stabilized --> Failsafe : RC link lost
    Autonomous --> Failsafe : RC link lost OR GPS lost
    Failsafe --> Failsafe : level wings + cut throttle
    Failsafe --> Manual : RC link restored<br/>(mode re-read from channel)
```

Structure ported from [INAV] `fc_core.c` + `rc_modes.c` + `failsafe.c`, simplified. Open: transition rules detail, detection timeouts, GPS-lost criteria, re-arm after failsafe — [Flight modes, arming & failsafe](./flight-modes-arming-failsafe.md).

### Navigation data flow

```mermaid
flowchart LR
    pvt["NAV-PVT @ 5 Hz<br/>lat/lon, COG, speed, alt,<br/>sats, hAcc/vAcc"] --> navcore["Waypoint execution &<br/>fixed-wing guidance<br/>[INAV navigation.c +<br/>navigation_fixedwing.c]"]
    geo["Geo math [INAV navigation_geo.c]<br/>+ sqrt_controller.c"] --> navcore
    wps["Waypoint list<br/>(entry method: fog)"] --> navcore
    navcore -->|"heading/altitude/throttle demands<br/>heading = GPS course-over-ground<br/>(no magnetometer)"| ctl["Control pipeline"]
    navcore -->|"GPS lost in AUTO"| fs["Failsafe: level + cut throttle"]
```

Open: guidance law confirmation (L1/cross-track vs bearing chase), turn coordination without rudder, advance criteria, GPS-only altitude, low-speed heading behavior — [Autonomous navigation design](./autonomous-navigation.md). Fog: mission entry, low-speed heading, altitude reference (map "Not yet specified").

### Telemetry

```mermaid
flowchart LR
    fc["FC state:<br/>attitude, GPS, mode, armed"] --> sel["Content selection<br/>[bespoke — this part]<br/>no battery frame (no sensing)"]
    sel --> enc["Frame encoders<br/>[INAV telemetry/crsf.c]<br/>GPS 0x02 · attitude 0x1E · mode 0x21"]
    enc --> uart["CRSF telemetry wire → RX → TX"]
```

Open: which frames at what rates, never starving the control loop — [Telemetry content spec](./telemetry-content.md).

### Web config & persistence

```mermaid
flowchart TB
    subgraph disarmed["Disarmed only"]
        ap["Pico W AP"] --> http["microdot server<br/>HTML forms"]
        http --> val["Validation<br/>ranges/defaults per parameter"]
        val --> store["JSON config in flash<br/>tmp + rename atomic write<br/>factory reset"]
    end
    store -.load at boot.-> params["Parameter set:<br/>PIDs, rates, mixing, failsafe, modes"]
```

All [bespoke] — INAV's CLI/MSP/EEPROM not ported. Open: schema, reboot-vs-live apply, SSID/security — [Web config & parameter persistence](./web-config-persistence.md).

### Pin map & power (placeholder — part open)

```mermaid
flowchart LR
    subgraph pico[Pico W]
        u0["UART0 → CRSF (RX + telemetry wire)"]
        u1["UART1 → GPS (UBX)"]
        i2c["I²C → MPU6050"]
        g1["GPIO → ESC (Oneshot125, PWM slice)"]
        g2["GPIO ×2 → servos (50 Hz)"]
    end
    bec["ESC BEC → VSYS/VBUS<br/>(USB-vs-BEC back-powering: open question)"]
```

Assignment provisional — exact pins and power safety decided in [Pin map & power](./pin-map-power.md).
