# esp32-airplane

Autonomous **flying wing** (ala voladora) flown by an **ESP32**, reusing the
algorithms developed for the AR.Drone 2.0 project.

> New here? Read in this order:
> 1. [`CHECKLIST.md`](CHECKLIST.md) — the **iterative definition checklist**
>    (this is the living document we keep closing)
> 2. [`docs/algorithm-mapping.md`](docs/algorithm-mapping.md) — which
>    AR.Drone algorithms transfer, which are rewritten, which are dropped
> 3. [`docs/rc-and-telemetry.md`](docs/rc-and-telemetry.md) — how RC input and
>    telemetry reach a **Radiomaster TX16S MKII (4in1)** (SBUS/Spektrum in,
>    S.Port/FPort out — SBUS itself is strictly one-way)
> 4. `firmware/esp32_airplane/` — starter firmware

---

## Locked baseline

| Decision | Value |
|----------|-------|
| Airframe | **Ala voladora** — elevons only, no rudder, yaw via bank |
| Sensors | IMU + baro + GPS + magnetometer + **LiDAR (TFmini)** + **radar (HLK-LD2450)** |
| Phase 1 autonomy | **Estabilización + RTH** |
| Phase 2 autonomy | Vuelo autónomo con cámara + LiDAR + radar |
| Propulsion | **Eléctrico + ESC (throttle PWM)** |
| RC input | **SBUS o Spektrum** (UART2, elegido y persistido por runtime) |
| IMU board | **GY-91** (MPU9250+BMP280) o **GY-87** (MPU6050+HMC5883L+BMP180) — `#if` en tiempo de compilación |
| Configuration | **WiFi portal** (STA→AP `wing-%04X`), guardado en NVS con CRC |

---

## Folder structure

```
esp32-airplane/
├── README.md                     # you are here
├── CHECKLIST.md                  # ★ iterative definition checklist
├── docs/
│   ├── algorithm-mapping.md      # AR.Drone → wing algorithm transfer matrix
│   └── rc-and-telemetry.md       # RC in (SBUS/Spektrum) + telemetry out (TX16S)
└── firmware/
    ├── Makefile                  # `make test` → 257 host unit tests
    │                             # `make test IMU=87` → GY-87 board instead
    ├── tests/
    │   ├── test_wing_core.cpp    # J2: mixing, envelope, L1, RTH, NMEA, LD2450, AHRS, failsafe (111)
    │   └── test_rc_settings.cpp  # settings/CRC, SBUS+Spektrum, sensor math (106)
    └── esp32_airplane/
        ├── esp32_airplane.ino    # entry point: setup/loop, fixed-rate slots, PWM, RC
        ├── config.h              # gains, limits, pin map, compile-time IMU select
        ├── settings.{h,cpp}      # persisted Settings blob (magic/version/CRC/NVS)
        ├── rc_input.{h,cpp}      # SBUS + Spektrum decode, channel map, expo, arm
        ├── wifi_config.{h,cpp}   # ESP32 HTTP config portal (I5)
        ├── ahrs.{h,cpp}          # gyro+accel+mag fusion (complementary)
        ├── control.{h,cpp}       # ADRC/PID loops + envelope protection + b0 ∝ q
        ├── mixing.{h,cpp}        # elevon mixing + adverse-yaw differential
        ├── guidance.{h,cpp}      # L1 path following, wing-specific RTH, geofence
        ├── gps_nav.{h,cpp}       # NMEA parser + Haversine (ported from AR.Drone)
        ├── failsafe.{h,cpp}      # failsafe FSM + preflight gate
        └── drivers/
            ├── imu_board.h       # board abstraction behind `#if IMU_GY91/GY87`
            ├── imu_gy91.cpp      # REAL: MPU9250 + AK8963 + BMP280
            ├── imu_gy87.cpp      # REAL: MPU6050 + HMC5883L + BMP180
            ├── sensor_math.{h,cpp} # LSB tables + BMP180/BMP280 datasheet compensation
            ├── imu.{h,cpp}       # thin adapter over board_imu_*
            ├── sensors.{h,cpp}   # baro/mag dispatch; GPS/TFmini still STUBs (C4/C6)
            └── ld2450.{h,cpp}    # REAL: HLK-LD2450 frame parser + track matcher (C7)
```

*GPS (C4) and LiDAR (C6) still emit synthetic signals. Wire real drivers
before §C of the checklist is marked done.*

### Compile-time IMU selection

```bash
cd esp32-airplane/firmware
make test            # GY-91 (default)
make test IMU=87     # GY-87
# Arduino IDE  -> -DIMU_GY87 in build_opt.h
# PlatformIO   -> build_flags = -DIMU_GY87
```

Both board drivers are always *compiled* (the Arduino IDE builds every `.cpp`
in the sketch folder); only one is active, and defining both or neither is a
`#error`.

### Run the tests

```bash
cd esp32-airplane/firmware && make test      # 257 assertions
# or from the repo-wide suite:
cd tools/simulator && make check             # quad (90) + wing (257) + wing SIL (64)
```

---

## Why these algorithms?

The single biggest insight from the AR.Drone work that carries over:

> **ADRC (ESO + NLSEF) is the best-fit controller for a wing in wind** —
> the ESO lumps gusts, turbulence and aerodynamic uncertainty into one
> "total disturbance" term `f̂` and cancels it, without needing an accurate
> model. But `b0` (control effectiveness) must be **scheduled with dynamic
> pressure**: `b0 ∝ q ∝ V²`, because elevon authority comes from airflow.

What **doesn't** carry over: the ETH spinning-flight prop-loss recovery
(a wing glides instead), quad hover/vertical-altitude logic (a wing uses
energy management / TECS), and onboard visual odometry (too heavy for ESP32).

Full matrix → [`docs/algorithm-mapping.md`](docs/algorithm-mapping.md).

---

## Top risks

1. **Stall** — the #1 killer of wings. Envelope protection (checklist **F3 / N4**)
   must be implemented and SIL-tested *before* anything autonomous moves a
   surface.
2. **CG placement** (A3) — wrong CG = unflyable regardless of software.
3. **Brown-out under throttle** (B4). The pin map (B2) is now written down in
   `config.h`, but still needs bench verification.
4. **RC link** (H2) — SBUS is one-way, so telemetry needs a parallel path
   (see `docs/rc-and-telemetry.md`); the Spektrum status byte needs checking
   against a real receiver.
5. **600 → 240 MHz, no double FPU** — `float` only in the hot loop.

---

## Development order

See the **K. Phase roadmap** section of the checklist. Summary:

`S0 define → S1 bench → S2 glide/trim → S3 RTH → S4 waypoints → S5 camera/LiDAR/radar → S6 swarm`

Software-in-the-loop comes before every flight: extend `tools/simulator/`
with a 6-DOF **fixed-wing** plant model (checklist **J1**) and run the same
unit-test style used for the quad (`make check`, 90/90 today).

---

## Related repo paths

- `src/flight/adrc.c` — ADRC controller to port
- `src/flight/flight_controller.c` — PID cascade + geofence to port
- `src/navigation/gps.c` — NMEA parser + Haversine + RTH to adapt
- `src/vision/obstacle.c`, `line_detect.c`, `flow_stage1.c` — phase 2 vision
- `docs/auto-recovery-navigation.md` — research: motor-fail, consensus, LD2450
- `tools/simulator/` — quad SIL (template for the wing SIL)
- `esp32-airplane/docs/rc-and-telemetry.md` — RC + telemetry feasibility
