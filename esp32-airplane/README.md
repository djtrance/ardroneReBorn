# esp32-airplane

Autonomous **flying wing** (ala voladora) flown by an **ESP32**, reusing the
algorithms developed for the AR.Drone 2.0 project.

> New here? Read in this order:
> 1. [`CHECKLIST.md`](CHECKLIST.md) — the **iterative definition checklist**
>    (this is the living document we keep closing)
> 2. [`docs/algorithm-mapping.md`](docs/algorithm-mapping.md) — which
>    AR.Drone algorithms transfer, which are rewritten, which are dropped
> 3. `firmware/esp32_airplane/` — starter firmware

---

## Locked baseline

| Decision | Value |
|----------|-------|
| Airframe | **Ala voladora** — elevons only, no rudder, yaw via bank |
| Sensors | IMU + baro + GPS + magnetometer + **LiDAR (TFmini)** + **radar (HLK-LD2450)** |
| Phase 1 autonomy | **Estabilización + RTH** |
| Phase 2 autonomy | Vuelo autónomo con cámara + LiDAR + radar |
| Propulsion | **Eléctrico + ESC (throttle PWM)** |

---

## Folder structure

```
esp32-airplane/
├── README.md                     # you are here
├── CHECKLIST.md                  # ★ iterative definition checklist
├── docs/
│   └── algorithm-mapping.md      # AR.Drone → wing algorithm transfer matrix
└── firmware/
    ├── Makefile                  # `make test` → 111 host unit tests
    ├── tests/
    │   └── test_wing_core.cpp    # J2: mixing, envelope, L1, RTH, NMEA, LD2450, AHRS, failsafe
    └── esp32_airplane/
        ├── esp32_airplane.ino    # entry point: setup/loop, fixed-rate slots, PWM
        ├── config.h              # all gains, limits, pin map (edit here first)
        ├── ahrs.{h,cpp}          # gyro+accel+mag fusion (complementary)
        ├── control.{h,cpp}       # ADRC/PID loops + envelope protection + b0 ∝ q
        ├── mixing.{h,cpp}        # elevon mixing + adverse-yaw differential
        ├── guidance.{h,cpp}      # L1 path following, wing-specific RTH, geofence
        ├── gps_nav.{h,cpp}       # NMEA parser + Haversine (ported from AR.Drone)
        ├── failsafe.{h,cpp}      # failsafe FSM + preflight gate
        └── drivers/
            ├── imu.{h,cpp}       # STUB: SPI/I2C IMU driver (C1)
            ├── sensors.{h,cpp}   # STUBs: BMP388, QMC5883, GPS UART, TFmini (C2/C3/C4/C6)
            └── ld2450.{h,cpp}    # REAL: HLK-LD2450 frame parser + track matcher (C7)
```

*The `drivers/` stubs (except `ld2450`) emit synthetic signals so the
firmware compiles and the control/guidance stack can be exercised before the
real sensor drivers land. Wire real drivers before §C of the checklist is
marked done.*

### Run the tests

```bash
cd esp32-airplane/firmware && make test      # 111 assertions
# or from the repo-wide suite:
cd tools/simulator && make check             # quad (90) + wing (111)
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
3. **Missing pin map** (B2) and **brown-out under throttle** (B4).
4. **600 → 240 MHz, no double FPU** — `float` only in the hot loop.

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
