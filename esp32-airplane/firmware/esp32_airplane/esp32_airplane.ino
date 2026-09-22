// ============================================================================
// esp32_airplane — autonomous flying wing on ESP32
//
// Phase 1 goal (locked baseline): stabilization + RTH.
// Reuses AR.Drone algorithms where they transfer; see
//   ../../../docs/algorithm-mapping.md   (transfer matrix)
//   ../../../CHECKLIST.md                (iterative definition checklist)
//
// Architecture (checklist E2/E3):
//   Core 0 — 400 Hz: IMU -> AHRS -> control -> envelope -> elevon mix -> PWM
//   Core 1 — 25 Hz : GPS/LiDAR/LD2450 parse, guidance (L1/RTH), telemetry, log
//
// NOTE: this is a bench-testable skeleton. Drivers under drivers/ are stubs;
//       real sensor drivers must land before checklist §C can be closed.
// ============================================================================

#include "config.h"
#include "ahrs.h"
#include "control.h"
#include "mixing.h"
#include "guidance.h"
#include "gps_nav.h"
#include "failsafe.h"
#include "drivers/imu.h"
#include "drivers/sensors.h"
#include "drivers/ld2450.h"

#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// Shared state (single writer per field; guarded where crossed)
// ---------------------------------------------------------------------------
static Attitude     g_att;
static GpsFix       g_fix;
static ControlCmd   g_cmd;
static ControlDebug g_dbg;
static NavState     g_nav;
static ElevonOut    g_out;
static FailsafeStatus g_fs;
static PreflightReport g_pf;
static Ld2450Frame  g_radar;
static bool         g_armed = false;
static uint32_t     g_loop_overruns = 0;

// Airspeed estimate: start with GPS ground speed (checklist C5 method 1).
// TODO: add pitot + wind triangle (N2/N3) before closing C5.
static float estimate_airspeed(const GpsFix& fix, const Attitude& att) {
    (void)att;
    float v = fix.ground_mps;
    if (v < 1.0f) v = V_CRUISE_MPS;   // no GPS speed yet -> assume cruise
    if (v > V_NE_MPS) v = V_NE_MPS;
    return v;
}

// ---------------------------------------------------------------------------
// PWM helpers — ESC + servos (checklist B3)
// ---------------------------------------------------------------------------
static inline uint32_t us_to_duty(uint16_t us) {
    // 16-bit @ 50 Hz: 20 ms period -> 65536 counts; 1 count = 0.305 us
    return ((uint32_t)us * ((1UL << PWM_RES_BITS) * PWM_FREQ_HZ)) / 1000000UL;
}

static void pwm_init() {
    // Arduino-ESP32 core 3.x API. For core 2.x use:
    //   ledcSetup(ch, PWM_FREQ_HZ, PWM_RES_BITS); ledcAttachPin(pin, ch);
    ledcAttach(PIN_ELEVON_L,  PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_ELEVON_R,  PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(PIN_ESC_THROTTLE, PWM_FREQ_HZ, PWM_RES_BITS);

    // Boot-safe: surfaces centered, throttle at arm/idle.
    ledcWrite(PIN_ELEVON_L,      us_to_duty(SERVO_CENTER_US));
    ledcWrite(PIN_ELEVON_R,      us_to_duty(SERVO_CENTER_US));
    ledcWrite(PIN_ESC_THROTTLE,  us_to_duty(ESC_ARM_US));
}

static void pwm_output(const ElevonOut& o) {
    ledcWrite(PIN_ELEVON_L,     us_to_duty(o.left_us));
    ledcWrite(PIN_ELEVON_R,     us_to_duty(o.right_us));
    ledcWrite(PIN_ESC_THROTTLE, us_to_duty(g_armed ? o.throttle_us : ESC_ARM_US));
}

// ---------------------------------------------------------------------------
// Telemetry (checklist I1/I2) — CSV line on USB serial for now.
// TODO: replace with MAVLink when I1 is decided.
// ---------------------------------------------------------------------------
static void telemetry_print(uint32_t now) {
    Serial.printf("%lu,phase=%s,armed=%d,roll=%.2f,pitch=%.2f,yaw=%.2f,"
                  "v=%.1f,phi_cmd=%.2f,thr=%.2f,b0r=%.1f,f=%.2f,"
                  "env[stall=%d bank=%d vne=%d g=%d],"
                  "fs=%s,gps=%d,hdop=%.1f,sv=%d,trk=%.0f,dHome=%.0f,"
                  "overruns=%lu\r\n",
                  (unsigned long)now,
                  phase_name(guidance_phase()), g_armed ? 1 : 0,
                  g_att.roll * 57.2958f, g_att.pitch * 57.2958f,
                  g_att.yaw * 57.2958f,
                  g_dbg.airspeed_est, g_dbg.phi_cmd_l, g_dbg.throttle_out,
                  g_dbg.b0_roll, g_dbg.f_roll,
                  g_dbg.env_stall ? 1 : 0, g_dbg.env_bank ? 1 : 0,
                  g_dbg.env_vne ? 1 : 0, g_dbg.env_g ? 1 : 0,
                  fs_event_name(g_fs.last_event),
                  g_fix.valid ? 1 : 0, g_fix.hdop, g_fix.num_sv,
                  g_fix.course_deg, g_nav.dist_home_m,
                  (unsigned long)g_loop_overruns);
}

// ---------------------------------------------------------------------------
// setup()
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println(F("\n== esp32_airplane : flying wing (phase 1: stabilize + RTH) =="));

    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    pwm_init();

    ahrs_init();
    control_init();
    guidance_init();
    failsafe_init();

    bool ok = true;
    ok &= imu_init();
    ok &= imu_calibrate_rest();     // C1 gyro bias at rest
    ok &= baro_init();
    ok &= mag_init();
    ok &= gps_init();
    ok &= lidar_init();

    Serial.printf("init: imu=%d baro=%d mag=%d gps=%d lidar=%d\n",
                  imu_healthy(), baro_healthy(), mag_healthy(),
                  gps_healthy(), lidar_healthy());
    if (!ok) Serial.println(F("!! sensor init incomplete — preflight will block arming"));

    // Arming gate (H6): we will not arm until AHRS converges + preflight passes.
    Serial.println(F("waiting for AHRS convergence..."));
}

// ---------------------------------------------------------------------------
// loop() — fixed-rate scheduler.  Core 1 does I/O + nav; control math runs
// at DT_RATE_HZ using micros()-driven slots (E3: no delay() in the hot path).
// ---------------------------------------------------------------------------
void loop() {
    static uint32_t t_prev     = micros();
    static uint32_t t_telem    = 0;
    static uint32_t t_nav      = 0;
    static uint32_t t_imu      = 0;
    static float    rc_age_ms  = 0;
    static float    gps_age_s  = 0;
    static bool     stall_latch = false;

    uint32_t now_us = micros();
    uint32_t elapsed_us = now_us - t_prev;

    // ---- 400 Hz control slot ---------------------------------------------
    if (elapsed_us >= (1000000UL / DT_RATE_HZ)) {
        float dt = elapsed_us / 1000000.0f;
        if (elapsed_us > (1000000UL / DT_RATE_HZ) * 3 / 2) g_loop_overruns++;
        t_prev = now_us;

        // Airspeed (C5): GPS-derived for now.
        float v = estimate_airspeed(g_fix, g_att);

        // Pilot/nav command selection: nav phase decides who drives.
        Phase ph = guidance_phase();
        g_cmd.nav_active = (ph != Phase::ARMED_MANUAL && ph != Phase::DISARMED);
        // TODO: read RC here (H2). Until then, neutral sticks => level flight.
        g_cmd.pitch = 0.0f;
        g_cmd.roll  = 0.0f;
        g_cmd.throttle = 0.35f;                  // fixed mid throttle for bench
        g_cmd.phi_cmd   = g_nav.phi_cmd;
        g_cmd.v_cmd     = g_nav.v_cmd;
        g_cmd.h_cmd     = g_nav.h_cmd;

        // Core control: AHRS -> loops -> envelope -> normalised cmds.
        control_update(g_att, g_cmd, v, dt, g_dbg);

        // Elevon mixing + adverse-yaw differential (D1/D2).
        g_out = mix_elevons(g_dbg.ctrl_pitch_out, g_dbg.ctrl_roll_out,
                            g_dbg.throttle_out);
        pwm_output(g_out);

        // ---- failsafe evaluation (H1) at control rate ---------------------
        bool fence = g_nav.fence_breach;
        g_fs = failsafe_update(FsEvent::NONE,
                               /*rc_ok=*/true, rc_age_ms,
                               g_fix.valid, gps_age_s,
                               VBAT_WARN_CELL + 0.15f,   // TODO real ADC (B4/H3)
                               imu_healthy(), g_dbg.env_stall,
                               fence, millis());
        if (g_fs.rth_requested && ph == Phase::ARMED_MANUAL) guidance_trigger_rth();
        if (g_fs.glide_requested && ph == Phase::ARMED_MANUAL) {
            // transition handled by guidance_update on next nav tick
        }
        stall_latch = g_dbg.env_stall;
        (void)stall_latch;
    }

    // ---- IMU + AHRS slot (DT_IMU_HZ) --------------------------------------
    if (now_us - t_imu >= (1000000UL / DT_IMU_HZ)) {
        t_imu = now_us;
        Vec3 a, g, m;
        bool imu_ok = imu_read(a, g);
        bool mag_ok = mag_read(m) && CFG_USE_MAG;
        if (imu_ok) {
            float dt = 1.0f / (float)DT_IMU_HZ;
            g_att = ahrs_update(a, g, m, mag_ok, dt);
        }
        if (!baro_healthy()) { /* H4: baro fallback */ }
    }

    // ---- nav / guidance slot (DT_NAV_HZ) ----------------------------------
    if (now_us - t_nav >= (1000000UL / DT_NAV_HZ)) {
        t_nav = now_us;
        gps_poll(g_fix);
        if (g_fix.valid) gps_age_s = 0; else gps_age_s += 1.0f / DT_NAV_HZ;

        // First good fix becomes home (G4). TODO: latch on arming, not first fix.
        static bool home_set = false;
        if (!home_set && gps_trustworthy(g_fix)) {
            guidance_set_home(g_fix.lat, g_fix.lon);
            home_set = true;
        }

        float pa, tc;
        baro_read(pa, tc);

        g_nav.cur_lat    = g_fix.lat;
        g_nav.cur_lon    = g_fix.lon;
        g_nav.alt_m      = g_fix.valid ? g_fix.alt_m : 0.0f;
        g_nav.ground_mps = g_fix.ground_mps;
        g_nav.course_deg = g_fix.course_deg;
        g_nav.gps_ok     = gps_trustworthy(g_fix);

        guidance_update(g_nav, 1.0f / (float)DT_NAV_HZ);

        // Preflight gate (H6) — reports why arming is blocked.
        g_pf = preflight_check(imu_healthy(), ahrs_converged(), mag_healthy(),
                               g_fix, baro_healthy(),
                               VBAT_WARN_CELL + 0.15f,
                               true, !g_nav.fence_breach);
        // Preflight gate arms the vehicle (H1/H6). TODO: also require a
        // physical pilot arm switch before this becomes autonomous.
        bool was_armed = g_armed;
        g_armed = g_pf.all_ok;
        if (g_armed && !was_armed) guidance_arm();
        if (!g_armed && was_armed) { guidance_disarm(); failsafe_init(); }

        // Radar (C7) — parse any pending bytes.
        // TODO: read UART2 bytes into Ld2450Stream::feed().
    }

    // ---- telemetry slot ----------------------------------------------------
    if (now_us - t_telem >= (1000000UL / 10)) {   // 10 Hz
        t_telem = now_us;
        telemetry_print(millis());
        digitalWrite(PIN_STATUS_LED, g_armed ? HIGH :
                     (millis() / 250) & 1);        // blink = not armed
    }
}
