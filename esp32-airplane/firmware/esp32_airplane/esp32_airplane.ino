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
#include "settings.h"
#include "rc_input.h"
#include "wifi_config.h"
#include "drivers/imu.h"
#include "drivers/sensors.h"
#include "drivers/ld2450.h"

#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// Shared state (single writer per field; guarded where crossed)
// ---------------------------------------------------------------------------
static Settings     g_set;            // persisted config (NVS-backed)
static RcFrame      g_rc_frame;
static RcInput      g_rc;
static uint32_t     g_rc_last_rx_ms = 0;
static bool         g_rc_ok = false;

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
// RC input (checklist H2) — SBUS or Spektrum satellite, selected at runtime
// through Settings.rc.proto and persisted across power-off.
// ---------------------------------------------------------------------------
static uint8_t s_rc_buf[64];
static uint8_t s_rc_len = 0;

static void rc_uart_init() {
    rc_input_defaults(g_rc);
    if (g_set.rc.proto == RC_PROTO_NONE) {
        Serial.println(F("[rc] disabled (RC_PROTO_NONE)"));
        return;
    }
    bool spek = (g_set.rc.proto == RC_PROTO_SPEKTRUM);
    uint32_t baud = spek ? SPEKTRUM_BAUD : SBUS_BAUD;

    // SBUS: 100000 8E2 with an inverted line. Spektrum: 115200 8N1.
    if (spek) {
        Serial2.begin(baud, SERIAL_8N1, PIN_RC_RX, PIN_RC_TX, false);
    } else {
        Serial2.begin(baud, SERIAL_8E2, PIN_RC_RX, PIN_RC_TX,
                      g_set.rc.sbus_inverted != 0);
    }

    // Resynchronise: drain until the line has been idle for 2 ms. Spektrum
    // frames are 16 bytes back-to-back with a ~7 ms gap, so a 2 ms quiet
    // window always lands between frames — without this the first byte we
    // ever see could be mid-frame and every channel would stay misaligned.
    uint32_t t0 = millis();
    while (millis() - t0 < 500) {
        while (Serial2.available() > 0) Serial2.read();
        delay(2);
        if (Serial2.available() == 0) break;
    }
    s_rc_len = 0;
    Serial.printf("[rc] %s on UART2 rx=%d tx=%d baud=%lu inverted=%d\n",
                  spek ? "Spektrum" : "SBUS", PIN_RC_RX, PIN_RC_TX,
                  (unsigned long)baud, g_set.rc.sbus_inverted);
}

static void rc_service(uint32_t now_ms) {
    if (g_set.rc.proto == RC_PROTO_NONE) {
        g_rc_ok = false;
        return;
    }
    const bool     spek = (g_set.rc.proto == RC_PROTO_SPEKTRUM);
    const uint8_t  flen = spek ? (uint8_t)SPEKTRUM_FRAME_LEN
                               : (uint8_t)SBUS_FRAME_LEN;

    int avail = Serial2.available();
    if (avail > 0) {
        int want = avail;
        if (want > (int)(sizeof(s_rc_buf) - s_rc_len))
            want = (int)sizeof(s_rc_buf) - s_rc_len;
        int got = Serial2.readBytes((char*)s_rc_buf + s_rc_len, want);
        if (got > 0) s_rc_len = (uint8_t)(s_rc_len + got);

        while (s_rc_len >= flen) {
            bool ok = spek ? spektrum_decode(s_rc_buf, g_rc_frame)
                           : sbus_decode(s_rc_buf, g_rc_frame);
            if (ok) {
                rc_map(g_rc_frame, g_set.rc, now_ms, g_rc);
                g_rc_last_rx_ms = now_ms;
                g_rc_ok = !g_rc.failsafe && !g_rc.frame_lost;
            } else {
                // SBUS footer/header mismatch => we started mid-frame.
                // Drop one byte and hunt for the real header.
                memmove(s_rc_buf, s_rc_buf + 1, s_rc_len - 1);
                s_rc_len--;
                continue;
            }
            memmove(s_rc_buf, s_rc_buf + flen, s_rc_len - flen);
            s_rc_len = (uint8_t)(s_rc_len - flen);
        }
    }

    // Loss timeout: a stopped stream must become "not ok" even though no new
    // (bad) frame ever arrives to say so.
    if ((int32_t)(now_ms - g_rc_last_rx_ms) > (int32_t)g_set.rc.loss_timeout_ms)
        g_rc_ok = false;
}

// Re-apply cached, non-Settings state after the portal saves. Called by
// wifi_config so a trim/servo change lands on the next control tick.
static void on_settings_changed(Settings& s) {
    mixing_load(&s.mix);
    // RC protocol or UART settings changed -> reopen the port.
    rc_uart_init();
}

// ---------------------------------------------------------------------------
// Telemetry (checklist I1/I2) — CSV line on USB serial for now.
// TODO: replace with MAVLink when I1 is decided; S.Port fields go through
//       docs/rc-and-telemetry.md once the RX side is validated.
// ---------------------------------------------------------------------------
static void telemetry_print(uint32_t now) {
    Serial.printf("%lu,phase=%s,armed=%d,roll=%.2f,pitch=%.2f,yaw=%.2f,"
                  "v=%.1f,phi_cmd=%.2f,thr=%.2f,b0r=%.1f,f=%.2f,"
                  "env[stall=%d bank=%d vne=%d g=%d],"
                  "fs=%s,rc=%d,rcage=%lu,rssi=%d,board=\"%s\","
                  "gps=%d,hdop=%.1f,sv=%d,trk=%.0f,dHome=%.0f,"
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
                  g_rc_ok ? 1 : 0,
                  (unsigned long)(now - g_rc_last_rx_ms),
                  g_rc_frame.link_quality,
                  sensors_board_name(),
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
    Serial.printf("\n== esp32_airplane : flying wing (phase 1: stabilize + RTH) ==\n");
    Serial.printf("board IMU: %s\n", IMU_BOARD_NAME);

    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, LOW);

    // Persisted configuration first: pins, RC protocol and mix all depend on
    // it. NVS miss => defaults (settings_load fills them in).
    if (!settings_load(g_set)) Serial.println(F("[cfg] no saved settings, using defaults"));
    mixing_load(&g_set.mix);

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    pwm_init();
    rc_uart_init();

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

    Serial.printf("init: imu=%d baro=%d mag=%d gps=%d@%u lidar=%d rc=%s\n",
                  imu_healthy(), baro_healthy(), mag_healthy(),
                  gps_healthy(), (unsigned)gps_baud(), lidar_healthy(),
                  g_set.rc.proto == RC_PROTO_NONE ? "off" : "on");
    if (!ok) Serial.println(F("!! sensor init incomplete — preflight will block arming"));

    // Configuration portal (I5). Runs for the whole session; changes are
    // written to NVS immediately so they survive a power cycle.
    wifi_config_begin(g_set, on_settings_changed);
    Serial.printf("[wifi] portal: SSID '%s' port %u\n",
                  wifi_config_ssid(), g_set.wifi.http_port);

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
    static float    gps_age_s  = 0;
    static bool     stall_latch = false;

    uint32_t now_us = micros();
    uint32_t elapsed_us = now_us - t_prev;
    uint32_t now_ms = millis();

    // ---- RC ingest ---------------------------------------------------------
    // SBUS frames arrive every ~9 ms, Spektrum every ~9 ms; the 400 Hz control
    // slot drains them well within one period without blocking.
    if (elapsed_us >= (1000000UL / DT_RATE_HZ)) rc_service(now_ms);

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
        // Phase 1 flies on RC only (H2): the sticks are the outer-loop
        // setpoints whenever guidance is not in charge.
        g_cmd.pitch    = g_rc.pitch;
        g_cmd.roll     = g_rc.roll;
        g_cmd.throttle = g_rc.throttle;
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
        float rc_age = g_rc_ok ? 0.0f : (float)(now_ms - g_rc_last_rx_ms);
        g_fs = failsafe_update(FsEvent::NONE,
                               /*rc_ok=*/g_rc_ok || g_set.rc.proto == RC_PROTO_NONE,
                               rc_age,
                               g_fix.valid, gps_age_s,
                               VBAT_WARN_CELL + 0.15f,   // TODO real ADC (B4/H3)
                               imu_healthy(), g_dbg.env_stall,
                               fence, now_ms);
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
                               g_rc_ok || g_set.rc.proto == RC_PROTO_NONE,
                               !g_nav.fence_breach);
        // Arming gate (H1/H6): preflight must pass AND the physical arm
        // switch must be asserted. With RC_PROTO_NONE (bench, no receiver) the
        // switch requirement is dropped so the stack can be exercised.
        bool arm_ok = (g_set.rc.proto == RC_PROTO_NONE) ? true : g_rc.arm;
        bool was_armed = g_armed;
        g_armed = g_pf.all_ok && arm_ok;
        if (g_armed && !was_armed) guidance_arm();
        if (!g_armed && was_armed) { guidance_disarm(); failsafe_init(); }

        // RTH switch / mode switch (H2) — pilot can always grab RTH back.
        if (g_armed && ph == Phase::ARMED_MANUAL &&
            (g_rc.rth || g_rc.mode == 1)) {
            guidance_trigger_rth();
        }

        // Radar (C7) — parse any pending bytes.
        // TODO: read UART2 bytes into Ld2450Stream::feed().
    }

    // ---- telemetry slot ----------------------------------------------------
    if (now_us - t_telem >= (1000000UL / 10)) {   // 10 Hz
        t_telem = now_us;
        telemetry_print(now_ms);
        digitalWrite(PIN_STATUS_LED, g_armed ? HIGH :
                     (millis() / 250) & 1);        // blink = not armed
    }

    // ---- configuration portal (I5) ----------------------------------------
    wifi_config_loop();
}
