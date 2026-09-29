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
#include "logger.h"
#include "mavlink.h"
#include "drivers/imu.h"
#include "drivers/sensors.h"
#include "drivers/ld2450.h"

#include <Arduino.h>
#include <Wire.h>
#include <WiFiUdp.h>

// ---------------------------------------------------------------------------
// Shared state (single writer per field; guarded where crossed)
// ---------------------------------------------------------------------------
static Settings     g_set;            // persisted config (NVS-backed)
static RcFrame      g_rc_frame;
static RcInput      g_rc;
static uint32_t     g_rc_last_rx_ms = 0;
static bool         g_rc_ok = false;

// MAVLink (I1) borrows one of the three UARTs when enabled in the portal;
// these report whether it currently owns a given one. UART0 = USB (shared
// with the CSV logger), UART1 = GPS, UART2 = RC receiver.
static bool mav_owns_uart(uint8_t u) {
    return g_set.mav.enable && g_set.mav.uart == u;
}
static bool mav_on_usb() { return mav_owns_uart(0); }

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
static float        s_home_alt_m = 0;  // latched with the home fix (G4), for
                                       // GLOBAL_POSITION_INT relative_alt
static bool         s_gps_up = false;  // gps_init() ran and the GPS is alive

// Last sensor samples kept for the CSV logger (I3): the IMU runs at
// DT_IMU_HZ but the log ticks at LOG_RATE_HZ, so the log reads the newest.
static Vec3  g_accel = {0, 0, 0};       // g
static Vec3  g_gyro  = {0, 0, 0};       // rad/s
static float g_baro_pa = 0, g_baro_c = 0;

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
    // UART2 taken by MAVLink (portal) => no receiver in sight; report loss
    // instead of stealing the telemetry bytes.
    if (g_set.rc.proto == RC_PROTO_NONE || mav_owns_uart(2)) {
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

// ---------------------------------------------------------------------------
// MAVLink telemetry (checklist I1) — v2 frames out, REQUEST_DATA_STREAM in,
// on the UART picked in the portal. docs/mavlink-telemetry.md has the wire
// rules, the message table and the UART conflict matrix.
//
//   UART0 (USB): shares the port with the CSV logger -> CSV moves to UDP
//   UART1       : GPS is disabled for the session (gps_init/poll skipped)
//   UART2       : RC is disabled for the session (rc_service skipped, which
//                 also means preflight fails and the arm switch goes dead)
// ---------------------------------------------------------------------------
static MavlinkRx       s_mav_rx;
static MavlinkSchedule s_mav_sched;
static uint8_t         s_mav_seq = 0;
static bool            s_mav_up  = false;
static Stream*         s_mav_port = nullptr;

static void mav_uart_init() {
    s_mav_up   = false;
    s_mav_port = nullptr;
    if (!g_set.mav.enable) return;

    if (g_set.mav.uart == 0) {
        Serial.begin(g_set.mav.baud);
        s_mav_port = &Serial;
        Serial.printf("[mav] UART0 @ %lu — CSV serial off, use capture.py --udp\n",
                      (unsigned long)g_set.mav.baud);
    } else {
        if (g_set.mav.tx_pin < 0 || g_set.mav.rx_pin < 0) {
            Serial.println(F("[mav] UART1/2 needs TX+RX pins — set them in the portal"));
            return;
        }
        HardwareSerial* hs = (g_set.mav.uart == 1) ? &Serial1 : &Serial2;
        hs->begin(g_set.mav.baud, SERIAL_8N1,
                  g_set.mav.rx_pin, g_set.mav.tx_pin, false);
        s_mav_port = hs;
        Serial.printf("[mav] UART%u @ %lu rx=%d tx=%d — %s disabled this session\n",
                      g_set.mav.uart, (unsigned long)g_set.mav.baud,
                      g_set.mav.rx_pin, g_set.mav.tx_pin,
                      g_set.mav.uart == 1 ? "GPS" : "RC");
    }

    mavlink_rx_init(&s_mav_rx);
    mav_schedule_init(&s_mav_sched, g_set.mav.rate_hz);
    s_mav_up = true;

    // First frame: a STATUSTEXT so the GCS console shows the link came up.
    uint8_t pl[MAV_PAYLOAD_MAX], fr[MAV_FRAME_MAX];
    uint8_t n = mav_pack_statustext(pl, MAV_SEVERITY_INFO, "mavlink up");
    size_t w = mavlink_encode(s_mav_seq++, g_set.mav.sysid, 1,
                              MAV_MSG_STATUSTEXT, pl, n, MAV_CRC_STATUSTEXT,
                              fr, sizeof(fr));
    if (w && s_mav_port) s_mav_port->write(fr, w);
}

static void mav_service(uint32_t now_ms) {
    if (!s_mav_up || !s_mav_port) return;
    Stream* p = s_mav_port;

    // ---- inbound: REQUEST_DATA_STREAM steers the fast rate ----------------
    int guard = 0;
    while (p->available() > 0 && guard++ < 512) {
        MavlinkRxMsg m;
        if (mavlink_rx_feed(&s_mav_rx, (uint8_t)p->read(), &m)) {
            uint8_t rate = 0;
            if (mavlink_rds_rate(&m, g_set.mav.sysid, &rate) &&
                rate != s_mav_sched.rate_hz) {
                mav_schedule_set_rate(&s_mav_sched, rate);
                Serial.printf("[mav] GCS set fast rate %u Hz\n", rate);
            }
        }
    }

    // ---- outbound ----------------------------------------------------------
    uint32_t due = mav_schedule_due(&s_mav_sched, now_ms);
    if (!due) return;

    uint8_t pl[MAV_PAYLOAD_MAX];
    uint8_t fr[MAV_FRAME_MAX];
    const uint8_t sysid = g_set.mav.sysid;
    auto send = [&](uint8_t msgid, uint8_t crc_extra, uint8_t plen) {
        size_t n = mavlink_encode(s_mav_seq++, sysid, 1, msgid, pl, plen,
                                  crc_extra, fr, sizeof(fr));
        if (n) p->write(fr, n);
    };

    if (due & MAV_DUE_HEARTBEAT) {
        uint8_t base = MAV_MODE_FLAG_CUSTOM_MODE;
        if (g_armed) base |= MAV_MODE_FLAG_SAFETY_ARMED;
        uint8_t n = mav_pack_heartbeat(pl, MAV_TYPE_FIXED_WING,
                                       MAV_AUTOPILOT_GENERIC, base,
                                       (uint32_t)guidance_phase(),
                                       g_armed ? MAV_STATE_ACTIVE
                                               : MAV_STATE_STANDBY);
        send(MAV_MSG_HEARTBEAT, MAV_CRC_HEARTBEAT, n);
    }
    if (due & MAV_DUE_ATTITUDE) {
        uint8_t n = mav_pack_attitude(pl, now_ms,
                                      g_att.roll, g_att.pitch, g_att.yaw,
                                      g_att.gx, g_att.gy, g_att.gz);
        send(MAV_MSG_ATTITUDE, MAV_CRC_ATTITUDE, n);
    }
    if (due & MAV_DUE_POSITION) {
        const float trk = g_fix.course_deg * (float)M_PI / 180.0f;
        const float gs  = g_fix.ground_mps;
        const int16_t vx = (int16_t)(gs * cosf(trk) * 100.0f);   // cm/s north
        const int16_t vy = (int16_t)(gs * sinf(trk) * 100.0f);   // cm/s east
        const uint16_t hdg = g_fix.valid
            ? (uint16_t)(g_fix.course_deg * 100.0f) : MAV_HDG_UNKNOWN;
        uint8_t n = mav_pack_global_position_int(
            pl, now_ms,
            (int32_t)(g_fix.lat * 1e7), (int32_t)(g_fix.lon * 1e7),
            (int32_t)(g_fix.alt_m * 1000.0f),
            (int32_t)((g_fix.alt_m - s_home_alt_m) * 1000.0f),
            vx, vy,
            0,                        // vz: no vertical-speed estimate (yet)
            hdg);
        send(MAV_MSG_GLOBAL_POSITION_INT, MAV_CRC_GLOBAL_POSITION_INT, n);
    }
    if (due & MAV_DUE_GPS) {
        const uint8_t fix_type = !g_fix.valid ? MAV_GPS_FIX_NONE
                                 : (g_fix.fix_quality >= 2 ? MAV_GPS_FIX_DGPS
                                                           : MAV_GPS_FIX_3D);
        const uint16_t eph = (g_fix.hdop <= 655.0f)
            ? (uint16_t)(g_fix.hdop * 100.0f) : (uint16_t)MAV_HDG_UNKNOWN;
        const uint16_t cog = g_fix.valid
            ? (uint16_t)(g_fix.course_deg * 100.0f) : (uint16_t)MAV_HDG_UNKNOWN;
        uint8_t n = mav_pack_gps_raw_int(
            pl, (uint64_t)now_ms * 1000ull, fix_type,
            (int32_t)(g_fix.lat * 1e7), (int32_t)(g_fix.lon * 1e7),
            (int32_t)(g_fix.alt_m * 1000.0f),
            eph, (uint16_t)MAV_HDG_UNKNOWN,      // epv unknown without a fix calc
            (uint16_t)(g_fix.ground_mps * 100.0f), cog, g_fix.num_sv);
        send(MAV_MSG_GPS_RAW_INT, MAV_CRC_GPS_RAW_INT, n);
    }
    if (due & MAV_DUE_SYS) {
        uint32_t sens = MAV_SYS_3D_GYRO | MAV_SYS_3D_ACCEL | MAV_SYS_3D_MAG |
                        MAV_SYS_ABSOLUTE_PRESSURE;
        if (gps_healthy())       sens |= MAV_SYS_GPS;
        if (g_set.rc.proto != RC_PROTO_NONE) sens |= MAV_SYS_RC_RECEIVER;
        uint32_t health = 0;
        if (imu_healthy())   health |= MAV_SYS_3D_GYRO | MAV_SYS_3D_ACCEL;
        if (mag_healthy())   health |= MAV_SYS_3D_MAG;
        if (baro_healthy())  health |= MAV_SYS_ABSOLUTE_PRESSURE;
        if (gps_healthy())   health |= MAV_SYS_GPS;
        if (g_rc_ok)         health |= MAV_SYS_RC_RECEIVER;
        uint8_t n = mav_pack_sys_status(pl, sens, sens, health,
                                        0,        // CPU load unknown
                                        0,        // battery ADC pending (B4/H3)
                                        -1, -1);  // current / percentage unknown
        send(MAV_MSG_SYS_STATUS, MAV_CRC_SYS_STATUS, n);
    }
    if (due & MAV_DUE_SERVO) {
        uint8_t n = mav_pack_servo_output_raw(pl, (uint32_t)now_ms * 1000u, 0,
                                              g_out.left_us, g_out.right_us,
                                              g_out.throttle_us,
                                              0, 0, 0, 0, 0);
        send(MAV_MSG_SERVO_OUTPUT_RAW, MAV_CRC_SERVO_OUTPUT_RAW, n);
    }
    if (due & MAV_DUE_VFR) {
        float thr = g_dbg.throttle_out;
        if (thr < 0.0f) thr = 0.0f;
        if (thr > 1.0f) thr = 1.0f;
        uint8_t n = mav_pack_vfr_hud(pl,
                                     0.0f,                 // no pitot yet (C5)
                                     g_fix.ground_mps,
                                     g_fix.valid ? (int16_t)g_fix.course_deg : 0,
                                     (uint16_t)(thr * 100.0f + 0.5f),
                                     g_fix.alt_m,
                                     0.0f);                // climb: baro later
        send(MAV_MSG_VFR_HUD, MAV_CRC_VFR_HUD, n);
    }
}

// Re-apply cached, non-Settings state after the portal saves. Called by
// wifi_config so a trim/servo change lands on the next control tick.
static void on_settings_changed(Settings& s) {
    mixing_load(&s.mix);
    // RC protocol or UART settings changed -> reopen the port, unless
    // MAVLink is holding UART2 this session.
    if (!mav_owns_uart(2)) rc_uart_init();
    // GPS: bring it up if it was skipped at boot because MAVLink had UART1,
    // or mark it down if MAVLink is about to take the port.
    if (mav_owns_uart(1)) {
        s_gps_up = false;
    } else if (!s_gps_up) {
        gps_init();
        s_gps_up = gps_healthy();
    }
    mav_uart_init();                 // runs last: its begin() owns the pins
}

// ---------------------------------------------------------------------------
// Real-time logger (checklist I3) — CSV over USB serial, plus UDP to the
// first ground tool that sends a datagram to LOG_UDP_PORT (a "HELLO" from
// tools/wing_logger/capture.py). Format is host-tested in logger.cpp; the
// schema and the analysis workflow live in docs/test-campaign.md.
// ---------------------------------------------------------------------------
static WiFiUDP    s_udp;
static IPAddress  s_peer_ip;
static uint16_t   s_peer_port = 0;       // 0 => nobody subscribed yet
static bool       s_udp_up = false;

static void log_udp_begin() {
    s_udp_up = s_udp.begin(LOG_UDP_PORT);
}

static void log_udp_service() {
    if (!s_udp_up) return;
    if (s_udp.parsePacket() > 0) {       // any datagram subscribes the sender
        s_peer_ip   = s_udp.remoteIP();
        s_peer_port = s_udp.remotePort();
        s_udp.flush();                   // payload itself is irrelevant
    }
}

static void log_emit(uint32_t now_ms) {
    LogSample s;
    s.t_ms   = now_ms;
    s.mode   = g_set.mix.passthrough;
    s.armed  = g_armed;
    s.rc_ok  = g_rc_ok;
    s.phase  = (uint8_t)guidance_phase();
    s.fs_evt = (uint8_t)g_fs.last_event;
    s.env    = (g_dbg.env_stall ? LOG_ENV_STALL : 0) |
               (g_dbg.env_bank  ? LOG_ENV_BANK  : 0) |
               (g_dbg.env_vne   ? LOG_ENV_VNE   : 0) |
               (g_dbg.env_g     ? LOG_ENV_G     : 0);
    s.wind   = LOG_WIND_UNKNOWN;         // F6 classifier lands later

    s.rc_p = g_rc.pitch;  s.rc_r = g_rc.roll;
    s.rc_t = g_rc.throttle; s.rc_y = g_rc.yaw;
    s.raw_p = g_rc_frame.raw[g_set.rc.ch_pitch];
    s.raw_r = g_rc_frame.raw[g_set.rc.ch_roll];
    s.raw_t = g_rc_frame.raw[g_set.rc.ch_throttle];
    s.raw_y = g_rc_frame.raw[g_set.rc.ch_yaw];

    s.ax = g_accel.x; s.ay = g_accel.y; s.az = g_accel.z;
    s.gx = g_gyro.x;  s.gy = g_gyro.y;  s.gz = g_gyro.z;
    s.baro_pa = g_baro_pa; s.baro_c = g_baro_c;

    s.roll  = g_att.roll  * 57.2957795f;
    s.pitch = g_att.pitch * 57.2957795f;
    s.yaw   = g_att.yaw   * 57.2957795f;

    s.fix     = g_fix.valid ? 1 : 0;
    s.sv      = g_fix.num_sv;
    s.hdop    = g_fix.hdop;
    s.gps_v   = g_fix.ground_mps;
    s.gps_alt = g_fix.alt_m;
    s.gps_trk = g_fix.course_deg;
    s.vest    = g_dbg.airspeed_est;
    s.phi_cmd = g_dbg.phi_cmd_l;
    s.l_us    = g_out.left_us;
    s.r_us    = g_out.right_us;
    s.thr_out = g_dbg.throttle_out;
    s.overruns = g_loop_overruns;

    char line[LOGGER_MAX_LINE];
    if (!logger_format(line, sizeof(line), s)) return;

    // Sink 1: USB serial — off while MAVLink speaks on UART0 (mixed binary
    // and CSV would corrupt both); UDP keeps streaming either way.
    if (!mav_on_usb()) Serial.print(line);
    if (s_udp_up && s_peer_port) {       // sink 2: UDP (after HELLO)
        s_udp.beginPacket(s_peer_ip, s_peer_port);
        s_udp.print(line);
        s_udp.endPacket();
    }
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
    if (mav_owns_uart(2)) {
        rc_input_defaults(g_rc);
        Serial.println(F("[rc] skipped — UART2 owned by MAVLink"));
    } else {
        rc_uart_init();
    }

    ahrs_init();
    control_init();
    guidance_init();
    failsafe_init();

    bool ok = true;
    ok &= imu_init();
    ok &= imu_calibrate_rest();     // C1 gyro bias at rest
    ok &= baro_init();
    ok &= mag_init();
    if (mav_owns_uart(1)) {
        Serial.println(F("[mav] GPS port borrowed — gps_init skipped"));
    } else {
        ok &= gps_init();
        s_gps_up = gps_healthy();
    }
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

    // MAVLink telemetry (I1): open the configured UART after every other
    // user of it has run, so its begin() wins the pins.
    mav_uart_init();

    // Real-time logger (I3): CSV schema marker for the ground tool, and the
    // UDP subscriber port (capture.py sends a HELLO to start the stream).
    if (!mav_on_usb()) Serial.println(logger_csv_header());
    log_udp_begin();

    // Arming gate (H6): we will not arm until AHRS converges + preflight passes.
    Serial.println(F("waiting for AHRS convergence..."));
}

// ---------------------------------------------------------------------------
// loop() — fixed-rate scheduler.  Core 1 does I/O + nav; control math runs
// at DT_RATE_HZ using micros()-driven slots (E3: no delay() in the hot path).
// ---------------------------------------------------------------------------
void loop() {
    static uint32_t t_prev     = micros();
    static uint32_t t_log      = 0;
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
        // Stage 1 (etapa 1, mix.passthrough): the pilot flies the aircraft —
        // sticks go straight to the elevon mixer and the TX curves are the
        // only controller. Nothing (envelope, RTH, ESO) may touch the surfaces
        // until the bench campaign (docs/test-campaign.md T0) passes.
        g_dbg.airspeed_est = v;
        if (g_set.mix.passthrough) {
            g_dbg.ctrl_pitch_out = g_rc.pitch;
            g_dbg.ctrl_roll_out  = g_rc.roll;
            g_dbg.throttle_out   = g_rc.throttle;
            g_dbg.phi_cmd_l      = 0.0f;
            g_out = mix_elevons(g_rc.pitch, g_rc.roll, g_rc.throttle);
        } else {
            control_update(g_att, g_cmd, v, dt, g_dbg);

            // Elevon mixing + adverse-yaw differential (D1/D2).
            g_out = mix_elevons(g_dbg.ctrl_pitch_out, g_dbg.ctrl_roll_out,
                                g_dbg.throttle_out);
        }
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
        if (!g_set.mix.passthrough && g_fs.rth_requested &&
            ph == Phase::ARMED_MANUAL) {
            guidance_trigger_rth();
        }
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
            g_accel = a;                      // newest sample for the logger
            g_gyro  = g;
            float dt = 1.0f / (float)DT_IMU_HZ;
            g_att = ahrs_update(a, g, m, mag_ok, dt);
        }
        if (!baro_healthy()) { /* H4: baro fallback */ }
    }

    // ---- nav / guidance slot (DT_NAV_HZ) ----------------------------------
    if (now_us - t_nav >= (1000000UL / DT_NAV_HZ)) {
        t_nav = now_us;
        if (!mav_owns_uart(1)) gps_poll(g_fix);
        if (g_fix.valid) gps_age_s = 0; else gps_age_s += 1.0f / DT_NAV_HZ;

        // First good fix becomes home (G4). TODO: latch on arming, not first fix.
        static bool home_set = false;
        if (!home_set && gps_trustworthy(g_fix)) {
            guidance_set_home(g_fix.lat, g_fix.lon);
            s_home_alt_m = g_fix.alt_m;      // for relative_alt in telemetry
            home_set = true;
        }

        float pa, tc;
        baro_read(pa, tc);
        g_baro_pa = pa;                       // for the CSV logger (I3)
        g_baro_c  = tc;

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
        // In passthrough (stage 1) guidance never drives the surfaces, so
        // don't even enter the phase: the mode switch stays inert.
        if (g_armed && !g_set.mix.passthrough &&
            ph == Phase::ARMED_MANUAL &&
            (g_rc.rth || g_rc.mode == 1)) {
            guidance_trigger_rth();
        }

        // Radar (C7) — parse any pending bytes.
        // TODO: read UART2 bytes into Ld2450Stream::feed().
    }

    // ---- real-time log slot (I3) + MAVLink (I1) ---------------------------
    if (now_us - t_log >= (1000000UL / LOG_RATE_HZ)) {
        t_log = now_us;
        log_udp_service();               // pick up / keep a ground subscriber
        log_emit(now_ms);
        mav_service(now_ms);             // due telemetry frames + RX pump
        digitalWrite(PIN_STATUS_LED, g_armed ? HIGH :
                     (millis() / 250) & 1);        // blink = not armed
    }

    // ---- configuration portal (I5) ----------------------------------------
    wifi_config_loop();
}
