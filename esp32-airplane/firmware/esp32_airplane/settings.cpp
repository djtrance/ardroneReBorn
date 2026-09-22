#include "settings.h"
#include "config.h"
#include <string.h>

// ---------------------------------------------------------------------------
// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) — small and well understood.
// ---------------------------------------------------------------------------
uint16_t settings_crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

// Offset of the trailing CRC field — everything before it is covered.
static const size_t kCrcOffset = offsetof(Settings, crc);

// ---------------------------------------------------------------------------
// Protocol-dependent raw stick ranges
//
//   SBUS        172 .. 1811 raw  (988 .. 2012 us), centre 992
//   Spektrum 1024    0 .. 1023  raw (903 .. 2095 us), centre 512
//   Spektrum 2048    0 .. 2047  raw (903 .. 2095 us), centre 1024
// The 1024/2048 distinction is discovered per-frame (Spektrum bit 0), so the
// decoder normalises to the 10-bit domain first and we keep 1024 here.
// ---------------------------------------------------------------------------
void rc_apply_proto_defaults(RcSettings& rc) {
    switch (rc.proto) {
        case RC_PROTO_SBUS:
            rc.min_raw      = 172;
            rc.mid_raw      = 992;
            rc.max_raw      = 1811;
            rc.deadband_raw = 8;
            break;
        case RC_PROTO_SPEKTRUM:
            rc.min_raw      = 0;
            rc.mid_raw      = 512;
            rc.max_raw      = 1023;
            rc.deadband_raw = 4;
            break;
        default:
            rc.min_raw      = 1000;
            rc.mid_raw      = 1500;
            rc.max_raw      = 2000;
            rc.deadband_raw = 0;
            break;
    }
    rc.loss_timeout_ms = RC_LOSS_TIMEOUT_MS;
}

// ---------------------------------------------------------------------------
// Defaults — the bench-safe starting point before anything is saved to NVS.
// ---------------------------------------------------------------------------
void settings_defaults(Settings& s) {
    memset(&s, 0, sizeof(s));
    s.magic   = SETTINGS_MAGIC;
    s.version = SETTINGS_VERSION;
    s.size    = (uint16_t)sizeof(Settings);

    // --- mix: mirror config.h ------------------------------------------------
    MixSettings& m = s.mix;
    m.elevon_l_trim_us  = ELEVON_L_TRIM_US;
    m.elevon_r_trim_us  = ELEVON_R_TRIM_US;
    m.pitch_span_us     = ELEVON_PITCH_MAX_US;
    m.roll_span_us      = ELEVON_ROLL_MAX_US;
    m.elevon_l_reverse  = 0;          // D1: validate on bench, flip if needed
    m.elevon_r_reverse  = 0;
    m.throttle_reverse  = 0;
    m.differential      = ELEVON_DIFFERENTIAL;
    m.esc_min_us        = ESC_MIN_US;
    m.esc_max_us        = ESC_MAX_US;
    m.esc_idle_us       = ESC_ARM_US;

    // --- RC ------------------------------------------------------------------
    RcSettings& rc = s.rc;
    rc.proto            = RC_PROTO_SBUS;
    rc.ch_pitch         = 1;          // AETR-ish; tune on bench (H2)
    rc.ch_roll          = 0;
    rc.ch_throttle      = 2;
    rc.ch_yaw           = 3;
    rc.ch_arm           = 4;          // aux1
    rc.ch_rth           = 5;          // aux2
    rc.ch_flightmode    = 6;          // aux3 (3-pos)
    rc.sbus_inverted    = 1;          // almost every SBUS source is inverted
    rc.arm_high_is_armed = 1;         // HIGH arms => switch DOWN disarms (safe)
    rc.expo              = 0.0f;
    rc_apply_proto_defaults(rc);

    // --- wifi ---------------------------------------------------------------
    WifiSettings& w = s.wifi;
    w.mode      = WIFI_MODE_AP;       // AP by default: always reachable
    w.ssid[0]   = 0;                  // filled by the portal on first boot
    w.pass[0]   = 0;
    strncpy(w.hostname, "wing-01", HOSTNAME_MAX);
    w.http_port = 80;

    // --- telemetry ----------------------------------------------------------
    TelemSettings& t = s.telem;
    t.enable     = 0;                 // off until the RX path is validated
    t.fast_only  = 0;
    t.rate_hz    = 50;
    t.fields_mask = TF_PHASE | TF_ENV_FLAGS | TF_AIRSPEED | TF_DIST_HOME |
                    TF_BEARING_HOME | TF_ALT_HOME | TF_GLIDE_MARGIN |
                    TF_ESO_ROLL | TF_ESO_PITCH | TF_B0 | TF_FENCE_MARGIN |
                    TF_PREFLIGHT | TF_LOOP_JITTER | TF_LINK_AGE | TF_IMU_MODEL;

    s.crc = settings_crc16((const uint8_t*)&s, kCrcOffset);
}

// ---------------------------------------------------------------------------
// Blob codec
// ---------------------------------------------------------------------------
bool settings_serialize(const Settings& s, uint8_t* buf, size_t cap, size_t* out_len) {
    if (!buf || !out_len) return false;
    if (cap < sizeof(Settings)) return false;

    Settings tmp = s;
    tmp.magic   = SETTINGS_MAGIC;
    tmp.version = SETTINGS_VERSION;
    tmp.size    = (uint16_t)sizeof(Settings);
    tmp.crc     = settings_crc16((const uint8_t*)&tmp, kCrcOffset);

    memcpy(buf, &tmp, sizeof(Settings));
    *out_len = sizeof(Settings);
    return true;
}

bool settings_deserialize(const uint8_t* buf, size_t len, Settings& out) {
    if (!buf || len != sizeof(Settings)) return false;

    Settings s;
    memcpy(&s, buf, sizeof(Settings));

    if (s.magic != SETTINGS_MAGIC) return false;
    if (s.version != SETTINGS_VERSION) return false;   // structure changed
    if (s.size != (uint16_t)sizeof(Settings)) return false;
    if (settings_crc16(buf, kCrcOffset) != s.crc) return false;

    out = s;
    settings_validate(out);      // clamp anything a corrupt-but-valid blob broke
    return true;
}

// ---------------------------------------------------------------------------
// Range clamping — a pilot fat-fingers a field in the portal, we refuse to
// fly with an absurd value rather than trusting it.
// ---------------------------------------------------------------------------
static int clamp_u16(uint16_t& v, uint16_t lo, uint16_t hi) {
    if (v < lo) { v = lo; return 1; }
    if (v > hi) { v = hi; return 1; }
    return 0;
}

static int clamp_f(float& v, float lo, float hi) {
    if (!(v >= lo)) { v = lo; return 1; }   // also catches NaN
    if (v > hi)     { v = hi; return 1; }
    return 0;
}

int settings_validate(Settings& s) {
    int fixed = 0;

    MixSettings& m = s.mix;
    fixed += clamp_u16(m.elevon_l_trim_us, SERVO_MIN_US, SERVO_MAX_US);
    fixed += clamp_u16(m.elevon_r_trim_us, SERVO_MIN_US, SERVO_MAX_US);
    fixed += clamp_u16(m.pitch_span_us, 50, 900);
    fixed += clamp_u16(m.roll_span_us, 50, 900);
    m.elevon_l_reverse = m.elevon_l_reverse ? 1 : 0;
    m.elevon_r_reverse = m.elevon_r_reverse ? 1 : 0;
    m.throttle_reverse = m.throttle_reverse ? 1 : 0;
    fixed += clamp_f(m.differential, 0.0f, 0.40f);
    fixed += clamp_u16(m.esc_min_us, 800, 1200);
    fixed += clamp_u16(m.esc_max_us, 1800, 2200);
    fixed += clamp_u16(m.esc_idle_us, m.esc_min_us, m.esc_max_us);

    RcSettings& rc = s.rc;
    if (rc.proto > RC_PROTO_SPEKTRUM) { rc.proto = RC_PROTO_NONE; fixed++; }
    if (rc.ch_pitch >= RC_MAX_CH) { rc.ch_pitch = 1; fixed++; }
    if (rc.ch_roll  >= RC_MAX_CH) { rc.ch_roll  = 0; fixed++; }
    if (rc.ch_throttle >= RC_MAX_CH) { rc.ch_throttle = 2; fixed++; }
    if (rc.ch_yaw   >= RC_MAX_CH) { rc.ch_yaw   = 3; fixed++; }
    if (rc.ch_arm    != 0xFF && rc.ch_arm    >= RC_MAX_CH) { rc.ch_arm = 0xFF; fixed++; }
    if (rc.ch_rth    != 0xFF && rc.ch_rth    >= RC_MAX_CH) { rc.ch_rth = 0xFF; fixed++; }
    if (rc.ch_flightmode != 0xFF && rc.ch_flightmode >= RC_MAX_CH) {
        rc.ch_flightmode = 0xFF; fixed++;
    }
    rc.sbus_inverted     = rc.sbus_inverted ? 1 : 0;
    rc.arm_high_is_armed = rc.arm_high_is_armed ? 1 : 0;
    if (rc.max_raw <= rc.mid_raw) { rc_apply_proto_defaults(rc); fixed++; }
    if (rc.mid_raw <= rc.min_raw) { rc_apply_proto_defaults(rc); fixed++; }
    fixed += clamp_u16(rc.deadband_raw, 0, 100);
    fixed += clamp_u16(rc.loss_timeout_ms, 100, 5000);
    fixed += clamp_f(rc.expo, 0.0f, 1.0f);

    WifiSettings& w = s.wifi;
    if (w.mode > WIFI_MODE_STA) { w.mode = WIFI_MODE_AP; fixed++; }
    w.ssid[SSID_MAX]   = 0;
    w.pass[PASS_MAX]   = 0;
    w.hostname[HOSTNAME_MAX] = 0;
    if (w.hostname[0] == 0) { strncpy(w.hostname, "wing-01", HOSTNAME_MAX); fixed++; }
    fixed += clamp_u16(w.http_port, 1, 65535);

    TelemSettings& t = s.telem;
    t.enable     = t.enable ? 1 : 0;
    t.fast_only  = t.fast_only ? 1 : 0;
    fixed += clamp_u16(t.rate_hz, 1, 200);

    return fixed;
}

// ---------------------------------------------------------------------------
// ESP32 NVS backend.  On the host this compiles to a stub so the unit tests
// can still link the module without pulling in the Arduino core.
// ---------------------------------------------------------------------------
#ifdef ARDUINO
#include <Preferences.h>

static const char* kNs = "wing";
static const char* kKey = "cfg";

bool settings_load(Settings& out) {
    settings_defaults(out);

    Preferences prefs;
    if (!prefs.begin(kNs, true)) return false;          // read-only
    size_t n = prefs.getBytesLength(kKey);
    if (n == 0) { prefs.end(); return false; }

    uint8_t buf[SETTINGS_BLOB_MAX];
    if (n > sizeof(buf)) { prefs.end(); return false; }
    size_t got = prefs.getBytes(kKey, buf, n);
    prefs.end();

    if (got != n) return false;
    return settings_deserialize(buf, n, out);           // false => defaults kept
}

bool settings_save(const Settings& s) {
    uint8_t buf[SETTINGS_BLOB_MAX];
    size_t len = 0;
    if (!settings_serialize(s, buf, sizeof(buf), &len)) return false;

    Preferences prefs;
    if (!prefs.begin(kNs, false)) return false;         // read-write
    size_t put = prefs.putBytes(kKey, buf, len);
    prefs.end();
    return put == len;
}
#else
// Host: no NVS. Callers that want persistence use the blob codec directly.
bool settings_load(Settings& out) {
    settings_defaults(out);
    return false;
}
bool settings_save(const Settings&) { return false; }
#endif
