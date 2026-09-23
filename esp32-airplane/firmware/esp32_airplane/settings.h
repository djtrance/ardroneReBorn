// settings.h — runtime configuration that MUST survive a power cycle.
//
// Checklist: B2 (pin defaults), D1/D2/D3 (servo reverse + mix), H2 (RC map),
//            I5 (WiFi config portal), I1 (telemetry enable).
//
// Everything here can be edited over the WiFi portal and is stored in ESP32
// NVS ("Preferences"). Compile-time constants stay in config.h — only values
// a pilot may want to tweak without reflashing live here.
//
// Layout is a plain-old-data struct so it can be memcpy'd to a blob, CRC'd and
// stored. Host tests exercise serialize/deserialize without any NVS.

#pragma once
#include <stdint.h>
#include <stddef.h>

#define SETTINGS_MAGIC      0x57494E47u   // "WING"
#define SETTINGS_VERSION    6u            // bump on any struct change
#define SETTINGS_BLOB_MAX   512

#define SSID_MAX            32
#define PASS_MAX            64
#define HOSTNAME_MAX        23

#ifndef RC_MAX_CH
#define RC_MAX_CH           16
#endif

// ---------------------------------------------------------------------------
// Elevon / throttle mixing (checklist D1/D2/D3) — set from the WiFi portal
// ---------------------------------------------------------------------------
struct MixSettings {
    uint16_t elevon_l_trim_us;        // mechanical trim, left surface
    uint16_t elevon_r_trim_us;        // mechanical trim, right surface
    uint16_t pitch_span_us;           // +/- us around trim for full pitch
    uint16_t roll_span_us;            // +/- us around trim for full roll
    uint8_t  elevon_l_reverse;        // 1 => invert that servo (D1)
    uint8_t  elevon_r_reverse;
    uint8_t  throttle_reverse;        // 1 => invert ESC signal (rare)
    uint8_t  passthrough;             // 1 => etapa 1: RX sticks -> elevon mix
                                      //      directly, no AHRS/control in the
                                      //      loop (test-campaign T0). Default
                                      //      ON: the first flights must behave
                                      //      like a plain RC plane.
    float    differential;            // adverse-yaw differential 0..0.4 (D2)
    uint16_t esc_min_us;              // throttle low (D4 arming)
    uint16_t esc_max_us;
    uint16_t esc_idle_us;             // idle while armed, sticks low
};

// ---------------------------------------------------------------------------
// RC input (checklist H2) — SBUS or Spektrum satellite
// ---------------------------------------------------------------------------
#define RC_PROTO_NONE       0
#define RC_PROTO_SBUS       1
#define RC_PROTO_SPEKTRUM   2

struct RcSettings {
    uint8_t  proto;                    // RC_PROTO_*
    uint8_t  ch_pitch;                 // channel index (0-based)
    uint8_t  ch_roll;
    uint8_t  ch_throttle;
    uint8_t  ch_yaw;
    uint8_t  ch_arm;                   // 0xFF = no dedicated switch
    uint8_t  ch_rth;
    uint8_t  ch_flightmode;            // 3-pos: manual / RTH / loiter
    uint8_t  sbus_inverted;            // 1 => invert line level (FrSky default)
    uint8_t  arm_high_is_armed;        // default 0 => switch HIGH disarms
    uint16_t min_raw;                  // protocol units at stick low
    uint16_t mid_raw;                  // stick centre
    uint16_t max_raw;                  // stick high
    uint16_t deadband_raw;             // raw units of centre deadband
    uint16_t loss_timeout_ms;          // failsafe after this without a frame
    float    expo;                     // pilot-stick expo 0..1 (shape only)
};

// ---------------------------------------------------------------------------
// WiFi + configuration portal (checklist I5)
// ---------------------------------------------------------------------------
#define WIFI_MODE_AP        0           // always run an access point
#define WIFI_MODE_STA       1           // join `ssid` if it works, else AP

struct WifiSettings {
    uint8_t  mode;                     // WIFI_MODE_*
    char     ssid[SSID_MAX + 1];
    char     pass[PASS_MAX + 1];
    char     hostname[HOSTNAME_MAX + 1];
    uint16_t http_port;                // config portal port
};

// ---------------------------------------------------------------------------
// Telemetry back to the RC radio (docs/rc-and-telemetry.md)
// ---------------------------------------------------------------------------
struct TelemSettings {
    uint8_t  enable;                   // 1 => FrSky S.Port output
    uint8_t  fast_only;                // 1 => only envelope/phase while flying
    uint16_t rate_hz;                  // S.Port response pacing
    uint16_t fields_mask;              // bitmask of DIY fields to publish
};

// DIY telemetry field IDs (docs/rc-and-telemetry.md §5.2)
#define TF_PHASE            (1u << 0)
#define TF_ENV_FLAGS        (1u << 1)
#define TF_AIRSPEED         (1u << 2)
#define TF_DIST_HOME        (1u << 3)
#define TF_BEARING_HOME     (1u << 4)
#define TF_ALT_HOME         (1u << 5)
#define TF_GLIDE_MARGIN     (1u << 6)
#define TF_ESO_ROLL         (1u << 7)
#define TF_ESO_PITCH        (1u << 8)
#define TF_B0               (1u << 9)
#define TF_FENCE_MARGIN     (1u << 10)
#define TF_PREFLIGHT        (1u << 11)
#define TF_LOOP_JITTER      (1u << 12)
#define TF_LINK_AGE         (1u << 13)
#define TF_IMU_MODEL        (1u << 14)

// ---------------------------------------------------------------------------
// The whole persisted blob
// ---------------------------------------------------------------------------
struct Settings {
    uint32_t magic;                    // SETTINGS_MAGIC
    uint16_t version;                  // SETTINGS_VERSION
    uint16_t size;                     // sizeof(Settings) at write time

    MixSettings   mix;
    RcSettings    rc;
    WifiSettings  wifi;
    TelemSettings telem;

    uint16_t crc;                      // CRC-16/CCITT over everything above
};

// --- lifecycle ---------------------------------------------------------------
void settings_defaults(Settings& s);

// --- blob codec (host-testable) ---------------------------------------------
uint16_t settings_crc16(const uint8_t* data, size_t len);
bool settings_serialize(const Settings& s, uint8_t* buf, size_t cap, size_t* out_len);
// Returns false on magic/version/size/CRC mismatch; on failure `out` keeps its
// previous contents so the caller can keep running on defaults.
bool settings_deserialize(const uint8_t* buf, size_t len, Settings& out);

// Clamp everything into safe ranges. Returns the number of fields that were
// corrected (0 = the blob was already sane).
int settings_validate(Settings& s);

// Re-fill raw stick ranges after the protocol changes (SBUS vs Spektrum).
void rc_apply_proto_defaults(RcSettings& rc);

// --- ESP32 NVS backend (compiles to a no-op stub on the host) ----------------
bool settings_load(Settings& out);     // false => `out` holds defaults
bool settings_save(const Settings& s);
