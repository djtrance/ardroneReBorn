#include "rc_input.h"
#include "config.h"
#include <string.h>

#ifdef ARDUINO
  #include <Arduino.h>
  #define RC_NOW_US()  ((uint32_t)esp_timer_get_time())
#else
  #define RC_NOW_US()  (0u)
#endif

static uint32_t s_last_decode_us = 0;

uint32_t rc_last_decode_us(void) { return s_last_decode_us; }

void rc_frame_clear(RcFrame& f) {
    memset(&f, 0, sizeof(f));
}

void rc_input_defaults(RcInput& in) {
    memset(&in, 0, sizeof(in));
    in.pitch = 0.0f;
    in.roll  = 0.0f;
    in.yaw   = 0.0f;
    in.throttle = 0.0f;
    in.mode  = 0;
}

// ===========================================================================
// SBUS — 25 bytes @ 100000 8E2 (inverted on the wire)
//
//   buf[0]     header 0x0F (SBUS2 also 0x1B / 0x2B)
//   buf[1..22] 16 channels x 11 bits packed little-endian = 176 bits
//   buf[23]    status: bit0 ch16, bit1 ch17, bit2 ch18,
//                     bit3 frame lost, bit4 failsafe
//   buf[24]    footer 0x00
//
// Channel n starts at bit 11*n of the data area, so the extraction below is
// written out channel by channel rather than as a generic bit loop — cheaper
// and easier to verify against the bit chart.
// ===========================================================================
bool sbus_decode(const uint8_t* buf, RcFrame& out) {
    uint32_t t0 = RC_NOW_US();
    rc_frame_clear(out);

    if (!buf) return false;
    if (buf[0] != SBUS_HEADER_1 && buf[0] != SBUS_HEADER_2 &&
        buf[0] != SBUS_HEADER_3) return false;
    if (buf[24] != 0x00) return false;              // footer must be 0

    const uint8_t* d = buf;

    out.raw[0]  = (uint16_t)(( d[1]          | (d[2]  << 8))              & 0x07FF);
    out.raw[1]  = (uint16_t)(((d[2]  >> 3)   | (d[3]  << 5))              & 0x07FF);
    out.raw[2]  = (uint16_t)(((d[3]  >> 6)   | (d[4]  << 2)  | (d[5]  << 10)) & 0x07FF);
    out.raw[3]  = (uint16_t)(((d[5]  >> 1)   | (d[6]  << 7))              & 0x07FF);
    out.raw[4]  = (uint16_t)(((d[6]  >> 4)   | (d[7]  << 4))              & 0x07FF);
    out.raw[5]  = (uint16_t)(((d[7]  >> 7)   | (d[8]  << 1)  | (d[9]  << 9))  & 0x07FF);
    out.raw[6]  = (uint16_t)(((d[9]  >> 2)   | (d[10] << 6))              & 0x07FF);
    out.raw[7]  = (uint16_t)(((d[10] >> 5)   | (d[11] << 3))              & 0x07FF);
    out.raw[8]  = (uint16_t)(( d[12]         | (d[13] << 8))              & 0x07FF);
    out.raw[9]  = (uint16_t)(((d[13] >> 3)   | (d[14] << 5))              & 0x07FF);
    out.raw[10] = (uint16_t)(((d[14] >> 6)   | (d[15] << 2)  | (d[16] << 10)) & 0x07FF);
    out.raw[11] = (uint16_t)(((d[16] >> 1)   | (d[17] << 7))              & 0x07FF);
    out.raw[12] = (uint16_t)(((d[17] >> 4)   | (d[18] << 4))              & 0x07FF);
    out.raw[13] = (uint16_t)(((d[18] >> 7)   | (d[19] << 1)  | (d[20] << 9))  & 0x07FF);
    out.raw[14] = (uint16_t)(((d[20] >> 2)   | (d[21] << 6))              & 0x07FF);
    out.raw[15] = (uint16_t)(((d[21] >> 5)   | (d[22] << 3))              & 0x07FF);

    out.count      = RC_MAX_CH;
    out.status     = d[23];
    out.frame_ok   = true;
    out.frame_lost = (d[23] & 0x08) != 0;
    out.failsafe   = (d[23] & 0x10) != 0;

    s_last_decode_us = RC_NOW_US() - t0;
    return true;
}

// ===========================================================================
// Spektrum satellite — 16 bytes @ 115200 8N1
//
//   byte 0     bit0 = 0 => 1024 mode (10-bit values),
//                    = 1 => 2048 mode (11-bit values);
//              bits1-7 frame counter / fade counter low
//   byte 1     frame counter / fade counter high
//   byte 2     status: failsafe in bit0, remaining bits are loss/hold counts
//   byte 3..14 six channels, 16-bit big-endian words
//              1024: bits15-10 = channel id (6b), bits9-0  = servo position
//              2048: bit15     = servo phase,
//                    bits14-11 = channel id (4b), bits10-0 = servo position
//   byte 15    signal quality (RSSI-ish)
//
// Values are normalised to the 10-bit domain on the way out so a single
// RcSettings min/mid/max works for both Spektrum sub-modes.
//
// NOTE / TODO bench validation: the byte-2 status layout differs between
// published sources (BoldPort AR6400 note vs. PX4 dsm.cpp). We follow the
// simple "bit0 = failsafe" reading and keep the raw byte in RcFrame.status so
// it can be checked against a real receiver before first flight (H2 blocker).
// ===========================================================================
bool spektrum_decode(const uint8_t* buf, RcFrame& out) {
    uint32_t t0 = RC_NOW_US();
    rc_frame_clear(out);
    if (!buf) return false;

    const bool eleven = (buf[0] & 0x01) != 0;

    for (int i = 0; i < 6; i++) {
        uint16_t w = (uint16_t)((buf[3 + 2 * i] << 8) | buf[4 + 2 * i]);

        uint16_t id, val;
        if (eleven) {
            id  = (uint16_t)((w >> 11) & 0x0F);   // bits14-11 (bit15 = phase)
            val = (uint16_t)(w & 0x07FF);         // 11-bit servo position
            val = (uint16_t)(val >> 1);           // -> 10-bit domain
        } else {
            id  = (uint16_t)((w >> 10) & 0x3F);   // bits15-10
            val = (uint16_t)(w & 0x03FF);         // 10-bit servo position
        }

        if (id < RC_MAX_CH) {
            out.raw[id] = val;
            out.count++;                          // distinct ids seen
        }
    }

    out.status       = buf[2];
    out.link_quality = buf[15];
    out.frame_ok     = true;
    out.failsafe     = (buf[2] & 0x01) != 0;
    out.frame_lost   = false;                     // see note above

    s_last_decode_us = RC_NOW_US() - t0;
    return true;
}

// ===========================================================================
// Mapping + normalisation
// ===========================================================================
float rc_raw_to_norm(uint16_t raw, const RcSettings& rc) {
    float v;
    if (raw >= rc.mid_raw) {
        uint16_t span = (uint16_t)(rc.max_raw - rc.mid_raw);
        if (span == 0) return 0.0f;
        v = (float)(raw - rc.mid_raw) / (float)span;
    } else {
        uint16_t span = (uint16_t)(rc.mid_raw - rc.min_raw);
        if (span == 0) return 0.0f;
        v = -(float)(rc.mid_raw - raw) / (float)span;
    }
    if (v >  1.0f) v =  1.0f;
    if (v < -1.0f) v = -1.0f;

    // Centre deadband — a mis-trimmed stick must not wobble the servos.
    float db = (float)rc.deadband_raw /
               (float)((raw >= rc.mid_raw) ? (rc.max_raw - rc.mid_raw)
                                           : (rc.mid_raw - rc.min_raw));
    if (v > -db && v < db) return 0.0f;
    return v;
}

float rc_apply_expo(float v, float expo) {
    if (expo <= 0.0f) return v;
    if (expo > 1.0f)  expo = 1.0f;
    // v' = (1-e)*v + e*v^3  — standard RC exponential, keeps the endpoints.
    float v3 = v * v * v;
    return (1.0f - expo) * v + expo * v3;
}

static inline uint8_t switch_pos(uint16_t raw, const RcSettings& rc) {
    // 3-pos switch: below / around / above centre.
    float n = rc_raw_to_norm(raw, rc);
    if (n < -0.33f) return 0;
    if (n >  0.33f) return 2;
    return 1;
}

bool rc_map(const RcFrame& f, const RcSettings& rc, uint32_t now_ms, RcInput& in) {
    if (!f.frame_ok) return false;

    auto get = [&](uint8_t ch) -> uint16_t {
        if (ch == 0xFF || ch >= RC_MAX_CH) return rc.mid_raw;
        return f.raw[ch];
    };

    float p = rc_apply_expo(rc_raw_to_norm(get(rc.ch_pitch),    rc), rc.expo);
    float r = rc_apply_expo(rc_raw_to_norm(get(rc.ch_roll),     rc), rc.expo);
    float y = rc_raw_to_norm(get(rc.ch_yaw), rc);

    // Throttle: raw min..max -> 0..1 (never expo'd — linearity matters here)
    float th = rc_raw_to_norm(get(rc.ch_throttle), rc);
    th = (th + 1.0f) * 0.5f;
    if (th < 0.0f) th = 0.0f;
    if (th > 1.0f) th = 1.0f;

    in.pitch    = p;
    in.roll     = r;
    in.yaw      = y;
    in.throttle = th;

    if (rc.ch_arm == 0xFF) {
        in.arm = false;                 // no switch configured: never arm
    } else {
        uint8_t sp = switch_pos(get(rc.ch_arm), rc);
        in.arm = rc.arm_high_is_armed ? (sp == 2) : (sp == 0);
    }

    in.rth  = (rc.ch_rth != 0xFF) &&
              (switch_pos(get(rc.ch_rth), rc) == 2);

    if (rc.ch_flightmode == 0xFF) {
        in.mode = 0;
    } else {
        uint8_t sp = switch_pos(get(rc.ch_flightmode), rc);
        in.mode = (sp == 2) ? 2 : (sp == 1 ? 1 : 0);
    }

    in.last_frame_ms = now_ms;
    in.ok = !f.failsafe && !f.frame_lost;
    return true;
}
