// rc_input.h — SBUS + Spektrum satellite decoders  (checklist H2, phase 1 RC)
//
// First flight stage is flown on RC only, so this path must be solid and it
// must be independent of GPS/vision. Two wire protocols are supported and
// selected at runtime through Settings.rc.proto (WiFi portal):
//
//   SBUS        100000 baud, 8E2, inverted, 25-byte frame, 16 channels
//   Spektrum    115200 baud, 8N1, 16-byte frame, 6 channels per frame
//
// Both decoders are pure functions over a byte buffer so they can be unit
// tested on the host with captured/replayed frames (checklist J2).
//
// Reference research: docs/rc-and-telemetry.md — SBUS carries NO telemetry
// (one-way). Telemetry back to the TX16S needs S.Port or FPort.

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "settings.h"

#define SBUS_FRAME_LEN      25
#define SPEKTRUM_FRAME_LEN  16
#define SBUS_BAUD           100000
#define SPEKTRUM_BAUD       115200

// SBUS frame lead byte (SBUS2 variants 0x1B/0x2B are also accepted).
#define SBUS_HEADER_1       0x0F
#define SBUS_HEADER_2       0x1B
#define SBUS_HEADER_3       0x2B

// ---------------------------------------------------------------------------
struct RcFrame {
    uint16_t raw[RC_MAX_CH];   // protocol units (see RcSettings min/mid/max)
    uint8_t  count;            // distinct channels present in this frame
    uint8_t  status;           // raw status/flags byte (debug + failsafe)
    uint8_t  link_quality;     // Spektrum: signal-quality byte
    bool     frame_ok;         // header + length looked sane
    bool     failsafe;
    bool     frame_lost;
};

void rc_frame_clear(RcFrame& f);

// Decode a complete frame. Return false (and leave `out` cleared) on garbage.
bool sbus_decode(const uint8_t* buf, RcFrame& out);
bool spektrum_decode(const uint8_t* buf, RcFrame& out);

// ---------------------------------------------------------------------------
// Normalised pilot sticks after channel mapping, inversion, deadband, expo.
// ---------------------------------------------------------------------------
struct RcInput {
    float    pitch;            // -1..+1, + = nose up
    float    roll;             // -1..+1, + = roll right
    float    yaw;              // -1..+1, + = yaw right
    float    throttle;         //  0..1
    bool     arm;              // arm switch asserted
    bool     rth;              // RTH switch asserted
    uint8_t  mode;             // 0 = manual, 1 = RTH, 2 = loiter
    bool     ok;               // fresh frame AND not failsafe
    uint32_t last_frame_ms;
};

void rc_input_defaults(RcInput& in);

// Raw protocol units -> -1..+1 with centre deadband applied.
float rc_raw_to_norm(uint16_t raw, const RcSettings& rc);

// Exponential curve, 0 = linear, 1 = fully soft at centre.
float rc_apply_expo(float v, float expo);

// Map one decoded frame into sticks. `now_ms` drives the loss timeout, so a
// caller that stops receiving frames automatically gets ok == false.
bool rc_map(const RcFrame& f, const RcSettings& rc, uint32_t now_ms, RcInput& in);

// Total pilot-input processing time of the last decode, in microseconds —
// feeds the loop-jitter telemetry field (I1, docs/rc-and-telemetry.md §5.2).
uint32_t rc_last_decode_us(void);
