#include "ld2450.h"
#include <string.h>
#include <math.h>

// Frame layout constants (checklist C7 — verify against firmware V2.02.x
// using the HLKRadarTool visualizer before trusting offsets).
#define HDR0 0xAA
#define HDR1 0xFF
#define HDR2 0x03
#define HDR3 0x00
#define FTR0 0x55
#define FTR1 0xCC
#define FRAME_LEN      30
#define HDR_LEN         6      // AA FF 03 00 + 2 cmd bytes
#define TARGET_STRIDE   8      // x(2) y(2) speed(2) res(2)

static inline int16_t s16le(const uint8_t* p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline uint16_t u16le(const uint8_t* p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

// A target is "present" when its range field is non-zero (Hi-Link convention).
static bool decode_targets(const uint8_t* d, Ld2450Frame& out) {
    out.count = 0;
    for (int t = 0; t < LD2450_MAX_TARGETS; t++) {
        const uint8_t* p = d + t * TARGET_STRIDE;
        Ld2450Target& tg = out.targets[t];
        tg.x_mm        = s16le(p);
        tg.y_mm        = s16le(p + 2);
        tg.speed_mmps  = s16le(p + 4);
        tg.resolution  = u16le(p + 6);
        // y_mm is the range; 0 means no object at this slot.
        tg.valid = (tg.y_mm != 0) && (tg.y_mm > 0);
        if (tg.valid) out.count++;
    }
    return out.count > 0;
}

bool ld2450_parse(const uint8_t* buf, size_t n, Ld2450Frame& out) {
    memset(&out, 0, sizeof(out));
    if (!buf || n < FRAME_LEN) return false;

    // Search for the header inside the buffer (handles misaligned reads).
    for (size_t i = 0; i + FRAME_LEN <= n; i++) {
        if (buf[i]   == HDR0 && buf[i+1] == HDR1 &&
            buf[i+2] == HDR2 && buf[i+3] == HDR3 &&
            buf[i + FRAME_LEN - 2] == FTR0 &&
            buf[i + FRAME_LEN - 1] == FTR1) {
            decode_targets(buf + i + HDR_LEN, out);
            out.frames_ok++;
            return true;
        }
    }
    out.frames_bad++;
    return false;
}

// --- Incremental stream reassembly -----------------------------------------
bool Ld2450Stream::feed(uint8_t byte, Ld2450Frame& out) {
    // Slide byte into the ring.
    if (_len < LD2450_FRAME_LEN_HINT) _buf[_len++] = byte;
    else {
        memmove(_buf, _buf + 1, LD2450_FRAME_LEN_HINT - 1);
        _buf[LD2450_FRAME_LEN_HINT - 1] = byte;
        _len = LD2450_FRAME_LEN_HINT;
    }

    if (_len < FRAME_LEN) return false;

    // Try to decode from every position that could be a header start.
    for (uint8_t i = 0; i + FRAME_LEN <= _len; i++) {
        if (_buf[i] == HDR0 && _buf[i+1] == HDR1 &&
            _buf[i+2] == HDR2 && _buf[i+3] == HDR3 &&
            _buf[i + FRAME_LEN - 2] == FTR0 &&
            _buf[i + FRAME_LEN - 1] == FTR1) {
            memset(&out, 0, sizeof(out));
            decode_targets(_buf + i + HDR_LEN, out);
            out.frames_ok++;
            // Drop consumed bytes so the next frame starts clean.
            uint8_t consumed = (uint8_t)(i + FRAME_LEN);
            memmove(_buf, _buf + consumed, _len - consumed);
            _len = (uint8_t)(_len - consumed);
            return true;
        }
    }
    return false;
}

// --- Nearest-neighbour track matching (C7) ---------------------------------
void ld2450_match_tracks(const Ld2450Frame& prev, const Ld2450Frame& cur,
                         int8_t assignment[LD2450_MAX_TARGETS]) {
    for (int i = 0; i < LD2450_MAX_TARGETS; i++) assignment[i] = -1;

    bool prev_used[LD2450_MAX_TARGETS] = {false, false, false};

    // For each current target, find the closest previous target in (x, y).
    for (int c = 0; c < LD2450_MAX_TARGETS; c++) {
        if (!cur.targets[c].valid) continue;
        float best = 1e12f;
        int   best_j = -1;
        for (int p = 0; p < LD2450_MAX_TARGETS; p++) {
            if (!prev.targets[p].valid || prev_used[p]) continue;
            float dx = (float)(cur.targets[c].x_mm - prev.targets[p].x_mm);
            float dy = (float)(cur.targets[c].y_mm - prev.targets[p].y_mm);
            float d2 = dx * dx + dy * dy;
            if (d2 < best) { best = d2; best_j = p; }
        }
        if (best_j >= 0 && best < (1500.0f * 1500.0f)) {   // 1.5 m gate @10 Hz
            assignment[c] = (int8_t)best_j;
            prev_used[best_j] = true;
        }
    }
}
