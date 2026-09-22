// drivers/ld2450.h — HLK-LD2450 24 GHz FMCW radar frame parser (checklist C7)
//
// UART 256000 8N1, 10 fps, 30-byte frames:
//   AA FF 03 00 | cmd(2) | 3 x (x_lo x_hi y_lo y_hi spd_lo spd_hi res_lo res_hi) | 55 CC
// Coordinates signed little-endian millimetres; y = range, x = lateral.
//
// Limitation (see algorithm-mapping §4.4): range is 6 m — useful for landing
// / low-alt / target detection, NOT cruise obstacle avoidance.
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#define LD2450_MAX_TARGETS 3

typedef struct {
    int16_t  x_mm;        // lateral offset
    int16_t  y_mm;        // range toward target (0 = no target)
    int16_t  speed_mmps;  // radial speed, mm/s
    uint16_t resolution;  // target resolution metric
    bool     valid;
} Ld2450Target;

typedef struct {
    Ld2450Target targets[LD2450_MAX_TARGETS];
    uint8_t      count;        // number of valid targets in this frame
    uint32_t     frames_ok;
    uint32_t     frames_bad;   // header/footer/CRC failures
} Ld2450Frame;

// Parse one complete frame. Returns true if header + footer matched.
// `buf` must contain at least LD2450_FRAME_LEN bytes starting at a header,
// or a larger buffer in which the header will be searched for.
bool ld2450_parse(const uint8_t* buf, size_t n, Ld2450Frame& out);

// Feed bytes incrementally (streaming). Returns true when a full frame was
// decoded into `out`. Handles partial reads across UART refills.
#define LD2450_FRAME_LEN_HINT 64   // reassembly window (> FRAME_LEN)

class Ld2450Stream {
public:
    Ld2450Stream() : _len(0) {}
    bool feed(uint8_t byte, Ld2450Frame& out);
    void reset() { _len = 0; }
private:
    uint8_t _buf[LD2450_FRAME_LEN_HINT];
    uint8_t _len;
};

// Nearest-neighbour track assignment across frames (keeps target IDs stable,
// checklist C7). Writes the assigned index for each new target, or -1.
void ld2450_match_tracks(const Ld2450Frame& prev, const Ld2450Frame& cur,
                         int8_t assignment[LD2450_MAX_TARGETS]);
