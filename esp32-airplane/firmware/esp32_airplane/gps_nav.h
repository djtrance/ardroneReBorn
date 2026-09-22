// gps_nav.h — NMEA parsing + great-circle math (ported from src/navigation/gps.c)
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

struct GpsFix {
    double lat, lon;       // degrees
    float  alt_m;          // metres (from GGA altitude)
    float  ground_mps;     // speed over ground
    float  course_deg;     // track over ground
    float  hdop;
    uint8_t num_sv;
    uint8_t fix_quality;   // 0 = no fix, 1 = GPS, 2 = DGPS
    uint32_t utc;          // hhmmss as uint
    bool   valid;
};

// Haversine great-circle distance in metres (direct port of gps.c).
double haversine_m(double lat1, double lon1, double lat2, double lon2);

// Initial bearing from p1 to p2, degrees [0,360).
double bearing_deg(double lat1, double lon1, double lat2, double lon2);

// Extract "ddmm.mmmm" NMEA coordinate + hemisphere -> decimal degrees.
static inline double nmea_to_deg(const char* s, char hemi) {
    // Format: ddmm.mmmm (lat) or dddmm.mmmm (lon).  Degrees = all but last 2.
    long whole = 0;
    double frac = 0.0, scale = 0.1;
    bool dot = false;
    for (const char* p = s; *p; ++p) {
        if (*p == '.') { dot = true; continue; }
        if (*p < '0' || *p > '9') break;
        if (!dot) { whole = whole * 10 + (*p - '0'); }
        else      { frac += (*p - '0') * scale; scale *= 0.1; }
    }
    double v = (double)whole + frac;
    int    deg = (int)(v / 100.0);        // strip the mm part
    double minutes = v - (double)deg * 100.0;
    double out = (double)deg + minutes / 60.0;
    if (hemi == 'S' || hemi == 'W') out = -out;
    return out;
}

// Feed one NMEA sentence (or a chunk containing sentences). Returns true if
// the fix struct was updated.
bool gps_parse(const char* line, GpsFix& fix);

// Checklist G4 gate: true when fix quality + satellite count + HDOP are
// trustworthy enough to arm / trust for RTH (C4).
bool gps_trustworthy(const GpsFix& f);

// --- Byte-stream front end (sensors.cpp pumps UART1 bytes through this) ----
// Assembles a line buffer, parses on '\n'/'\r', and resynchronises on a '$'
// so a truncated or corrupted frame never poisons the next good one.
void     gps_line_reset();               // drop partial line + zero counter
bool     gps_feed_byte(char c, GpsFix& fix);   // true when `fix` was updated
void     gps_feed(const char* data, size_t n, GpsFix& fix);
uint32_t gps_lines_seen();               // checksum-valid lines since reset
                                             // (== "the module is talking")

// --- u-blox 6 (UBX) config packets — sent by gps_init() at boot -----------
// Pure builders: sync (B5 62) + class/id/len + payload + 8-bit Fletcher
// checksum over class..payload (u-blox 6 spec §31). Return the total packet
// length in bytes, or 0 if `out_max` is too small / a value is out of spec.
int ubx_build_cfg_rate(uint8_t* out, size_t out_max, uint16_t meas_ms);
int ubx_build_cfg_msg_nmea(uint8_t* out, size_t out_max,
                           uint8_t nmea_id, uint8_t uart1_rate);
// UBX-CFG-PRT: re-baud UART1 (the line-rate jump to GPS_BAUD_HI). sensors.cpp
// verifies it took by probing for NMEA on the new baud and reverts if mute.
int ubx_build_cfg_prt_uart(uint8_t* out, size_t out_max, uint32_t baud);
