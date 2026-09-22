// gps_nav.h — NMEA parsing + great-circle math (ported from src/navigation/gps.c)
#pragma once
#include <stdint.h>
#include <stdbool.h>

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
