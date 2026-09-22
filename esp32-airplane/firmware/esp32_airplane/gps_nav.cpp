#include "gps_nav.h"
#include "config.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232

// ---------------------------------------------------------------------------
// Great-circle maths — direct port of src/navigation/gps.c
// ---------------------------------------------------------------------------
double haversine_m(double lat1, double lon1, double lat2, double lon2) {
    double r = 6371000.0;
    double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD;
    double dp = (lat2 - lat1) * DEG2RAD;
    double dl = (lon2 - lon1) * DEG2RAD;
    double a = sin(dp / 2) * sin(dp / 2) +
               cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return (float)(r * c);
}

double bearing_deg(double lat1, double lon1, double lat2, double lon2) {
    double p1 = lat1 * DEG2RAD, p2 = lat2 * DEG2RAD;
    double dl = (lon2 - lon1) * DEG2RAD;
    double y = sin(dl) * cos(p2);
    double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    double br = atan2(y, x) * RAD2DEG;
    if (br < 0) br += 360.0;
    return br;
}

// ---------------------------------------------------------------------------
// Minimal NMEA checksum validation
// ---------------------------------------------------------------------------
static bool nmea_checksum_ok(const char* s) {
    if (s[0] != '$') return false;
    const char* star = strchr(s, '*');
    if (!star || !star[1] || !star[2]) return false;
    uint8_t cs = 0;
    for (const char* p = s + 1; p < star; ++p) cs ^= (uint8_t)*p;
    char want[3] = { (char)((cs >> 4) < 10 ? '0' + (cs >> 4) : 'A' + (cs >> 4) - 10),
                     (char)((cs & 0xF) < 10 ? '0' + (cs & 0xF) : 'A' + (cs & 0xF) - 10), 0 };
    return (toupper(star[1]) == want[0]) && (toupper(star[2]) == want[1]);
}

// ---------------------------------------------------------------------------
// GGA — position fix
// ---------------------------------------------------------------------------
static bool parse_gga(char* body, GpsFix& fix) {
    // $GPGGA,hhmmss.ss,lat,N,lon,E,fix,numsv,hdop,alt,M,...
    char* tok[15];
    int n = 0;
    char* p = strtok(body, ",");
    while (p && n < 15) { tok[n++] = p; p = strtok(NULL, ","); }
    if (n < 10) return false;

    // NOTE: buf+7 already consumed "$GPGGA,", so tok[0] is the UTC time.
    //   [0]=time [1]=lat [2]=N [3]=lon [4]=E [5]=quality [6]=sats
    //   [7]=hdop [8]=alt [9]=M ...
    if (tok[0] && *tok[0]) fix.utc = (uint32_t)atof(tok[0]);
    fix.fix_quality = (uint8_t)atoi(tok[5]);
    fix.num_sv      = (uint8_t)atoi(tok[6]);
    fix.hdop        = (float)atof(tok[7]);
    fix.alt_m       = (float)atof(tok[8]);
    if (fix.fix_quality > 0 && tok[1] && tok[2] && tok[3] && tok[4]) {
        fix.lat = nmea_to_deg(tok[1], tok[2][0]);
        fix.lon = nmea_to_deg(tok[3], tok[4][0]);
        fix.valid = true;
    } else {
        fix.valid = false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// RMC — velocity + course
// ---------------------------------------------------------------------------
static bool parse_rmc(char* body, GpsFix& fix) {
    // buf+7 strips "$GPRMC,", so:
    //   [0]=time [1]=status [2]=lat [3]=N [4]=lon [5]=E
    //   [6]=knots [7]=course [8]=date ...
    char* tok[12];
    int n = 0;
    char* p = strtok(body, ",");
    while (p && n < 12) { tok[n++] = p; p = strtok(NULL, ","); }
    if (n < 8) return false;

    bool active = (tok[1] && tok[1][0] == 'A');
    if (active) {
        float knots = (float)atof(tok[6]);
        fix.ground_mps = knots * 0.514444f;          // 1 knot = 0.5144 m/s
        fix.course_deg = (float)atof(tok[7]);
        if (tok[2] && tok[3] && tok[4] && tok[5]) {
            fix.lat = nmea_to_deg(tok[2], tok[3][0]);
            fix.lon = nmea_to_deg(tok[4], tok[5][0]);
            fix.valid = true;
        }
    }
    return true;
}

bool gps_parse(const char* line, GpsFix& fix) {
    if (!line || line[0] != '$') return false;
    if (!nmea_checksum_ok(line)) return false;

    char buf[96];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;

    char* star = strchr(buf, '*');
    if (star) *star = 0;

    if (strncmp(buf, "$GPGGA", 6) == 0 || strncmp(buf, "$GNGGA", 6) == 0)
        return parse_gga(buf + 7, fix);
    if (strncmp(buf, "$GPRMC", 6) == 0 || strncmp(buf, "$GNRMC", 6) == 0)
        return parse_rmc(buf + 7, fix);
    return false;
}

bool gps_trustworthy(const GpsFix& f) {
    return f.valid && f.fix_quality >= 1 &&
           f.num_sv >= 6 && f.hdop > 0.0f && f.hdop <= GPS_MAX_HDOP;
}
