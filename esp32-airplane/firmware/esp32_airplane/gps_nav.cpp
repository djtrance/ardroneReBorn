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

// Split a comma-separated body PRESERVING empty fields. strtok() collapses
// consecutive delimiters, which would shift every index on a sentence with
// empty fields — and a no-fix GGA/RMC is mostly empty, exactly the case the
// GPS_LOSS failsafe must see.
static int split_fields(char* body, char* tok[], int max) {
    int n = 0;
    char* p = body;
    while (n < max) {
        tok[n++] = p;
        char* c = strchr(p, ',');
        if (!c) break;
        *c = '\0';
        p = c + 1;
    }
    return n;
}

// ---------------------------------------------------------------------------
// GGA — position fix
// ---------------------------------------------------------------------------
static bool parse_gga(char* body, GpsFix& fix) {
    // $GPGGA,hhmmss.ss,lat,N,lon,E,fix,numsv,hdop,alt,M,...
    char* tok[15];
    int n = split_fields(body, tok, 15);
    if (n < 10) return false;

    //   [0]=time [1]=lat [2]=N [3]=lon [4]=E [5]=quality [6]=sats
    //   [7]=hdop [8]=alt [9]=M ...
    if (tok[0] && *tok[0]) fix.utc = (uint32_t)atof(tok[0]);
    fix.fix_quality = (uint8_t)atoi(tok[5]);
    fix.num_sv      = (uint8_t)atoi(tok[6]);
    fix.hdop        = (float)atof(tok[7]);
    fix.alt_m       = (float)atof(tok[8]);
    if (fix.fix_quality > 0 && tok[1] && tok[1][0] && tok[3] && tok[3][0]) {
        fix.lat = nmea_to_deg(tok[1], tok[2][0]);
        fix.lon = nmea_to_deg(tok[3], tok[4][0]);
        fix.valid = true;
    } else {
        fix.valid = false;      // fix dropped: clear the stale flag NOW so
    }                           // gps_trustworthy()/failsafe react immediately
    return true;
}

// ---------------------------------------------------------------------------
// RMC — velocity + course
// ---------------------------------------------------------------------------
static bool parse_rmc(char* body, GpsFix& fix) {
    //   [0]=time [1]=status [2]=lat [3]=N [4]=lon [5]=E
    //   [6]=knots [7]=course [8]=date ...
    char* tok[12];
    int n = split_fields(body, tok, 12);
    if (n < 8) return false;

    bool active = (tok[1] && tok[1][0] == 'A');
    if (!active) {
        fix.valid = false;      // status V (void) — no trustworthy position
        return true;
    }
    float knots = (float)atof(tok[6]);
    fix.ground_mps = knots * 0.514444f;          // 1 knot = 0.5144 m/s
    fix.course_deg = (float)atof(tok[7]);
    if (tok[2] && tok[2][0] && tok[4] && tok[4][0]) {
        fix.lat = nmea_to_deg(tok[2], tok[3][0]);
        fix.lon = nmea_to_deg(tok[4], tok[5][0]);
        fix.valid = true;
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

// ---------------------------------------------------------------------------
// Byte-stream front end — the u-blox 6 streams NMEA continuously, so the
// UART ISR/poll hands us arbitrary slices. This assembles complete lines and
// only counts checksum-valid ones (gps_lines_seen == "the module is talking
// at the right baud", which is what gps_healthy() reports).
// ---------------------------------------------------------------------------
static char     s_line[GPS_LINE_MAX];
static uint16_t s_line_len = 0;
static uint32_t s_lines_seen = 0;

void gps_line_reset() {
    s_line_len  = 0;
    s_lines_seen = 0;
}

uint32_t gps_lines_seen() { return s_lines_seen; }

bool gps_feed_byte(char c, GpsFix& fix) {
    if (c == '$') {
        // New sentence header: drop whatever partial frame we were holding
        // (recovers from a truncated frame or line noise mid-burst).
        s_line_len = 0;
    } else if (c == '\n' || c == '\r') {
        if (s_line_len == 0) return false;      // blank line / CR-LF pair
        s_line[s_line_len] = '\0';
        s_line_len = 0;
        if (nmea_checksum_ok(s_line)) s_lines_seen++;
        return gps_parse(s_line, fix);
    }
    if (s_line_len >= (uint16_t)(sizeof(s_line) - 1)) {
        s_line_len = 0;                         // overlong garbage: drop it
        return false;
    }
    s_line[s_line_len++] = c;
    return false;
}

void gps_feed(const char* data, size_t n, GpsFix& fix) {
    for (size_t i = 0; i < n; ++i) gps_feed_byte(data[i], fix);
}

// ---------------------------------------------------------------------------
// UBX config packets (u-blox 6 spec GPS.G6-SW-10018, §31 CFG-* messages)
//
// CK_A/CK_B = 8-bit Fletcher over class, id, length(2) and the payload —
// verified against the spec's own worked example:
//   CFG-RATE 200 ms => B5 62 06 08 06 00 C8 00 01 00 01 00 DE 6A
// ---------------------------------------------------------------------------
static void ubx_checksum(const uint8_t* p, size_t n, uint8_t* ck) {
    uint8_t a = 0, b = 0;
    for (size_t i = 0; i < n; ++i) { a = (uint8_t)(a + p[i]); b = (uint8_t)(b + a); }
    ck[0] = a; ck[1] = b;
}

// UBX-CFG-RATE (0x06 0x08): measRate(ms) | navRate(cycles) | timeRef.
// navRate is fixed at 1 on u-blox 5/6 (the spec says it cannot be changed);
// timeRef 1 = GPS time. measRate floor 200 ms = 5 Hz, the NEO-6M maximum.
int ubx_build_cfg_rate(uint8_t* out, size_t out_max, uint16_t meas_ms) {
    if (!out || out_max < 14) return 0;
    if (meas_ms < 200 || meas_ms > 1000) return 0;   // spec window we allow
    static const uint8_t hdr[4] = { 0x06, 0x08, 0x06, 0x00 };
    out[0] = 0xB5; out[1] = 0x62;
    memcpy(out + 2, hdr, 4);
    out[6]  = (uint8_t)(meas_ms & 0xFF);        // measRate LE
    out[7]  = (uint8_t)(meas_ms >> 8);
    out[8]  = 0x01; out[9] = 0x00;              // navRate = 1 (fixed on u-blox 6)
    out[10] = 0x01; out[11] = 0x00;             // timeRef = GPS
    ubx_checksum(out + 2, 10, out + 12);
    return 14;
}

// UBX-CFG-MSG (0x06 0x01), 8-byte payload form: msgClass, msgID, then the
// rate for each I/O target (DDC, UART1, UART2, USB, SPI, reserved). We only
// ever touch UART1 — everything else stays 0.
// NMEA sentence class is 0xF0: GGA=00 GLL=01 GSA=02 GSV=03 RMC=04 VTG=05.
int ubx_build_cfg_msg_nmea(uint8_t* out, size_t out_max,
                           uint8_t nmea_id, uint8_t uart1_rate) {
    if (!out || out_max < 16) return 0;
    if (uart1_rate > 1) return 0;                // on/off + 1x per epoch only
    out[0] = 0xB5; out[1] = 0x62;
    out[2] = 0x06; out[3] = 0x01;                // class CFG, id MSG
    out[4] = 0x08; out[5] = 0x00;                // payload length 8
    out[6] = 0xF0; out[7] = nmea_id;             // NMEA sentence
    out[8]  = 0x00;                              // DDC   (I2C)
    out[9]  = uart1_rate;                        // UART1 <- us
    out[10] = 0x00;                              // UART2
    out[11] = 0x00;                              // USB
    out[12] = 0x00;                              // SPI
    out[13] = 0x00;                              // reserved
    ubx_checksum(out + 2, 12, out + 14);
    return 16;
}

// UBX-CFG-PRT (0x06 0x00), 20-byte UART payload: portID, reserved0, txReady,
// mode, baudRate, inProtoMask, outProtoMask, flags, reserved2.
//   mode 0x000008D0 = 8 data bits, no parity, 1 stop bit (u-center's value:
//     charLen bits7-6 = 11, parity bits11-9 = 10x ("no parity"), stop = 00)
//   baud limited to the spec's allowed list (4800..460800).
// This is the packet that raises the GPS line rate: a 145-byte GGA takes
// 151 ms to clock out at 9600 vs 13 ms at 115200 — i.e. the fix the nav loop
// sees is that much fresher.
int ubx_build_cfg_prt_uart(uint8_t* out, size_t out_max, uint32_t baud) {
    if (!out || out_max < 28) return 0;
    switch (baud) {
        case 4800: case 9600: case 19200: case 38400:
        case 57600: case 115200: case 230400: case 460800:
            break;
        default: return 0;                    // not in the spec's baud list
    }
    out[0] = 0xB5; out[1] = 0x62;
    out[2] = 0x06; out[3] = 0x00;             // class CFG, id PRT
    out[4] = 0x14; out[5] = 0x00;             // payload length 20
    out[6]  = 0x01;                           // portID = UART1
    out[7]  = 0x00;                           // reserved0
    out[8]  = 0x00; out[9] = 0x00;            // txReady = off
    out[10] = 0xD0; out[11] = 0x08; out[12] = 0x00; out[13] = 0x00;  // 8N1
    out[14] = (uint8_t)(baud & 0xFF);         // baudRate LE
    out[15] = (uint8_t)((baud >> 8) & 0xFF);
    out[16] = (uint8_t)((baud >> 16) & 0xFF);
    out[17] = (uint8_t)((baud >> 24) & 0xFF);
    out[18] = 0x03; out[19] = 0x00;           // inProto  = UBX | NMEA
    out[20] = 0x03; out[21] = 0x00;           // outProto = UBX | NMEA
    out[22] = 0x00; out[23] = 0x00;           // flags
    out[24] = 0x00; out[25] = 0x00;           // reserved2
    ubx_checksum(out + 2, 24, out + 26);
    return 28;
}
