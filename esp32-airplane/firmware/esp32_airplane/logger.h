// logger.h — real-time CSV flight/calibration logger (checklist I3)
//
// One sample struct -> one CSV line. The line goes to every registered sink;
// on the aircraft that is USB serial (always) and UDP port LOG_UDP_PORT
// (once a ground tool subscribes by sending any datagram — see
// tools/wing_logger/capture.py). The formatter itself is pure C so the exact
// on-wire format is unit-tested on the host: if a column is added or a
// decimal moves, `make test` fails before anything is flashed.
//
// Column meaning and the analysis workflow live in
//   docs/test-campaign.md  (logger section + T0..T3 test campaign)
#pragma once
#include <stddef.h>
#include <stdint.h>

#define LOGGER_MAX_LINE     384          // worst-case line incl. NUL

// env flag bits (packed into LogSample.env)
#define LOG_ENV_STALL       0x01
#define LOG_ENV_BANK        0x02
#define LOG_ENV_VNE         0x04
#define LOG_ENV_G           0x08

// wind classification (F6 — classifier not implemented yet; logs pre-wire
// the field so old and new logs share one schema)
#define LOG_WIND_UNKNOWN    0
#define LOG_WIND_CALM       1            // |w| <  WIND_CALM_MAX_MPS
#define LOG_WIND_SOFT       2
#define LOG_WIND_STRONG     3

// All floats are SI/normalised as documented per field; attitude in degrees
// for human-readable analysis. Field order matches logger_csv_header().
struct LogSample {
    uint32_t t_ms;                // millis() since boot
    uint8_t  mode;                // 0 = stabilize, 1 = passthrough (etapa 1)
    uint8_t  armed;
    uint8_t  rc_ok;
    uint8_t  phase;               // guidance Phase (guidance.h)
    uint8_t  fs_evt;              // FsEvent (failsafe.h)
    uint8_t  env;                 // LOG_ENV_* bitmask
    uint8_t  wind;                // LOG_WIND_* class
    float    rc_p, rc_r, rc_t, rc_y;   // mapped sticks after expo [-1,1]/[0,1]
    uint16_t raw_p, raw_r, raw_t, raw_y; // raw channel units (TX curve debug)
    float    ax, ay, az;          // accel, g (last IMU sample)
    float    gx, gy, gz;          // gyro, rad/s
    float    baro_pa, baro_c;
    float    roll, pitch, yaw;    // AHRS, deg
    uint8_t  fix, sv;
    float    hdop, gps_v, gps_alt, gps_trk, vest;   // m/s, m, deg, m/s
    float    phi_cmd;             // commanded bank, rad
    uint16_t l_us, r_us;          // surface commands, us
    float    thr_out;             // [0,1]
    uint32_t overruns;
};

// Header line (single, NUL-terminated, includes the trailing newline).
const char* logger_csv_header(void);

// Format one sample. Returns bytes written (excl. NUL), or 0 on any failure
// (null buffer, cap too small, snprintf error) — callers must drop the line
// rather than emit a truncated one.
size_t logger_format(char* buf, size_t cap, const LogSample& s);
