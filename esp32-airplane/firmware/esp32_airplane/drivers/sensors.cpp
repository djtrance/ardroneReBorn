#include "sensors.h"
#include "imu_board.h"
#include "../config.h"
#include <math.h>
#include <string.h>            // memset (GPS probe scratch fix)
#ifdef ARDUINO
  #include <Arduino.h>          // Serial1, delay/millis — GPS UART (C4)
#endif

// ===========================================================================
// Barometer + magnetometer are board-selected (config.h); GPS (u-blox 6) is
// implemented below, LiDAR is still a stub — see the TODO on it before
// closing checklist section C.
// ===========================================================================

// --- Barometer -------------------------------------------------------------
static bool  s_baro_ok = false;
static float s_sea_level_pa = 101325.0f;

bool baro_init() {
    s_baro_ok = board_baro_init();
    return s_baro_ok;
}

bool baro_read(float& pressure_pa, float& temp_c) {
    if (!s_baro_ok) return false;
    return board_baro_read(pressure_pa, temp_c);
}

float baro_altitude_m(float pressure_pa, float sea_level_pa) {
    if (pressure_pa <= 0.0f) return 0.0f;
    // ISA barometric formula: h = 44330 * (1 - (p/p0)^0.1903)
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}

bool baro_healthy() { return s_baro_ok; }

void baro_set_sea_level_pa(float pa) {
    if (pa > 80000.0f && pa < 110000.0f) s_sea_level_pa = pa;
}
float baro_sea_level_pa() { return s_sea_level_pa; }

// --- Magnetometer ----------------------------------------------------------
static bool s_mag_ok = false;

bool mag_init() {
    s_mag_ok = board_mag_init();
    return s_mag_ok;
}

bool mag_read(Vec3& field_ut) {
    if (!s_mag_ok) return false;
    return board_mag_read(field_ut);
}

bool mag_healthy() { return s_mag_ok; }

// --- GPS — u-blox 6 (NEO-6M) on UART1, checklist C4 -----------------------
// Factory default is 9600 8N1 with GGA+GLL+GSA+GSV+RMC+VTG+TXT at 1 Hz
// (u-blox 6 spec App. A.5/A.11). Boot sequence:
//
//   1. Open 9600 and push the full config blindly (5 Hz + GGA/RMC only).
//      At factory baud the module flips within a frame, so the probe below
//      sees it quickly.
//   2. Probe for a checksum-valid NMEA line ("line seen" == "it lives on
//      this baud"). Not seen? Try GPS_BAUD_HI — a module whose config
//      survived in battery-backup RAM boots at 115200; re-push config there.
//      Nowhere? assume factory and let gps_poll() keep listening — health
//      stays false so preflight blocks arming.
//   3. Alive at 9600 and GPS_USE_HI_BAUD: UBX-CFG-PRT -> 115200, switch our
//      UART, and PROBE AGAIN. Only a line seen on the new baud counts as
//      success; otherwise revert (with its own re-probe, covering a module
//      that did switch but whose lines were missed) so the GPS is never left
//      mute. Win: 145-byte GGA latency 151 ms -> 13 ms, plus ~16x headroom
//      for GSV/GSA diagnostics later.
//
// Nothing is saved to flash: every packet is rebuilt on each boot, so a
// factory-fresh or wiped module needs no u-center setup. No ACK is read —
// UBX-CFG-* is idempotent and the probe (not the ACK) is what decides.
static bool     s_gps_ok = false;
static uint32_t s_gps_baud = GPS_BAUD;

// NMEA sentence ids we toggle (u-blox class 0xF0). We parse GGA (position)
// and RMC (speed/course) only — GGA also carries hdop/num_sv, so GSA/GSV
// are pure bandwidth here.
enum { NMEA_GGA = 0x00, NMEA_GLL = 0x01, NMEA_GSA = 0x02,
       NMEA_GSV = 0x03, NMEA_RMC = 0x04, NMEA_VTG = 0x05 };

#ifdef ARDUINO
static void gps_uart_begin(uint32_t baud) {
    Serial1.end();
    Serial1.begin(baud, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    while (Serial1.available() > 0) (void)Serial1.read();   // drop stale/garbage
}

// Block up to `ms` waiting for ONE checksum-valid NMEA line. The module
// emits NMEA from boot whatever its rate, so this doubles as a baud probe.
// (Timeout is >1 full cycle at the factory 1 Hz rate — see config.h.)
static bool gps_wait_line(uint32_t ms) {
    GpsFix scratch;
    memset(&scratch, 0, sizeof(scratch));
    uint32_t t0    = millis();
    uint32_t seen0 = gps_lines_seen();
    while (millis() - t0 < ms) {
        while (Serial1.available() > 0)
            gps_feed_byte((char)Serial1.read(), scratch);
        if (gps_lines_seen() > seen0) return true;
        delay(2);
    }
    return false;
}

// Rate + sentence slimming (GGA+RMC every epoch, everything else off),
// sent at whatever baud the module was last heard on.
static void gps_send_config() {
    uint8_t pkt[28];
    static const uint8_t k_keep[] = { NMEA_GGA, NMEA_RMC };
    static const uint8_t k_drop[] = { NMEA_GLL, NMEA_GSA, NMEA_GSV, NMEA_VTG };
    for (unsigned i = 0; i < sizeof(k_keep); ++i) {
        int n = ubx_build_cfg_msg_nmea(pkt, sizeof(pkt), k_keep[i], 1);
        if (n > 0) Serial1.write(pkt, n);
    }
    for (unsigned i = 0; i < sizeof(k_drop); ++i) {
        int n = ubx_build_cfg_msg_nmea(pkt, sizeof(pkt), k_drop[i], 0);
        if (n > 0) Serial1.write(pkt, n);
    }
    int n = ubx_build_cfg_rate(pkt, sizeof(pkt), GPS_RATE_MS);
    if (n > 0) Serial1.write(pkt, n);
    Serial1.flush();                           // config fully clocked out
}
#endif

bool gps_init() {
    gps_line_reset();
    s_gps_ok    = true;
    s_gps_baud  = GPS_BAUD;
#ifdef ARDUINO
    // 1) Factory-side attempt: config first, then listen.
    gps_uart_begin(GPS_BAUD);
    gps_send_config();
    bool alive = gps_wait_line(GPS_PROBE_TIMEOUT_MS);

    // 2) Maybe the module retained a previous config at the high baud.
    if (!alive) {
        gps_uart_begin(GPS_BAUD_HI);
        if (gps_wait_line(GPS_PROBE_TIMEOUT_MS)) {
            alive = true;
            s_gps_baud = GPS_BAUD_HI;
            gps_send_config();
        } else {
            gps_uart_begin(GPS_BAUD);        // leave the UART where factory
        }                                    // would be; polls keep trying
    }

    // 3) Raise the line rate — verified, never blind.
#if GPS_USE_HI_BAUD
    if (alive && s_gps_baud != GPS_BAUD_HI) {
        uint8_t pkt[28];
        int n = ubx_build_cfg_prt_uart(pkt, sizeof(pkt), GPS_BAUD_HI);
        if (n > 0) { Serial1.write(pkt, n); Serial1.flush(); }
        gps_uart_begin(GPS_BAUD_HI);         // module re-bauds as it processes
        if (gps_wait_line(GPS_PROBE_TIMEOUT_MS)) {
            s_gps_baud = GPS_BAUD_HI;         // verified on the new rate
        } else {
            gps_uart_begin(GPS_BAUD);        // CFG-PRT didn't take...
            if (gps_wait_line(GPS_PROBE_TIMEOUT_MS)) {
                s_gps_baud = GPS_BAUD;        // ...module stayed at factory
            } else {
                gps_uart_begin(GPS_BAUD_HI); // ...or it DID switch and we just
                if (gps_wait_line(GPS_PROBE_TIMEOUT_MS)) {   // missed the lines
                    s_gps_baud = GPS_BAUD_HI;
                } else {
                    gps_uart_begin(GPS_BAUD); // dead either way: sit on factory
                    s_gps_baud = GPS_BAUD;
                }
            }
        }
    }
#endif
#endif
    return s_gps_ok;
}

int gps_available() {
#ifdef ARDUINO
    return Serial1.available();
#else
    return 0;                // host: drive gps_feed_byte() directly (tests)
#endif
}

void gps_poll(GpsFix& fix) {
#ifdef ARDUINO
    while (Serial1.available() > 0)
        gps_feed_byte((char)Serial1.read(), fix);
#else
    (void)fix;
#endif
}

// Healthy = UART initialised AND at least one checksum-valid NMEA line seen
// (so a wrong baud, dead module or unconnected RX all report unhealthy).
bool gps_healthy() { return s_gps_ok && gps_lines_seen() > 0; }

// Line rate actually negotiated at boot (telemetry / init log, checklist C4).
uint32_t gps_baud() { return s_gps_baud; }

// --- LiDAR TFmini ----------------------------------------------------------
static bool s_lidar_ok = false;

bool lidar_init() {
    // TODO: Serial2.begin(LIDAR_BAUD, ..., PIN_LIDAR_RX, -1);
    s_lidar_ok = CFG_USE_LIDAR;
    return s_lidar_ok;
}

bool lidar_poll(LidarSample& s) {
    if (!s_lidar_ok) return false;
    // TODO: parse TFmini frame: 0x59 0x59 dist_l dist_h strength_l strength_h ...
    s.range_m = -1.0f;
    s.valid   = false;
    return true;
}

bool lidar_healthy() { return s_lidar_ok; }

// --- Board identification (telemetry field TF_IMU_MODEL) -------------------
const char* sensors_board_name() { return board_sensor_name(); }
