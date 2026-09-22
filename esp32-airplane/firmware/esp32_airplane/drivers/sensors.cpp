#include "sensors.h"
#include "imu_board.h"
#include "../config.h"
#include <math.h>
#ifdef ARDUINO
  #include <Arduino.h>          // Serial1, delay — GPS UART (C4)
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
// (u-blox 6 spec App. A.5/A.11). We keep the factory baud (never re-baud —
// a failed baud change would leave the module mute with no ACK to notice)
// and instead slim the output to GGA+RMC at GPS_RATE_MS (4 Hz), which fits
// the 9600-baud line with ~40% headroom.
//
// Everything is re-sent on every boot, so a factory-fresh or wiped module
// needs no manual u-center setup. No ACK is read: UBX-CFG-* is idempotent,
// and the parser only ever accepts checksum-valid NMEA, so a half-applied
// config degrades to "fewer sentences", never to garbage fixes.
static bool s_gps_ok = false;

// NMEA sentence ids we toggle (u-blox class 0xF0). We parse GGA (position)
// and RMC (speed/course) only — GGA also carries hdop/num_sv, so GSA/GSV
// are pure bandwidth here.
enum { NMEA_GGA = 0x00, NMEA_GLL = 0x01, NMEA_GSA = 0x02,
       NMEA_GSV = 0x03, NMEA_RMC = 0x04, NMEA_VTG = 0x05 };

bool gps_init() {
    gps_line_reset();
    s_gps_ok = true;
#ifdef ARDUINO
    Serial1.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    delay(100);                                  // module finishes booting
    uint8_t pkt[16];
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
    {
        int n = ubx_build_cfg_rate(pkt, sizeof(pkt), GPS_RATE_MS);
        if (n > 0) Serial1.write(pkt, n);
    }
    Serial1.flush();                             // config fully clocked out
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
