#include "sensors.h"
#include "imu_board.h"
#include "../config.h"
#include <math.h>

// ===========================================================================
// Barometer + magnetometer are board-selected (config.h); GPS and LiDAR are
// still stubs — see the TODO on each before closing checklist section C.
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

// --- GPS -------------------------------------------------------------------
static bool s_gps_ok = false;

bool gps_init() {
    // TODO: Serial1.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
    s_gps_ok = true;
    return s_gps_ok;
}

int gps_available() {
    // TODO: return Serial1.available();
    return 0;
}

void gps_poll(GpsFix& fix) {
    // TODO: read bytes, split on '\n', call gps_parse(line, fix).
    (void)fix;
}

bool gps_healthy() { return s_gps_ok; }

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
