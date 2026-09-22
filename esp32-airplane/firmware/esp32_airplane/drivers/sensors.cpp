#include "sensors.h"
#include "../config.h"
#include <math.h>

// ===========================================================================
// All drivers in this file are STUBS so the firmware compiles and the control
// + guidance stack can be exercised with synthetic data. Replace each TODO
// before marking the corresponding checklist section (§C) done.
// ===========================================================================

// --- Barometer -------------------------------------------------------------
static bool s_baro_ok = false;
static float s_sea_level_pa = 101325.0f;

bool baro_init() {
    // TODO: Wire.begin(); BMP388 chip-id check at BARO_I2C_ADDR; soft IIR.
    s_baro_ok = true;
    return s_baro_ok;
}

bool baro_read(float& pressure_pa, float& temp_c) {
    if (!s_baro_ok) return false;
    // TODO: real read.
    pressure_pa = s_sea_level_pa;   // stub: sea level => 0 m relative
    temp_c = 25.0f;
    return true;
}

float baro_altitude_m(float pressure_pa, float sea_level_pa) {
    if (pressure_pa <= 0.0f) return 0.0f;
    // ISA barometric formula: h = 44330 * (1 - (p/p0)^0.1903)
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}

bool baro_healthy() { return s_baro_ok; }

// --- Magnetometer ----------------------------------------------------------
static bool s_mag_ok = false;

bool mag_init() {
    // TODO: QMC5883 init at MAG_I2C_ADDR; load soft/hard-iron cal (C3).
    s_mag_ok = CFG_USE_MAG;
    return s_mag_ok;
}

bool mag_read(Vec3& field_ut) {
    if (!s_mag_ok) return false;
    // TODO: real read. Stub: horizontal field pointing north.
    field_ut = {22.0f, 0.0f, -40.0f};
    return true;
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
