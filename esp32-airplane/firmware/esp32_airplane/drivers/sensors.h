// drivers/sensors.h — baro, magnetometer, GPS UART, LiDAR stubs (checklist C2/C3/C4/C6)
#pragma once
#include "../ahrs.h"
#include "../gps_nav.h"
#include <stdbool.h>

// --- Barometer (BMP388) ----------------------------------------------------
bool  baro_init();
bool  baro_read(float& pressure_pa, float& temp_c);
float baro_altitude_m(float pressure_pa, float sea_level_pa);   // barometric formula
bool  baro_healthy();

// --- Magnetometer (QMC5883) ------------------------------------------------
bool mag_init();
bool mag_read(Vec3& field_ut);
bool mag_healthy();

// --- GPS (UART NMEA) -------------------------------------------------------
bool gps_init();
// Pump bytes from the UART; call gps_available() then gps_poll().
int  gps_available();
// Reads up to `max` bytes, parses complete sentences, updates `fix`.
void gps_poll(GpsFix& fix);
bool gps_healthy();

// --- LiDAR TFmini ----------------------------------------------------------
struct LidarSample { float range_m; bool valid; };
bool lidar_init();
bool lidar_poll(LidarSample& s);
bool lidar_healthy();
