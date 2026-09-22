// drivers/sensors.h — baro, magnetometer, GPS UART, LiDAR  (checklist C2/C3/C4/C6)
//
// Baro and magnetometer are implemented by the compile-time-selected board
// driver (drivers/imu_gy91.cpp / imu_gy87.cpp) — see config.h:
//   GY-91 -> BMP280 + AK8963      GY-87 -> BMP180 + HMC5883L
#pragma once
#include "../ahrs.h"
#include "../gps_nav.h"
#include <stdbool.h>

// --- Barometer -------------------------------------------------------------
bool  baro_init();
bool  baro_read(float& pressure_pa, float& temp_c);
float baro_altitude_m(float pressure_pa, float sea_level_pa);   // barometric formula
bool  baro_healthy();
// Seed the sea-level reference (defaults to ISA 101325 Pa).
void  baro_set_sea_level_pa(float pa);
float baro_sea_level_pa();

// --- Magnetometer ----------------------------------------------------------
bool mag_init();
bool mag_read(Vec3& field_ut);      // microtesla, body frame
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

// --- Board identity (telemetry field TF_IMU_MODEL) -------------------------
const char* sensors_board_name();
