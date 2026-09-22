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

// --- GPS — u-blox 6 / NEO-6M (UART1 NMEA) ----------------------------- (C4)
// gps_init(): UART1 @ 9600 8N1 (factory baud), sends UBX-CFG-MSG to keep
// GGA+RMC / silence GLL+GSA+GSV+VTG, then UBX-CFG-RATE for GPS_RATE_MS
// (4 Hz). gps_healthy(): the module has produced a checksum-valid line.
// Byte-level path (gps_feed_byte / gps_feed / UBX builders) lives in
// gps_nav.h and is host-unit-tested.
bool gps_init();
int  gps_available();               // bytes waiting in the UART buffer
void gps_poll(GpsFix& fix);         // pump UART -> line assembler -> parser
bool gps_healthy();

// --- LiDAR TFmini ----------------------------------------------------------
struct LidarSample { float range_m; bool valid; };
bool lidar_init();
bool lidar_poll(LidarSample& s);
bool lidar_healthy();

// --- Board identity (telemetry field TF_IMU_MODEL) -------------------------
const char* sensors_board_name();
