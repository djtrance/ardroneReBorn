// drivers/imu_board.h — board-level sensor bundle.
//
// A "board" is one physical module: accelerometer+gyroscope, barometer and
// magnetometer ship together and share the I2C bus, so they are selected as a
// unit. The selection is COMPILE-TIME (config.h):
//
//   -DIMU_GY91   GY-91  -> MPU9250 + BMP280 (+ AK8963 inside the MPU9250)
//   -DIMU_GY87   GY-87  -> MPU6050 + HMC5883L + BMP180
//
// Both board files are always compiled (the Arduino IDE builds every .cpp in
// the sketch folder) but each one is wrapped in `#if defined(IMU_...)`, so
// exactly one provides these symbols. Selecting both or neither is a
// #error in config.h.
//
// On the host (no ARDUINO) both still compile and return a plausible
// stationary signal, so the control/guidance stack stays unit-testable.

#pragma once
#include "../ahrs.h"

// --- accelerometer + gyroscope (checklist C1) -------------------------------
bool board_imu_init();
bool board_imu_read(Vec3& accel_g, Vec3& gyro_rps);   // g and rad/s
bool board_imu_calibrate_rest();                      // capture gyro bias
bool board_imu_healthy();

// --- barometer (checklist C2) ----------------------------------------------
bool board_baro_init();
bool board_baro_read(float& pressure_pa, float& temp_c);
bool board_baro_healthy();

// --- magnetometer (checklist C3) -------------------------------------------
bool board_mag_init();
bool board_mag_read(Vec3& field_ut);                  // microtesla, body frame
bool board_mag_healthy();

// Human-readable part name, for telemetry (TF_IMU_MODEL) and boot logs.
const char* board_sensor_name();

// Mount rotation of the sensor package relative to the body frame, in radians.
// GY-91 and GY-87 breakouts both sit flat with +X forward on most mounts, so
// this defaults to identity; set it once the board orientation is fixed (A4).
void board_mount_rotation(float rpy_rad[3]);
