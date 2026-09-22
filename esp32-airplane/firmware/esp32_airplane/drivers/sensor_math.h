// drivers/sensor_math.h — pure sensor conversion, no bus, no Arduino.
//
// Every formula here is transcribed from the part's datasheet so it can be
// unit-tested on the host (checklist J2) before the I2C transactions are ever
// exercised on hardware.
//
//   MPU6050 / MPU9250   accel + gyro full-scale scaling
//   HMC5883L            gain-scaled magnetometer
//   AK8963              MPU9250's internal magnetometer (+ ASA adjust)
//   BMP180              datasheet pressure/temperature compensation
//   BMP280              datasheet pressure/temperature compensation
//
// The BMP180 and BMP280 vectors below come straight from the datasheet
// worked examples, which makes them good regression tests.

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "../ahrs.h"

// ---------------------------------------------------------------------------
// MPU6050 / MPU9250 — accel and gyro share the same full-scale encoding
//   ACCEL_CONFIG[4:3] = 00 +/-2g   16384 LSB/g
//                       01 +/-4g    8192 LSB/g
//                       10 +/-8g    4096 LSB/g
//                       11 +/-16g   2048 LSB/g
//   GYRO_CONFIG[4:3]  = 00 +/-250dps   131 LSB/dps
//                       01 +/-500dps 65.5 LSB/dps
//                       10 +/-1000dps 32.8 LSB/dps
//                       11 +/-2000dps  16.4 LSB/dps
//   `fs_bits` is the 2-bit field value (0..3).
// ---------------------------------------------------------------------------
float mpu_accel_lsb_per_g(uint8_t fs_bits);
float mpu_gyro_lsb_per_dps(uint8_t fs_bits);
Vec3  mpu_accel_to_g(const int16_t raw[3], uint8_t fs_bits);
Vec3  mpu_gyro_to_dps(const int16_t raw[3], uint8_t fs_bits);

// ---------------------------------------------------------------------------
// HMC5883L — 12-bit, gain set by CONFIG_B.
// Default gain 0b010 => 1090 LSB/Ga, and 1 Ga = 100 uT.
// ---------------------------------------------------------------------------
float hmc_lsb_per_ut(uint8_t config_b);
Vec3  hmc_to_ut(const int16_t raw_xyz[3], uint8_t config_b);

// ---------------------------------------------------------------------------
// AK8963 — inside the MPU9250.
//   CNTL1 bit4 = 1 -> 16-bit mode (4912 / 32768 uT per LSB)
//               = 0 -> 14-bit mode (4912 / 8192  uT per LSB)
//   ASA registers (fuse ROM) adjust sensitivity: gain = (asa - 128)/256 + 1
// ---------------------------------------------------------------------------
float ak8963_lsb_per_ut(bool sixteen_bit);
Vec3  ak8963_to_ut(const int16_t raw_xyz[3], bool sixteen_bit,
                   const float asa[3]);

// ---------------------------------------------------------------------------
// BMP180 — calibration block read from 0xAA..0xBF (11 x int16 big-endian).
// ---------------------------------------------------------------------------
struct Bmp180Cal {
    int16_t   ac1, ac2, ac3;
    uint16_t  ac4, ac5, ac6;
    int16_t   b1, b2, mb, mc, md;
};
// Decode the 22-byte calibration block (0xAA..0xBF).
bool bmp180_parse_cal(const uint8_t raw22[22], Bmp180Cal& out);
// Temperature in degC from the uncompensated temperature word.
float bmp180_compensate_temp_c(const Bmp180Cal& c, int32_t ut);
// Pressure in Pa. `b5` is the intermediate from the temperature step — call
// bmp180_compensate_temp_c() first (the datasheet algorithm needs it).
float bmp180_compensate_press_pa(const Bmp180Cal& c, int32_t up,
                                 uint8_t oss, int32_t b5);
// Convenience: run both steps, returning pressure and temperature together.
void  bmp180_compensate(const Bmp180Cal& c, int32_t up, int32_t ut,
                        uint8_t oss, float& press_pa, float& temp_c);

// ---------------------------------------------------------------------------
// BMP280 — calibration block read from 0x88..0x9F (24 bytes, little-endian).
// ---------------------------------------------------------------------------
struct Bmp280Cal {
    uint16_t t1;  int16_t t2,  t3;
    uint16_t p1;  int16_t p2,  p3,  p4,  p5,  p6,  p7,  p8,  p9;
};
bool  bmp280_parse_cal(const uint8_t raw24[24], Bmp280Cal& out);
// Returns degC; `t_fine` is the shared intermediate the pressure step needs.
float bmp280_compensate_temp_c(const Bmp280Cal& c, int32_t adc_t, int32_t& t_fine);
float bmp280_compensate_press_pa(const Bmp280Cal& c, int32_t adc_p, int32_t t_fine);
void  bmp280_compensate(const Bmp280Cal& c, int32_t adc_p, int32_t adc_t,
                        float& press_pa, float& temp_c);
