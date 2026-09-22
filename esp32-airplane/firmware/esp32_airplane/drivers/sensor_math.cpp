#include "sensor_math.h"
#include <string.h>
#include <math.h>

// ===========================================================================
// MPU6050 / MPU9250
// ===========================================================================
float mpu_accel_lsb_per_g(uint8_t fs_bits) {
    switch (fs_bits & 0x03) {
        case 0: return 16384.0f;    // +/- 2g
        case 1: return  8192.0f;    // +/- 4g
        case 2: return  4096.0f;    // +/- 8g
        default:return  2048.0f;    // +/- 16g
    }
}

float mpu_gyro_lsb_per_dps(uint8_t fs_bits) {
    switch (fs_bits & 0x03) {
        case 0: return 131.0f;      // +/- 250 dps
        case 1: return  65.5f;      // +/- 500 dps
        case 2: return  32.8f;      // +/- 1000 dps
        default:return  16.4f;      // +/- 2000 dps
    }
}

Vec3 mpu_accel_to_g(const int16_t raw[3], uint8_t fs_bits) {
    float s = mpu_accel_lsb_per_g(fs_bits);
    Vec3 v;
    v.x = (float)raw[0] / s;
    v.y = (float)raw[1] / s;
    v.z = (float)raw[2] / s;
    return v;
}

Vec3 mpu_gyro_to_dps(const int16_t raw[3], uint8_t fs_bits) {
    float s = mpu_gyro_lsb_per_dps(fs_bits);
    Vec3 v;
    v.x = (float)raw[0] / s;
    v.y = (float)raw[1] / s;
    v.z = (float)raw[2] / s;
    return v;
}

// ===========================================================================
// HMC5883L — gain table from CONFIG_B[7:5]. 1 Ga = 100 uT.
// ===========================================================================
float hmc_lsb_per_ut(uint8_t config_b) {
    static const float lsb_per_ga[8] = {
        1370.0f, 1090.0f, 820.0f, 660.0f,
         440.0f,  390.0f, 330.0f, 230.0f
    };
    float gain = lsb_per_ga[(config_b >> 5) & 0x07];
    return gain / 100.0f;           // LSB per uT
}

Vec3 hmc_to_ut(const int16_t raw_xyz[3], uint8_t config_b) {
    // Register order on the wire is X, Z, Y — the caller must already have
    // reordered into raw_xyz = {X, Y, Z}.
    float s = hmc_lsb_per_ut(config_b);
    Vec3 v;
    v.x = (float)raw_xyz[0] / s;
    v.y = (float)raw_xyz[1] / s;
    v.z = (float)raw_xyz[2] / s;
    return v;
}

// ===========================================================================
// AK8963 (inside MPU9250)
// ===========================================================================
float ak8963_lsb_per_ut(bool sixteen_bit) {
    // Full scale is +/- 4912 uT in both modes.
    return sixteen_bit ? (4912.0f / 32768.0f)   // 0.1499 uT/LSB
                       : (4912.0f / 8192.0f);    // 0.5996 uT/LSB
}

Vec3 ak8963_to_ut(const int16_t raw_xyz[3], bool sixteen_bit, const float asa[3]) {
    float s = ak8963_lsb_per_ut(sixteen_bit);
    // Sensitivity-adjustment: gain = (ASA - 128)/256 + 1, ASA = 0 -> 128.
    float g[3] = {1.0f, 1.0f, 1.0f};
    if (asa) {
        g[0] = (asa[0] - 128.0f) / 256.0f + 1.0f;
        g[1] = (asa[1] - 128.0f) / 256.0f + 1.0f;
        g[2] = (asa[2] - 128.0f) / 256.0f + 1.0f;
    }
    Vec3 v;
    v.x = (float)raw_xyz[0] * s * g[0];
    v.y = (float)raw_xyz[1] * s * g[1];
    v.z = (float)raw_xyz[2] * s * g[2];
    return v;
}

// ===========================================================================
// BMP180 — Bosch worked example (datasheet section 3.5):
//   cal   AC1=408 AC2=-72 AC3=-14383 AC4=32741 AC5=32757 AC6=23153
//         B1=6190 B2=4 MB=-32767 MC=-8711 MD=2868
//   UT=27898  UP=23843  oss=0
//   => T = 150 (15.0 C), p = 69965 Pa
// ===========================================================================
static inline int16_t be16s(const uint8_t* p) {
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static inline uint16_t be16u(const uint8_t* p) {
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}
static inline int16_t le16s(const uint8_t* p) {
    return (int16_t)(((uint16_t)p[1] << 8) | p[0]);
}
static inline uint16_t le16u(const uint8_t* p) {
    return (uint16_t)(((uint16_t)p[1] << 8) | p[0]);
}

bool bmp180_parse_cal(const uint8_t raw22[22], Bmp180Cal& out) {
    if (!raw22) return false;
    out.ac1 = be16s(raw22 + 0);
    out.ac2 = be16s(raw22 + 2);
    out.ac3 = be16s(raw22 + 4);
    out.ac4 = be16u(raw22 + 6);
    out.ac5 = be16u(raw22 + 8);
    out.ac6 = be16u(raw22 + 10);
    out.b1  = be16s(raw22 + 12);
    out.b2  = be16s(raw22 + 14);
    out.mb  = be16s(raw22 + 16);
    out.mc  = be16s(raw22 + 18);
    out.md  = be16s(raw22 + 20);
    // A part with no calibration reads all 0x00 or all 0xFF.
    if (out.ac1 == 0 || out.ac1 == -1) return false;
    if (out.ac4 == 0 || out.ac4 == 0xFFFF) return false;
    return true;
}

// Kept as the two-step datasheet sequence because the pressure step needs B5.
static int32_t bmp180_b5(const Bmp180Cal& c, int32_t ut) {
    int32_t x1 = ((ut - (int32_t)c.ac6) * (int32_t)c.ac5) / 32768;
    int32_t x2 = ((int32_t)c.mc * 2048) / (x1 + (int32_t)c.md);
    return x1 + x2;
}

float bmp180_compensate_temp_c(const Bmp180Cal& c, int32_t ut) {
    int32_t b5 = bmp180_b5(c, ut);
    return (float)((b5 + 8) / 16) / 10.0f;       // 0.1 C units
}

float bmp180_compensate_press_pa(const Bmp180Cal& c, int32_t up,
                                 uint8_t oss, int32_t b5) {
    if (oss > 3) oss = 3;

    // NOTE: Bosch's reference algorithm uses division, not bit shifts. For
    // negative intermediates `/` truncates toward zero while `>>` floors, and
    // that one-count difference is what makes the worked example come out at
    // 69965 Pa instead of 69964. Keep the divisions.
    int32_t b6 = b5 - 4000;

    int32_t x1 = (b6 * b6) / 4096;
    x1 = (x1 * (int32_t)c.b2) / 2048;
    int32_t x2 = ((int32_t)c.ac2 * b6) / 2048;
    int32_t x3 = x1 + x2;
    int32_t b3 = ((((int32_t)c.ac1 * 4 + x3) << oss) + 2) / 4;

    x1 = ((int32_t)c.ac3 * b6) / 8192;
    x2 = ((int32_t)c.b1 * ((b6 * b6) / 4096)) / 65536;
    x3 = ((x1 + x2) + 2) / 4;

    uint32_t b4 = ((uint32_t)c.ac4 * (uint32_t)(x3 + 32768)) / 32768u;
    uint32_t b7 = ((uint32_t)((int32_t)up - b3)) * (uint32_t)(50000u >> oss);
    if (b4 == 0) return 0.0f;

    int32_t p;
    if (b7 < 0x80000000u) p = (int32_t)((b7 * 2u) / b4);
    else                  p = (int32_t)((b7 / b4) * 2u);

    x1 = (p / 256) * (p / 256);
    x1 = (x1 * 3038) / 65536;
    x2 = (-7357 * p) / 65536;
    p  = p + ((x1 + x2 + 3791) / 16);
    return (float)p;
}

void bmp180_compensate(const Bmp180Cal& c, int32_t up, int32_t ut,
                       uint8_t oss, float& press_pa, float& temp_c) {
    int32_t b5 = bmp180_b5(c, ut);
    temp_c   = (float)((b5 + 8) / 16) / 10.0f;
    press_pa = bmp180_compensate_press_pa(c, up, oss, b5);
}

// ===========================================================================
// BMP280 — Bosch reference integer code (datasheet section 3.11.3).
//   adc_T = 519888 with the standard dig_T set => T = 25.08 C
//   returns p in Pa (the reference code yields Q24.8)
// ===========================================================================
bool bmp280_parse_cal(const uint8_t raw24[24], Bmp280Cal& out) {
    if (!raw24) return false;
    out.t1 = le16u(raw24 + 0);
    out.t2 = le16s(raw24 + 2);
    out.t3 = le16s(raw24 + 4);
    out.p1 = le16u(raw24 + 6);
    out.p2 = le16s(raw24 + 8);
    out.p3 = le16s(raw24 + 10);
    out.p4 = le16s(raw24 + 12);
    out.p5 = le16s(raw24 + 14);
    out.p6 = le16s(raw24 + 16);
    out.p7 = le16s(raw24 + 18);
    out.p8 = le16s(raw24 + 20);
    out.p9 = le16s(raw24 + 22);
    if (out.t1 == 0 || out.t1 == 0xFFFF) return false;
    if (out.p1 == 0 || out.p1 == 0xFFFF) return false;
    return true;
}

float bmp280_compensate_temp_c(const Bmp280Cal& c, int32_t adc_t, int32_t& t_fine) {
    int32_t var1 = ((((adc_t >> 3) - ((int32_t)c.t1 << 1)) * (int32_t)c.t2) >> 11);
    int32_t d = (adc_t >> 4) - (int32_t)c.t1;
    int32_t var2 = (((((d * d) >> 12) * (int32_t)c.t3)) >> 14);
    t_fine = var1 + var2;
    return (float)((t_fine * 5 + 128) >> 8) / 100.0f;
}

float bmp280_compensate_press_pa(const Bmp280Cal& c, int32_t adc_p, int32_t t_fine) {
    int64_t var1 = (int64_t)t_fine - 128000;
    int64_t var2 = var1 * var1 * (int64_t)c.p6;
    var2 = var2 + ((var1 * (int64_t)c.p5) << 17);
    var2 = var2 + (((int64_t)c.p4) << 35);
    var1 = ((var1 * var1 * (int64_t)c.p3) >> 8) +
           ((var1 * (int64_t)c.p2) << 12);
    var1 = ((((int64_t)1 << 47) + var1) * (int64_t)c.p1) >> 33;
    if (var1 == 0) return 0.0f;                  // division by zero guard

    int64_t p = 1048576 - (int64_t)adc_p;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)c.p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)c.p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)c.p7) << 4);

    return (float)((double)p / 256.0);           // Q24.8 -> Pa
}

void bmp280_compensate(const Bmp280Cal& c, int32_t adc_p, int32_t adc_t,
                       float& press_pa, float& temp_c) {
    int32_t t_fine = 0;
    temp_c   = bmp280_compensate_temp_c(c, adc_t, t_fine);
    press_pa = bmp280_compensate_press_pa(c, adc_p, t_fine);
}
