// drivers/imu_gy91.cpp — GY-91 board: MPU9250 + BMP280 (+ internal AK8963)
//
// Active only when config.h selects -DIMU_GY91 (the default).
// I2C transactions run under ARDUINO; on the host the same code returns a
// stationary 1 g / 0 rad/s signal so the control stack stays testable.

#include "../config.h"                 // must come first: it defines IMU_GY91

#if defined(IMU_GY91)

#include "imu_board.h"
#include "sensor_math.h"
#include <math.h>
#include <string.h>

#ifdef ARDUINO
  #include <Wire.h>
  #define GY91_BUS 1
#endif

// --- register map -----------------------------------------------------------
#define MPU9250_ADDR        0x68
#define REG_SMPLRT_DIV      0x19
#define REG_CONFIG          0x1A
#define REG_GYRO_CONFIG     0x1B
#define REG_ACCEL_CONFIG    0x1C
#define REG_ACCEL_CONFIG2   0x1D
#define REG_INT_PIN_CFG     0x37
#define REG_USER_CTRL       0x6A
#define REG_PWR_MGMT_1      0x6B
#define REG_ACCEL_XOUT_H    0x3B
#define REG_WHO_AM_I        0x75
#define WHO_AM_I_9250       0x71    // also 0x73 = MPU9255, 0x68 = MPU6500

#define AK8963_ADDR         0x0C
#define AK8963_WIA          0x00    // -> 0x48
#define AK8963_ST1          0x02
#define AK8963_HXL          0x03    // 7 bytes: X,Y,Z + ST2
#define AK8963_CNTL1        0x0A
#define AK8963_PWR          0x0B
#define AK8963_ASAX         0x10

#define BMP280_ADDR         BARO_I2C_ADDR
#define BMP280_ID_REG       0xD0    // -> 0x58
#define BMP280_RESET        0xE0
#define BMP280_CONFIG       0xF5
#define BMP280_CTRL         0xF4
#define BMP280_PRESS_MSB    0xF7    // 6 bytes: press then temp, 20-bit

// Full-scale selections written at init (see sensor_math.h for the scalings)
#define GYRO_FS_BITS        0x03    // +/- 2000 dps
#define ACCEL_FS_BITS       0x03    // +/- 16 g
#define AK8963_16BIT        true

// --- state ------------------------------------------------------------------
static bool s_imu_ok  = false;
static bool s_baro_ok = false;
static bool s_mag_ok  = false;

static Vec3     s_gyro_bias = {0.0f, 0.0f, 0.0f};
#ifdef ARDUINO
static Bmp280Cal s_baro_cal;
static int32_t  s_t_fine = 0;
static float    s_asa[3] = {128.0f, 128.0f, 128.0f};
#endif

// Samples averaged during the at-rest gyro calibration.
#define GYRO_CAL_SAMPLES    400

// ---------------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------------
#ifdef ARDUINO
static bool i2c_write(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static bool i2c_read(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t n) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;   // repeated start
    if (Wire.requestFrom((int)addr, (int)n) != (int)n) return false;
    for (uint8_t i = 0; i < n; i++) buf[i] = (uint8_t)Wire.read();
    return true;
}
#endif

const char* board_sensor_name() { return IMU_BOARD_NAME; }

void board_mount_rotation(float rpy_rad[3]) {
    rpy_rad[0] = rpy_rad[1] = rpy_rad[2] = 0.0f;   // TODO set once mounted (A4)
}

// ===========================================================================
// IMU — MPU9250
// ===========================================================================
bool board_imu_init() {
    s_imu_ok = false;
#ifdef ARDUINO
    uint8_t who = 0;
    if (!i2c_read(MPU9250_ADDR, REG_WHO_AM_I, &who, 1)) return false;
    if (who != WHO_AM_I_9250 && who != 0x73 && who != 0x68) return false;

    // Reset, then park the clock on the X-gyro PLL (more stable than the
    // internal RC oscillator, which drifts with temperature).
    i2c_write(MPU9250_ADDR, REG_PWR_MGMT_1, 0x80);
    delay(100);
    i2c_write(MPU9250_ADDR, REG_PWR_MGMT_1, 0x01);
    delay(200);

    // 1 kHz internal rate / (1 + div). IMU_SAMPLE_HZ = 500 -> div = 1.
    uint8_t div = (uint8_t)((1000 / IMU_SAMPLE_HZ) - 1);
    i2c_write(MPU9250_ADDR, REG_SMPLRT_DIV, div);

    // DLPF 184 Hz: passes the 400 Hz control bandwidth with modest aliasing.
    i2c_write(MPU9250_ADDR, REG_CONFIG,        0x01);
    i2c_write(MPU9250_ADDR, REG_ACCEL_CONFIG2, 0x01);
    i2c_write(MPU9250_ADDR, REG_GYRO_CONFIG,   (uint8_t)(GYRO_FS_BITS  << 3));
    i2c_write(MPU9250_ADDR, REG_ACCEL_CONFIG,  (uint8_t)(ACCEL_FS_BITS << 3));

    // Direct the AK8963 onto the shared bus instead of the MPU's internal
    // I2C master, then verify it answers.
    i2c_write(MPU9250_ADDR, REG_USER_CTRL,  0x00);
    i2c_write(MPU9250_ADDR, REG_INT_PIN_CFG, 0x12);   // bypass + clear-on-read

    s_imu_ok = true;
#else
    s_imu_ok = true;      // host: synthetic signal (checklist J2)
#endif
    return s_imu_ok;
}

bool board_imu_read(Vec3& accel_g, Vec3& gyro_rps) {
    if (!s_imu_ok) return false;
#ifdef ARDUINO
    uint8_t raw[14];
    if (!i2c_read(MPU9250_ADDR, REG_ACCEL_XOUT_H, raw, 14)) return false;

    int16_t a[3], g[3];
    a[0] = (int16_t)((raw[0] << 8)  | raw[1]);
    a[1] = (int16_t)((raw[2] << 8)  | raw[3]);
    a[2] = (int16_t)((raw[4] << 8)  | raw[5]);
    // raw[6..7] = temperature (unused for control)
    g[0] = (int16_t)((raw[8] << 8)  | raw[9]);
    g[1] = (int16_t)((raw[10] << 8) | raw[11]);
    g[2] = (int16_t)((raw[12] << 8) | raw[13]);

    accel_g  = mpu_accel_to_g(a, ACCEL_FS_BITS);
    Vec3 dps = mpu_gyro_to_dps(g, GYRO_FS_BITS);

    const float D2R = (float)M_PI / 180.0f;
    gyro_rps.x = (dps.x * D2R) - s_gyro_bias.x;
    gyro_rps.y = (dps.y * D2R) - s_gyro_bias.y;
    gyro_rps.z = (dps.z * D2R) - s_gyro_bias.z;
    return true;
#else
    (void)accel_g; (void)gyro_rps;
    accel_g  = {0.0f, 0.0f, 1.0f};
    gyro_rps = {0.0f, 0.0f, 0.0f};
    return true;
#endif
}

bool board_imu_calibrate_rest() {
#ifdef ARDUINO
    if (!s_imu_ok) return false;
    Vec3 sum = {0.0f, 0.0f, 0.0f};
    int ok = 0;
    for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
        Vec3 a, g;
        if (board_imu_read(a, g)) {           // still includes the old bias
            sum.x += g.x; sum.y += g.y; sum.z += g.z;
            ok++;
        }
        delay(1000 / IMU_SAMPLE_HZ);
    }
    if (ok < GYRO_CAL_SAMPLES / 2) return false;
    s_gyro_bias.x = sum.x / (float)ok;
    s_gyro_bias.y = sum.y / (float)ok;
    s_gyro_bias.z = sum.z / (float)ok;
    return true;
#else
    s_gyro_bias = {0.0f, 0.0f, 0.0f};
    return true;
#endif
}

bool board_imu_healthy() { return s_imu_ok; }

// ===========================================================================
// Barometer — BMP280
// ===========================================================================
bool board_baro_init() {
    s_baro_ok = false;
#ifdef ARDUINO
    uint8_t id = 0;
    if (!i2c_read(BMP280_ADDR, BMP280_ID_REG, &id, 1)) return false;
    if (id != 0x58) return false;

    i2c_write(BMP280_ADDR, BMP280_RESET, 0xB6);
    delay(5);

    uint8_t cal[24];
    if (!i2c_read(BMP280_ADDR, 0x88, cal, 24)) return false;
    if (!bmp280_parse_cal(cal, s_baro_cal)) return false;

    // Standby 0.5 ms, IIR x4 (filters prop-wash pressure pulses), then
    // temperature x2 + pressure x16 oversampling in normal mode.
    i2c_write(BMP280_ADDR, BMP280_CONFIG,  0x10);
    i2c_write(BMP280_ADDR, BMP280_CTRL,     0x57);

    s_baro_ok = true;
#else
    s_baro_ok = true;
#endif
    return s_baro_ok;
}

bool board_baro_read(float& pressure_pa, float& temp_c) {
    if (!s_baro_ok) return false;
#ifdef ARDUINO
    uint8_t raw[6];
    if (!i2c_read(BMP280_ADDR, BMP280_PRESS_MSB, raw, 6)) return false;

    int32_t adc_p = (int32_t)(((uint32_t)raw[0] << 12) |
                              ((uint32_t)raw[1] << 4)  |
                              ((uint32_t)raw[2] >> 4));
    int32_t adc_t = (int32_t)(((uint32_t)raw[3] << 12) |
                              ((uint32_t)raw[4] << 4)  |
                              ((uint32_t)raw[5] >> 4));

    temp_c     = bmp280_compensate_temp_c(s_baro_cal, adc_t, s_t_fine);
    pressure_pa = bmp280_compensate_press_pa(s_baro_cal, adc_p, s_t_fine);
    return pressure_pa > 0.0f;
#else
    pressure_pa = 101325.0f;
    temp_c      = 25.0f;
    return true;
#endif
}

bool board_baro_healthy() { return s_baro_ok; }

// ===========================================================================
// Magnetometer — AK8963 inside the MPU9250 (reached through the bypass path)
// ===========================================================================
bool board_mag_init() {
    s_mag_ok = false;
    if (!CFG_USE_MAG) return true;         // not fitted: report neutral
#ifdef ARDUINO
    uint8_t who = 0;
    if (!i2c_read(AK8963_ADDR, AK8963_WIA, &who, 1)) return false;
    if (who != 0x48) return false;

    // Read the factory sensitivity-adjustment fuse ROM, then power down and
    // switch to 16-bit continuous mode at 100 Hz (0x16).
    i2c_write(AK8963_ADDR, AK8963_PWR,    0x00);
    i2c_write(AK8963_ADDR, AK8963_CNTL1,  0x0F);
    delay(10);

    uint8_t asa[3] = {128, 128, 128};
    if (i2c_read(AK8963_ADDR, AK8963_ASAX, asa, 3)) {
        s_asa[0] = (float)asa[0];
        s_asa[1] = (float)asa[1];
        s_asa[2] = (float)asa[2];
    }

    i2c_write(AK8963_ADDR, AK8963_PWR,   0x00);
    delay(10);
    i2c_write(AK8963_ADDR, AK8963_CNTL1, 0x16);
    delay(10);

    s_mag_ok = true;
#else
    s_mag_ok = true;
#endif
    return s_mag_ok;
}

bool board_mag_read(Vec3& field_ut) {
    if (!s_mag_ok) { field_ut = {0.0f, 0.0f, 0.0f}; return false; }
#ifdef ARDUINO
    uint8_t st1 = 0;
    if (!i2c_read(AK8963_ADDR, AK8963_ST1, &st1, 1)) return false;
    if (!(st1 & 0x01)) return false;               // no new sample yet

    uint8_t raw[7];
    if (!i2c_read(AK8963_ADDR, AK8963_HXL, raw, 7)) return false;
    if (raw[6] & 0x08) return false;               // magnetic overflow

    int16_t xyz[3];
    xyz[0] = (int16_t)((raw[1] << 8) | raw[0]);    // little-endian
    xyz[1] = (int16_t)((raw[3] << 8) | raw[2]);
    xyz[2] = (int16_t)((raw[5] << 8) | raw[4]);

    field_ut = ak8963_to_ut(xyz, AK8963_16BIT, s_asa);
    return true;
#else
    field_ut = {22.0f, 0.0f, -40.0f};              // synthetic, north-pointing
    return true;
#endif
}

bool board_mag_healthy() { return s_mag_ok; }

#endif // IMU_GY91
