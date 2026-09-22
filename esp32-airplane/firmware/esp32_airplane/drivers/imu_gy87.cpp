// drivers/imu_gy87.cpp — GY-87 board: MPU6050 + HMC5883L + BMP180
//
// Active only when config.h selects -DIMU_GY87.
// Same structure as imu_gy91.cpp; the I2C transactions are the only part that
// differs (different parts, different register maps).

#include "../config.h"                 // must come first: it defines IMU_GY87

#if defined(IMU_GY87)

#include "imu_board.h"
#include "sensor_math.h"
#include <math.h>
#include <string.h>

#ifdef ARDUINO
  #include <Wire.h>
#endif

// --- register map -----------------------------------------------------------
#define MPU6050_ADDR        IMU_I2C_ADDR
#define MPU_WHO_AM_I        0x75    // -> 0x68 (AD0 low) or 0x69 (AD0 high)
#define MPU_PWR_MGMT_1      0x6B
#define MPU_SMPLRT_DIV      0x19
#define MPU_CONFIG          0x1A
#define MPU_GYRO_CONFIG     0x1B
#define MPU_ACCEL_CONFIG    0x1C
#define MPU_ACCEL_XOUT_H    0x3B

#define HMC_ADDR            MAG_I2C_ADDR
#define HMC_CONFIG_A        0x00
#define HMC_CONFIG_B        0x01
#define HMC_MODE            0x02
#define HMC_DATA_X_MSB      0x03    // wire order is X, Z, Y
#define HMC_ID_A            0x0A    // -> 0x48, 0x34, 0x33

#define BMP180_ADDR         BARO_I2C_ADDR
#define BMP180_ID_REG       0xD0    // -> 0x55
#define BMP180_CAL          0xAA    // 22 bytes, big-endian
#define BMP180_CTRL         0xF4
#define BMP180_DATA         0xF6
#define BMP180_OSS          1       // standard mode, 8 ms conversion

// Full-scale selections written at init
#define GYRO_FS_BITS        0x03    // +/- 2000 dps
#define ACCEL_FS_BITS       0x03    // +/- 16 g
#define HMC_CONFIG_A_VAL    0x54    // 4-sample average, 30 Hz
#define HMC_CONFIG_B_VAL    0x20    // gain 1090 LSB/Ga

// --- state ------------------------------------------------------------------
static bool      s_imu_ok  = false;
static bool      s_baro_ok = false;
static bool      s_mag_ok  = false;
static Vec3      s_gyro_bias = {0.0f, 0.0f, 0.0f};
#ifdef ARDUINO
static Bmp180Cal s_baro_cal;
static float     s_last_temp_c = 25.0f;
#endif

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
    if (Wire.endTransmission(false) != 0) return false;
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
// IMU — MPU6050
// ===========================================================================
bool board_imu_init() {
    s_imu_ok = false;
#ifdef ARDUINO
    uint8_t who = 0;
    if (!i2c_read(MPU6050_ADDR, MPU_WHO_AM_I, &who, 1)) return false;
    if (who != 0x68 && who != 0x69) return false;

    i2c_write(MPU6050_ADDR, MPU_PWR_MGMT_1, 0x80);      // reset
    delay(100);
    i2c_write(MPU6050_ADDR, MPU_PWR_MGMT_1, 0x01);      // X-gyro PLL
    delay(200);

    uint8_t div = (uint8_t)((1000 / IMU_SAMPLE_HZ) - 1);
    i2c_write(MPU6050_ADDR, MPU_SMPLRT_DIV,  div);
    i2c_write(MPU6050_ADDR, MPU_CONFIG,      0x01);     // DLPF 184 Hz
    i2c_write(MPU6050_ADDR, MPU_GYRO_CONFIG, (uint8_t)(GYRO_FS_BITS  << 3));
    i2c_write(MPU6050_ADDR, MPU_ACCEL_CONFIG,(uint8_t)(ACCEL_FS_BITS << 3));

    s_imu_ok = true;
#else
    s_imu_ok = true;
#endif
    return s_imu_ok;
}

bool board_imu_read(Vec3& accel_g, Vec3& gyro_rps) {
    if (!s_imu_ok) return false;
#ifdef ARDUINO
    uint8_t raw[14];
    if (!i2c_read(MPU6050_ADDR, MPU_ACCEL_XOUT_H, raw, 14)) return false;

    int16_t a[3], g[3];
    a[0] = (int16_t)((raw[0] << 8)  | raw[1]);
    a[1] = (int16_t)((raw[2] << 8)  | raw[3]);
    a[2] = (int16_t)((raw[4] << 8)  | raw[5]);
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
        if (board_imu_read(a, g)) {
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
// Barometer — BMP180 (two-step: the pressure conversion needs B5 from the
// temperature step, so a temperature read is always performed first)
// ===========================================================================
bool board_baro_init() {
    s_baro_ok = false;
#ifdef ARDUINO
    uint8_t id = 0;
    if (!i2c_read(BMP180_ADDR, BMP180_ID_REG, &id, 1)) return false;
    if (id != 0x55) return false;

    uint8_t cal[22];
    if (!i2c_read(BMP180_ADDR, BMP180_CAL, cal, 22)) return false;
    if (!bmp180_parse_cal(cal, s_baro_cal)) return false;

    s_baro_ok = true;
#else
    s_baro_ok = true;
#endif
    return s_baro_ok;
}

bool board_baro_read(float& pressure_pa, float& temp_c) {
    if (!s_baro_ok) return false;
#ifdef ARDUINO
    // --- uncompensated temperature ---------------------------------------
    if (!i2c_write(BMP180_ADDR, BMP180_CTRL, 0x2E)) return false;
    delay(5);
    uint8_t t[2];
    if (!i2c_read(BMP180_ADDR, BMP180_DATA, t, 2)) return false;
    int32_t ut = (int32_t)(((uint32_t)t[0] << 8) | t[1]);

    // --- uncompensated pressure ------------------------------------------
    if (!i2c_write(BMP180_ADDR, BMP180_CTRL,
                   (uint8_t)(0x34 | (BMP180_OSS << 6)))) return false;
    delay(BMP180_OSS == 0 ? 5 : (BMP180_OSS == 1 ? 8 :
                                 (BMP180_OSS == 2 ? 14 : 26)));
    uint8_t p[3];
    if (!i2c_read(BMP180_ADDR, BMP180_DATA, p, 3)) return false;
    int32_t raw = (int32_t)(((uint32_t)p[0] << 16) |
                            ((uint32_t)p[1] << 8)  | p[2]);
    int32_t up  = raw >> (8 - BMP180_OSS);

    bmp180_compensate(s_baro_cal, up, ut, BMP180_OSS, pressure_pa, temp_c);
    s_last_temp_c = temp_c;
    return pressure_pa > 0.0f;
#else
    pressure_pa = 101325.0f;
    temp_c      = 25.0f;
    return true;
#endif
}

bool board_baro_healthy() { return s_baro_ok; }

// ===========================================================================
// Magnetometer — HMC5883L
// ===========================================================================
bool board_mag_init() {
    s_mag_ok = false;
    if (!CFG_USE_MAG) return true;
#ifdef ARDUINO
    uint8_t id[3] = {0, 0, 0};
    if (!i2c_read(HMC_ADDR, HMC_ID_A, id, 3)) return false;
    if (id[0] != 0x48 || id[1] != 0x34 || id[2] != 0x33) return false;

    i2c_write(HMC_ADDR, HMC_CONFIG_A, HMC_CONFIG_A_VAL);
    i2c_write(HMC_ADDR, HMC_CONFIG_B, HMC_CONFIG_B_VAL);
    i2c_write(HMC_ADDR, HMC_MODE,     0x00);     // continuous measurement

    s_mag_ok = true;
#else
    s_mag_ok = true;
#endif
    return s_mag_ok;
}

bool board_mag_read(Vec3& field_ut) {
    if (!s_mag_ok) { field_ut = {0.0f, 0.0f, 0.0f}; return false; }
#ifdef ARDUINO
    uint8_t raw[6];
    if (!i2c_read(HMC_ADDR, HMC_DATA_X_MSB, raw, 6)) return false;

    // Wire order is X, Z, Y — reorder into the {X, Y, Z} the maths expects.
    int16_t xyz[3];
    xyz[0] = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
    xyz[2] = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
    xyz[1] = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);

    // Overflow sentinel: all axes at the negative rail means the sensor
    // latched, not that the field is real.
    if (xyz[0] == -4096 && xyz[1] == -4096 && xyz[2] == -4096) return false;

    field_ut = hmc_to_ut(xyz, HMC_CONFIG_B_VAL);
    return true;
#else
    field_ut = {22.0f, 0.0f, -40.0f};
    return true;
#endif
}

bool board_mag_healthy() { return s_mag_ok; }

#endif // IMU_GY87
