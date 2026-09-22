// config.h — ALL tunable parameters and the pin map live here.
// Checklist references: B2 (pins), A6 (envelope), F2 (bandwidths), G4 (RTH).
//
// Every value marked TODO must be filled from the airframe before first flight.

#pragma once
#include <stdint.h>

// ---------------------------------------------------------------------------
// Build options
// ---------------------------------------------------------------------------
#define CFG_USE_MAG             1       // magnetometer for heading (checklist C3)
#define CFG_USE_LIDAR           1       // TFmini (C6)
#define CFG_USE_LD2450          1       // HLK-LD2450 radar (C7)
#define CFG_USE_AIRSPEED_GPS    1       // airspeed from GPS+wind (C5, start here)
#define CFG_CONTROLLER_ADRC     1       // 1 = ADRC (F1), 0 = PID cascade

// ---------------------------------------------------------------------------
// Pin map — checklist B2  (VALIDATE ON BENCH WITH PROPS REMOVED)
// ---------------------------------------------------------------------------
// LEDC PWM outputs (core 3.x Arduino API: ledcAttach(pin, freq, bits))
#define PIN_ELEVON_L            25      // LEDC
#define PIN_ELEVON_R            26      // LEDC
#define PIN_ESC_THROTTLE        27      // LEDC

// Buses
#define PIN_I2C_SDA             21
#define PIN_I2C_SCL             22
#define PIN_IMU_CS              5       // SPI chip select (IMU on SPI)
#define PIN_IMU_SCK             18
#define PIN_IMU_MISO            19
#define PIN_IMU_MOSI            23

#define PIN_GPS_RX              16      // UART2 (or UART1) — GPS TX -> ESP RX
#define PIN_GPS_TX              17
#define PIN_LIDAR_RX            34      // UART / soft-serial RX (input only pin)
#define PIN_RADAR_RX            35      // HLK-LD2450 RX

#define PIN_VBAT_ADC            32      // ADC1 battery divider (B4)
#define PIN_STATUS_LED          2
#define PIN_BUZZER              4       // optional (I5)
#define PIN_RC_INPUT            15      // i-BUS / CRSF (H2)

// ---------------------------------------------------------------------------
// PWM (ESC + servos) — checklist B3
// ---------------------------------------------------------------------------
#define PWM_FREQ_HZ             50      // 50 Hz: ESC + hobby servos
#define PWM_RES_BITS            16      // 16-bit -> ~0.3 us resolution
#define SERVO_MIN_US            1000
#define SERVO_MAX_US            2000
#define SERVO_CENTER_US         1500
#define ESC_MIN_US              1000     // idle
#define ESC_MAX_US              2000
#define ESC_ARM_US              1000     // arm at min throttle (D4)

// Per-surface mechanical trim & limits (D3) — TODO measure on bench
#define ELEVON_L_TRIM_US        1500
#define ELEVON_R_TRIM_US        1500
#define ELEVON_PITCH_MAX_US     400      // +/- us around trim, pitch
#define ELEVON_ROLL_MAX_US      400      // +/- us around trim, roll
#define CMD_RATE_LIMIT_DPS      400.0f   // deg/s output slew (D5)

// Adverse-yaw differential (D2) 0.0 .. 0.4
#define ELEVON_DIFFERENTIAL     0.15f

// ---------------------------------------------------------------------------
// Airframe performance envelope — checklist A6  ***BLOCKER***
// TODO: measure V_stall in flight test card J4-2 before trusting these.
// ---------------------------------------------------------------------------
#define V_STALL_MPS             9.0f     // TODO measure
#define V_MIN_MPS               11.7f    // 1.3 * V_STALL
#define V_CRUISE_MPS            15.0f
#define V_NE_MPS                30.0f     // never exceed
#define PHI_MAX_DEG             50.0f    // max bank
#define N_MAX_POS               3.0f      // +g
#define N_MAX_NEG              -1.5f      // -g
#define CLIMB_MAX_MPS           6.0f
#define SINK_MAX_MPS           -6.0f

// Mass / geometry (A1, A2) — TODO
#define MASS_KG                 0.80f     // TODO
#define WING_AREA_M2            0.30f     // TODO  (for future lift model)
#define RHO_SEA_LEVEL           1.225f

// ---------------------------------------------------------------------------
// Control bandwidths — checklist F2  (ADRC: wc = controller, wo = observer)
// ---------------------------------------------------------------------------
#if CFG_CONTROLLER_ADRC
  #define ROLL_RATE_WC          12.0f
  #define ROLL_RATE_WO          40.0f
  #define PITCH_RATE_WC         10.0f
  #define PITCH_RATE_WO         35.0f
  #define ROLL_ATT_WC           2.5f
  #define PITCH_ATT_WC          2.0f
  #define AIRSPEED_WC           0.6f
  #define AIRSPEED_WO           2.0f
  // b0 at reference airspeed; scheduled b0 = B0_REF * q/q_ref, clamped
  #define B0_ROLL_REF           9.0f      // TODO identify in SIL (F1)
  #define B0_PITCH_REF          6.0f      // TODO identify in SIL
  #define B0_MIN_SCALE          0.25f
  #define B0_MAX_SCALE          2.00f
#endif

// PID fallback (CFG_CONTROLLER_ADRC == 0)
#define PID_ROLL_RATE_KP        0.06f
#define PID_ROLL_RATE_KI        0.15f
#define PID_ROLL_RATE_KD        0.002f
#define PID_PITCH_RATE_KP       0.08f
#define PID_PITCH_RATE_KI       0.15f
#define PID_PITCH_RATE_KD       0.003f
#define PID_ROLL_ATT_KP         3.5f
#define PID_PITCH_ATT_KP        3.5f
#define PID_AIRSPEED_KP         0.08f
#define PID_AIRSPEED_KI         0.03f

// Reference dynamic pressure for b0 scheduling
#define Q_REF                   (0.5f * RHO_SEA_LEVEL * V_CRUISE_MPS * V_CRUISE_MPS)

// ---------------------------------------------------------------------------
// Loop rates — checklist E2/E3
// ---------------------------------------------------------------------------
#define DT_RATE_HZ              400      // rate loop + PWM out (core 0)
#define DT_ATT_HZ               100      // attitude + envelope
#define DT_NAV_HZ               25       // guidance / RTH
#define DT_IMU_HZ               500      // IMU read + AHRS
#define RATE_DT                 (1.0f / (float)DT_RATE_HZ)

// AHRS complementary filter weight (C8)
#define AHRS_ALPHA              0.98f
#define AHRS_YAW_ALPHA          0.95f

// ---------------------------------------------------------------------------
// Sensors — checklist C*
// ---------------------------------------------------------------------------
#define IMU_SAMPLE_HZ           500
#define IMU_GYRO_FS_DPS         2000
#define IMU_ACCEL_FS_G          16
#define BARO_I2C_ADDR           0x76     // BMP388 — check vs mag (C2)
#define MAG_I2C_ADDR            0x0D     // QMC5883
#define GPS_BAUD                9600     // NMEA (C4)
#define GPS_MIN_FIX_S           10       // seconds of stable 3D fix
#define GPS_MAX_HDOP            2.0f     // trust threshold
#define LIDAR_BAUD              115200
#define LD2450_BAUD             256000   // NOTE: unusual rate (C7)
#define LD2450_FRAME_LEN        30
#define LD2450_NTARGETS         3

// Battery thresholds (H3) — TODO per-cell chemistry
#define VBAT_WARN_CELL          3.60f
#define VBAT_RTH_CELL           3.50f
#define VBAT_CRIT_CELL          3.35f
#define VBAT_CELLS              3        // TODO

// ---------------------------------------------------------------------------
// Navigation / guidance — checklist G*
// ---------------------------------------------------------------------------
#define HOME_ACCEPT_RADIUS_M    30.0f    // waypoint acceptance radius
#define RTH_ALTITUDE_AGL_M      40.0f
#define RTH_LOITER_RADIUS_M     40.0f    // sized by PHI_MAX at cruise
#define L1_LOOKAHEAD_M          30.0f    // G2  L1 = 2 * R_turn guideline
#define GEOFENCE_RADIUS_M       300.0f
#define GEOFENCE_ALT_MIN_M      5.0f
#define GEOFENCE_ALT_MAX_M      120.0f
#define FLARE_ALT_M             1.8f     // G4
#define V_BEST_GLIDE_MPS        13.0f    // H5 deadstick glide speed

// Failsafe timing (H2)
#define RC_LOSS_TIMEOUT_MS      800
#define NAV_LOSS_TIMEOUT_MS     3000
#define PREFLIGHT_MAX_JITTER_US 200

// ---------------------------------------------------------------------------
// Logging (I3)
// ---------------------------------------------------------------------------
#define LOG_RATE_HZ             25
#define LOG_RING_KB             256
