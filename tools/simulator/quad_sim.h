#ifndef QUAD_SIM_H
#define QUAD_SIM_H

#include <stdint.h>
#include <stdbool.h>
#include "../../src/flight/adrc.h"

/* 6-DOF Quadcopter Physics Simulator
 *
 * For testing ADRC vs PID rate control in closed-loop.
 * Models: gravity, motor thrust, aerodynamic drag, inertia,
 * gyro/accel sensor noise, wind disturbances.
 *
 * Motor layout (AR.Drone 2.0 style, + configuration):
 *        front
 *       M0  M1
 *        \ /
 *    left  X  right
 *        / \
 *       M3  M2
 *        rear
 *
 * Control inputs: roll, pitch, yaw, throttle  (each -1..1)
 */

#define QUAD_SIM_DT    0.002f   /* 500 Hz physics */
#define QUAD_CTRL_DT   0.002f   /* 500 Hz control loop */

/* AR.Drone 2.0 approximate parameters */
#define QUAD_MASS       0.42f   /* kg (with battery) */
#define QUAD_ARM_LEN    0.225f  /* m (motor to CG distance) */
#define QUAD_Ixx        0.0028f /* kg·m² (roll inertia) */
#define QUAD_Iyy        0.0028f /* kg·m² (pitch inertia) */
#define QUAD_Izz        0.0055f /* kg·m² (yaw inertia) */
#define QUAD_THRUST_MAX 4.5f    /* N per motor at max RPM */
#define QUAD_DRAGCoeff  0.3f    /* aerodynamic drag coefficient */
#define QUAD_GRAVITY    9.81f   /* m/s² */

/* 3D vector */
typedef struct {
    float x, y, z;
} vec3_t;

/* Quaternion */
typedef struct {
    float w, x, y, z;
} quat_t;

/* Sensor readings (simulated) */
typedef struct {
    vec3_t accel;       /* m/s² (body frame, includes gravity) */
    vec3_t gyro;        /* rad/s (body frame) */
    float  baro_alt;    /* m (altitude from barometer) */
    float  baro_vel;    /* m/s (vertical velocity from baro) */
    float  gps_lat;     /* deg */
    float  gps_lon;     /* deg */
    float  gps_alt;     /* m */
    vec3_t gps_vel;     /* m/s (NED) */
    float  gps_hdop;    /* horizontal dilution of precision */
    float  temperature; /* °C */
} quad_sensors_t;

/* Wind disturbance */
typedef struct {
    vec3_t velocity;    /* m/s in NED frame */
    float  turbulence;  /* turbulence intensity (0..1) */
} quad_wind_t;

/* Motor commands (internal) */
typedef struct {
    float motor[4];     /* 0..1 per motor */
} quad_motors_t;

/* Full quadcopter state */
typedef struct {
    /* Position (NED frame, meters) */
    vec3_t pos;

    /* Velocity (NED frame, m/s) */
    vec3_t vel;

    /* Orientation (quaternion, body-to-NED) */
    quat_t quat;

    /* Angular velocity (body frame, rad/s) */
    vec3_t omega;

    /* Acceleration (body frame, m/s²) */
    vec3_t accel_body;

    /* Motor RPMs (normalized 0..1) */
    float motor_rpm[4];

    /* Altitude (barometer, m) */
    float baro_alt;

    /* GPS */
    float gps_lat;
    float gps_lon;
    float gps_alt;

    /* Timing */
    float time_s;
    uint32_t step_count;

    /* Sensor noise */
    float accel_noise;  /* m/s² std dev */
    float gyro_noise;   /* rad/s std dev */
    float baro_noise;   /* m std dev */
    float gps_noise;    /* m std dev */
} quad_state_t;

/* ADRC controller for each axis */
typedef struct {
    adrc_state_t roll_adrc;
    adrc_state_t pitch_adrc;
    adrc_state_t yaw_adrc;
    adrc_state_t alt_adrc;
    adrc_gains_t roll_gains;
    adrc_gains_t pitch_gains;
    adrc_gains_t yaw_gains;
    adrc_gains_t alt_gains;
} quad_adrc_controllers_t;

/* PID controller for each axis (for comparison) */
typedef struct {
    float kp, ki, kd;
    float imax;
    float integral;
    float prev_error;
} quad_pid_axis_t;

typedef struct {
    quad_pid_axis_t roll;
    quad_pid_axis_t pitch;
    quad_pid_axis_t yaw;
    quad_pid_axis_t alt;
} quad_pid_controllers_t;

/* Initialize quadcopter state (on ground, all zeros) */
void quad_init(quad_state_t *s);

/* Set initial altitude and hover state */
void quad_set_hover(quad_state_t *s, float alt_m);

/* Initialize ADRC controllers with default gains */
void quad_adrc_init(quad_adrc_controllers_t *c);

/* Initialize PID controllers with default gains */
void quad_pid_init(quad_pid_controllers_t *c);

/* Run one physics step (called at QUAD_SIM_DT)
 * Commands: roll, pitch, yaw, throttle (-1..1)
 * Wind disturbance applied if non-null
 */
void quad_step(quad_state_t *s, float roll, float pitch,
               float yaw, float throttle, const quad_wind_t *wind);

/* Run ADRC control step
 * Returns motor commands: roll, pitch, yaw, throttle (-1..1)
 */
void quad_adrc_control(quad_adrc_controllers_t *c,
                       const quad_state_t *s,
                       float target_roll, float target_pitch,
                       float target_yaw_rate, float target_alt,
                       float dt,
                       float *cmd_roll, float *cmd_pitch,
                       float *cmd_yaw, float *cmd_throttle);

/* Run PID control step (for comparison) */
void quad_pid_control(quad_pid_controllers_t *c,
                      const quad_state_t *s,
                      float target_roll, float target_pitch,
                      float target_yaw_rate, float target_alt,
                      float dt,
                      float *cmd_roll, float *cmd_pitch,
                      float *cmd_yaw, float *cmd_throttle);

/* Reset ADRC controllers */
void quad_adrc_reset(quad_adrc_controllers_t *c);

/* Reset PID controllers */
void quad_pid_reset(quad_pid_controllers_t *c);

/* Convert normalized motor commands to individual motor RPMs */
void quad_motor_mix(float cmd_roll, float cmd_pitch,
                    float cmd_yaw, float cmd_throttle,
                    float motors[4]);

#endif /* QUAD_SIM_H */
