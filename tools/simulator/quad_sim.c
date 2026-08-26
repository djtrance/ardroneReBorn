#include "quad_sim.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ================================================================== */
/*  Utility functions                                                  */
/* ================================================================== */

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float randf(void) {
    /* Box-Muller transform for Gaussian noise */
    float u1 = (float)rand() / (float)RAND_MAX;
    float u2 = (float)rand() / (float)RAND_MAX;
    if (u1 < 1e-6f) u1 = 1e-6f;
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}

/* Quaternion to rotation matrix (body-to-NED) */
static void quat_to_dcm(const quat_t *q, float R[3][3]) {
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;
    R[0][0] = 1 - 2*(qy*qy + qz*qz);
    R[0][1] = 2*(qx*qy - qz*qw);
    R[0][2] = 2*(qx*qz + qy*qw);
    R[1][0] = 2*(qx*qy + qz*qw);
    R[1][1] = 1 - 2*(qx*qx + qz*qz);
    R[1][2] = 2*(qy*qz - qx*qw);
    R[2][0] = 2*(qx*qz - qy*qw);
    R[2][1] = 2*(qy*qz + qx*qw);
    R[2][2] = 1 - 2*(qx*qx + qy*qy);
}

/* Rotate vector from body to NED frame */
static vec3_t rotate_body_to_ned(const quat_t *q, const vec3_t *v) {
    float R[3][3];
    quat_to_dcm(q, R);
    vec3_t out;
    out.x = R[0][0]*v->x + R[0][1]*v->y + R[0][2]*v->z;
    out.y = R[1][0]*v->x + R[1][1]*v->y + R[1][2]*v->z;
    out.z = R[2][0]*v->x + R[2][1]*v->y + R[2][2]*v->z;
    return out;
}

/* Quaternion integration: q̇ = 0.5 * q ⊗ ω (first-order Euler) */
static quat_t quat_integrate(const quat_t *q, const vec3_t *omega, float dt) {
    /* q̇ = 0.5 * q ⊗ (0, ωx, ωy, ωz) */
    float qw = q->w, qx = q->x, qy = q->y, qz = q->z;
    float wx = omega->x, wy = omega->y, wz = omega->z;

    float dqdt_w = 0.5f * (-qx*wx - qy*wy - qz*wz);
    float dqdt_x = 0.5f * ( qw*wx + qy*wz - qz*wy);
    float dqdt_y = 0.5f * ( qw*wy - qx*wz + qz*wx);
    float dqdt_z = 0.5f * ( qw*wz + qx*wy - qy*wx);

    /* Euler step: q_new = q + q̇·dt */
    quat_t q_new;
    q_new.w = qw + dqdt_w * dt;
    q_new.x = qx + dqdt_x * dt;
    q_new.y = qy + dqdt_y * dt;
    q_new.z = qz + dqdt_z * dt;

    /* Normalize */
    float norm = sqrtf(q_new.w*q_new.w + q_new.x*q_new.x +
                       q_new.y*q_new.y + q_new.z*q_new.z);
    if (norm > 1e-6f) {
        q_new.w /= norm; q_new.x /= norm;
        q_new.y /= norm; q_new.z /= norm;
    }
    return q_new;
}

/* ================================================================== */
/*  Quadcopter Initialization                                           */
/* ================================================================== */

void quad_init(quad_state_t *s) {
    memset(s, 0, sizeof(quad_state_t));
    s->quat.w = 1.0f;  /* identity rotation */
    s->accel_noise = 0.1f;
    s->gyro_noise = 0.01f;
    s->baro_noise = 0.1f;
    s->gps_noise = 1.0f;
}

void quad_set_hover(quad_state_t *s, float alt_m) {
    s->pos.z = -alt_m;  /* NED: z is down */
    s->baro_alt = alt_m;
    s->gps_alt = alt_m;
}

/* ================================================================== */
/*  Motor Mixing                                                       */
/* ================================================================== */

void quad_motor_mix(float cmd_roll, float cmd_pitch,
                    float cmd_yaw, float cmd_throttle,
                    float motors[4]) {
    /* + configuration mixing
     * M0 (front-left):  throttle + pitch + roll - yaw
     * M1 (front-right): throttle + pitch - roll + yaw
     * M2 (rear-right):  throttle - pitch - roll - yaw
     * M3 (rear-left):   throttle - pitch + roll + yaw
     */
    float t = cmd_throttle;
    float r = cmd_roll;
    float p = cmd_pitch;
    float y = cmd_yaw;

    motors[0] = clampf(t + p + r - y, 0.0f, 1.0f);
    motors[1] = clampf(t + p - r + y, 0.0f, 1.0f);
    motors[2] = clampf(t - p - r - y, 0.0f, 1.0f);
    motors[3] = clampf(t - p + r + y, 0.0f, 1.0f);
}

/* ================================================================== */
/*  Physics Step (6-DOF)                                               */
/* ================================================================== */

void quad_step(quad_state_t *s, float cmd_roll, float cmd_pitch,
               float cmd_yaw, float cmd_throttle, const quad_wind_t *wind) {
    float dt = QUAD_SIM_DT;
    float motors[4];

    /* Motor mixing */
    quad_motor_mix(cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle, motors);
    for (int i = 0; i < 4; i++) s->motor_rpm[i] = motors[i];

    /* --- Forces in body frame --- */
    /* Total thrust (all motors, N) */
    float total_thrust = 0.0f;
    for (int i = 0; i < 4; i++) {
        total_thrust += motors[i] * QUAD_THRUST_MAX;
    }

    /* Thrust vector in body frame (z-up in body) */
    vec3_t thrust_body = { 0.0f, 0.0f, -total_thrust };

    /* Rotate thrust to NED frame */
    vec3_t thrust_ned = rotate_body_to_ned(&s->quat, &thrust_body);

    /* Gravity in NED frame */
    vec3_t gravity = { 0.0f, 0.0f, QUAD_GRAVITY };

    /* Aerodynamic drag in NED frame (proportional to v²) */
    vec3_t drag_ned = { 0.0f, 0.0f, 0.0f };
    float speed = sqrtf(s->vel.x*s->vel.x + s->vel.y*s->vel.y + s->vel.z*s->vel.z);
    if (speed > 0.01f) {
        float drag_mag = QUAD_DRAGCoeff * speed * speed;
        drag_ned.x = -drag_mag * s->vel.x / speed;
        drag_ned.y = -drag_mag * s->vel.y / speed;
        drag_ned.z = -drag_mag * s->vel.z / speed;
    }

    /* Wind disturbance (aerodynamic drag on airframe) */
    vec3_t wind_force = { 0.0f, 0.0f, 0.0f };
    if (wind) {
        /* Wind applies force proportional to relative velocity squared */
        float rel_vx = wind->velocity.x - s->vel.x;
        float rel_vy = wind->velocity.y - s->vel.y;
        float rel_vz = wind->velocity.z - s->vel.z;
        float drag_area = 0.08f;  /* effective drag area in m² */
        float air_density = 1.225f;  /* kg/m³ */
        float Cd = 1.2;  /* drag coefficient for bluff body */

        float rel_speed_xy = sqrtf(rel_vx*rel_vx + rel_vy*rel_vy);
        if (rel_speed_xy > 0.01f) {
            float drag_xy = 0.5f * air_density * Cd * drag_area * rel_speed_xy * rel_speed_xy;
            wind_force.x = drag_xy * rel_vx / rel_speed_xy;
            wind_force.y = drag_xy * rel_vy / rel_speed_xy;
        }
        float drag_z = 0.5f * air_density * Cd * drag_area * 0.02f * rel_vz * fabsf(rel_vz);
        wind_force.z = drag_z;

        /* Add turbulence */
        if (wind->turbulence > 0.0f) {
            float t = wind->turbulence;
            wind_force.x += randf() * t * 0.3f;
            wind_force.y += randf() * t * 0.3f;
            wind_force.z += randf() * t * 0.1f;
        }
    }

    /* Total acceleration in NED */
    vec3_t acc_ned;
    acc_ned.x = (thrust_ned.x + drag_ned.x + wind_force.x) / QUAD_MASS;
    acc_ned.y = (thrust_ned.y + drag_ned.y + wind_force.y) / QUAD_MASS;
    acc_ned.z = (thrust_ned.z + drag_ned.z + wind_force.z) / QUAD_MASS + gravity.z;

    /* Integrate velocity and position (Euler) */
    s->vel.x += acc_ned.x * dt;
    s->vel.y += acc_ned.y * dt;
    s->vel.z += acc_ned.z * dt;
    s->pos.x += s->vel.x * dt;
    s->pos.y += s->vel.y * dt;
    s->pos.z += s->vel.z * dt;

    /* Ground constraint */
    if (s->pos.z > 0.0f) {
        s->pos.z = 0.0f;
        if (s->vel.z > 0.0f) s->vel.z = 0.0f;
    }

    /* --- Torques in body frame --- */
    float arm = QUAD_ARM_LEN;
    /* Roll torque (M0+M3 vs M1+M2) */
    float tau_roll = arm * QUAD_THRUST_MAX *
        ((motors[0] + motors[3]) - (motors[1] + motors[2]));
    /* Pitch torque (front vs rear) */
    float tau_pitch = arm * QUAD_THRUST_MAX *
        ((motors[0] + motors[1]) - (motors[2] + motors[3]));
    /* Yaw torque (CW vs CCW motors) */
    float k_yaw = 0.015f;  /* yaw torque coefficient */
    float tau_yaw = k_yaw * QUAD_THRUST_MAX *
        ((motors[1] + motors[3]) - (motors[0] + motors[2]));

    /* Angular acceleration (torque / inertia) */
    vec3_t alpha;
    alpha.x = tau_roll / QUAD_Ixx;
    alpha.y = tau_pitch / QUAD_Iyy;
    alpha.z = tau_yaw / QUAD_Izz;

    /* Gyro damping (simulate internal friction) */
    float damping = 0.02f;
    alpha.x -= damping * s->omega.x;
    alpha.y -= damping * s->omega.y;
    alpha.z -= damping * s->omega.z;

    /* Integrate angular velocity */
    s->omega.x += alpha.x * dt;
    s->omega.y += alpha.y * dt;
    s->omega.z += alpha.z * dt;

    /* Integrate quaternion */
    s->quat = quat_integrate(&s->quat, &s->omega, dt);

    /* --- Body-frame acceleration (for accelerometer) --- */
    /* a_body = R^T * (a_ned - g) */
    vec3_t a_ned_no_grav;
    a_ned_no_grav.x = acc_ned.x;
    a_ned_no_grav.y = acc_ned.y;
    a_ned_no_grav.z = acc_ned.z - gravity.z;
    float R[3][3];
    quat_to_dcm(&s->quat, R);
    /* Transpose: R^T */
    s->accel_body.x = R[0][0]*a_ned_no_grav.x + R[1][0]*a_ned_no_grav.y + R[2][0]*a_ned_no_grav.z;
    s->accel_body.y = R[0][1]*a_ned_no_grav.x + R[1][1]*a_ned_no_grav.y + R[2][1]*a_ned_no_grav.z;
    s->accel_body.z = R[0][2]*a_ned_no_grav.x + R[1][2]*a_ned_no_grav.y + R[2][2]*a_ned_no_grav.z;
    /* Add gravity component in body frame */
    vec3_t g_body = rotate_body_to_ned(&(quat_t){s->quat.w, -s->quat.x, -s->quat.y, -s->quat.z},
                                       &(vec3_t){0, 0, -QUAD_GRAVITY});
    s->accel_body.x += g_body.x;
    s->accel_body.y += g_body.y;
    s->accel_body.z += g_body.z;

    /* --- Update sensors --- */
    s->baro_alt = -s->pos.z;  /* NED: altitude = -z */
    s->gps_lat = 47.3977f + s->pos.y * 9.0e-6f;  /* approximate */
    s->gps_lon = 8.5456f + s->pos.x * 1.1e-5f;
    s->gps_alt = s->baro_alt;

    s->time_s += dt;
    s->step_count++;
}

/* ================================================================== */
/*  ADRC Controller                                                     */
/* ================================================================== */

void quad_adrc_init(quad_adrc_controllers_t *c) {
    memset(c, 0, sizeof(quad_adrc_controllers_t));

    /* Roll ADRC: wc=12, wo=40, b0=15000 (matches plant gain ~700 rad/s² * scale) */
    c->roll_gains = (adrc_gains_t){ .wc = 12.0f, .wo = 40.0f, .b0 = 15000.0f, .out_max = 1.0f };
    adrc_init(&c->roll_adrc, &c->roll_gains, QUAD_CTRL_DT);

    /* Pitch ADRC: same as roll (symmetric) */
    c->pitch_gains = (adrc_gains_t){ .wc = 12.0f, .wo = 40.0f, .b0 = 15000.0f, .out_max = 1.0f };
    adrc_init(&c->pitch_adrc, &c->pitch_gains, QUAD_CTRL_DT);

    /* Yaw ADRC: slower */
    c->yaw_gains = (adrc_gains_t){ .wc = 8.0f, .wo = 30.0f, .b0 = 5000.0f, .out_max = 1.0f };
    adrc_init(&c->yaw_adrc, &c->yaw_gains, QUAD_CTRL_DT);

    /* Altitude ADRC: faster response, gravity compensation */
    c->alt_gains = (adrc_gains_t){ .wc = 12.0f, .wo = 40.0f, .b0 = 150.0f, .out_max = 1.0f };
    adrc_init(&c->alt_adrc, &c->alt_gains, QUAD_CTRL_DT);
}

void quad_adrc_reset(quad_adrc_controllers_t *c) {
    adrc_reset(&c->roll_adrc, 0.0f);
    adrc_reset(&c->pitch_adrc, 0.0f);
    adrc_reset(&c->yaw_adrc, 0.0f);
    adrc_reset(&c->alt_adrc, 0.0f);
}

void quad_adrc_control(quad_adrc_controllers_t *c,
                       const quad_state_t *s,
                       float target_roll, float target_pitch,
                       float target_yaw_rate, float target_alt,
                       float dt,
                       float *cmd_roll, float *cmd_pitch,
                       float *cmd_yaw, float *cmd_throttle) {
    /* Convert target angles to rate targets (simplified P controller for outer loop) */
    /* Output is in deg/s (ADRC expects deg/s for rate tracking) */
    float roll_angle = atan2f(2.0f*(s->quat.w*s->quat.x + s->quat.y*s->quat.z),
                              1.0f - 2.0f*(s->quat.x*s->quat.x + s->quat.y*s->quat.y));
    float pitch_angle = asinf(2.0f*(s->quat.w*s->quat.y - s->quat.z*s->quat.x));
    float roll_rate_target = (target_roll - roll_angle) * 10.0f * 57.2958f;
    float pitch_rate_target = (target_pitch - pitch_angle) * 10.0f * 57.2958f;

    /* ADRC rate controllers */
    *cmd_roll = adrc_update(&c->roll_adrc, &c->roll_gains,
                            roll_rate_target, s->omega.x * 57.2958f, dt, 1.0f);
    *cmd_pitch = adrc_update(&c->pitch_adrc, &c->pitch_gains,
                             pitch_rate_target, s->omega.y * 57.2958f, dt, 1.0f);
    *cmd_yaw = adrc_update(&c->yaw_adrc, &c->yaw_gains,
                           target_yaw_rate * 57.2958f, s->omega.z * 57.2958f, dt, 1.0f);

    /* Altitude ADRC */
    float alt_error = target_alt - (-s->pos.z);
    float alt_vel_target = clampf(alt_error * 3.0f, -3.0f, 3.0f);
    float vz_ms = -s->vel.z;
    *cmd_throttle = adrc_update(&c->alt_adrc, &c->alt_gains,
                                alt_vel_target, vz_ms, dt, 1.0f);
    /* Add gravity feedforward */
    *cmd_throttle += QUAD_MASS * QUAD_GRAVITY / (4.0f * QUAD_THRUST_MAX);
    *cmd_throttle = clampf(*cmd_throttle, 0.0f, 1.0f);
}

/* ================================================================== */
/*  PID Controller (for comparison)                                     */
/* ================================================================== */

void quad_pid_init(quad_pid_controllers_t *c) {
    memset(c, 0, sizeof(quad_pid_controllers_t));

    /* Roll PID */
    c->roll.kp = 3.0f; c->roll.ki = 0.5f; c->roll.kd = 0.3f;
    c->roll.imax = 0.5f;

    /* Pitch PID */
    c->pitch.kp = 3.0f; c->pitch.ki = 0.5f; c->pitch.kd = 0.3f;
    c->pitch.imax = 0.5f;

    /* Yaw PID */
    c->yaw.kp = 2.0f; c->yaw.ki = 0.3f; c->yaw.kd = 0.2f;
    c->yaw.imax = 0.3f;

    /* Altitude PID */
    c->alt.kp = 0.8f; c->alt.ki = 0.2f; c->alt.kd = 0.15f;
    c->alt.imax = 0.4f;
}

void quad_pid_reset(quad_pid_controllers_t *c) {
    c->roll.integral = 0; c->roll.prev_error = 0;
    c->pitch.integral = 0; c->pitch.prev_error = 0;
    c->yaw.integral = 0; c->yaw.prev_error = 0;
    c->alt.integral = 0; c->alt.prev_error = 0;
}

static float pid_update(quad_pid_axis_t *p, float error, float dt) {
    p->integral += error * dt;
    if (p->integral > p->imax) p->integral = p->imax;
    if (p->integral < -p->imax) p->integral = -p->imax;

    float dterm = (error - p->prev_error) / dt;
    p->prev_error = error;

    float out = p->kp * error + p->ki * p->integral + p->kd * dterm;
    return clampf(out, -1.0f, 1.0f);
}

void quad_pid_control(quad_pid_controllers_t *c,
                      const quad_state_t *s,
                      float target_roll, float target_pitch,
                      float target_yaw_rate, float target_alt,
                      float dt,
                      float *cmd_roll, float *cmd_pitch,
                      float *cmd_yaw, float *cmd_throttle) {
    /* Convert target angles to rate targets (same outer loop as ADRC) */
    float roll_angle = atan2f(2.0f*(s->quat.w*s->quat.x + s->quat.y*s->quat.z),
                              1.0f - 2.0f*(s->quat.x*s->quat.x + s->quat.y*s->quat.y));
    float pitch_angle = asinf(2.0f*(s->quat.w*s->quat.y - s->quat.z*s->quat.x));

    float roll_rate_target = (target_roll - roll_angle) * 10.0f * 57.2958f;
    float pitch_rate_target = (target_pitch - pitch_angle) * 10.0f * 57.2958f;

    /* Rate controllers */
    float roll_error = roll_rate_target - s->omega.x * 57.2958f;
    float pitch_error = pitch_rate_target - s->omega.y * 57.2958f;
    float yaw_error = target_yaw_rate * 57.2958f - s->omega.z * 57.2958f;

    *cmd_roll = pid_update(&c->roll, roll_error, dt);
    *cmd_pitch = pid_update(&c->pitch, pitch_error, dt);
    *cmd_yaw = pid_update(&c->yaw, yaw_error, dt);

    /* Altitude PID */
    float alt_error = target_alt - (-s->pos.z);
    *cmd_throttle = pid_update(&c->alt, alt_error, dt);
    /* Add gravity feedforward */
    *cmd_throttle += QUAD_MASS * QUAD_GRAVITY / (4.0f * QUAD_THRUST_MAX);
    *cmd_throttle = clampf(*cmd_throttle, 0.0f, 1.0f);
}
