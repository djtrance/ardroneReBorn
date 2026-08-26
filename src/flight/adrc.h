#ifndef ADRC_H
#define ADRC_H

#include <stdint.h>
#include <stdbool.h>

/* Active Disturbance Rejection Control (ADRC)
 *
 * Based on ADRC-inav implementation (Boyyt357/ADRC-inav).
 * Replaces traditional PID rate control with:
 *   - Extended State Observer (ESO): estimates angular rate + disturbance
 *   - Non-Linear State Error Feedback (NLSEF): PD + disturbance cancel
 *   - Bandwidth parameterization: tuning via wc (controller) and wo (observer)
 *
 * Mathematical model (second-order plant):
 *   ẍ = f(x, ẋ, t, w) + b·u
 *   where f = total disturbance (internal + external)
 *         b = plant gain estimate
 *         u = control input
 *
 * ESO estimates states z1 (position), z2 (velocity), z3 (disturbance)
 * and the control law cancels z3, reducing to a simple double integrator.
 *
 * Advantages over PID:
 *   - No integral windup (disturbance is estimated, not accumulated)
 *   - Better disturbance rejection (wind gusts, voltage drops, CG shifts)
 *   - Simpler tuning: 3 parameters (wc, wo, b0) instead of P/I/D
 *   - Smooth stick feel: linear response, no derivative kick
 */

/* ADRC axis configuration (maps to PID P/I/D in INAV convention) */
typedef struct {
    float wc;     /* Controller bandwidth (rad/s). From P gain. Higher = faster. */
    float wo;     /* Observer bandwidth (rad/s). From I gain. Should be 3-5x wc. */
    float b0;     /* Plant gain estimate. From D gain. Approximate system gain. */
    float out_max; /* Output saturation */
} adrc_gains_t;

/* ADRC state (per axis) */
typedef struct {
    /* ESO states */
    float z1;     /* Estimated angular rate (position) */
    float z2;     /* Estimated angular acceleration (velocity) */
    float z3;     /* Estimated total disturbance */

    /* Precalculated gains */
    float kp;     /* = wc² */
    float kd;     /* = 2·wc (critical damping) */
    float l1;     /* Observer gain 1 */
    float l2;     /* Observer gain 2 */
    float l3;     /* Observer gain 3 */
    float b0;     /* Plant gain */

    /* Previous control output (for ESO) */
    float last_output;

    /* Output */
    float output;
} adrc_state_t;

/* Initialize ADRC state from gains and sample time */
void adrc_init(adrc_state_t *s, const adrc_gains_t *gains, float dt);

/* Reset ESO states (call on arming / mode change) */
void adrc_reset(adrc_state_t *s, float initial_rate);

/* Run one ADRC step
 *
 * Parameters:
 *   s          - ADRC state
 *   gains      - ADRC gains (wc, wo, b0)
 *   rate_target - desired angular rate (deg/s for pitch/roll, deg/s for yaw)
 *   gyro_rate   - measured angular rate from IMU (deg/s)
 *   dt          - time step (seconds)
 *   out_max     - output saturation
 *
 * Returns: control output (motor command, typically -1..1)
 */
float adrc_update(adrc_state_t *s, const adrc_gains_t *gains,
                  float rate_target, float gyro_rate,
                  float dt, float out_max);

/* Get ESO-estimated states (for logging/telemetry) */
void adrc_get_states(const adrc_state_t *s,
                     float *z1_rate, float *z2_accel, float *z3_disturbance);

/* Get precalculated gains (for debugging) */
void adrc_get_gains(const adrc_state_t *s,
                    float *kp, float *kd, float *l1, float *l2, float *l3);

/* Convenience: convert PID P/I/D to ADRC wc/wo/b0
 *
 * INAV convention:
 *   P gain  → wc  (controller bandwidth)
 *   I gain  → wo  (observer bandwidth)
 *   D gain  → b0  (plant gain estimate × scale)
 */
adrc_gains_t adrc_from_pid(float p, float i, float d);

#endif /* ADRC_H */
