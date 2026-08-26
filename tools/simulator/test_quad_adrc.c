#include "quad_sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int total_tests = 0;
static int passed_tests = 0;

#define CHECK(cond, msg) do { \
  total_tests++; \
  if (!(cond)) { \
    fprintf(stderr, "  FAIL: %s\n", msg); \
  } else { \
    passed_tests++; \
  } \
} while(0)

#define CHECK_NEAR(a, b, tol, msg) do { \
  total_tests++; \
  if (fabsf((a) - (b)) > (tol)) { \
    fprintf(stderr, "  FAIL: %s (%.4f != %.4f, tol=%.4f)\n", msg, (double)(a), (double)(b), (double)(tol)); \
  } else { \
    passed_tests++; \
  } \
} while(0)

/* ================================================================== */
/*  Test 1: Quad initialization and hover                               */
/* ================================================================== */

static void test_quad_init(void) {
  printf("  Test: Quad initialization and hover\n");

  quad_state_t s;
  quad_init(&s);
  quad_set_hover(&s, 2.0f);

  CHECK_NEAR(s.baro_alt, 2.0f, 0.01f, "Baro altitude = 2.0m");
  CHECK_NEAR(s.pos.z, -2.0f, 0.01f, "NED z = -2.0m");
  CHECK_NEAR(s.quat.w, 1.0f, 0.001f, "Quaternion w = 1.0");
}

/* ================================================================== */
/*  Test 2: Motor mixing                                                */
/* ================================================================== */

static void test_motor_mix(void) {
  printf("  Test: Motor mixing\n");

  float motors[4];

  /* Pure throttle */
  quad_motor_mix(0, 0, 0, 0.5f, motors);
  CHECK_NEAR(motors[0], 0.5f, 0.01f, "M0 = throttle");
  CHECK_NEAR(motors[1], 0.5f, 0.01f, "M1 = throttle");
  CHECK_NEAR(motors[2], 0.5f, 0.01f, "M2 = throttle");
  CHECK_NEAR(motors[3], 0.5f, 0.01f, "M3 = throttle");

  /* Pure roll right */
  quad_motor_mix(0.3f, 0, 0, 0.5f, motors);
  CHECK(motors[0] > motors[1], "Roll right: M0 > M1");
  CHECK(motors[3] > motors[2], "Roll right: M3 > M2");
}

/* ================================================================== */
/*  Test 3: Physics step - hover stability                             */
/* ================================================================== */

static void test_hover_stability(void) {
  printf("  Test: Hover stability (ADRC)\n");

  quad_state_t s;
  quad_init(&s);
  quad_set_hover(&s, 2.0f);

  quad_adrc_controllers_t adrc;
  quad_adrc_init(&adrc);

  float target_alt = 2.0f;
  float max_alt_dev = 0.0f;

  /* Run for 2 seconds */
  for (int i = 0; i < 1000; i++) {
    float cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle;
    quad_adrc_control(&adrc, &s, 0, 0, 0, target_alt,
                      QUAD_SIM_DT, &cmd_roll, &cmd_pitch, &cmd_yaw, &cmd_throttle);
    quad_step(&s, cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle, NULL);

    float alt = -s.pos.z;
    float dev = fabsf(alt - target_alt);
    if (dev > max_alt_dev) max_alt_dev = dev;
  }

  printf("    Final alt: %.3f m (target: %.1f m)\n", -s.pos.z, target_alt);
  printf("    Max alt deviation: %.3f m\n", max_alt_dev);

  CHECK(max_alt_dev < 0.5f, "ADRC hover: max altitude deviation < 0.5m");
  CHECK_NEAR(s.pos.x, 0.0f, 0.1f, "ADRC hover: no horizontal drift");
  CHECK_NEAR(s.pos.y, 0.0f, 0.1f, "ADRC hover: no horizontal drift");
}

/* ================================================================== */
/*  Test 4: ADRC step response                                         */
/* ================================================================== */

static void test_adrc_step_response(void) {
  printf("  Test: ADRC step response (altitude 0→3m)\n");

  quad_state_t s;
  quad_init(&s);
  quad_set_hover(&s, 0.0f);

  quad_adrc_controllers_t adrc;
  quad_adrc_init(&adrc);

  float target = 3.0f;
  float final_alt = 0.0f;

  /* Run for 3 seconds */
  for (int i = 0; i < 1500; i++) {
    float cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle;
    quad_adrc_control(&adrc, &s, 0, 0, 0, target,
                      QUAD_SIM_DT, &cmd_roll, &cmd_pitch, &cmd_yaw, &cmd_throttle);
    quad_step(&s, cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle, NULL);
    final_alt = -s.pos.z;
  }

  printf("    Final alt: %.3f m (target: %.1f m)\n", final_alt, target);

  CHECK(fabsf(final_alt - target) < 0.5f, "ADRC step: reaches target within 0.5m");
}

/* ================================================================== */
/*  Test 5: ADRC disturbance rejection (wind)                          */
/* ================================================================== */

static void test_adrc_wind_rejection(void) {
  printf("  Test: ADRC vs PID wind disturbance rejection\n");

  /* --- ADRC --- */
  quad_state_t s_adrc;
  quad_init(&s_adrc);
  quad_set_hover(&s_adrc, 2.0f);

  quad_adrc_controllers_t adrc;
  quad_adrc_init(&adrc);

  quad_wind_t wind = { .velocity = {2.0f, 0, 0}, .turbulence = 0.1f };
  float target = 2.0f;
  float max_dev_adrc = 0.0f;
  float final_alt_adrc = 0.0f;

  for (int i = 0; i < 2000; i++) {
    float cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle;
    quad_adrc_control(&adrc, &s_adrc, 0, 0, 0, target,
                      QUAD_SIM_DT, &cmd_roll, &cmd_pitch, &cmd_yaw, &cmd_throttle);
    quad_step(&s_adrc, cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle, &wind);

    float alt = -s_adrc.pos.z;
    float dev = fabsf(alt - target);
    if (dev > max_dev_adrc) max_dev_adrc = dev;
    final_alt_adrc = alt;
  }

  /* --- PID --- */
  quad_state_t s_pid;
  quad_init(&s_pid);
  quad_set_hover(&s_pid, 2.0f);

  quad_pid_controllers_t pid;
  quad_pid_init(&pid);

  float max_dev_pid = 0.0f;
  float final_alt_pid = 0.0f;

  for (int i = 0; i < 2000; i++) {
    float cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle;
    quad_pid_control(&pid, &s_pid, 0, 0, 0, target,
                     QUAD_SIM_DT, &cmd_roll, &cmd_pitch, &cmd_yaw, &cmd_throttle);
    quad_step(&s_pid, cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle, &wind);

    float alt = -s_pid.pos.z;
    float dev = fabsf(alt - target);
    if (dev > max_dev_pid) max_dev_pid = dev;
    final_alt_pid = alt;
  }

  printf("    ADRC: max_dev=%.3f m, final=%.3f m\n", max_dev_adrc, final_alt_adrc);
  printf("    PID:  max_dev=%.3f m, final=%.3f m\n", max_dev_pid, final_alt_pid);

  CHECK(max_dev_adrc < max_dev_pid * 1.5f,
        "ADRC wind rejection comparable or better than PID");
  CHECK(max_dev_adrc < 1.0f,
        "ADRC wind: max deviation < 1.0m");
}

/* ================================================================== */
/*  Test 6: ADRC roll response                                         */
/* ================================================================== */

static void test_adrc_roll(void) {
  printf("  Test: ADRC roll angle response\n");

  quad_state_t s;
  quad_init(&s);
  quad_set_hover(&s, 2.0f);

  quad_adrc_controllers_t adrc;
  quad_adrc_init(&adrc);

  float target_alt = 2.0f;
  float target_roll = 0.2f;  /* ~11.5 degrees */
  float final_roll = 0.0f;

  /* Run for 2 seconds */
  for (int i = 0; i < 1000; i++) {
    float cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle;
    quad_adrc_control(&adrc, &s, target_roll, 0, 0, target_alt,
                      QUAD_SIM_DT, &cmd_roll, &cmd_pitch, &cmd_yaw, &cmd_throttle);
    quad_step(&s, cmd_roll, cmd_pitch, cmd_yaw, cmd_throttle, NULL);

    /* Current roll angle */
    final_roll = atan2f(2.0f*(s.quat.w*s.quat.x + s.quat.y*s.quat.z),
                        1.0f - 2.0f*(s.quat.x*s.quat.x + s.quat.y*s.quat.y));
  }

  printf("    Final roll: %.3f rad (target: %.3f rad)\n", final_roll, target_roll);

  CHECK(fabsf(final_roll - target_roll) < 0.1f,
        "ADRC roll reaches target within 0.1 rad");
  CHECK(fabsf(-s.pos.z - target_alt) < 0.5f,
        "ADRC roll: altitude maintained");
}

/* ================================================================== */
/*  Test 7: ADRC vs PID comparison summary                             */
/* ================================================================== */

static void test_adrc_vs_pid_summary(void) {
  printf("  Test: ADRC vs PID full comparison\n");

  float dt = QUAD_SIM_DT;
  float target_alt = 3.0f;
  quad_wind_t wind = { .velocity = {1.5f, 1.0f, 0}, .turbulence = 0.05f };

  /* --- ADRC --- */
  quad_state_t s_adrc;
  quad_init(&s_adrc);
  quad_set_hover(&s_adrc, 0.0f);
  quad_adrc_controllers_t adrc;
  quad_adrc_init(&adrc);

  float max_dev_adrc = 0, total_error_adrc = 0;
  int steps = 2000;

  for (int i = 0; i < steps; i++) {
    float cr, cp, cy, ct;
    quad_adrc_control(&adrc, &s_adrc, 0, 0, 0, target_alt,
                      dt, &cr, &cp, &cy, &ct);
    quad_step(&s_adrc, cr, cp, cy, ct, &wind);

    float alt = -s_adrc.pos.z;
    float err = fabsf(alt - target_alt);
    if (err > max_dev_adrc) max_dev_adrc = err;
    total_error_adrc += err;
  }
  float iae_adrc = total_error_adrc * dt;

  /* --- PID --- */
  quad_state_t s_pid;
  quad_init(&s_pid);
  quad_set_hover(&s_pid, 0.0f);
  quad_pid_controllers_t pid;
  quad_pid_init(&pid);

  float max_dev_pid = 0, total_error_pid = 0;

  for (int i = 0; i < steps; i++) {
    float cr, cp, cy, ct;
    quad_pid_control(&pid, &s_pid, 0, 0, 0, target_alt,
                     dt, &cr, &cp, &cy, &ct);
    quad_step(&s_pid, cr, cp, cy, ct, &wind);

    float alt = -s_pid.pos.z;
    float err = fabsf(alt - target_alt);
    if (err > max_dev_pid) max_dev_pid = err;
    total_error_pid += err;
  }
  float iae_pid = total_error_pid * dt;

  printf("    ┌─────────────────┬───────────┬───────────┐\n");
  printf("    │ Metric          │    ADRC   │     PID   │\n");
  printf("    ├─────────────────┼───────────┼───────────┤\n");
  printf("    │ Max deviation   │  %6.3f m │  %6.3f m │\n", max_dev_adrc, max_dev_pid);
  printf("    │ IAE (int. error)│  %6.3f   │  %6.3f   │\n", iae_adrc, iae_pid);
  printf("    │ Final alt       │  %6.3f m │  %6.3f m │\n", -s_adrc.pos.z, -s_pid.pos.z);
  printf("    └─────────────────┴───────────┴───────────┘\n");

  CHECK(max_dev_adrc < max_dev_pid * 2.0f,
        "ADRC max deviation ≤ 2× PID");
  CHECK(iae_adrc < iae_pid * 2.0f,
        "ADRC IAE ≤ 2× PID");
  CHECK(fabsf(-s_adrc.pos.z - target_alt) < 1.0f,
        "ADRC reaches target altitude");
}

/* ================================================================== */
/*  Test 8: ADRC convergence time                                      */
/* ================================================================== */

static void test_adrc_convergence(void) {
  printf("  Test: ADRC convergence time\n");

  quad_state_t s;
  quad_init(&s);
  quad_set_hover(&s, 0.0f);

  quad_adrc_controllers_t adrc;
  quad_adrc_init(&adrc);

  float target = 5.0f;
  float settle_time = -1.0f;

  for (int i = 0; i < 3000; i++) {
    float cr, cp, cy, ct;
    quad_adrc_control(&adrc, &s, 0, 0, 0, target,
                      QUAD_SIM_DT, &cr, &cp, &cy, &ct);
    quad_step(&s, cr, cp, cy, ct, NULL);

    float alt = -s.pos.z;
    if (settle_time < 0 && fabsf(alt - target) < 0.2f) {
      settle_time = s.time_s;
    }
  }

  printf("    Settle time (±0.2m): %.2f s\n", settle_time);

  CHECK(settle_time > 0, "ADRC converges to target");
  CHECK(settle_time < 5.0f, "ADRC settles in < 5s");
}

/* ================================================================== */
/*  Main                                                               */
/* ================================================================== */

int main(void) {
  printf("=== Quadcopter ADRC vs PID Simulator Tests ===\n\n");

  srand(42);  /* deterministic seed for reproducibility */

  test_quad_init();
  test_motor_mix();
  test_hover_stability();
  test_adrc_step_response();
  test_adrc_wind_rejection();
  test_adrc_roll();
  test_adrc_vs_pid_summary();
  test_adrc_convergence();

  printf("\n=== Results: %d/%d passed ===\n", passed_tests, total_tests);
  return (passed_tests == total_tests) ? 0 : 1;
}
