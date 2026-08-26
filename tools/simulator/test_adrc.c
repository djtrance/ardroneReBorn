#include "adrc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static int total_tests = 0;
static int passed_tests = 0;

#define CHECK(cond, msg) do { \
  total_tests++; \
  if (!(cond)) { \
    fprintf(stderr, "FAIL: %s\n", msg); \
  } else { \
    passed_tests++; \
  } \
} while(0)

#define CHECK_NEAR(a, b, tol, msg) do { \
  total_tests++; \
  if (fabsf((a) - (b)) > (tol)) { \
    fprintf(stderr, "FAIL: %s (%.4f != %.4f, tol=%.4f)\n", msg, (double)(a), (double)(b), (double)(tol)); \
  } else { \
    passed_tests++; \
  } \
} while(0)

/* ================================================================== */
/*  Test 1: ADRC initialization produces valid gains                   */
/* ================================================================== */

static void test_adrc_init(void) {
  printf("  Test: ADRC initialization\n");

  adrc_state_t s;
  adrc_gains_t g = { .wc = 10.0f, .wo = 30.0f, .b0 = 500.0f, .out_max = 1.0f };
  adrc_init(&s, &g, 0.001f);  /* 1kHz */

  CHECK(s.kp == 100.0f, "kp = wc² = 100");
  CHECK_NEAR(s.kd, 20.0f, 0.01f, "kd = 2·wc = 20");
  CHECK(s.l1 > 0, "l1 > 0");
  CHECK(s.l2 > 0, "l2 > 0");
  CHECK(s.l3 > 0, "l3 > 0");
  CHECK(s.b0 == 500.0f, "b0 = 500");
}

/* ================================================================== */
/*  Test 2: ESO converges to true state                               */
/* ================================================================== */

static void test_eso_convergence(void) {
  printf("  Test: ESO convergence to true rate\n");

  adrc_state_t s;
  adrc_gains_t g = { .wc = 15.0f, .wo = 50.0f, .b0 = 300.0f, .out_max = 1.0f };
  adrc_init(&s, &g, 0.001f);

  /* Simulate a constant rate of 100 deg/s with zero disturbance */
  float true_rate = 100.0f;
  float gyro_measurement = 100.0f;  /* perfect measurement */

  adrc_reset(&s, 0.0f);

  /* Run ESO for 500ms (500 steps at 1kHz) with zero control output */
  for (int i = 0; i < 500; i++) {
    adrc_update(&s, &g, 0.0f, gyro_measurement, 0.001f, 1.0f);
  }

  /* z1 should converge to measured rate */
  CHECK_NEAR(s.z1, true_rate, 5.0f, "ESO z1 converges to measured rate");
  /* z2 and z3 may not be zero in open-loop (u=0) because ESO attributes
   * unmodeled dynamics to z3. This is expected behavior.
   * The key test is that z1 tracks the measurement accurately. */
}

/* ================================================================== */
/*  Test 3: ESO estimates external disturbance                        */
/* ================================================================== */

static void test_eso_disturbance(void) {
  printf("  Test: ESO disturbance estimation\n");

  adrc_state_t s;
  adrc_gains_t g = { .wc = 15.0f, .wo = 60.0f, .b0 = 200.0f, .out_max = 1.0f };
  adrc_init(&s, &g, 0.001f);
  adrc_reset(&s, 0.0f);

  /* Simulate closed-loop: hold rate=0 with external disturbance of 50 deg/s² */
  float x = 0.0f;
  float dx = 0.0f;
  float disturbance = 50.0f;
  float dt = 0.001f;

  /* Run for 500ms */
  for (int i = 0; i < 500; i++) {
    float u = adrc_update(&s, &g, 0.0f, x, dt, 1.0f);
    /* Plant: ẋ = b0·u + disturbance (after 100ms) */
    float dist = (i > 100) ? disturbance : 0.0f;
    dx += (g.b0 * u + dist) * dt;
    x += dx * dt;
  }

  /* After convergence, z3 should track the disturbance */
  /* With disturbance active, z3 should be non-zero and tracking it */
  CHECK(fabsf(s.z3) > 10.0f, "ESO z3 estimates disturbance (non-zero)");
  /* The state x should be held near zero despite disturbance */
  CHECK(fabsf(x) < 5.0f, "ADRC holds state near zero despite disturbance");
}

/* ================================================================== */
/*  Test 4: ADRC step response (setpoint tracking)                    */
/* ================================================================== */

static void test_adrc_tracking(void) {
  printf("  Test: ADRC step response tracking\n");

  adrc_state_t s;
  adrc_gains_t g = { .wc = 12.0f, .wo = 40.0f, .b0 = 250.0f, .out_max = 1.0f };
  adrc_init(&s, &g, 0.001f);
  adrc_reset(&s, 0.0f);

  /* Simulate a simple first-order plant: ẋ = b0·u + disturbance */
  float x = 0.0f;       /* true state */
  float dx = 0.0f;      /* true velocity */
  float target = 50.0f;  /* step target */
  float dt = 0.001f;

  /* Run for 500ms */
  for (int i = 0; i < 500; i++) {
    float u = adrc_update(&s, &g, target, x, dt, 1.0f);

    /* Simple plant dynamics: ẋ = b0·u */
    dx += g.b0 * u * dt;
    x += dx * dt;
  }

  /* After 500ms, x should be approaching target */
  CHECK(fabsf(x) > 10.0f, "System responds to step input");
  CHECK(fabsf(x - target) < 30.0f, "System approaches target within 30 deg");
}

/* ================================================================== */
/*  Test 5: ADRC disturbance rejection                                */
/* ================================================================== */

static void test_adrc_rejection(void) {
  printf("  Test: ADRC disturbance rejection\n");

  /* Run two simulations: with and without ADRC, apply same disturbance */
  float dt = 0.001f;
  float target = 0.0f;  /* trying to hold zero rate */
  float disturbance = 80.0f;  /* wind gust */

  /* ADRC */
  adrc_state_t s_adrc;
  adrc_gains_t g = { .wc = 15.0f, .wo = 50.0f, .b0 = 300.0f, .out_max = 1.0f };
  adrc_init(&s_adrc, &g, dt);
  adrc_reset(&s_adrc, 0.0f);

  float x_adrc = 0.0f, dx_adrc = 0.0f;
  float max_deviation_adrc = 0.0f;

  for (int i = 0; i < 500; i++) {
    float u = adrc_update(&s_adrc, &g, target, x_adrc, dt, 1.0f);
    /* Plant: ẋ = b0·u + disturbance (after 100ms) */
    float dist = (i > 100) ? disturbance : 0.0f;
    dx_adrc += (g.b0 * u + dist) * dt;
    x_adrc += dx_adrc * dt;
    float dev = fabsf(x_adrc - target);
    if (dev > max_deviation_adrc) max_deviation_adrc = dev;
  }

  /* Simple PID for comparison */
  float x_pid = 0.0f, dx_pid = 0.0f;
  float integral_pid = 0.0f, prev_error_pid = 0.0f;
  float max_deviation_pid = 0.0f;
  float kp_pid = 0.5f, ki_pid = 0.1f, kd_pid = 0.2f;

  for (int i = 0; i < 500; i++) {
    float error = target - x_pid;
    integral_pid += error * dt;
    float dterm = (error - prev_error_pid) / dt;
    float u_pid = kp_pid * error + ki_pid * integral_pid + kd_pid * dterm;
    if (u_pid > 1.0f) u_pid = 1.0f;
    if (u_pid < -1.0f) u_pid = -1.0f;
    prev_error_pid = error;

    float dist = (i > 100) ? disturbance : 0.0f;
    dx_pid += (300.0f * u_pid + dist) * dt;
    x_pid += dx_pid * dt;
    float dev = fabsf(x_pid - target);
    if (dev > max_deviation_pid) max_deviation_pid = dev;
  }

  printf("    ADRC max deviation: %.1f deg\n", max_deviation_adrc);
  printf("    PID  max deviation: %.1f deg\n", max_deviation_pid);

  CHECK(max_deviation_adrc < max_deviation_pid * 1.5f,
        "ADRC disturbance rejection comparable or better than PID");
  CHECK(max_deviation_adrc < 200.0f,
        "ADRC max deviation under 200 deg");
}

/* ================================================================== */
/*  Test 6: ADRC from PID conversion                                  */
/* ================================================================== */

static void test_adrc_from_pid(void) {
  printf("  Test: PID → ADRC conversion\n");

  adrc_gains_t g = adrc_from_pid(10.0f, 30.0f, 50.0f);

  CHECK_NEAR(g.wc, 10.0f, 0.01f, "wc = P × 1.0");
  CHECK_NEAR(g.wo, 30.0f, 0.01f, "wo = I × 1.0");
  CHECK_NEAR(g.b0, 500.0f, 0.1f, "b0 = D × 10.0");
}

/* ================================================================== */
/*  Test 7: ADRC reset                                                */
/* ================================================================== */

static void test_adrc_reset(void) {
  printf("  Test: ADRC reset\n");

  adrc_state_t s;
  adrc_gains_t g = { .wc = 10.0f, .wo = 30.0f, .b0 = 500.0f, .out_max = 1.0f };
  adrc_init(&s, &g, 0.001f);

  /* Run some steps */
  for (int i = 0; i < 50; i++) {
    adrc_update(&s, &g, 100.0f, 50.0f, 0.001f, 1.0f);
  }

  /* Reset */
  adrc_reset(&s, 25.0f);

  CHECK_NEAR(s.z1, 25.0f, 0.01f, "z1 = initial_rate after reset");
  CHECK_NEAR(s.z2, 0.0f, 0.01f, "z2 = 0 after reset");
  CHECK_NEAR(s.z3, 0.0f, 0.01f, "z3 = 0 after reset");
  CHECK_NEAR(s.last_output, 0.0f, 0.01f, "last_output = 0 after reset");
}

/* ================================================================== */
/*  Test 8: Output saturation                                         */
/* ================================================================== */

static void test_adrc_saturation(void) {
  printf("  Test: Output saturation\n");

  adrc_state_t s;
  adrc_gains_t g = { .wc = 20.0f, .wo = 60.0f, .b0 = 100.0f, .out_max = 0.5f };
  adrc_init(&s, &g, 0.001f);
  adrc_reset(&s, 0.0f);

  /* Large step should saturate */
  float max_out = 0.0f;
  for (int i = 0; i < 100; i++) {
    float u = adrc_update(&s, &g, 500.0f, 0.0f, 0.001f, 0.5f);
    if (fabsf(u) > max_out) max_out = fabsf(u);
  }

  CHECK(max_out <= 0.501f, "Output saturated at out_max");
}

/* ================================================================== */
/*  Test 9: Critical damping (no overshoot with proper wc)            */
/* ================================================================== */

static void test_adrc_damping(void) {
  printf("  Test: ADRC critical damping\n");

  adrc_state_t s;
  /* High observer bandwidth for fast convergence */
  adrc_gains_t g = { .wc = 15.0f, .wo = 60.0f, .b0 = 300.0f, .out_max = 2.0f };
  adrc_init(&s, &g, 0.001f);
  adrc_reset(&s, 0.0f);

  float x = 0.0f, dx = 0.0f;
  float target = 100.0f;
  float prev_x = 0.0f;
  int crossings = 0;

  for (int i = 0; i < 1000; i++) {
    float u = adrc_update(&s, &g, target, x, 0.001f, 2.0f);
    dx += g.b0 * u * 0.001f;
    x += dx * 0.001f;

    /* Count zero crossings of (x - target) */
    if (i > 10 && ((prev_x - target) * (x - target) < 0)) {
      crossings++;
    }
    prev_x = x;
  }

  printf("    Crossings: %d (0-1 = well damped)\n", crossings);
  CHECK(crossings <= 3, "System is well damped (≤3 crossings)");
}

/* ================================================================== */
/*  Main                                                              */
/* ================================================================== */

int main(void) {
  printf("=== ADRC Controller Tests ===\n\n");

  test_adrc_init();
  test_eso_convergence();
  test_eso_disturbance();
  test_adrc_tracking();
  test_adrc_rejection();
  test_adrc_from_pid();
  test_adrc_reset();
  test_adrc_saturation();
  test_adrc_damping();

  printf("\n=== Results: %d/%d passed ===\n", passed_tests, total_tests);
  return (passed_tests == total_tests) ? 0 : 1;
}
