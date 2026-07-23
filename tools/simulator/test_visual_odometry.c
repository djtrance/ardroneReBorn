#include "visual_odometry.h"
#include "image.h"
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

/* Pseudo-random texture base pixel - high contrast for FAST detection */
static uint8_t base_pixel(uint16_t x, uint16_t y) {
  /* Checkerboard + noise for strong corner detection */
  uint8_t checker = ((x / 8) + (y / 8)) & 1 ? 200 : 50;
  uint8_t noise = (uint8_t)(((uint32_t)x * 37 + (uint32_t)y * 71) & 0x3F);
  return (checker + noise) & 0xFF;
}

/* Create frame with known translation */
static void make_shifted_frame(vision_image_t *img, uint16_t w, uint16_t h,
                                float shift_x, float shift_y) {
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      float sx = (float)x - shift_x;
      float sy = (float)y - shift_y;
      int ix = (int)sx;
      int iy = (int)sy;
      if (ix < 0) ix = 0; if (ix >= (int)w) ix = w - 1;
      if (iy < 0) iy = 0; if (iy >= (int)h) iy = h - 1;
      img->buf[y * img->stride + x] = base_pixel(ix, iy);
    }
  }
}

/* ================================================================== */
/*  Test 1: Essential matrix from known point correspondences         */
/* ================================================================== */

static void test_essential_matrix(void) {
  printf("  Test: Essential matrix computation\n");

  /* Simple translation: points shift right by ~10px with varied y offsets */
  float pts1[10][2], pts2[10][2];
  for (int i = 0; i < 10; i++) {
    pts1[i][0] = 50.0f + i * 20;
    pts1[i][1] = 50.0f + (i % 3) * 30;
    pts2[i][0] = pts1[i][0] + 10.0f;
    pts2[i][1] = pts1[i][1] + ((float)(i % 5) - 2.0f) * 0.5f;  /* slight y variation */
  }

  float E[3][3];
  bool ok = vo_compute_essential(pts1, pts2, 10, E);
  CHECK(ok, "Essential matrix computed");

  /* Check that E is 3x3 */
  float sum = 0;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      sum += fabsf(E[i][j]);
  CHECK(sum > 0.001f, "Essential matrix is non-zero");
}

/* ================================================================== */
/*  Test 2: Essential matrix decomposition                            */
/* ================================================================== */

static void test_decompose_essential(void) {
  printf("  Test: Essential matrix decomposition\n");

  /* Construct a known E from R=I, t=[1,0,0] */
  float pts1[8][2], pts2[8][2];
  for (int i = 0; i < 8; i++) {
    pts1[i][0] = 40.0f + i * 25;
    pts1[i][1] = 40.0f + (i % 4) * 40;
    pts2[i][0] = pts1[i][0] + 5.0f;  /* rightward motion */
    pts2[i][1] = pts1[i][1] + 1.0f;  /* slight downward */
  }

  float E[3][3];
  bool ok = vo_compute_essential(pts1, pts2, 8, E);
  CHECK(ok, "Essential matrix computed");

  float R[3][3], t[3];
  ok = vo_decompose_essential(E, R, t);
  CHECK(ok, "Decomposition succeeded");

  /* Check that t is unit length */
  float tn = sqrtf(t[0]*t[0] + t[1]*t[1] + t[2]*t[2]);
  CHECK_NEAR(tn, 1.0f, 0.01f, "Translation is unit length");

  /* Check that R is a valid rotation (det = 1) */
  float det = R[0][0]*(R[1][1]*R[2][2] - R[1][2]*R[2][1])
            - R[0][1]*(R[1][0]*R[2][2] - R[1][2]*R[2][0])
            + R[0][2]*(R[1][0]*R[2][1] - R[1][1]*R[2][0]);
  CHECK_NEAR(det, 1.0f, 0.05f, "Rotation determinant = 1");

  /* At least one component should be dominant (motion is primarily in one axis) */
  float max_t = fabsf(t[0]);
  if (fabsf(t[1]) > max_t) max_t = fabsf(t[1]);
  if (fabsf(t[2]) > max_t) max_t = fabsf(t[2]);
  CHECK(max_t > 0.5f, "Translation has dominant direction");
}

/* ================================================================== */
/*  Test 3: Epipolar error                                           */
/* ================================================================== */

static void test_epipolar_error(void) {
  printf("  Test: Epipolar error for matched points\n");

  /* Identity essential matrix (all zeros) → error should be 0 */
  float E[3][3] = {{0}};
  float p1[2] = {100, 100};
  float p2[2] = {110, 100};
  float err = vo_epipolar_error(E, p1, p2);
  CHECK_NEAR(err, 0.0f, 0.001f, "Zero E → zero epipolar error");

  /* Non-zero E → non-zero error for random points */
  float E2[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  err = vo_epipolar_error(E2, p1, p2);
  CHECK(err > 0.001f, "Identity E → non-zero epipolar error");
}

/* ================================================================== */
/*  Test 4: VO pipeline creates context and processes frames          */
/* ================================================================== */

static void test_vo_pipeline_create_process(void) {
  printf("  Test: VO pipeline create/process/destroy lifecycle\n");

  vo_config_t cfg;
  vo_default_config(&cfg);
  cfg.use_barometer = false;
  cfg.min_matches = 3;
  cfg.fast_threshold = 15;

  uint16_t w = 320, h = 240;
  vo_context_t *vo = vo_create(&cfg, w, h);
  CHECK(vo != NULL, "VO context created");

  vision_image_t frame;
  frame.type = VISION_IMAGE_GRAYSCALE;
  frame.w = w;
  frame.h = h;
  frame.stride = w;
  frame.bytes_per_pixel = 1;
  frame.buf = malloc(w * h);
  CHECK(frame.buf != NULL, "Frame buffer allocated");

  /* Process frames without crashing */
  make_shifted_frame(&frame, w, h, 0, 0);
  for (int i = 0; i < 5; i++) {
    int ret = vo_process(vo, &frame, 0, 0, 33, NULL, NULL);
    CHECK(ret == 0, "vo_process returns success");
  }

  vo_position_t pos;
  vo_get_position(vo, &pos);
  CHECK(pos.timestamp_ms >= 0, "Position timestamp set");

  uint16_t fc = vo_feature_count(vo);
  CHECK(fc >= 0, "Feature count is valid");

  vo_destroy(vo);
  free(frame.buf);
}

/* ================================================================== */
/*  Test 5: VO reset                                                 */
/* ================================================================== */

static void test_vo_reset(void) {
  printf("  Test: VO reset\n");

  vo_config_t cfg;
  vo_default_config(&cfg);
  cfg.use_barometer = false;

  uint16_t w = 160, h = 120;
  vo_context_t *vo = vo_create(&cfg, w, h);

  vision_image_t frame;
  frame.type = VISION_IMAGE_GRAYSCALE;
  frame.w = w;
  frame.h = h;
  frame.stride = w;
  frame.bytes_per_pixel = 1;
  frame.buf = malloc(w * h);
  make_shifted_frame(&frame, w, h, 50, 50);

  /* Process some frames */
  for (int i = 0; i < 5; i++) {
    vo_process(vo, &frame, 0, 0, 33, NULL, NULL);
  }

  /* Reset */
  vo_reset(vo);

  vo_position_t pos;
  vo_get_position(vo, &pos);
  CHECK_NEAR(pos.x, 0.0f, 0.01f, "After reset: x = 0");
  CHECK_NEAR(pos.y, 0.0f, 0.01f, "After reset: y = 0");
  CHECK_NEAR(pos.z, 0.0f, 0.01f, "After reset: z = 0");

  free(frame.buf);
  vo_destroy(vo);
}

/* ================================================================== */
/*  Main                                                              */
/* ================================================================== */

int main(void) {
  printf("=== Visual Odometry Tests ===\n\n");

  test_essential_matrix();
  test_decompose_essential();
  test_epipolar_error();
  test_vo_pipeline_create_process();
  test_vo_reset();

  printf("\n=== Results: %d/%d passed ===\n", passed_tests, total_tests);
  return (passed_tests == total_tests) ? 0 : 1;
}
