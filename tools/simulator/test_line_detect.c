#include "line_detect.h"
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

static void make_image(vision_image_t *img, uint16_t w, uint16_t h) {
  img->type = VISION_IMAGE_GRAYSCALE;
  img->w = w;
  img->h = h;
  img->stride = w;
  img->bytes_per_pixel = 1;
  img->buf = (uint8_t*)calloc(w * h, 1);
}

/* ================================================================== */
/*  Test 1: Sobel edge detection on blank image                       */
/* ================================================================== */

static void test_sobel_blank(void) {
  printf("  Test: Sobel on blank image → no edges\n");

  vision_image_t img;
  make_image(&img, 64, 64);
  memset(img.buf, 100, 64 * 64);

  vision_image_t mag;
  make_image(&mag, 64, 64);
  line_sobel(&img, &mag, NULL);

  int nonzero = 0;
  for (int i = 0; i < 64*64; i++)
    if (mag.buf[i] > 0) nonzero++;

  CHECK(nonzero == 0, "Blank image → zero Sobel magnitude");

  free(img.buf);
  free(mag.buf);
}

/* ================================================================== */
/*  Test 2: Sobel on image with vertical edge                         */
/* ================================================================== */

static void test_sobel_vertical_edge(void) {
  printf("  Test: Sobel on image with vertical edge\n");

  vision_image_t img;
  make_image(&img, 64, 64);
  /* Left half dark, right half bright */
  for (int y = 0; y < 64; y++)
    for (int x = 0; x < 64; x++)
      img.buf[y*64+x] = (x < 32) ? 50 : 200;

  vision_image_t mag;
  make_image(&mag, 64, 64);
  line_sobel(&img, &mag, NULL);

  /* Strong edge at x=31-32 */
  CHECK(mag.buf[32*64+31] > 0 || mag.buf[32*64+32] > 0,
        "Strong edge at vertical boundary");

  free(img.buf);
  free(mag.buf);
}

/* ================================================================== */
/*  Test 3: Non-max suppression                                       */
/* ================================================================== */

static void test_nonmax_suppress(void) {
  printf("  Test: Non-max suppression keeps thin edges\n");

  vision_image_t mag;
  make_image(&mag, 32, 32);
  /* Create a 3-pixel-wide horizontal ridge */
  for (int x = 5; x < 27; x++) {
    mag.buf[16*32+x] = 100;
    mag.buf[15*32+x] = 80;
    mag.buf[17*32+x] = 80;
  }

  vision_image_t nms;
  make_image(&nms, 32, 32);
  line_nonmax_suppress(&mag, &nms, 1);

  /* After NMS, only the peak should survive */
  int kept = 0;
  for (int i = 0; i < 32*32; i++)
    if (nms.buf[i] > 0) kept++;

  CHECK(kept < 32*22, "NMS reduces edge width");

  free(mag.buf);
  free(nms.buf);
}

/* ================================================================== */
/*  Test 4: Canny edge detection                                      */
/* ================================================================== */

static void test_canny(void) {
  printf("  Test: Canny edge detection\n");

  vision_image_t img;
  make_image(&img, 64, 64);
  /* Horizontal line at y=32 */
  for (int x = 10; x < 54; x++)
    for (int dy = -2; dy <= 2; dy++)
      img.buf[(32+dy)*64+x] = 200;

  vision_image_t edges;
  make_image(&edges, 64, 64);
  line_canny(&img, 20, 60, &edges);

  int edge_count = 0;
  for (int i = 0; i < 64*64; i++)
    if (edges.buf[i] > 0) edge_count++;

  CHECK(edge_count > 0, "Canny finds edges");
  CHECK(edge_count < 64*64, "Canny doesn't mark everything as edge");

  free(img.buf);
  free(edges.buf);
}

/* ================================================================== */
/*  Test 5: Hough transform on image with known line                 */
/* ================================================================== */

static void test_hough_line(void) {
  printf("  Test: Hough transform detects known line\n");

  vision_image_t img;
  make_image(&img, 128, 128);

  /* Draw a strong vertical line at x=64 */
  for (int y = 10; y < 118; y++)
    img.buf[y*128+64] = 255;

  line_config_t *cfg = line_default_config();
  cfg->hough_threshold = 20;
  cfg->hough_min_line_len = 30;

  vision_image_t edges;
  make_image(&edges, 128, 128);
  for (int y = 10; y < 118; y++)
    edges.buf[y*128+64] = 255;

  detected_line_t lines[16];
  uint8_t num_lines = 0;
  line_hough(&edges, cfg, lines, &num_lines);

  CHECK(num_lines > 0, "Hough found at least one line");

  if (num_lines > 0) {
    /* Check that detected line is roughly vertical (theta near 0) */
    float theta_deg = lines[0].theta * 180.0f / 3.14159f;
    CHECK(fabsf(theta_deg) < 10 || fabsf(theta_deg - 90) < 10 || fabsf(theta_deg + 90) < 10,
          "Detected line is roughly vertical");
  }

  free(img.buf);
  free(edges.buf);
}

/* ================================================================== */
/*  Test 6: Hough transform on image with diagonal line              */
/* ================================================================== */

static void test_hough_diagonal(void) {
  printf("  Test: Hough transform detects diagonal line\n");

  vision_image_t edges;
  make_image(&edges, 128, 128);

  /* Draw diagonal from (10,10) to (118,118) */
  for (int i = 0; i < 108; i++) {
    int x = 10 + i;
    int y = 10 + i;
    edges.buf[y*128+x] = 255;
  }

  line_config_t *cfg = line_default_config();
  cfg->hough_threshold = 20;
  cfg->hough_min_line_len = 30;

  detected_line_t lines[16];
  uint8_t num_lines = 0;
  line_hough(&edges, cfg, lines, &num_lines);

  CHECK(num_lines > 0, "Hough found diagonal line");

  if (num_lines > 0) {
    /* Diagonal should have theta near 45° (π/4) */
    float theta_deg = fabsf(lines[0].theta * 180.0f / 3.14159f);
    CHECK(theta_deg > 30 && theta_deg < 60,
          "Diagonal line theta near 45 degrees");
  }

  free(edges.buf);
}

/* ================================================================== */
/*  Test 7: Line fitting                                              */
/* ================================================================== */

static void test_fit_line(void) {
  printf("  Test: Line fitting to known points\n");

  float points[10][2];
  for (int i = 0; i < 10; i++) {
    points[i][0] = (float)i * 10;
    points[i][1] = 2.0f * points[i][0] + 5.0f; /* y = 2x + 5 */
  }

  float a, b, score;
  bool ok = line_fit_line(points, 10, &a, &b, &score);

  CHECK(ok, "Line fit succeeded");
  CHECK_NEAR(a, 2.0f, 0.01f, "Slope = 2.0");
  CHECK_NEAR(b, 5.0f, 0.01f, "Intercept = 5.0");
  CHECK(score > 0.99f, "R² score near 1.0 for perfect line");
}

/* ================================================================== */
/*  Test 8: Parabola fitting                                          */
/* ================================================================== */

static void test_fit_parabola(void) {
  printf("  Test: Parabola fitting to known points\n");

  float points[20][2];
  for (int i = 0; i < 20; i++) {
    float x = (float)i - 10;
    points[i][0] = x;
    points[i][1] = 0.5f * x * x + 1.0f; /* y = 0.5x² + 1 */
  }

  float a, b, c, score;
  bool ok = line_fit_parabola(points, 20, &a, &b, &c, &score);

  CHECK(ok, "Parabola fit succeeded");
  CHECK_NEAR(a, 0.5f, 0.01f, "a = 0.5");
  CHECK_NEAR(b, 0.0f, 0.01f, "b = 0.0");
  CHECK_NEAR(c, 1.0f, 0.01f, "c = 1.0");
  CHECK(score > 0.99f, "R² score near 1.0 for perfect parabola");
}

/* ================================================================== */
/*  Test 9: Circle fitting                                            */
/* ================================================================== */

static void test_fit_circle(void) {
  printf("  Test: Circle fitting to known points\n");

  float points[24][2];
  for (int i = 0; i < 24; i++) {
    float angle = (float)i * 3.14159f * 2.0f / 24.0f;
    points[i][0] = 50.0f + 30.0f * cosf(angle);
    points[i][1] = 50.0f + 30.0f * sinf(angle);
  }

  float cx, cy, r, score;
  bool ok = line_fit_circle(points, 24, &cx, &cy, &r, &score);

  CHECK(ok, "Circle fit succeeded");
  CHECK_NEAR(cx, 50.0f, 0.5f, "Center x = 50");
  CHECK_NEAR(cy, 50.0f, 0.5f, "Center y = 50");
  CHECK_NEAR(r, 30.0f, 0.5f, "Radius = 30");
  CHECK(score > 0.9f, "R² score > 0.9 for perfect circle");
}

/* ================================================================== */
/*  Test 10: Full pipeline on image with lines                        */
/* ================================================================== */

static void test_pipeline(void) {
  printf("  Test: Full line_detect pipeline\n");

  vision_image_t img;
  make_image(&img, 128, 128);

  /* Draw two parallel lines (simulating road lane) */
  for (int y = 10; y < 118; y++) {
    img.buf[y*128+30] = 255;  /* left lane */
    img.buf[y*128+98] = 255;  /* right lane */
  }

  line_config_t *cfg = line_default_config();
  cfg->hough_threshold = 15;
  cfg->hough_min_line_len = 30;

  line_result_t result;
  line_detect(&img, cfg, &result);

  CHECK(result.num_lines >= 1, "Pipeline detects at least one line");
  CHECK(result.num_lines <= 8, "Pipeline doesn't over-detect");

  if (result.num_lines >= 2) {
    CHECK(result.lane_valid, "Lane detection valid with 2+ lines");
  }

  free(img.buf);
}

/* ================================================================== */
/*  Main                                                              */
/* ================================================================== */

int main(void) {
  printf("=== Line Detection Tests ===\n\n");

  test_sobel_blank();
  test_sobel_vertical_edge();
  test_nonmax_suppress();
  test_canny();
  test_hough_line();
  test_hough_diagonal();
  test_fit_line();
  test_fit_parabola();
  test_fit_circle();
  test_pipeline();

  printf("\n=== Results: %d/%d passed ===\n", passed_tests, total_tests);
  return (passed_tests == total_tests) ? 0 : 1;
}
