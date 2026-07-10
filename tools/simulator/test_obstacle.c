#include "obstacle.h"
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

/* Base texture generator: pseudo-random per-pixel values.
 * Uses a=53, b=91. Verified: no (dx,dy) ≠ (0,0) in [-8,8] satisfies
 * 53*dx + 91*dy ≡ 0 (mod 128). Also no offset ≠ (3,0) in [-4,4] matches
 * the asymmetry shift (3,0) hash value 159 mod 128 = 31. */
static uint8_t base_pixel(uint16_t x, uint16_t y, uint8_t bg) {
  (void)bg;
  return (uint8_t)(((uint32_t)x * 53 + (uint32_t)y * 91) & 0x7F) + 64;
}

/* Create a grayscale image with pseudo-random texture and optional vertical stripe */
static void make_test_image(vision_image_t *img, uint16_t w, uint16_t h,
                             uint8_t stripe, uint16_t stripe_x) {
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      uint8_t val = base_pixel(x, y, 0);
      if (stripe_x > 0 && x > stripe_x - 3 && x < stripe_x + 3) {
        val = stripe;
      }
      img->buf[y * img->stride + x] = val;
    }
  }
}

/* Make a zoomed version (simulates approaching a wall).
 * Samples the same base texture as make_test_image. */
static void make_zoomed_image(vision_image_t *img, uint16_t w, uint16_t h,
                               float zoom) {
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      float sx = (x - w/2) / zoom + w/2;
      float sy = (y - h/2) / zoom + h/2;
      int ix = (int)sx;
      int iy = (int)sy;
      if (ix < 0) ix = 0; if (ix >= (int)w) ix = w-1;
      if (iy < 0) iy = 0; if (iy >= (int)h) iy = h-1;
      img->buf[y * img->stride + x] = base_pixel(ix, iy, 0);
    }
  }
}

/* Test 1: no motion → no looming */
static void test_no_motion(void) {
  vision_image_t f1, f2;
  image_create(&f1, 160, 120, VISION_IMAGE_GRAYSCALE);
  image_create(&f2, 160, 120, VISION_IMAGE_GRAYSCALE);

  make_test_image(&f1, 160, 120, 255, 0);
  make_test_image(&f2, 160, 120, 255, 0);

  obstacle_t *obs = obstacle_create(160, 120);
  obstacle_result_t res;

  obstacle_process(obs, &f1, &f2, &res);
  CHECK(res.looming == 0, "no motion → no looming");

  obstacle_destroy(obs);
  image_destroy(&f1);
  image_destroy(&f2);
}

/* Test 2: looming with zoomed frames */
static void test_looming(void) {
  vision_image_t f1, f2;
  image_create(&f1, 160, 120, VISION_IMAGE_GRAYSCALE);
  image_create(&f2, 160, 120, VISION_IMAGE_GRAYSCALE);

  /* Both from same base texture, f2 is zoomed in relative to f1 */
  make_zoomed_image(&f1, 160, 120, 1.0f);
  make_zoomed_image(&f2, 160, 120, 1.06f);

  obstacle_t *obs = obstacle_create(160, 120);
  obstacle_result_t res;

  /* Compare f2 (zoomed) vs f1 (original) */
  obstacle_process(obs, &f2, &f1, &res);

  CHECK(res.looming > 0, "zoom in → looming > 0");

  obstacle_destroy(obs);
  image_destroy(&f1);
  image_destroy(&f2);
}

/* Test 3: asymmetry — obstacle on right (right half expands more) */
static void test_asymmetry(void) {
  vision_image_t f1, f2;
  image_create(&f1, 160, 120, VISION_IMAGE_GRAYSCALE);
  image_create(&f2, 160, 120, VISION_IMAGE_GRAYSCALE);

  /* Frame 1: uniform pseudo-random texture */
  make_test_image(&f1, 160, 120, 255, 0);

  /* Frame 2: right half shifted outward by 3px (simulates obstacle approach:
   * features on the right move further right in the current frame).
   * f2[x, y] samples base_pixel(x-3, y) for x > 80 (content was 3px left
   * in prev frame, now expanded outward). */
  for (uint16_t y = 0; y < 120; y++) {
    for (uint16_t x = 0; x < 160; x++) {
      uint8_t val;
      if (x > 80) {
        int sx = x - 3;
        if (sx < 0) sx = 0;
        val = base_pixel(sx, y, 0);
      } else {
        val = base_pixel(x, y, 0);
      }
      f2.buf[y * f2.stride + x] = val;
    }
  }

  obstacle_t *obs = obstacle_create(160, 120);
  obstacle_result_t res;

  obstacle_process(obs, &f1, &f1, &res);
  obstacle_process(obs, &f2, &f1, &res);

  CHECK(res.asymmetry != 0, "right side shift → asymmetry != 0");

  obstacle_destroy(obs);
  image_destroy(&f1);
  image_destroy(&f2);
}

/* Test 4: vertical line detection */
static void test_vertical_line(void) {
  vision_image_t frame;
  image_create(&frame, 160, 120, VISION_IMAGE_GRAYSCALE);

  /* Bright vertical stripe at x=80 on textured background */
  make_test_image(&frame, 160, 120, 255, 80);

  obstacle_t *obs = obstacle_create(160, 120);
  obstacle_result_t res;

  obstacle_process(obs, &frame, &frame, &res);

  CHECK(res.line_found, "vertical stripe → line found");
  CHECK(res.line_x >= 70 && res.line_x <= 90, "line_x near stripe at 80");
  CHECK(res.line_strength > 20, "line strength > 20");

  obstacle_destroy(obs);
  image_destroy(&frame);
}

/* Test 5: no vertical line on uniform image */
static void test_no_vertical_line(void) {
  vision_image_t frame;
  image_create(&frame, 160, 120, VISION_IMAGE_GRAYSCALE);
  memset(frame.buf, 128, 160 * 120);

  obstacle_t *obs = obstacle_create(160, 120);
  obstacle_result_t res;

  obstacle_process(obs, &frame, &frame, &res);

  CHECK(!res.line_found, "uniform image → no line found");

  obstacle_destroy(obs);
  image_destroy(&frame);
}

/* Test 6: approach sequence — zooming in produces looming > 0 */
static void test_approach_sequence(void) {
  vision_image_t frames[4];
  for (int i = 0; i < 4; i++) {
    image_create(&frames[i], 160, 120, VISION_IMAGE_GRAYSCALE);
  }

  /* Frames at increasing zoom levels */
  float zooms[] = {1.0f, 1.02f, 1.04f, 1.06f};
  for (int i = 0; i < 4; i++) {
    make_zoomed_image(&frames[i], 160, 120, zooms[i]);
  }

  obstacle_t *obs = obstacle_create(160, 120);
  obstacle_result_t res;
  int looming_count = 0;

  for (int i = 1; i < 4; i++) {
    obstacle_process(obs, &frames[i], &frames[i-1], &res);
    if (res.looming > 0) looming_count++;
  }

  CHECK(looming_count > 0, "approach sequence → at least one frame with looming > 0");

  for (int i = 0; i < 4; i++) image_destroy(&frames[i]);
  obstacle_destroy(obs);
}

int main(void) {
  printf("=== Obstacle Detection Tests ===\n\n");

  test_no_motion();
  test_looming();
  test_asymmetry();
  test_vertical_line();
  test_no_vertical_line();
  test_approach_sequence();

  printf("\n%d/%d tests passed\n", passed_tests, total_tests);
  return passed_tests == total_tests ? 0 : 1;
}
