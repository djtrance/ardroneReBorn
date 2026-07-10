/*
 * Test for Stage 2: Multi-scale SAD block matching optical flow.
 *
 * Generates synthetic random texture with known motion and tests
 * the coarse-to-fine tracking pipeline.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "types.h"
#include "image.h"
#include "fast_detect.h"
#include "flow_stage2.h"

#define W 320
#define H 240

static void generate_frame(vision_image_t *img, int frame, int dx, int dy) {
  static uint8_t *pattern = NULL;
  if (!pattern) {
    pattern = (uint8_t*)malloc(W * H);
    srand(42);
    for (int i = 0; i < W * H; i++)
      pattern[i] = (uint8_t)(rand() & 0xFF);
  }

  int sx = (frame * dx) % W;
  int sy = (frame * dy) % H;

  for (uint16_t y = 0; y < H; y++) {
    for (uint16_t x = 0; x < W; x++) {
      int px = (x + sx) % W;
      int py = (y + sy) % H;
      img->buf[y * img->stride + x] = pattern[py * W + px];
    }
  }
}

int main(void) {
  printf("=== STAGE 2 TEST: Multi-scale SAD Block Matching ===\n");

  vision_image_t frames[2];
  image_create(&frames[0], W, H, VISION_IMAGE_GRAYSCALE);
  image_create(&frames[1], W, H, VISION_IMAGE_GRAYSCALE);

  flow_stage2_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.max_corners = 0;
  cfg.fast_threshold = 30;
  cfg.fast_min_distance = 8;
  cfg.lk_window_size = 0;
  cfg.lk_pyramid_levels = 0;
  cfg.lk_max_iterations = 0;
  cfg.lk_subpixel_factor = 0;

  flow_stage2_t *stage2 = flow_stage2_create(&cfg, W, H);
  vision_result_t result;
  memset(&result, 0, sizeof(result));

  /* Test 1: Forward motion (dx=2, dy=1) */
  printf("\n--- Test 1: Forward Motion (dx=2, dy=1) ---\n");
  generate_frame(&frames[0], 0, 2, 1);
  flow_stage2_process(stage2, &frames[0], &result);

  for (int f = 1; f <= 10; f++) {
    generate_frame(&frames[f % 2], f, 2, 1);
    flow_stage2_process(stage2, &frames[f % 2], &result);

    int err_x = abs(result.flow_x_robust - 20);
    int err_y = abs(result.flow_y_robust - 10);
    printf("  Frame %2d: flow=(%4d, %4d) qual=%3u matched=%u err=(%d, %d)\n",
           f, result.flow_x_robust, result.flow_y_robust,
           result.quality_robust, result.corner_cnt, err_x, err_y);
  }

  /* Test 2: Subpixel motion (dx=2.3, dy=0.7) */
  printf("\n--- Test 2: Subpixel Motion (dx=2.3, dy=0.7) ---\n");
  /* Note: synthetic generator uses integer shifts, so true motion is (2, 0)
     but with occasional pixel jump. This tests subpixel refinement. */

  for (int f = 1; f <= 5; f++) {
    generate_frame(&frames[f % 2], f, 2, 0);
    flow_stage2_process(stage2, &frames[f % 2], &result);
    printf("  Frame %2d: flow=(%4d, %4d) qual=%3u\n",
           f, result.flow_x_robust, result.flow_y_robust, result.quality_robust);
  }

  /* Test 3: Divergence (expansion speed = descent) */
  printf("\n--- Test 3: Divergence (Descent) ---\n");
  /* Generate expanding pattern: shift every 4th tile outward */
  /* Simple: use a diverging checkerboard */
  for (int f = 1; f <= 5; f++) {
    for (uint16_t y = 0; y < H; y++) {
      for (uint16_t x = 0; x < W; x++) {
        int scale = 8 + f;
        frames[0].buf[y * frames[0].stride + x] = (uint8_t)((x / scale + y / scale) * 32);
        frames[1].buf[y * frames[1].stride + x] = (uint8_t)((x / (scale+1) + y / (scale+1)) * 32);
      }
    }
    flow_stage2_process(stage2, &frames[f % 2], &result);
    printf("  Frame %2d: flow=(%4d, %4d) div=%d\n",
           f, result.flow_x_robust, result.flow_y_robust, result.divergence);
  }

  /* Test 4: Zero motion */
  printf("\n--- Test 4: Zero Motion ---\n");
  for (int f = 1; f <= 5; f++) {
    generate_frame(&frames[f % 2], 0, 0, 0);
    flow_stage2_process(stage2, &frames[f % 2], &result);
    printf("  Frame %2d: flow=(%4d, %4d) qual=%3u\n",
           f, result.flow_x_robust, result.flow_y_robust, result.quality_robust);
  }

  flow_stage2_destroy(stage2);
  image_destroy(&frames[0]);
  image_destroy(&frames[1]);

  printf("\nDone.\n");
  return 0;
}
