/*
 * Test for Stage 3 (Rotation) and Stage 4 (Pattern Detection).
 *
 * Stage 3: Generates synthetic flow fields with known rotation
 *          and verifies that rotation_estimate recovers it.
 *
 * Stage 4: Generates synthetic landing pad patterns and verifies
 *          that pattern_detect_landing_pad finds them.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "types.h"
#include "image.h"
#include "rotation.h"
#include "pattern.h"

#define W 320
#define H 240

/* Generate synthetic flow vectors that simulate rotation */
/* Generate flow field: generate points, then compute flow from known rotation */
static uint16_t gen_rotation_flow(int32_t *flow_x, int32_t *flow_y,
                                   uint16_t *px, uint16_t *py,
                                   uint16_t max_count, float yaw_deg_per_frame) {
  float cx = W / 2.0f;
  float cy = H / 2.0f;
  float yaw_rad = yaw_deg_per_frame * 3.14159f / 180.0f;
  uint16_t count = 0;

  /* Centered grid: symmetric around (160, 120) */
  for (int16_t gy = -5; gy <= 5 && count < max_count; gy++) {
    for (int16_t gx = -5; gx <= 5 && count < max_count; gx++) {
      int16_t x = (int16_t)(cx + gx * 24);
      int16_t y = (int16_t)(cy + gy * 18);
      if (x < 0 || x >= (int16_t)W || y < 0 || y >= (int16_t)H)
        continue;

      px[count] = (uint16_t)x;
      py[count] = (uint16_t)y;

      float rx = x - cx;
      float ry = y - cy;
      float dist = sqrtf(rx * rx + ry * ry);

      if (dist > 1.0f) {
        float mag = dist * yaw_rad;
        flow_x[count] = (int32_t)((-ry / dist) * mag * 10);
        flow_y[count] = (int32_t)((rx / dist) * mag * 10);
      } else {
        flow_x[count] = 0;
        flow_y[count] = 0;
      }
      count++;
    }
  }
  return count;
}

/* Generate a synthetic landing pad image (solid dark circle) */
static void gen_landing_pad(vision_image_t *img, uint16_t cx, uint16_t cy,
                             uint16_t size) {
  uint16_t half = size / 2;
  for (uint16_t y = 0; y < H; y++) {
    for (uint16_t x = 0; x < W; x++) {
      /* Background: light gray with slight texture */
      img->buf[y * img->stride + x] = 180 + ((x + y * 3) % 20);

      /* Dark circle (landing pad) */
      int16_t dx = (int16_t)x - (int16_t)cx;
      int16_t dy = (int16_t)y - (int16_t)cy;
      if (dx * dx + dy * dy <= (int32_t)half * half) {
        img->buf[y * img->stride + x] = 25 + ((x * 7 + y * 13) % 15);  /* dark texture */
      }
    }
  }
}

int main(void) {
  printf("=== STAGE 3 + 4 TEST ===\n");

  /* ---- Stage 3: Rotation ---- */
  printf("\n--- Stage 3: Visual Rotation Estimation ---\n");

  rotation_ctx_t *rot = rotation_create(W, H);

  /* Helper: call estimate N times to let filter converge */
  #define CONVERGE(n) for (int _cv = 0; _cv < (n); _cv++) { \
    rotation_estimate(rot, fpx, fpy, ppx, ppy, count, &_q); }

  int32_t fpx[200], fpy[200];
  uint16_t ppx[200], ppy[200];
  uint8_t _q;
  uint16_t count;

  /* Test 3a: 1 degree yaw per frame */
  printf("\nTest 3a: Yaw = +1.0 deg/frame (clockwise)\n");
  count = gen_rotation_flow(fpx, fpy, ppx, ppy, 200, 1.0f);
  CONVERGE(10);
  int32_t yaw = rotation_estimate(rot, fpx, fpy, ppx, ppy, count, &_q);
  printf("  Estimated: %d millidegrees/frame (expected: 1000) qual=%u\n",
         yaw, _q);

  /* Test 3b: 3 degrees yaw per frame */
  printf("\nTest 3b: Yaw = +3.0 deg/frame\n");
  rotation_reset(rot);
  count = gen_rotation_flow(fpx, fpy, ppx, ppy, 200, 3.0f);
  CONVERGE(10);
  yaw = rotation_estimate(rot, fpx, fpy, ppx, ppy, count, &_q);
  printf("  Estimated: %d millidegrees/frame (expected: ~3000) qual=%u\n",
         yaw, _q);

  /* Test 3c: Negative yaw (counter-clockwise) */
  printf("\nTest 3c: Yaw = -2.0 deg/frame\n");
  rotation_reset(rot);
  count = gen_rotation_flow(fpx, fpy, ppx, ppy, 200, -2.0f);
  CONVERGE(10);
  yaw = rotation_estimate(rot, fpx, fpy, ppx, ppy, count, &_q);
  printf("  Estimated: %d millidegrees/frame (expected: ~-2000) qual=%u\n",
         yaw, _q);

  /* Test 3d: Zero rotation (pure translation) with centered grid */
  printf("\nTest 3d: Pure translation (flow_x=10, flow_y=5)\n");
  rotation_reset(rot);
  count = gen_rotation_flow(fpx, fpy, ppx, ppy, 200, 0.0f);
  for (uint16_t i = 0; i < count; i++) {
    fpx[i] = 10 * 10;  /* flow_x = 10 pixels */
    fpy[i] = 5 * 10;
  }

  CONVERGE(10);
  yaw = rotation_estimate(rot, fpx, fpy, ppx, ppy, count, &_q);
  printf("  Estimated: %d millidegrees/frame (expected: ~0) qual=%u\n",
         yaw, _q);

  /* ---- Stage 4: Pattern Detection ---- */
  printf("\n--- Stage 4: Landing Pad Detection ---\n");

  pattern_ctx_t *pat = pattern_create(W, H);
  pattern_set_pad_size(pat, 300);  /* 30cm pad */

  vision_image_t frame;
  image_create(&frame, W, H, VISION_IMAGE_GRAYSCALE);

  /* Test 4a: Landing pad at center */
  printf("\nTest 4a: Landing pad at center, size=40px\n");
  gen_landing_pad(&frame, W / 2, H / 2, 40);
  landing_pad_t pad;
  pattern_detect_landing_pad(pat, &frame, &pad);
  printf("  Found: %s\n", pad.found ? "YES" : "NO");
  if (pad.found) {
    printf("  Position: (%u, %u)  size=%u  confidence=%u  dist=%.0fcm\n",
           pad.center_x, pad.center_y,
           pad.size, pad.confidence, pad.distance_est);
  }

  /* Test 4b: Landing pad off-center */
  printf("\nTest 4b: Landing pad at (80, 60), size=60px\n");
  gen_landing_pad(&frame, 80, 60, 60);
  pattern_detect_landing_pad(pat, &frame, &pad);
  printf("  Found: %s\n", pad.found ? "YES" : "NO");
  if (pad.found) {
    printf("  Position: (%u, %u)  size=%u  confidence=%u\n",
           pad.center_x, pad.center_y,
           pad.size, pad.confidence);
  }

  /* Test 4c: No landing pad (random texture) */
  printf("\nTest 4c: No landing pad (random texture)\n");
  srand(42);
  for (uint32_t i = 0; i < (uint32_t)W * H; i++)
    frame.buf[i] = (uint8_t)(rand() & 0xFF);
  pattern_detect_landing_pad(pat, &frame, &pad);
  printf("  Found: %s (expected: NO)\n", pad.found ? "YES" : "NO");

  /* Test 4d: Landing pad tracking with flow */
  printf("\nTest 4d: Track landing pad with flow (dx=4, dy=2)\n");
  gen_landing_pad(&frame, 160, 120, 50);
  pattern_detect_landing_pad(pat, &frame, &pad);
  printf("  Initial: (%u, %u)\n", pad.center_x, pad.center_y);

  pattern_track_landing_pad(pat, 40, 20, &pad);
  printf("  After flow(4,2): (%u, %u) (expected: ~164, 122)\n",
         pad.center_x, pad.center_y);

  /* Test 4e: Home snapshot */
  printf("\nTest 4e: Home snapshot\n");
  srand(123);
  for (uint32_t i = 0; i < (uint32_t)W * H; i++)
    frame.buf[i] = (uint8_t)(rand() & 0xFF);
  pattern_snapshot_home(pat, &frame);

  home_marker_t home;
  pattern_detect_home(pat, &frame, &home);
  printf("  Home found: %s (placeholder)\n", home.found ? "YES" : "NO");

  /* Cleanup */
  rotation_destroy(rot);
  pattern_destroy(pat);
  image_destroy(&frame);

  printf("\nDone.\n");
  return 0;
}
