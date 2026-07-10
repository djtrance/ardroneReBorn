#include "rotation.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

struct rotation_ctx_s {
  uint16_t image_width;
  uint16_t image_height;
  float    center_x;
  float    center_y;

  int32_t  cumulative_yaw;
  float    filtered_yaw_rate;
};

rotation_ctx_t* rotation_create(uint16_t image_width, uint16_t image_height) {
  rotation_ctx_t *ctx = (rotation_ctx_t*)calloc(1, sizeof(rotation_ctx_t));
  if (!ctx) return NULL;

  ctx->image_width = image_width;
  ctx->image_height = image_height;
  ctx->center_x = image_width / 2.0f;
  ctx->center_y = image_height / 2.0f;
  ctx->cumulative_yaw = 0;
  ctx->filtered_yaw_rate = 0.0f;

  return ctx;
}

void rotation_destroy(rotation_ctx_t *ctx) {
  free(ctx);
}

int32_t rotation_estimate(rotation_ctx_t *ctx,
                           const int32_t *flow_x, const int32_t *flow_y,
                           const uint16_t *positions_x,
                           const uint16_t *positions_y,
                           uint16_t count,
                           uint8_t *quality) {
  if (count == 0) {
    if (quality) *quality = 0;
    return 0;
  }

  float cx = ctx->center_x;
  float cy = ctx->center_y;

  /*
   * Rotation estimation using cross-product method.
   *
   * For a point at (rx, ry) from center with flow (dx, dy),
   * the rotation angle θ satisfies:
   *   cross = dx * (-ry) + dy * rx = θ * (rx² + ry²)
   *
   * Because for rotation: (dx, dy) = θ * (-ry, rx)
   * So cross = θ(-ry)(-ry) + θ(rx)(rx) = θ(rx² + ry²)
   *
   * Therefore θ = cross / (rx² + ry²)
   *
   * We average θ over all points weighted by r² (to suppress noise
   * from points near the center where rotation signal is weak).
   */

  int64_t total_cross = 0;
  int64_t total_r2 = 0;
  int valid = 0;

  for (uint16_t i = 0; i < count; i++) {
    int32_t dx = flow_x[i];     /* subpixel * 10 */
    int32_t dy = flow_y[i];

    float rx = positions_x[i] - cx;
    float ry = positions_y[i] - cy;
    float r2 = rx * rx + ry * ry;

    if (r2 < 4.0f) continue;    /* skip center */

    /* Cross product: dx*(-ry) + dy*rx */
    int64_t cross = (int64_t)((float)dx * (-ry) + (float)dy * rx);

    /* Weight by r²: points farther from center contribute more */
    total_cross += cross;
    total_r2 += (int64_t)r2;
    valid++;
  }

  if (valid < 3 || total_r2 == 0) {
    if (quality) *quality = 0;
    return 0;
  }

  /*
   * θ_rad = cross / r²
   * But cross is in subpixel * 10 pixels * pixels = subpixel·pixels²
   * And r² is in pixels²
   * So θ = cross / r² * (1/10) rad  (correcting subpixel factor)
   *
   * θ_mdeg = θ_rad * 180/π * 1000
   *        = cross / r² / 10 * 57296
   *        = cross * 57296 / (r² * 10)
   *        = cross * 5729.6 / r²
   */

  /* Use fixed-point: cross * 57296 / (r² * 10) */
  int32_t yaw_mdeg = (int32_t)(total_cross * 57296LL / (total_r2 * 10LL));

  /* Low-pass filter. Use higher alpha for faster tracking. */
  const float alpha = 0.6f;
  ctx->filtered_yaw_rate = ctx->filtered_yaw_rate * (1.0f - alpha)
                          + (float)yaw_mdeg * alpha;

  ctx->cumulative_yaw += (int32_t)ctx->filtered_yaw_rate;

  /* Quality: based on valid count and flow coherence */
  uint8_t q = (uint8_t)(valid * 255 / count);
  if (q > 255) q = 255;
  if (quality) *quality = q;

  return (int32_t)ctx->filtered_yaw_rate;
}

int32_t rotation_fuse_with_gyro(int32_t gyro_yaw_rate,
                                 int32_t visual_yaw_rate,
                                 uint32_t dt_ms, float alpha) {
  (void)dt_ms;
  return (int32_t)(alpha * visual_yaw_rate + (1.0f - alpha) * gyro_yaw_rate);
}

void rotation_reset(rotation_ctx_t *ctx) {
  ctx->cumulative_yaw = 0;
  ctx->filtered_yaw_rate = 0.0f;
}
