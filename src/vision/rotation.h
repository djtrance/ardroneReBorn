#ifndef VISION_ROTATION_H
#define VISION_ROTATION_H

#include "types.h"

/* Stage 3: Visual rotation detection from optical flow field.
 *
 * Uses tangential component of flow vectors to estimate yaw rate.
 * For each flow vector at position (x,y) relative to image center (cx,cy):
 *   Radial component:   flow · (x-cx, y-cy) / |r|  → translation/divergence
 *   Tangential component: flow · perpendicular to r → rotation (yaw)
 *
 * Positive yaw = clockwise rotation (drone yaw right).
 */

/* Rotation estimation context */
typedef struct rotation_ctx_s rotation_ctx_t;

/* Create rotation context */
rotation_ctx_t* rotation_create(uint16_t image_width, uint16_t image_height);

/* Destroy rotation context */
void rotation_destroy(rotation_ctx_t *ctx);

/* Estimate yaw rate from a set of flow vectors.
 *
 * Parameters:
 *   ctx      - rotation context
 *   flow_x   - array of flow vectors X (subpixel * 10)
 *   flow_y   - array of flow vectors Y (subpixel * 10)
 *   positions_x - array of tile/corner X positions
 *   positions_y - array of tile/corner Y positions
 *   count    - number of flow vectors
 *   yaw_rate - output: estimated yaw rate (millidegrees/frame)
 *   quality  - output: estimation quality (0-255)
 *
 * Returns the estimated yaw rate in millidegrees per frame.
 */
int32_t rotation_estimate(rotation_ctx_t *ctx,
                           const int32_t *flow_x, const int32_t *flow_y,
                           const uint16_t *positions_x,
                           const uint16_t *positions_y,
                           uint16_t count,
                           uint8_t *quality);

/* Fuse visual yaw with gyro data (complementary filter).
 *
 * Parameters:
 *   gyro_yaw_rate   - gyro yaw rate in millidegrees/s
 *   visual_yaw_rate - visual yaw rate in millidegrees/s
 *   dt_ms           - time step in milliseconds
 *   alpha           - fusion factor (0.0 = gyro only, 1.0 = visual only)
 *
 * Returns fused yaw rate in millidegrees/s.
 */
int32_t rotation_fuse_with_gyro(int32_t gyro_yaw_rate,
                                 int32_t visual_yaw_rate,
                                 uint32_t dt_ms, float alpha);

/* Reset cumulative yaw */
void rotation_reset(rotation_ctx_t *ctx);

#endif /* VISION_ROTATION_H */
