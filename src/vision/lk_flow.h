#ifndef LK_FLOW_H
#define LK_FLOW_H

#include "types.h"
#include "fast_detect.h"

/* Lucas-Kanade sparse optical flow with image pyramid.
 *
 * Derived from Paparazzi opticflow_calculator.c and OpenCV LK.
 *
 * Pipeline:
 *   1. Build Gaussian pyramid of both frames (3-4 levels)
 *   2. Detect FAST9 corners on current frame (lowest resolution level)
 *   3. Track corners through pyramid, coarse-to-fine
 *   4. Flow = accumulated displacement across all levels
 *   5. Outlier rejection via median filtering
 */

/* LK tracking result for one corner */
typedef struct {
  uint16_t x;        /* original corner x (full resolution) */
  uint16_t y;        /* original corner y (full resolution) */
  int32_t  flow_x;   /* subpixel flow (factor = LK_SUBPIXEL_FACTOR) */
  int32_t  flow_y;
  uint32_t error;    /* matching error (SSD) */
  uint8_t  status;   /* 0 = lost, 1 = tracked, 2 = rejected */
} lk_track_t;

/* LK context */
typedef struct lk_context_s lk_context_t;

/* Create LK context */
lk_context_t* lk_context_create(const flow_stage2_config_t *cfg,
                                uint16_t image_width,
                                uint16_t image_height);

/* Destroy LK context */
void lk_context_destroy(lk_context_t *ctx);

/* Track corners from previous frame to current frame.
 *
 * Parameters:
 *   ctx    - LK context
 *   prev   - previous grayscale frame
 *   curr   - current grayscale frame
 *   tracks - output: tracked corner positions and flow
 *   max_tracks - maximum number of tracks to output
 *
 * Returns number of successfully tracked corners.
 */
uint16_t lk_track(lk_context_t *ctx,
                  const vision_image_t *prev,
                  const vision_image_t *curr,
                  lk_track_t *tracks,
                  uint16_t max_tracks);

/* Get the current corner set (after detection, before tracking) */
void lk_get_corners(const lk_context_t *ctx, fast_corners_t *corners);

/* Update configuration at runtime */
void lk_reconfigure(lk_context_t *ctx, const flow_stage2_config_t *cfg);

#define LK_SUBPIXEL_FACTOR 10

#endif /* LK_FLOW_H */
