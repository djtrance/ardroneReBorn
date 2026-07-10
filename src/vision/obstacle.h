#ifndef VISION_OBSTACLE_H
#define VISION_OBSTACLE_H

#include "types.h"

/* Obstacle detection for the front camera.
 *
 * Three detectors run on each frame:
 *   1. LOOMING — symmetric divergence in the center region (wall ahead)
 *   2. ASYMMETRY — left vs right divergence imbalance (obstacle on one side)
 *   3. VERTICAL LINE — dominant vertical edge for corridor centering
 *
 * All processing at 320×240 grayscale using SAD block matching between
 * consecutive frames. The same pyramid can later be shared with Stage 2.
 */

/* Obstacle detection result */
typedef struct {
  /* Looming: 0..255, 0 = nothing, 255 = imminent collision */
  uint8_t  looming;
  /* Asymmetry: -128..127, negative = obstacle on left, positive = right */
  int8_t   asymmetry;
  /* Confidence: 0..255 */
  uint8_t  confidence;

  /* Vertical line: dominant edge near image center */
  bool     line_found;
  uint16_t line_x;         /* column of the line at image bottom (pixels) */
  int16_t  line_angle;     /* angle from vertical (centidegrees, + = lean right) */
  uint8_t  line_strength;  /* 0..255 */
} obstacle_result_t;

/* Opaque context */
typedef struct obstacle_s obstacle_t;

/* Create obstacle detection context for a given image size.
 * 'image_width' and 'image_height' should be the processing resolution (e.g. 320×240). */
obstacle_t* obstacle_create(uint16_t image_width, uint16_t image_height);

/* Destroy context */
void obstacle_destroy(obstacle_t *ctx);

/* Process a new frame.
 * 'frame' and 'prev_frame' must be grayscale images of the configured size.
 * 'prev_frame' should be the previous frame (or NULL on first call).
 * 'result' is filled with detection outputs. */
void obstacle_process(obstacle_t *ctx,
                      const vision_image_t *frame,
                      const vision_image_t *prev_frame,
                      obstacle_result_t *result);

/* Reconfigure parameters at runtime (not yet used, reserved). */
void obstacle_reconfigure(obstacle_t *ctx);

#endif /* VISION_OBSTACLE_H */
