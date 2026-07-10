#ifndef VISION_PATTERN_H
#define VISION_PATTERN_H

#include "types.h"

/* Stage 4: Pattern detection for navigation.
 *
 * Features:
 *   - Landing pad detection (high-contrast target on ground)
 *   - Precision landing guidance (center + size tracking)
 *   - Visual return-to-home (feature snapshot matching)
 *
 * Landing pad: dark cross/square on light background, or concentric circles.
 * Detection uses adaptive threshold + connected components.
 */

#define PATTERN_MAX_BLOBS 64

/* Blob structure from connected component analysis */
typedef struct {
  uint16_t x;         /* centroid X */
  uint16_t y;         /* centroid Y */
  uint32_t area;      /* pixel count */
  uint16_t min_x;     /* bounding box */
  uint16_t max_x;
  uint16_t min_y;
  uint16_t max_y;
  uint16_t width;     /* bounding box dimensions */
  uint16_t height;
  float    aspect;    /* width/height ratio */
  uint8_t  mean;      /* mean intensity */
} pattern_blob_t;

/* Blob list */
typedef struct {
  pattern_blob_t blobs[PATTERN_MAX_BLOBS];
  uint16_t       count;
} pattern_blobs_t;

/* Landing pad detection result */
typedef struct {
  bool     found;
  uint16_t center_x;       /* pixel X of pad center */
  uint16_t center_y;       /* pixel Y of pad center */
  uint16_t size;           /* apparent size (max dimension in pixels) */
  uint8_t  confidence;     /* 0-255 */
  float    distance_est;   /* estimated distance in cm (from known size) */
} landing_pad_t;

/* Home marker detection result */
typedef struct {
  bool     found;
  int16_t  angle;          /* angle to home (centidegrees, 0=straight ahead) */
  int16_t  distance;       /* estimated distance (cm) */
  uint8_t  confidence;
} home_marker_t;

/* Pattern detection context */
typedef struct pattern_ctx_s pattern_ctx_t;

/* Create pattern detection context */
pattern_ctx_t* pattern_create(uint16_t image_width, uint16_t image_height);

/* Destroy pattern detection context */
void pattern_destroy(pattern_ctx_t *ctx);

/* Detect landing pad in grayscale image.
 *
 * Algorithm:
 *   1. Adaptive threshold (local mean - offset) to binarize
 *   2. Connected component labeling
 *   3. Filter blobs by size, aspect ratio, position
 *   4. Score remaining blobs for landing pad match
 *
 * Parameters:
 *   ctx  - pattern context
 *   img  - grayscale input image
 *   pad  - output: detected landing pad
 */
void pattern_detect_landing_pad(pattern_ctx_t *ctx,
                                 const vision_image_t *img,
                                 landing_pad_t *pad);

/* Find all blobs in thresholded image */
void pattern_find_blobs(pattern_ctx_t *ctx,
                         const vision_image_t *img,
                         uint8_t threshold,
                         pattern_blobs_t *blobs);

/* Track landing pad across frames using flow (Stage 1 output).
 * Updates the expected position of the pad based on estimated flow. */
void pattern_track_landing_pad(pattern_ctx_t *ctx,
                                int32_t flow_x, int32_t flow_y,
                                landing_pad_t *pad);

/* Set known landing pad physical size (mm) for distance estimation */
void pattern_set_pad_size(pattern_ctx_t *ctx, uint16_t size_mm);

/* Detect home marker (for return-to-home).
 * Uses corner matching if a snapshot was taken at takeoff. */
void pattern_detect_home(pattern_ctx_t *ctx,
                          const vision_image_t *img,
                          home_marker_t *home);

/* Take a snapshot of the current position for return-to-home. */
void pattern_snapshot_home(pattern_ctx_t *ctx, const vision_image_t *img);

#endif /* VISION_PATTERN_H */
