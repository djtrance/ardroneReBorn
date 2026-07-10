#ifndef FAST_DETECT_H
#define FAST_DETECT_H

#include "types.h"
#include <stdint.h>

/* FAST corner detection (FAST-9)
 *
 * Portable C implementation, derived from Paparazzi fast_rosten.c.
 * Detects corners using the segment test: 9 contiguous pixels on the
 * Bresenham circle of radius 3 must all be brighter or darker than
 * the center pixel ± threshold.
 *
 * Typical parameters for 320x240:
 *   threshold = 30
 *   min_distance = 8
 *   max_corners = 50-200
 */

#define FAST_MAX_CORNERS 512

/* Corner structure */
typedef struct {
  uint16_t x;       /* pixel x coordinate */
  uint16_t y;       /* pixel y coordinate */
  uint32_t score;   /* corner strength (higher = stronger) */
} fast_corner_t;

/* List of detected corners */
typedef struct {
  fast_corner_t corners[FAST_MAX_CORNERS];
  uint16_t      count;
} fast_corners_t;

/* Detect FAST-9 corners in a grayscale image.
 *
 * Parameters:
 *   img        - grayscale input image
 *   threshold  - intensity difference threshold (typical: 20-40)
 *   corners    - output: detected corners
 *
 * Returns number of corners detected.
 */
uint16_t fast_detect(const vision_image_t *img, uint8_t threshold,
                     fast_corners_t *corners);

/* Score a corner candidate (sum of absolute differences on circle).
 * Higher score = stronger corner. */
uint32_t fast_corner_score(const vision_image_t *img, uint16_t x, uint16_t y,
                           uint8_t threshold);

/* Non-maximum suppression: filter corners that are too close to each other.
 *
 * Parameters:
 *   corners      - input/output: corner list to filter
 *   min_distance - minimum pixel distance between corners
 *   max_corners  - keep at most this many (best scores)
 */
void fast_nonmax_suppression(fast_corners_t *corners, uint8_t min_distance,
                             uint16_t max_corners);

#endif /* FAST_DETECT_H */
