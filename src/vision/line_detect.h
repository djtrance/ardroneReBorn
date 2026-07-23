#ifndef LINE_DETECT_H
#define LINE_DETECT_H

#include "types.h"
#include <stdint.h>
#include <stdbool.h>

/* Maximum detected lines/curves */
#define LINE_DETECT_MAX_LINES    32
#define LINE_DETECT_MAX_CIRCLES  8

/* Line representation (rho, theta form) */
typedef struct {
  float rho;         /* distance from origin (pixels) */
  float theta;       /* angle from x-axis (radians, -pi/2 to pi/2) */
  uint16_t votes;    /* Hough accumulator votes */
  uint16_t x1, y1;   /* start point (image coords) */
  uint16_t x2, y2;   /* end point (image coords) */
  float length;      /* line length in pixels */
  uint8_t strength;  /* edge strength 0-255 */
} detected_line_t;

/* Circle representation */
typedef struct {
  float cx, cy;      /* center (image coords) */
  float radius;      /* radius in pixels */
  uint16_t votes;    /* Hough accumulator votes */
  uint8_t strength;  /* edge strength 0-255 */
} detected_circle_t;

/* Curve type */
typedef enum {
  CURVE_LINE,
  CURVE_PARABOLA,
  CURVE_CIRCLE
} curve_type_t;

/* Fitted curve (for lane/road detection) */
typedef struct {
  curve_type_t type;
  float coeffs[4];   /* ax^2 + bx + c (parabola), or cx,cy,r (circle), or a,b (line y=ax+b) */
  float score;       /* fit quality 0.0-1.0 */
  uint16_t num_inliers;
  float x_min, x_max; /* horizontal extent */
} detected_curve_t;

/* Detection configuration */
typedef struct {
  /* Sobel/edge detection */
  uint8_t  sobel_threshold;    /* edge magnitude threshold (default: 30) */
  uint8_t  canny_low;          /* Canny low threshold (0=disable Canny, use raw Sobel) */
  uint8_t  canny_high;         /* Canny high threshold */
  uint8_t  suppress_radius;    /* non-max suppression radius (default: 2) */

  /* Hough line detection */
  uint8_t  hough_rho;          /* rho resolution in pixels (default: 1) */
  uint8_t  hough_theta_deg;    /* theta resolution in degrees (default: 1) */
  uint16_t hough_threshold;    /* minimum votes to detect a line (default: 40) */
  uint16_t hough_min_line_len; /* minimum line length in pixels (default: 30) */
  uint16_t hough_max_line_gap; /* maximum gap between line segments (default: 10) */

  /* Hough circle detection */
  uint16_t circle_threshold;   /* minimum votes for circle center */
  uint16_t circle_min_radius;
  uint16_t circle_max_radius;

  /* Region of interest */
  uint16_t roi_x, roi_y;
  uint16_t roi_w, roi_h;

  /* Output */
  bool     draw_edges;         /* if true, write edge map to debug image */
  bool     draw_lines;         /* if true, overlay lines on debug image */
} line_config_t;

/* Detection result */
typedef struct {
  detected_line_t   lines[LINE_DETECT_MAX_LINES];
  uint8_t           num_lines;

  detected_circle_t circles[LINE_DETECT_MAX_CIRCLES];
  uint8_t           num_circles;

  detected_curve_t  curves[8];
  uint8_t           num_curves;

  /* Edge image (if draw_edges=true) */
  vision_image_t   *edge_image;

  /* For road/lane detection: left/right lane lines */
  int16_t           lane_offset_x;   /* horizontal offset from center (px) */
  int16_t           lane_center_x;   /* center x of detected lane (px) */
  float             lane_heading;    /* lane heading angle (rad, 0=straight) */
  bool              lane_valid;
} line_result_t;

/* Lifecycle */
line_config_t* line_default_config(void);
void line_config_set_roi(line_config_t *cfg, uint16_t x, uint16_t y, uint16_t w, uint16_t h);

/* Main detection function (input: grayscale image, output: line_result_t) */
void line_detect(const vision_image_t *img, const line_config_t *cfg, line_result_t *result);

/* Individual components (usable standalone) */

/* Sobel edge detection → magnitude image */
void line_sobel(const vision_image_t *img, vision_image_t *magnitude, vision_image_t *direction);

/* Canny-style edge detection with non-max suppression */
void line_canny(const vision_image_t *img, uint8_t low, uint8_t high, vision_image_t *edges);

/* Hough transform for lines (from edge image) */
void line_hough(const vision_image_t *edges, const line_config_t *cfg,
                detected_line_t *lines, uint8_t *num_lines);

/* Hough transform for circles */
void line_hough_circles(const vision_image_t *edges, const line_config_t *cfg,
                         detected_circle_t *circles, uint8_t *num_circles);

/* Non-max suppression on edge magnitude */
void line_nonmax_suppress(const vision_image_t *magnitude, vision_image_t *suppressed,
                           uint8_t radius);

/* Utility: fit line to set of 2D points */
bool line_fit_line(const float points[][2], uint16_t n, float *a, float *b, float *score);

/* Utility: fit parabola y = ax^2 + bx + c */
bool line_fit_parabola(const float points[][2], uint16_t n, float *a, float *b, float *c, float *score);

/* Utility: fit circle using least squares */
bool line_fit_circle(const float points[][2], uint16_t n, float *cx, float *cy, float *r, float *score);

/* Lane/road detection helper: extract lane from detected lines */
void line_find_lane(const detected_line_t *lines, uint8_t num_lines,
                     uint16_t image_center_x, uint16_t image_w, line_result_t *result);

/* Debug: print detected lines to stdout */
void line_print(const line_result_t *result);

#endif /* LINE_DETECT_H */
