#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "types.h"
#include "fast_detect.h"
#include "lk_flow.h"

/* Visual Odometry for AR.Drone 2.0
 *
 * Monocular visual odometry using FAST corners + Lucas-Kanade tracking
 * + essential matrix decomposition for rotation/translation recovery.
 *
 * Pipeline per frame:
 *   1. Detect FAST-9 corners (or reuse from previous)
 *   2. Track corners via LK pyramid (coarse-to-fine)
 *   3. Reject outliers (RANSAC or fundamental matrix check)
 *   4. Compute essential matrix (8-point algorithm)
 *   5. Decompose E → R, t via SVD
 *   6. Integrate translation with scale from barometer/GPS
 *   7. Output position + velocity in body frame
 *
 * Scale recovery:
 *   - Monocular VO gives translation up to scale
 *   - We recover absolute scale using:
 *     a) Barometer altitude (vertical scale)
 *     b) Known ground height when near ground
 *     c) GPS when available (external scale factor)
 */

/* Maximum tracked features */
#define VO_MAX_FEATURES 256

/* Feature tracking state */
typedef struct {
    uint16_t id;        /* unique feature ID */
    float    x_prev;    /* previous frame position (subpixel) */
    float    y_prev;
    float    x_curr;    /* current frame position (subpixel) */
    float    y_curr;
    uint8_t  age;       /* frames since detection */
    uint8_t  status;    /* 0=lost, 1=tracked, 2=matched */
} vo_feature_t;

/* Feature map (all tracked features) */
typedef struct {
    vo_feature_t features[VO_MAX_FEATURES];
    uint16_t     count;
    uint32_t     next_id;
} vo_feature_map_t;

/* Essential matrix decomposition result */
typedef struct {
    float R[3][3];      /* rotation matrix (3x3) */
    float t[3];         /* translation vector (unit, up to scale) */
    float inlier_ratio; /* fraction of inliers (0-1) */
    int   num_inliers;
    int   num_matches;
    bool  valid;        /* decomposition successful */
} vo_motion_t;

/* Position estimate (body frame, accumulated) */
typedef struct {
    float x;            /* forward (m) */
    float y;            /* right (m) */
    float z;            /* down (m) - NED convention */
    float vx;           /* velocity forward (m/s) */
    float vy;           /* velocity right (m/s) */
    float vz;           /* velocity down (m/s) */
    float roll;         /* roll (radians) */
    float pitch;        /* pitch (radians) */
    float yaw;          /* yaw (radians) */
    uint32_t timestamp_ms;
    bool  valid;
} vo_position_t;

/* Visual odometry configuration */
typedef struct {
    /* Feature detection */
    uint8_t  fast_threshold;     /* FAST corner threshold (20-40) */
    uint8_t  min_feature_dist;   /* min distance between features (px) */
    uint16_t max_features;       /* max features to track */

    /* Tracking */
    uint8_t  lk_pyramid_levels;  /* LK pyramid levels (3-4) */
    uint8_t  lk_window_size;     /* LK window half-size (4-7) */
    uint8_t  lk_max_iterations;  /* LK iterations (10-20) */

    /* Outlier rejection */
    float    ransac_threshold;   /* epipolar distance threshold (px) */
    int      min_matches;        /* minimum matches for valid motion */

    /* Scale recovery */
    float    known_height_m;     /* known altitude for scale (0=use barometer) */
    bool     use_barometer;      /* use barometer for vertical scale */
    bool     use_gps;            /* use GPS for absolute scale */

    /* Camera intrinsics (normalized) */
    float    fx;                 /* focal length x (normalized) */
    float    fy;                 /* focal length y (normalized) */
    float    cx;                 /* principal point x (normalized) */
    float    cy;                 /* principal point y (normalized) */
} vo_config_t;

/* Visual odometry context */
typedef struct vo_context_s vo_context_t;

/* Create visual odometry context */
vo_context_t* vo_create(const vo_config_t *cfg,
                        uint16_t image_width,
                        uint16_t image_height);

/* Destroy context */
void vo_destroy(vo_context_t *ctx);

/* Process a new frame.
 *
 * Parameters:
 *   ctx      - VO context
 *   frame    - current grayscale frame
 *   altitude_m - current altitude from barometer (for scale, 0 if unavailable)
 *   gps_scale  - GPS-based scale factor (1.0 if unavailable)
 *   dt_ms      - time since last frame (ms)
 *   motion     - output: estimated R, t (optional, NULL to skip)
 *   position   - output: accumulated position (optional, NULL to skip)
 *
 * Returns 0 on success, negative on error.
 */
int vo_process(vo_context_t *ctx,
               const vision_image_t *frame,
               float altitude_m,
               float gps_scale,
               uint32_t dt_ms,
               vo_motion_t *motion,
               vo_position_t *position);

/* Reset VO state (e.g., after landing) */
void vo_reset(vo_context_t *ctx);

/* Get current position estimate */
void vo_get_position(const vo_context_t *ctx, vo_position_t *pos);

/* Get current feature count */
uint16_t vo_feature_count(const vo_context_t *ctx);

/* Get default configuration */
void vo_default_config(vo_config_t *cfg);

/* Update configuration at runtime */
void vo_reconfigure(vo_context_t *ctx, const vo_config_t *cfg);

/* ---- Math utilities (exposed for testing) ---- */

/* Compute essential matrix from matched points (8-point algorithm) */
bool vo_compute_essential(float pts1[][2], float pts2[][2],
                          int n, float E[3][3]);

/* Decompose essential matrix into R, t */
bool vo_decompose_essential(float E[3][3], float R[3][3], float t[3]);

/* Check point correspondence against epipolar constraint */
float vo_epipolar_error(float E[3][3],
                        const float p1[2], const float p2[2]);

#endif /* VISUAL_ODOMETRY_H */
