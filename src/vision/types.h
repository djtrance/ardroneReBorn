#ifndef VISION_TYPES_H
#define VISION_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* Image types */
typedef enum {
  VISION_IMAGE_GRAYSCALE,
  VISION_IMAGE_UYVY,
  VISION_IMAGE_YUV420
} vision_image_type_t;

/* Image structure (portable, no architecture dependencies) */
typedef struct {
  vision_image_type_t type;
  uint16_t            w;
  uint16_t            h;
  uint16_t            stride;       /* bytes per row (may differ from w*bytes_per_pixel) */
  uint8_t             bytes_per_pixel;
  uint8_t            *buf;         /* owned buffer */
  uint32_t            buf_size;
  uint32_t            timestamp_us; /* monotonic timestamp */
} vision_image_t;

/* 2D point */
typedef struct {
  int32_t x;
  int32_t y;
} vision_point_t;

/* Optical flow vector (subpixel) */
typedef struct {
  vision_point_t pos;       /* integer pixel position */
  int32_t        flow_x;    /* flow in subpixel units */
  int32_t        flow_y;
  uint32_t       error;     /* matching error */
} vision_flow_t;

/* 2D float vector */
typedef struct {
  float x;
  float y;
} vision_vec2f_t;

/* 3D float vector */
typedef struct {
  float x;
  float y;
  float z;
} vision_vec3f_t;

/* Complete pipeline result */
typedef struct {
  /* Stage 1: fast flow (every frame) */
  int32_t  flow_x_fast;
  int32_t  flow_y_fast;
  uint8_t  quality_fast;
  uint32_t sad_score;

  /* Stage 2: robust flow (when available) */
  int32_t  flow_x_robust;
  int32_t  flow_y_robust;
  int32_t  divergence;
  int16_t  focus_x;
  int16_t  focus_y;
  uint8_t  corner_cnt;
  uint8_t  quality_robust;

  /* Stage 3: rotation */
  int32_t  yaw_rate;         /* millidegrees/s */
  int32_t  cumulative_yaw;   /* millidegrees */

  /* Stage 4: pattern detection */
  int16_t  landing_x;
  int16_t  landing_y;
  uint16_t landing_size;
  int16_t  home_angle;
  int16_t  home_distance;

  /* Fused output (body frame, cm/s) */
  int32_t  velocity_x;
  int32_t  velocity_y;
  int32_t  velocity_z;

  /* Altitude from ground (cm) */
  int32_t  altitude_cm;

  /* Metadata */
  uint32_t frame_id;
  float    fps_capture;
  float    fps_stage1;
  float    fps_stage2;
} vision_result_t;

/* SAD block matching configuration */
typedef struct {
  uint8_t tile_size;        /* 8 or 16 */
  uint8_t search_range;     /* typically 4-12 pixels */
  uint16_t image_width;     /* processed width (may be downsampled) */
  uint16_t image_height;    /* processed height */
  uint8_t  subsample;       /* 1=full, 2=half, 4=quarter */
  uint8_t  min_quality;     /* minimum quality to accept (0-255) */
} flow_stage1_config_t;

/* FAST+ LK configuration */
typedef struct {
  uint16_t max_corners;
  uint8_t  fast_threshold;
  uint8_t  fast_min_distance;
  uint8_t  lk_window_size;
  uint8_t  lk_pyramid_levels;
  uint8_t  lk_max_iterations;
  uint8_t  lk_subpixel_factor;
} flow_stage2_config_t;

/* Pipeline configuration */
typedef struct {
  flow_stage1_config_t stage1_cfg;
  flow_stage2_config_t stage2_cfg;
  uint8_t              stage2_interval;   /* run stage2 every N frames */
  bool                 enable_rotation;
  bool                 enable_pattern;
  const char          *camera_device;
  uint16_t             capture_width;
  uint16_t             capture_height;
} vision_pipeline_config_t;

#endif /* VISION_TYPES_H */
