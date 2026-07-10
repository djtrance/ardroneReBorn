#include "obstacle.h"
#include "image.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* === Configuration === */

/* SAD grid for looming/asymmetry */
#define OBST_GRID_COLS     8
#define OBST_GRID_ROWS     6
#define OBST_TILE_SIZE     16
#define OBST_SEARCH_RANGE  4

/* Vertical line detection */
#define OBST_LINE_ROWS     4       /* sample rows in lower half */
#define OBST_LINE_SKIP     2       /* subsample for gradient */
#define OBST_LINE_RADIUS   60      /* search radius from last line (pixels) */

/* Looming thresholds */
#define OBST_MIN_RADIAL    15      /* min radial flow*10 to count as looming */
#define OBST_LOOM_TILES    4       /* min tiles with radial flow */

/* Context */
struct obstacle_s {
  uint16_t w;
  uint16_t h;

  /* SAD grid tile positions */
  uint16_t tile_x[OBST_GRID_COLS * OBST_GRID_ROWS];
  uint16_t tile_y[OBST_GRID_COLS * OBST_GRID_ROWS];
  uint16_t num_tiles;

  /* Previous frame for SAD matching (NULL = first call) */
  vision_image_t prev_frame;

  /* Vertical line tracking */
  uint16_t last_line_x;
  bool     has_last_line;
};

obstacle_t* obstacle_create(uint16_t image_width, uint16_t image_height) {
  obstacle_t *ctx = (obstacle_t*)calloc(1, sizeof(obstacle_t));
  if (!ctx) return NULL;

  ctx->w = image_width;
  ctx->h = image_height;

  /* Build SAD tile grid: evenly spaced across the image */
  uint16_t margin_x = OBST_TILE_SIZE / 2 + 4;
  uint16_t margin_y = OBST_TILE_SIZE / 2 + 4;
  uint16_t step_x = (image_width - 2 * margin_x) / OBST_GRID_COLS;
  uint16_t step_y = (image_height - 2 * margin_y) / OBST_GRID_ROWS;

  ctx->num_tiles = 0;
  for (uint8_t r = 0; r < OBST_GRID_ROWS && ctx->num_tiles < OBST_GRID_COLS * OBST_GRID_ROWS; r++) {
    for (uint8_t c = 0; c < OBST_GRID_COLS && ctx->num_tiles < OBST_GRID_COLS * OBST_GRID_ROWS; c++) {
      ctx->tile_x[ctx->num_tiles] = margin_x + c * step_x;
      ctx->tile_y[ctx->num_tiles] = margin_y + r * step_y;
      ctx->num_tiles++;
    }
  }

  /* Allocate previous frame buffer */
  image_create(&ctx->prev_frame, image_width, image_height, VISION_IMAGE_GRAYSCALE);

  ctx->has_last_line = false;
  ctx->last_line_x = image_width / 2;

  return ctx;
}

void obstacle_destroy(obstacle_t *ctx) {
  if (ctx) {
    image_destroy(&ctx->prev_frame);
    free(ctx);
  }
}

/* Compute looming score from per-tile flows.
 * Returns score 0..255 where higher = stronger looming. */
static uint8_t compute_looming(const int16_t *flow_x, const int16_t *flow_y,
                                const uint16_t *tile_x, const uint16_t *tile_y,
                                uint16_t num_tiles,
                                float cx, float cy,
                                int8_t *out_asymmetry) {
  float div_left = 0, div_right = 0;
  int left_cnt = 0, right_cnt = 0;
  float div_total = 0;
  int total_cnt = 0;

  for (uint16_t i = 0; i < num_tiles; i++) {
    float dx = flow_x[i] / 10.0f;
    float dy = flow_y[i] / 10.0f;
    float px = tile_x[i] + OBST_TILE_SIZE / 2.0f - cx;
    float py = tile_y[i] + OBST_TILE_SIZE / 2.0f - cy;
    float r2 = px * px + py * py;

    if (r2 < 4.0f) continue;  /* skip center tile */

    float r = sqrtf(r2);
    /* SAD returns offset of prev relative to current frame (prev = current + offset).
     * Outward expansion: current feature was CLOSER to center in prev frame,
     * so offset points INWARD (prev[inward] = current[outward]).
     * This makes (dx*px + dy*py) < 0 for expansion and radial < 0.
     * We negate to get positive expansion metric. */
    float radial = -(dx * px + dy * py) / r;  /* >0 = expansion, <0 = contraction */

    if (radial > OBST_MIN_RADIAL / 10.0f) {
      float normalized = radial / r;  /* expansion ratio */
      div_total += normalized;
      total_cnt++;

      if (px < -8) {
        div_left += normalized;
        left_cnt++;
      } else if (px > 8) {
        div_right += normalized;
        right_cnt++;
      }
    }
  }

  if (out_asymmetry) {
    float left_avg = left_cnt > 0 ? div_left / left_cnt : 0;
    float right_avg = right_cnt > 0 ? div_right / right_cnt : 0;
    float asym = (left_cnt > 2 || right_cnt > 2) ? (left_avg - right_avg) : 0.0f;
    if (asym > 0.03f)      *out_asymmetry = (int8_t)(asym * 1000 > 127 ? 127 : (int8_t)(asym * 1000));
    else if (asym < -0.03f)*out_asymmetry = (int8_t)(asym * 1000 < -128 ? -128 : (int8_t)(asym * 1000));
    else                    *out_asymmetry = 0;
  }

  if (total_cnt < OBST_LOOM_TILES) return 0;

  float avg_div = div_total / total_cnt;
  /* Scale: expansion of 0.01 → ~128, 0.02+ → 255 */
  float score = avg_div * 12800.0f;
  if (score > 255.0f) score = 255.0f;
  if (score < 0)      score = 0;
  return (uint8_t)score;
}

/* Detect the dominant vertical line in the lower half of the image.
 * Uses horizontal gradient accumulation + sliding window tracking. */
static bool detect_vertical_line(const vision_image_t *frame,
                                  uint16_t cx,
                                  uint16_t *out_line_x,
                                  int16_t *out_line_angle,
                                  uint8_t *out_strength) {
  uint16_t w = frame->w;
  uint16_t h = frame->h;
  uint16_t stride = frame->stride;
  const uint8_t *buf = frame->buf;

  /* Sample rows in lower half, compute horizontal gradient per column */
  uint16_t y_start = h / 2;
  uint16_t y_end = h - OBST_TILE_SIZE;

  if (y_end <= y_start) {
    *out_line_x = w / 2;
    *out_line_angle = 0;
    *out_strength = 0;
    return false;
  }

  uint16_t num_rows = 0;
  int32_t *col_sum = (int32_t*)calloc(w, sizeof(int32_t));
  if (!col_sum) return false;

  for (uint16_t y = y_start; y < y_end; y += OBST_LINE_SKIP) {
    for (uint16_t x = 2; x < w - 2; x++) {
      int32_t gx = (int32_t)buf[y * stride + x + 1] - (int32_t)buf[y * stride + x - 1];
      if (gx < 0) gx = -gx;
      col_sum[x] += gx;
    }
    num_rows++;
  }

  if (num_rows == 0) {
    free(col_sum);
    return false;
  }

  /* Find strongest edge column within search window */
  int search_start = (int)cx - OBST_LINE_RADIUS;
  int search_end   = (int)cx + OBST_LINE_RADIUS;
  if (search_start < 2) search_start = 2;
  if (search_end > (int)w - 2) search_end = w - 2;

  uint16_t best_col = cx;
  int32_t best_score = 0;
  for (int x = search_start; x < search_end; x++) {
    if (col_sum[x] > best_score) {
      best_score = col_sum[x];
      best_col = (uint16_t)x;
    }
  }

  free(col_sum);

  /* Compute line angle by tracking the edge at multiple y positions */
  #define VLINE_SAMPLE_ROWS 4
  uint16_t sample_ys[VLINE_SAMPLE_ROWS];
  int sample_xs[VLINE_SAMPLE_ROWS];
  int sample_cnt = 0;

  for (uint8_t i = 0; i < VLINE_SAMPLE_ROWS; i++) {
    uint16_t sy = y_start + (y_end - y_start) * (i + 1) / (VLINE_SAMPLE_ROWS + 1);
    if (sy >= h) continue;

    /* Find best edge in a narrow window around best_col at this row */
    int local_start = (int)best_col - 8;
    int local_end   = (int)best_col + 8;
    if (local_start < 2) local_start = 2;
    if (local_end > (int)w - 2) local_end = w - 2;

    int best_local_x = (int)best_col;
    int32_t best_local_score = 0;
    for (int x = local_start; x < local_end; x++) {
      int32_t gx = (int32_t)buf[sy * stride + x + 1] - (int32_t)buf[sy * stride + x - 1];
      if (gx < 0) gx = -gx;
      if (gx > best_local_score) {
        best_local_score = gx;
        best_local_x = x;
      }
    }

    sample_ys[sample_cnt] = sy;
    sample_xs[sample_cnt] = best_local_x;
    sample_cnt++;
  }

  /* Fit line x = a*y + b using least squares */
  if (sample_cnt >= 2) {
    float sum_y = 0, sum_x = 0, sum_yy = 0, sum_xy = 0;
    for (int i = 0; i < sample_cnt; i++) {
      float fy = (float)sample_ys[i];
      float fx = (float)sample_xs[i];
      sum_y += fy;   sum_x += fx;
      sum_yy += fy * fy;
      sum_xy += fy * fx;
    }
    float n = (float)sample_cnt;
    float denom = n * sum_yy - sum_y * sum_y;
    if (fabsf(denom) > 1.0f) {
      float a = (n * sum_xy - sum_y * sum_x) / denom;  /* dx/dy (pixels per row) */
      float b = (sum_x - a * sum_y) / n;

      /* Compute line x at bottom row */
      float x_bottom = a * (float)(h - 1) + b;
      if (x_bottom < 0) x_bottom = 0;
      if (x_bottom >= w) x_bottom = (float)(w - 1);
      *out_line_x = (uint16_t)(x_bottom + 0.5f);

      /* Angle from vertical: arctan(dx/dy) in centidegrees */
      float angle_rad = atan2f(a, 1.0f);
      *out_line_angle = (int16_t)(angle_rad * 18000.0f / 3.14159265f);
    } else {
      *out_line_x = best_col;
      *out_line_angle = 0;
    }
  } else {
    *out_line_x = best_col;
    *out_line_angle = 0;
  }

  /* Strength = normalized gradient score */
  uint32_t max_possible = num_rows * 255 * 5;  /* 5 pixel window at each row */
  uint32_t score_norm = (uint32_t)best_score * 255 / (max_possible > 0 ? max_possible : 1);
  *out_strength = score_norm > 255 ? 255 : (uint8_t)score_norm;

  return *out_strength > 20;
}

void obstacle_process(obstacle_t *ctx,
                      const vision_image_t *frame,
                      const vision_image_t *prev_frame,
                      obstacle_result_t *result) {
  memset(result, 0, sizeof(*result));

  if (!frame || !prev_frame) return;

  float cx = ctx->w / 2.0f;
  float cy = ctx->h / 2.0f;

  /* === Looming + Asymmetry from SAD flow grid === */
  int16_t flow_x[OBST_GRID_COLS * OBST_GRID_ROWS];
  int16_t flow_y[OBST_GRID_COLS * OBST_GRID_ROWS];
  uint32_t flow_sad[OBST_GRID_COLS * OBST_GRID_ROWS];

  image_sad_block_many(frame, prev_frame,
                        ctx->tile_x, ctx->tile_y,
                        flow_x, flow_y, flow_sad,
                        ctx->num_tiles, OBST_TILE_SIZE, OBST_SEARCH_RANGE);

  result->looming = compute_looming(flow_x, flow_y,
                                     ctx->tile_x, ctx->tile_y,
                                     ctx->num_tiles, cx, cy,
                                     &result->asymmetry);

  /* Confidence: based on number of tiles with valid (non-zero) flow */
  uint16_t valid_cnt = 0;
  for (uint16_t i = 0; i < ctx->num_tiles; i++) {
    if (flow_x[i] != 0 || flow_y[i] != 0) valid_cnt++;
  }
  result->confidence = (uint8_t)((uint32_t)valid_cnt * 255 / ctx->num_tiles);

  /* === Vertical line detection === */
  uint16_t track_cx = ctx->has_last_line ? ctx->last_line_x : ctx->w / 2;
  result->line_found = detect_vertical_line(frame,
                                             track_cx,
                                             &result->line_x,
                                             &result->line_angle,
                                             &result->line_strength);

  if (result->line_found) {
    ctx->last_line_x = result->line_x;
    ctx->has_last_line = true;
  }
}
