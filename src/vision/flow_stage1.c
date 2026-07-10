#include "flow_stage1.h"
#include "image.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_TILES 512
#define HISTOGRAM_SIZE 64

struct flow_stage1_s {
  flow_stage1_config_t config;

  vision_image_t prev_frame;   /* previous grayscale frame */
  bool           has_prev;

  /* tile grid computed once */
  uint16_t tile_x[MAX_TILES];
  uint16_t tile_y[MAX_TILES];
  uint16_t num_tiles;

  /* histogram for outlier rejection */
  int16_t  hist_x[HISTOGRAM_SIZE];
  int16_t  hist_y[HISTOGRAM_SIZE];
  int32_t  hist_weight[HISTOGRAM_SIZE];
  uint16_t hist_count;

  /* Per-tile flow results from last process() call (for diagnostics/Stage 3) */
  int16_t  last_flow_x[MAX_TILES];
  int16_t  last_flow_y[MAX_TILES];
  uint32_t last_sad[MAX_TILES];
  uint16_t last_matched;

  /* stats */
  uint32_t total_frames;
  uint32_t total_time_us;
};

static void compute_tile_grid(flow_stage1_t *ctx) {
  const flow_stage1_config_t *cfg = &ctx->config;
  uint16_t img_w = cfg->image_width;
  uint16_t img_h = cfg->image_height;
  uint8_t ts = cfg->tile_size;
  uint8_t sr = cfg->search_range;

  uint16_t margin = sr + ts / 2;
  int range_w = (int)img_w - 2 * (int)margin - (int)ts;
  int range_h = (int)img_h - 2 * (int)margin - (int)ts;

  ctx->num_tiles = 0;
  if (range_w <= 0 || range_h <= 0) return;

  /* Dense step (overlapping tiles) */
  uint8_t step_x = ts / 2;
  uint8_t step_y = ts / 2;

  /* If dense grid exceeds MAX_TILES, distribute tiles evenly across full image */
  int dense_cols = range_w / step_x + 1;
  int dense_rows = range_h / step_y + 1;
  if (dense_cols * dense_rows > MAX_TILES) {
    float opt = sqrtf((float)range_w * range_h / MAX_TILES);
    step_x = (uint8_t)(opt + 0.5f);
    step_y = step_x;
    if (step_x < 1) step_x = 1;
    /* Increment step until grid fits within MAX_TILES */
    while (step_x < 255 &&
           (range_w / step_x + 1) * (range_h / step_y + 1) > MAX_TILES) {
      step_x++; step_y++;
    }
  }

  for (uint16_t y = margin; y + margin + ts <= img_h && ctx->num_tiles < MAX_TILES; y += step_y) {
    for (uint16_t x = margin; x + margin + ts <= img_w && ctx->num_tiles < MAX_TILES; x += step_x) {
      ctx->tile_x[ctx->num_tiles] = x;
      ctx->tile_y[ctx->num_tiles] = y;
      ctx->num_tiles++;
    }
  }
}

flow_stage1_t* flow_stage1_create(const flow_stage1_config_t *cfg) {
  flow_stage1_t *ctx = (flow_stage1_t*)calloc(1, sizeof(flow_stage1_t));
  if (!ctx) return NULL;

  memcpy(&ctx->config, cfg, sizeof(flow_stage1_config_t));
  ctx->has_prev = false;

  /* Create previous frame buffer */
  if (!image_create(&ctx->prev_frame, cfg->image_width, cfg->image_height,
                    VISION_IMAGE_GRAYSCALE)) {
    free(ctx);
    return NULL;
  }

  compute_tile_grid(ctx);

  ctx->total_frames = 0;
  ctx->total_time_us = 0;

  return ctx;
}

void flow_stage1_destroy(flow_stage1_t *ctx) {
  if (ctx) {
    image_destroy(&ctx->prev_frame);
    free(ctx);
  }
}

static int16_t histogram_bin_index(int32_t value) {
  /* map integer flow to bin index, allowing small clusters */
  return (int16_t)(value + HISTOGRAM_SIZE / 2);
}

static void histogram_add(flow_stage1_t *ctx, int16_t dx, int16_t dy, int32_t weight) {
  int16_t bx = histogram_bin_index(dx);
  int16_t by = histogram_bin_index(dy);

  if (bx < 0 || bx >= HISTOGRAM_SIZE || by < 0 || by >= HISTOGRAM_SIZE)
    return;

  /* check if this vector is already in histogram */
  for (uint16_t i = 0; i < ctx->hist_count; i++) {
    if (ctx->hist_x[i] == dx && ctx->hist_y[i] == dy) {
      ctx->hist_weight[i] += weight;
      return;
    }
  }

  /* new entry */
  if (ctx->hist_count < HISTOGRAM_SIZE) {
    ctx->hist_x[ctx->hist_count] = dx;
    ctx->hist_y[ctx->hist_count] = dy;
    ctx->hist_weight[ctx->hist_count] = weight;
    ctx->hist_count++;
  }
}

static void find_dominant_flow(flow_stage1_t *ctx, int32_t *out_dx, int32_t *out_dy,
                                uint8_t *out_quality) {
  if (ctx->hist_count == 0) {
    *out_dx = 0;
    *out_dy = 0;
    *out_quality = 0;
    return;
  }

  /* Find highest weight */
  int32_t max_weight = 0;
  uint16_t max_idx = 0;
  for (uint16_t i = 0; i < ctx->hist_count; i++) {
    if (ctx->hist_weight[i] > max_weight) {
      max_weight = ctx->hist_weight[i];
      max_idx = i;
    }
  }

  *out_dx = ctx->hist_x[max_idx];
  *out_dy = ctx->hist_y[max_idx];

  /* Find second-highest peak that is spatially separate (≥3px away) */
  int32_t second_weight = 0;
  for (uint16_t i = 0; i < ctx->hist_count; i++) {
    if (i == max_idx) continue;
    int32_t dist_x = ctx->hist_x[i] - ctx->hist_x[max_idx];
    int32_t dist_y = ctx->hist_y[i] - ctx->hist_y[max_idx];
    int32_t dist2 = dist_x * dist_x + dist_y * dist_y;
    if (dist2 >= 9 && ctx->hist_weight[i] > second_weight) {
      second_weight = ctx->hist_weight[i];
    }
  }

  /* Quality: ratio of dominant peak to sum of all weights, scaled to 0-255 */
  int32_t total_weight = 0;
  for (uint16_t i = 0; i < ctx->hist_count; i++) {
    total_weight += ctx->hist_weight[i];
  }

  if (total_weight > 0) {
    /* Peak ratio: how much of the flow field agrees */
    int32_t peak_ratio = max_weight * 255 / total_weight;

    /* Suppress quality if second peak is too close in strength */
    if (second_weight > 0 && second_weight > max_weight / 2) {
      peak_ratio = peak_ratio / 2;
    }

    *out_quality = (uint8_t)(peak_ratio > 255 ? 255 : peak_ratio);
  } else {
    *out_quality = 0;
  }
}

void flow_stage1_process(flow_stage1_t *ctx, const vision_image_t *frame,
                          vision_result_t *result) {
  if (!ctx->has_prev) {
    /* First frame: store and return zero flow */
    image_copy(frame, &ctx->prev_frame);
    ctx->has_prev = true;
    result->flow_x_fast = 0;
    result->flow_y_fast = 0;
    result->quality_fast = 0;
    result->sad_score = 0;
    return;
  }

  uint32_t t0 = 0;
  /* Record start time using clock() or similar */
  /* We'll use a portable approach; platforms can override */

  /* Reset histogram */
  ctx->hist_count = 0;

  uint32_t total_sad = 0;
  ctx->last_matched = 0;

  const flow_stage1_config_t *cfg = &ctx->config;

  for (uint16_t i = 0; i < ctx->num_tiles; i++) {
    uint16_t tx = ctx->tile_x[i];
    uint16_t ty = ctx->tile_y[i];

    uint32_t best_sad = UINT32_MAX;
    int16_t best_dx = 0, best_dy = 0;

    uint8_t sr = cfg->search_range;
    uint8_t ts = cfg->tile_size;

    /* Full search in search window */
    for (int16_t dy = -(int16_t)sr; dy <= (int16_t)sr; dy++) {
      for (int16_t dx = -(int16_t)sr; dx <= (int16_t)sr; dx++) {
        int32_t px = (int32_t)tx + dx;
        int32_t py = (int32_t)ty + dy;

        if (px < 0 || py < 0 || px + ts > frame->w || py + ts > frame->h)
          continue;

        uint32_t sad = image_sad_block(frame, tx, ty, &ctx->prev_frame,
                                        (uint16_t)px, (uint16_t)py, ts);
        if (sad < best_sad) {
          best_sad = sad;
          best_dx = dx;
          best_dy = dy;
        }
      }
    }

    if (best_sad < UINT32_MAX) {
      /* Subpixel refinement via parabola fit */
      uint32_t sad_c = image_sad_block(frame, tx, ty, &ctx->prev_frame,
        (uint16_t)((int32_t)tx + best_dx), (uint16_t)((int32_t)ty + best_dy), ts);
      uint32_t sad_xn = image_sad_block(frame, tx, ty, &ctx->prev_frame,
        (uint16_t)((int32_t)tx + best_dx - 1), (uint16_t)((int32_t)ty + best_dy), ts);
      uint32_t sad_xp = image_sad_block(frame, tx, ty, &ctx->prev_frame,
        (uint16_t)((int32_t)tx + best_dx + 1), (uint16_t)((int32_t)ty + best_dy), ts);
      uint32_t sad_yn = image_sad_block(frame, tx, ty, &ctx->prev_frame,
        (uint16_t)((int32_t)tx + best_dx), (uint16_t)((int32_t)ty + best_dy - 1), ts);
      uint32_t sad_yp = image_sad_block(frame, tx, ty, &ctx->prev_frame,
        (uint16_t)((int32_t)tx + best_dx), (uint16_t)((int32_t)ty + best_dy + 1), ts);

      int32_t sub_x = 0, sub_y = 0;
      int32_t denom_x = (int32_t)(sad_xn + sad_xp - 2 * sad_c);
      if (denom_x != 0) {
        sub_x = ((int32_t)(sad_xn - sad_xp)) * 10 / (2 * denom_x);
      }
      int32_t denom_y = (int32_t)(sad_yn + sad_yp - 2 * sad_c);
      if (denom_y != 0) {
        sub_y = ((int32_t)(sad_yn - sad_yp)) * 10 / (2 * denom_y);
      }

      int16_t flow_x = (int16_t)(best_dx * 10 + sub_x);
      int16_t flow_y = (int16_t)(best_dy * 10 + sub_y);

      /* Store per-tile data for diagnostics / Stage 3 */
      ctx->last_flow_x[ctx->last_matched] = flow_x;
      ctx->last_flow_y[ctx->last_matched] = flow_y;
      ctx->last_sad[ctx->last_matched] = best_sad;

      /* Weight: inverse of SAD (higher SAD = lower confidence) */
      int32_t weight = best_sad > 0 ? (256 * 256 * ts * ts) / (int32_t)(best_sad + 1) : 255;
      weight = (weight > 255) ? 255 : weight;

      histogram_add(ctx, flow_x, flow_y, weight);
      total_sad += best_sad;
      ctx->last_matched++;
    }
  }

  /* Find dominant flow from histogram */
  int32_t dom_dx = 0, dom_dy = 0;
  uint8_t quality = 0;
  find_dominant_flow(ctx, &dom_dx, &dom_dy, &quality);

  result->flow_x_fast = dom_dx;
  result->flow_y_fast = dom_dy;
  result->quality_fast = quality;
  result->sad_score = ctx->last_matched > 0 ? total_sad / ctx->last_matched : UINT32_MAX;

  /* Store current frame as previous */
  image_copy(frame, &ctx->prev_frame);

  ctx->total_frames++;
}

void flow_stage1_reconfigure(flow_stage1_t *ctx, const flow_stage1_config_t *cfg) {
  memcpy(&ctx->config, cfg, sizeof(flow_stage1_config_t));
  image_destroy(&ctx->prev_frame);
  image_create(&ctx->prev_frame, cfg->image_width, cfg->image_height, VISION_IMAGE_GRAYSCALE);
  compute_tile_grid(ctx);
}

uint16_t flow_stage1_get_tile_flows(const flow_stage1_t *ctx,
    const uint16_t **positions_x, const uint16_t **positions_y,
    const int16_t **tile_flow_x, const int16_t **tile_flow_y,
    const uint32_t **tile_sad) {
  if (positions_x) *positions_x = ctx->tile_x;
  if (positions_y) *positions_y = ctx->tile_y;
  if (tile_flow_x) *tile_flow_x = ctx->last_flow_x;
  if (tile_flow_y) *tile_flow_y = ctx->last_flow_y;
  if (tile_sad)    *tile_sad    = ctx->last_sad;
  return ctx->last_matched;
}

void flow_stage1_stats(const flow_stage1_t *ctx, uint32_t *avg_process_us,
                       uint32_t *tiles_matched, uint8_t *quality) {
  (void)avg_process_us;
  (void)quality;
  if (tiles_matched) *tiles_matched = ctx->num_tiles;
}
