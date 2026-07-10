#include "flow_stage2.h"
#include "image.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/*
 * Stage 2: Robust multi-scale SAD block matching.
 *
 * Uses a coarse-to-fine pyramid approach:
 *   Level 2 (1/4 scale):  SAD search ±3 → effective ±12 at full res
 *   Level 1 (1/2 scale):  SAD search ±3 → effective ±6 at full res
 *   Level 0 (full scale): SAD search ±2 → subpixel refinement
 *
 * Total effective search range: ±12 pixels (vs ±6 in Stage 1)
 * Total tiles: ~64 (1/8 of Stage 1) for speed
 */

#define STAGE2_MAX_TILES 128
#define STAGE2_PYR_LEVELS 3

typedef struct {
  uint16_t x;
  uint16_t y;
  int32_t  flow_x;    /* subpixel flow * 10 */
  int32_t  flow_y;
  uint32_t sad;
  bool     valid;
} stage2_tile_t;

struct flow_stage2_s {
  flow_stage2_config_t config;
  uint16_t image_width;
  uint16_t image_height;

  /* Previous frame (grayscale, full resolution) */
  vision_image_t prev_frame;
  bool has_previous;

  /* Pyramid buffers */
  vision_image_t prev_pyr[STAGE2_PYR_LEVELS];
  vision_image_t curr_pyr[STAGE2_PYR_LEVELS];

  /* Tile grid (computed at full resolution, scaled per level) */
  uint16_t tile_x[STAGE2_MAX_TILES];
  uint16_t tile_y[STAGE2_MAX_TILES];
  uint16_t num_tiles;

  /* Per-tile results */
  stage2_tile_t tiles[STAGE2_MAX_TILES];
};

static void compute_tile_grid(flow_stage2_t *ctx) {
  uint16_t w = ctx->image_width;
  uint16_t h = ctx->image_height;

  ctx->num_tiles = 0;

  /* Grid with spacing = 24px → ~(320/24) * (240/24) ≈ 13*10 = 130 tiles */
  uint8_t spacing = 24;
  uint8_t margin = 16;

  for (uint16_t y = margin; y + 8 + margin <= h; y += spacing) {
    for (uint16_t x = margin; x + 8 + margin <= w; x += spacing) {
      if (ctx->num_tiles >= STAGE2_MAX_TILES) break;
      ctx->tile_x[ctx->num_tiles] = x;
      ctx->tile_y[ctx->num_tiles] = y;
      ctx->num_tiles++;
    }
  }
}

static void build_grayscale_pyramid(const vision_image_t *src,
                                     vision_image_t *pyr, int levels) {
  /* Level 0: copy from source */
  image_copy(src, &pyr[0]);

  /* Build higher levels: Gaussian blur + 2x downsample */
  for (int l = 1; l < levels; l++) {
    uint16_t pw = pyr[l - 1].w;
    uint16_t ph = pyr[l - 1].h;
    uint16_t nw = pw / 2;
    uint16_t nh = ph / 2;

    const int kernel[5] = {1, 4, 6, 4, 1};
    const int ksum = 16;

    for (uint16_t ny = 0; ny < nh; ny++) {
      for (uint16_t nx = 0; nx < nw; nx++) {
        int sum = 0;
        for (int ky = -2; ky <= 2; ky++) {
          for (int kx = -2; kx <= 2; kx++) {
            int sy = (int)(ny * 2) + ky;
            int sx = (int)(nx * 2) + kx;
            if (sy < 0) sy = 0;
            if (sy >= (int)ph) sy = (int)ph - 1;
            if (sx < 0) sx = 0;
            if (sx >= (int)pw) sx = (int)pw - 1;
            sum += pyr[l - 1].buf[sy * pyr[l - 1].stride + sx]
                 * kernel[ky + 2] * kernel[kx + 2];
          }
        }
        pyr[l].buf[ny * pyr[l].stride + nx] = (uint8_t)(sum / (ksum * ksum));
      }
    }
  }
}

/* SAD at a specific pyramid level */
static uint32_t sad_at_level(const vision_image_t *curr_pyr,
                              const vision_image_t *prev_pyr, int level,
                              uint16_t tx, uint16_t ty,
                              int16_t dx, int16_t dy, uint8_t ts) {
  uint8_t shift = (uint8_t)(1 << level);
  uint16_t cx = (tx >> level);  /* round down */
  uint16_t cy = (ty >> level);

  /* Account for rounding: adjust to nearest pixel in level */
  cx = (tx + shift / 2) >> level;
  cy = (ty + shift / 2) >> level;

  uint16_t px = (uint16_t)((int16_t)cx + dx);
  uint16_t py = (uint16_t)((int16_t)cy + dy);

  uint32_t sad = 0;
  for (uint8_t fy = 0; fy < ts; fy++) {
    for (uint8_t fx = 0; fx < ts; fx++) {
      uint8_t cv = curr_pyr->buf[(cy + fy) * curr_pyr->stride + (cx + fx)];
      uint8_t pv = prev_pyr->buf[(py + fy) * prev_pyr->stride + (px + fx)];
      int16_t diff = (int16_t)cv - (int16_t)pv;
      sad += (uint32_t)(diff < 0 ? -diff : diff);
    }
  }
  return sad;
}

static int16_t search_level(const vision_image_t *curr_pyr,
                             const vision_image_t *prev_pyr, int level,
                             uint16_t tx, uint16_t ty,
                             uint8_t tile_size, uint8_t search_range,
                             int16_t init_dx, int16_t init_dy,
                             int32_t *out_flow_x, int32_t *out_flow_y) {
  uint32_t best_sad = UINT32_MAX;
  int16_t best_dx = 0, best_dy = 0;

  /* Search around initial guess */
  for (int16_t dy = -(int16_t)search_range + init_dy;
       dy <= (int16_t)search_range + init_dy; dy++) {
    for (int16_t dx = -(int16_t)search_range + init_dx;
         dx <= (int16_t)search_range + init_dx; dx++) {
      uint32_t sad = sad_at_level(curr_pyr, prev_pyr, level,
                                   tx, ty, dx, dy, tile_size);
      if (sad < best_sad) {
        best_sad = sad;
        best_dx = dx;
        best_dy = dy;
      }
    }
  }

  /* Subpixel refinement at full resolution */
  if (level == 0 && best_sad < UINT32_MAX) {
    uint16_t cx = tx, cy = ty;
    int16_t px = (int16_t)cx + best_dx;
    int16_t py = (int16_t)cy + best_dy;

    uint32_t sad_c = image_sad_block(curr_pyr, cx, cy, prev_pyr,
                                      (uint16_t)px, (uint16_t)py, tile_size);
    uint32_t sad_xn = image_sad_block(curr_pyr, cx, cy, prev_pyr,
                                       (uint16_t)(px - 1), (uint16_t)py, tile_size);
    uint32_t sad_xp = image_sad_block(curr_pyr, cx, cy, prev_pyr,
                                       (uint16_t)(px + 1), (uint16_t)py, tile_size);
    uint32_t sad_yn = image_sad_block(curr_pyr, cx, cy, prev_pyr,
                                       (uint16_t)px, (uint16_t)(py - 1), tile_size);
    uint32_t sad_yp = image_sad_block(curr_pyr, cx, cy, prev_pyr,
                                       (uint16_t)px, (uint16_t)(py + 1), tile_size);

    int32_t sub_x = 0, sub_y = 0;
    int32_t denom_x = (int32_t)(sad_xn + sad_xp - 2 * sad_c);
    if (denom_x != 0)
      sub_x = ((int32_t)(sad_xn - sad_xp)) * 10 / (2 * denom_x);
    int32_t denom_y = (int32_t)(sad_yn + sad_yp - 2 * sad_c);
    if (denom_y != 0)
      sub_y = ((int32_t)(sad_yn - sad_yp)) * 10 / (2 * denom_y);

    *out_flow_x = (int32_t)best_dx * 10 + sub_x;
    *out_flow_y = (int32_t)best_dy * 10 + sub_y;
  } else {
    int scale = 1 << level;
    *out_flow_x = (int32_t)best_dx * 10 * scale;
    *out_flow_y = (int32_t)best_dy * 10 * scale;
  }

  return (int16_t)best_sad;
}

flow_stage2_t* flow_stage2_create(const flow_stage2_config_t *cfg,
                                   uint16_t image_width,
                                   uint16_t image_height) {
  flow_stage2_t *ctx = (flow_stage2_t*)calloc(1, sizeof(flow_stage2_t));
  if (!ctx) return NULL;

  memcpy(&ctx->config, cfg, sizeof(flow_stage2_config_t));
  ctx->image_width = image_width;
  ctx->image_height = image_height;
  ctx->has_previous = false;

  image_create(&ctx->prev_frame, image_width, image_height, VISION_IMAGE_GRAYSCALE);

  /* Pre-allocate pyramid buffers */
  for (int l = 0; l < STAGE2_PYR_LEVELS; l++) {
    uint16_t pw = image_width >> l;
    uint16_t ph = image_height >> l;
    if (pw < 8 || ph < 8) break;
    image_create(&ctx->prev_pyr[l], pw, ph, VISION_IMAGE_GRAYSCALE);
    image_create(&ctx->curr_pyr[l], pw, ph, VISION_IMAGE_GRAYSCALE);
  }

  compute_tile_grid(ctx);

  return ctx;
}

void flow_stage2_destroy(flow_stage2_t *ctx) {
  if (ctx) {
    image_destroy(&ctx->prev_frame);
    for (int l = 0; l < STAGE2_PYR_LEVELS; l++) {
      image_destroy(&ctx->prev_pyr[l]);
      image_destroy(&ctx->curr_pyr[l]);
    }
    free(ctx);
  }
}

static int32_t compute_divergence(const stage2_tile_t *tiles, uint16_t count,
                                   uint16_t img_w, uint16_t img_h,
                                   int16_t *focus_x, int16_t *focus_y) {
  if (count < 4) return 0;

  float cx = img_w / 2.0f;
  float cy = img_h / 2.0f;
  float div_sum = 0.0f;
  int valid = 0;
  float fcx = 0, fcy = 0;
  int fcount = 0;

  for (uint16_t i = 0; i < count; i++) {
    if (!tiles[i].valid) continue;

    float dx = tiles[i].flow_x / 10.0f;
    float dy = tiles[i].flow_y / 10.0f;
    float px = tiles[i].x - cx;
    float py = tiles[i].y - cy;
    float r2 = px * px + py * py;

    if (r2 > 4.0f) {
      float radial = (dx * px + dy * py) / sqrtf(r2);
      div_sum += radial / sqrtf(r2);
      valid++;
    }

    if (fabsf(dx) > 0.1f || fabsf(dy) > 0.1f) {
      fcx += (float)tiles[i].x - dy * tiles[i].x / (dx + 0.001f);
      fcy += (float)tiles[i].y - dx * tiles[i].y / (dy + 0.001f);
      fcount++;
    }
  }

  if (valid > 0) {
    if (focus_x) *focus_x = (int16_t)(fcx / (fcount > 0 ? fcount : 1));
    if (focus_y) *focus_y = (int16_t)(fcy / (fcount > 0 ? fcount : 1));
    return (int32_t)(div_sum * 1000 / valid);
  }
  return 0;
}

void flow_stage2_process(flow_stage2_t *ctx,
                          const vision_image_t *curr,
                          vision_result_t *result) {
  if (!ctx->has_previous) {
    image_copy(curr, &ctx->prev_frame);
    result->flow_x_robust = 0;
    result->flow_y_robust = 0;
    result->divergence = 0;
    result->focus_x = 0;
    result->focus_y = 0;
    result->corner_cnt = 0;
    result->quality_robust = 0;
    ctx->has_previous = true;
    return;
  }

  /* Build pyramids for prev and curr */
  build_grayscale_pyramid(&ctx->prev_frame, ctx->prev_pyr, STAGE2_PYR_LEVELS);
  build_grayscale_pyramid(curr, ctx->curr_pyr, STAGE2_PYR_LEVELS);

  uint8_t tile_size = 8;

  /* Track each tile through the pyramid, coarse-to-fine */
  uint32_t total_sad = 0;
  uint16_t matched = 0;

  for (uint16_t i = 0; i < ctx->num_tiles; i++) {
    uint16_t tx = ctx->tile_x[i];
    uint16_t ty = ctx->tile_y[i];

    /* Level 2 (coarse): search ±3, effective ±12 at full res */
    int16_t dx_2 = 0, dy_2 = 0;
    int32_t f2x, f2y;
    search_level(&ctx->curr_pyr[2], &ctx->prev_pyr[2], 2,
                 tx, ty, tile_size, 3, 0, 0, &f2x, &f2y);
    dx_2 = (int16_t)(f2x / 10 / 4);  /* convert to pixel at level 2 */

    /* Level 1 (medium): search ±3 around Level 2 guess, effective ±6 */
    int32_t f1x, f1y;
    search_level(&ctx->curr_pyr[1], &ctx->prev_pyr[1], 1,
                 tx, ty, tile_size, 3, dx_2 * 2, dy_2 * 2, &f1x, &f1y);
    int16_t dx_1 = (int16_t)(f1x / 10 / 2);
    int16_t dy_1 = (int16_t)(f1y / 10 / 2);

    /* Level 0 (fine): search ±2 around Level 1 guess, with subpixel */
    int32_t f0x, f0y;
    uint32_t sad = search_level(&ctx->curr_pyr[0], &ctx->prev_pyr[0], 0,
                                 tx, ty, tile_size, 2,
                                 dx_1, dy_1, &f0x, &f0y);

    /* Store result */
    ctx->tiles[i].x = tx;
    ctx->tiles[i].y = ty;
    ctx->tiles[i].flow_x = f0x;
    ctx->tiles[i].flow_y = f0y;
    ctx->tiles[i].sad = sad;

    /* Quality: reject if SAD too high or flow too large */
    uint32_t max_sad = (uint32_t)tile_size * tile_size * 64;
    ctx->tiles[i].valid = (sad < max_sad &&
                           abs(f0x) < 500 && abs(f0y) < 500);

    if (ctx->tiles[i].valid) {
      total_sad += sad;
      matched++;
    }
  }

  /* Compute dominant flow via trimmed mean (exclude outliers) */
  if (matched > 0) {
    int64_t sum_x = 0, sum_y = 0;
    int valid = 0;

    for (uint16_t i = 0; i < ctx->num_tiles; i++) {
      if (ctx->tiles[i].valid) {
        sum_x += ctx->tiles[i].flow_x;
        sum_y += ctx->tiles[i].flow_y;
        valid++;
      }
    }

    if (valid > 0) {
      result->flow_x_robust = (int32_t)(sum_x / valid);
      result->flow_y_robust = (int32_t)(sum_y / valid);
    }

    result->divergence = compute_divergence(ctx->tiles, ctx->num_tiles,
                                             ctx->image_width, ctx->image_height,
                                             &result->focus_x, &result->focus_y);
    result->corner_cnt = (uint8_t)(valid > 255 ? 255 : valid);
    result->quality_robust = (uint8_t)(total_sad / matched > 100 ? 0 : 255 - total_sad / matched);
  } else {
    result->flow_x_robust = 0;
    result->flow_y_robust = 0;
    result->divergence = 0;
    result->corner_cnt = 0;
    result->quality_robust = 0;
  }

  image_copy(curr, &ctx->prev_frame);
}

void flow_stage2_reconfigure(flow_stage2_t *ctx,
                              const flow_stage2_config_t *cfg) {
  (void)cfg;
  (void)ctx;
}
