#include "lk_flow.h"
#include "image.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Maximum pyramid levels */
#define LK_MAX_LEVELS 5

/* LK window (half-size) — a small window around each corner */
#define LK_WIN_HALF 4

/* Structure for one level of the pyramid */
typedef struct {
  uint16_t w;
  uint16_t h;
  uint16_t stride;
  uint8_t *data;      /* pixel data */
  bool     owned;     /* if true, data is dynamically allocated */
} pyramid_level_t;

struct lk_context_s {
  flow_stage2_config_t config;
  uint16_t image_width;
  uint16_t image_height;

  /* Corner detection */
  fast_corners_t corners;

  /* Buffers */
  pyramid_level_t curr_pyr[LK_MAX_LEVELS];
  pyramid_level_t prev_pyr[LK_MAX_LEVELS];
  uint8_t num_levels;

  /* Derivative buffer (pre-allocated for current level) */
  int16_t *Ix;
  int16_t *Iy;
  int16_t *It;
  uint32_t deriv_size;

  /* Tracked corners from previous frame */
  lk_track_t   prev_tracks[FAST_MAX_CORNERS];
  uint16_t      prev_track_count;
  bool          has_previous;

  /* Scratch buffer for pyramid building */
  uint8_t *scratch_buf;
  uint32_t scratch_size;
};

static void build_pyramid(const vision_image_t *src,
                           pyramid_level_t *pyr,
                           uint8_t num_levels,
                           uint8_t *scratch,
                           uint32_t scratch_size) {
  if (num_levels < 1) return;

  /* Level 0: copy from source */
  pyr[0].w = src->w;
  pyr[0].h = src->h;
  pyr[0].stride = src->stride;
  pyr[0].data = src->buf;
  pyr[0].owned = false;

  /* Build higher levels */
  for (uint8_t level = 1; level < num_levels; level++) {
    uint16_t pw = pyr[level - 1].w;
    uint16_t ph = pyr[level - 1].h;
    uint16_t nw = pw / 2;
    uint16_t nh = ph / 2;

    pyr[level].w = nw;
    pyr[level].h = nh;
    pyr[level].stride = nw;
    pyr[level].data = scratch + (level - 1) * (size_t)nw * nh;
    pyr[level].owned = true;

    if (!pyr[level].data) {
      pyr[level].w = 0;
      return;
    }

    /* 5-tap Gaussian blur + 2x downsampling */
    const int kernel[5] = {1, 4, 6, 4, 1};
    const int kernel_sum = 16;

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
            sum += pyr[level - 1].data[sy * pyr[level - 1].stride + sx]
                 * kernel[ky + 2] * kernel[kx + 2];
          }
        }
        pyr[level].data[ny * pyr[level].stride + nx] = (uint8_t)(sum / (kernel_sum * kernel_sum));
      }
    }
  }
}

/* Bilinear interpolation of a pixel at float coordinates */
static inline float sample_bilinear(const uint8_t *data, uint16_t stride,
                                     float x, float y,
                                     uint16_t w, uint16_t h) {
  if (x < 0 || x >= w - 1 || y < 0 || y >= h - 1)
    return data[(int)(y + 0.5f) * stride + (int)(x + 0.5f)];

  int ix = (int)x;
  int iy = (int)y;
  float fx = x - ix;
  float fy = y - iy;

  float v00 = (float)data[iy * stride + ix];
  float v10 = (float)data[iy * stride + ix + 1];
  float v01 = (float)data[(iy + 1) * stride + ix];
  float v11 = (float)data[(iy + 1) * stride + ix + 1];

  return (v00 * (1 - fx) + v10 * fx) * (1 - fy)
       + (v01 * (1 - fx) + v11 * fx) * fy;
}

/* Compute spatial and temporal derivatives for a window around (cx, cy)
 * at the given pyramid level, with previous frame warped by (u, v).
 * Returns the 2x2 system and error vector. */
static bool lk_compute_derivatives(const pyramid_level_t *curr_level,
                                    const pyramid_level_t *prev_level,
                                    float cx, float cy, float u, float v,
                                    int half_win,
                                    float *Gxx, float *Gxy, float *Gyy,
                                    float *bx, float *by) {
  *Gxx = *Gxy = *Gyy = *bx = *by = 0.0f;

  uint16_t w = curr_level->w;
  uint16_t h = curr_level->h;

  /* Check bounds for the window */
  int icx = (int)cx;
  int icy = (int)cy;
  if (icx < half_win || icx + half_win >= (int)w ||
      icy < half_win || icy + half_win >= (int)h)
    return false;

  for (int dy = -half_win; dy <= half_win; dy++) {
    for (int dx = -half_win; dx <= half_win; dx++) {
      int px = icx + dx;
      int py = icy + dy;

      if (px < 1 || px >= (int)w - 1 || py < 1 || py >= (int)h - 1)
        continue;

      /* Spatial gradients in CURRENT frame (central differences) */
      float Ix = ((float)curr_level->data[py * curr_level->stride + px + 1]
                - (float)curr_level->data[py * curr_level->stride + px - 1]) * 0.5f;

      float Iy = ((float)curr_level->data[(py + 1) * curr_level->stride + px]
                - (float)curr_level->data[(py - 1) * curr_level->stride + px]) * 0.5f;

      /* Temporal difference: curr - warped prev */
      float prev_val = sample_bilinear(prev_level->data, prev_level->stride,
                                        (float)px + u, (float)py + v,
                                        prev_level->w, prev_level->h);
      float It = (float)curr_level->data[py * curr_level->stride + px] - prev_val;

      *Gxx += Ix * Ix;
      *Gxy += Ix * Iy;
      *Gyy += Iy * Iy;
      *bx  += Ix * It;
      *by  += Iy * It;
    }
  }

  return true;
}

/* Solve 2x2 system for flow:
 *   [Gxx  Gxy] [u]   -[bx]
 *   [Gxy  Gyy] [v] = -[by]
 *
 * Returns false if matrix is singular (ill-conditioned). */
static bool solve_lk(float Gxx, float Gxy, float Gyy, float bx, float by,
                      float *u, float *v) {
  float det = Gxx * Gyy - Gxy * Gxy;

  /* Reject if determinant is too small (minimum eigenvalue check) */
  float trace = Gxx + Gyy;
  if (det < 1e-7f || det < 0.01f * trace * trace) {
    *u = 0.0f;
    *v = 0.0f;
    return false;
  }

  *u = (-bx * Gyy + by * Gxy) / det;
  *v = (-by * Gxx + bx * Gxy) / det;

  return true;
}

lk_context_t* lk_context_create(const flow_stage2_config_t *cfg,
                                 uint16_t image_width,
                                 uint16_t image_height) {
  lk_context_t *ctx = (lk_context_t*)calloc(1, sizeof(lk_context_t));
  if (!ctx) return NULL;

  memcpy(&ctx->config, cfg, sizeof(flow_stage2_config_t));
  ctx->image_width = image_width;
  ctx->image_height = image_height;
  ctx->has_previous = false;

  /* Compute number of pyramid levels */
  ctx->num_levels = cfg->lk_pyramid_levels;
  if (ctx->num_levels < 1) ctx->num_levels = 1;
  if (ctx->num_levels > LK_MAX_LEVELS) ctx->num_levels = LK_MAX_LEVELS;

  /* Allocate pyramid scratch buffers
   * Level 0: original image (not owned)
   * Levels 1..N-1: downsampled images (owned)
   */
  uint32_t total_scratch = 0;
  uint16_t pw = image_width, ph = image_height;
  for (uint8_t i = 1; i < ctx->num_levels; i++) {
    pw /= 2;
    ph /= 2;
    if (pw < 4 || ph < 4) {
      ctx->num_levels = i;
      break;
    }
    total_scratch += (uint32_t)pw * ph;
  }

  if (total_scratch > 0) {
    ctx->scratch_buf = (uint8_t*)malloc(total_scratch);
    ctx->scratch_size = total_scratch;
  }

  /* Derivative buffers for largest level (level 0) */
  ctx->deriv_size = (uint32_t)image_width * image_height;
  ctx->Ix = (int16_t*)malloc(ctx->deriv_size * sizeof(int16_t) * 3);
  ctx->Iy = ctx->Ix + ctx->deriv_size;
  ctx->It = ctx->Iy + ctx->deriv_size;

  if (!ctx->Ix) {
    lk_context_destroy(ctx);
    return NULL;
  }

  return ctx;
}

void lk_context_destroy(lk_context_t *ctx) {
  if (ctx) {
    free(ctx->scratch_buf);
    free(ctx->Ix);
    free(ctx);
  }
}

uint16_t lk_track(lk_context_t *ctx,
                  const vision_image_t *prev,
                  const vision_image_t *curr,
                  lk_track_t *tracks,
                  uint16_t max_tracks) {
  if (!ctx->has_previous) {
    /* First frame: detect corners, store, return zero flow */
    fast_detect(curr, ctx->config.fast_threshold, &ctx->corners);
    fast_nonmax_suppression(&ctx->corners,
                            ctx->config.fast_min_distance,
                            ctx->config.max_corners);

    ctx->prev_track_count = 0;
    for (uint16_t i = 0; i < ctx->corners.count && i < FAST_MAX_CORNERS; i++) {
      ctx->prev_tracks[i].x = ctx->corners.corners[i].x;
      ctx->prev_tracks[i].y = ctx->corners.corners[i].y;
      ctx->prev_tracks[i].flow_x = 0;
      ctx->prev_tracks[i].flow_y = 0;
      ctx->prev_tracks[i].error = 0;
      ctx->prev_tracks[i].status = 1;
      ctx->prev_track_count++;
    }

    ctx->has_previous = true;
    return 0;
  }

  /* Build pyramids */
  build_pyramid(prev, ctx->prev_pyr, ctx->num_levels,
                ctx->scratch_buf, ctx->scratch_size);
  build_pyramid(curr, ctx->curr_pyr, ctx->num_levels,
                ctx->scratch_buf, ctx->scratch_size);

  /* Track each previous corner through the pyramid */
  uint16_t tracked = 0;
  int half_win = LK_WIN_HALF;

  for (uint16_t i = 0; i < ctx->prev_track_count && tracked < max_tracks; i++) {
    if (ctx->prev_tracks[i].status == 0) continue; /* skip lost ones */

    /* Start from the top of the pyramid, with initial guess from previous flow */
    float guess_x = ctx->prev_tracks[i].flow_x / (float)LK_SUBPIXEL_FACTOR;
    float guess_y = ctx->prev_tracks[i].flow_y / (float)LK_SUBPIXEL_FACTOR;

    /* Corner position at full resolution */
    float pos_x = (float)ctx->prev_tracks[i].x;
    float pos_y = (float)ctx->prev_tracks[i].y;

    uint8_t last_level = ctx->num_levels - 1;
    int iter = ctx->config.lk_max_iterations;

    for (int8_t level = (int8_t)last_level; level >= 0; level--) {
      float scale = 1.0f / (float)(1 << level);

      float cx = pos_x * scale;
      float cy = pos_y * scale;

      float u = guess_x * scale;
      float v = guess_y * scale;

      const pyramid_level_t *curr_lvl = &ctx->curr_pyr[level];
      const pyramid_level_t *prev_lvl = &ctx->prev_pyr[level];

      /* Newton-Raphson iteration */
      for (int it = 0; it < iter; it++) {
        float Gxx, Gxy, Gyy, bx, by;

        if (!lk_compute_derivatives(curr_lvl, prev_lvl,
                                     cx, cy, u, v,
                                     half_win, &Gxx, &Gxy, &Gyy, &bx, &by)) {
          u = v = 0.0f;
          break;
        }

        float du, dv;
        if (!solve_lk(Gxx, Gxy, Gyy, bx, by, &du, &dv)) {
          u = v = 0.0f;
          break;
        }

        u += du;
        v += dv;

        if (du * du + dv * dv < 0.01f) break;
      }

      guess_x = u * 2.0f;
      guess_y = v * 2.0f;
    }

    /* Compute error (SSD) at full resolution */
    uint32_t ssd = 0;
    uint16_t fpx = (uint16_t)(pos_x + 0.5f);
    uint16_t fpy = (uint16_t)(pos_y + 0.5f);
    uint16_t fpx_flow = (uint16_t)(pos_x + guess_x + 0.5f);
    uint16_t fpy_flow = (uint16_t)(pos_y + guess_y + 0.5f);

    int margin = half_win;
    if (fpx >= (uint16_t)margin && fpx + margin < curr->w &&
        fpy >= (uint16_t)margin && fpy + margin < curr->h &&
        fpx_flow >= (uint16_t)margin && fpx_flow + margin < prev->w &&
        fpy_flow >= (uint16_t)margin && fpy_flow + margin < prev->h) {
      for (int dy = -margin; dy <= margin; dy++) {
        for (int dx = -margin; dx <= margin; dx++) {
          int diff = (int)curr->buf[(fpy + dy) * curr->stride + (fpx + dx)]
                   - (int)prev->buf[(fpy_flow + dy) * prev->stride + (fpx_flow + dx)];
          ssd += (uint32_t)(diff * diff);
        }
      }
    } else {
      ssd = UINT32_MAX;
    }

    /* Store result */
    tracks[tracked].x = (uint16_t)pos_x;
    tracks[tracked].y = (uint16_t)pos_y;
    tracks[tracked].flow_x = (int32_t)(guess_x * LK_SUBPIXEL_FACTOR);
    tracks[tracked].flow_y = (int32_t)(guess_y * LK_SUBPIXEL_FACTOR);
    tracks[tracked].error = ssd;

    /* Reject if SSD is too high or flow is unreasonably large */
    uint32_t max_ssd = (uint32_t)((2 * half_win + 1) * (2 * half_win + 1) * 256);
    if (ssd < max_ssd && fabsf(guess_x) < 50.0f && fabsf(guess_y) < 50.0f) {
      tracks[tracked].status = 1;
    } else {
      tracks[tracked].status = 0;
      tracks[tracked].flow_x = 0;
      tracks[tracked].flow_y = 0;
    }

    tracked++;
  }

  /* Detect new corners on current frame for next iteration */
  fast_detect(curr, ctx->config.fast_threshold, &ctx->corners);
  fast_nonmax_suppression(&ctx->corners,
                          ctx->config.fast_min_distance,
                          ctx->config.max_corners);

  /* Build next prev_tracks from current corners + tracked flow */
  ctx->prev_track_count = 0;

  for (uint16_t i = 0; i < ctx->corners.count && ctx->prev_track_count < FAST_MAX_CORNERS; i++) {
    ctx->prev_tracks[ctx->prev_track_count].x = ctx->corners.corners[i].x;
    ctx->prev_tracks[ctx->prev_track_count].y = ctx->corners.corners[i].y;
    ctx->prev_tracks[ctx->prev_track_count].flow_x = 0;
    ctx->prev_tracks[ctx->prev_track_count].flow_y = 0;
    ctx->prev_tracks[ctx->prev_track_count].error = 0;
    ctx->prev_tracks[ctx->prev_track_count].status = 1;
    ctx->prev_track_count++;
  }

  return tracked;
}

void lk_get_corners(const lk_context_t *ctx, fast_corners_t *corners) {
  memcpy(corners, &ctx->corners, sizeof(fast_corners_t));
}

void lk_reconfigure(lk_context_t *ctx, const flow_stage2_config_t *cfg) {
  memcpy(&ctx->config, cfg, sizeof(flow_stage2_config_t));
}
