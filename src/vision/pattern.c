#include "pattern.h"
#include "image.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* Minimum landing pad size in pixels */
#define MIN_PAD_SIZE 8
#define MAX_PAD_SIZE 240

/* Connected component labeling uses a two-pass algorithm with
 * equivalence resolution via union-find. */

typedef struct {
  uint16_t parent;
  uint32_t area;
  uint64_t sum_x;
  uint64_t sum_y;
  uint16_t min_x, max_x;
  uint16_t min_y, max_y;
  uint32_t sum_intensity;
} cc_region_t;

#define MAX_REGIONS 256

struct pattern_ctx_s {
  uint16_t image_width;
  uint16_t image_height;

  /* Binary image buffer for thresholding */
  uint8_t *binary;
  uint32_t binary_size;

  /* Label buffer for connected components */
  uint16_t *labels;
  uint32_t label_count;

  /* Region table for union-find */
  cc_region_t regions[MAX_REGIONS];
  uint16_t num_regions;

  /* Landing pad tracking */
  landing_pad_t last_pad;
  bool has_last_pad;

  /* Known physical pad size (mm) */
  uint16_t pad_size_mm;

  /* Home snapshot */
  vision_image_t home_snapshot;
  bool has_home_snapshot;

  /* Camera parameters for distance estimation */
  float focal_length_px;    /* focal length in pixels */
};

pattern_ctx_t* pattern_create(uint16_t image_width, uint16_t image_height) {
  pattern_ctx_t *ctx = (pattern_ctx_t*)calloc(1, sizeof(pattern_ctx_t));
  if (!ctx) return NULL;

  ctx->image_width = image_width;
  ctx->image_height = image_height;

  /* Allocate binary image buffer */
  ctx->binary_size = (uint32_t)image_width * image_height;
  ctx->binary = (uint8_t*)malloc(ctx->binary_size);
  ctx->labels = (uint16_t*)malloc(ctx->binary_size * sizeof(uint16_t));

  if (!ctx->binary || !ctx->labels) {
    free(ctx->binary);
    free(ctx->labels);
    free(ctx);
    return NULL;
  }

  ctx->has_last_pad = false;
  ctx->has_home_snapshot = false;
  ctx->pad_size_mm = 300;  /* default: 30cm landing pad */

  /* AR.Drone 2.0 bottom camera: 320x240, ~82° FOV
   * focal_length_px = 0.5 * W / tan(FOV/2) = 160 / tan(41°) ≈ 184 */
  ctx->focal_length_px = 184.0f;

  return ctx;
}

void pattern_destroy(pattern_ctx_t *ctx) {
  if (ctx) {
    free(ctx->binary);
    free(ctx->labels);
    if (ctx->has_home_snapshot)
      image_destroy(&ctx->home_snapshot);
    free(ctx);
  }
}

/* Find parent in union-find with path compression */
static uint16_t find_parent(cc_region_t *regions, uint16_t x) {
  while (regions[x].parent != x) {
    regions[x].parent = regions[regions[x].parent].parent;
    x = regions[x].parent;
  }
  return x;
}

/* Union two regions */
static void union_regions(cc_region_t *regions, uint16_t a, uint16_t b) {
  uint16_t pa = find_parent(regions, a);
  uint16_t pb = find_parent(regions, b);
  if (pa != pb)
    regions[pb].parent = pa;
}

void pattern_find_blobs(pattern_ctx_t *ctx,
                         const vision_image_t *img,
                         uint8_t threshold,
                         pattern_blobs_t *blobs) {
  blobs->count = 0;

  if (!img->buf || !ctx->binary) return;

  uint16_t w = img->w;
  uint16_t h = img->h;

  /* Step 1: Binarize */
  for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
    ctx->binary[i] = (img->buf[i] < threshold) ? 1 : 0;
  }

  /* Step 2: First pass - assign labels with equivalence resolution */
  uint16_t next_label = 1;
  memset(ctx->labels, 0, ctx->binary_size * sizeof(uint16_t));
  memset(ctx->regions, 0, sizeof(ctx->regions));

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      uint32_t idx = y * w + x;
      if (!ctx->binary[idx]) continue;

      uint16_t labels_above[8];
      int nl = 0;

      /* Check 8-connected neighbors:
       *   [ul] [a] [ur]
       *   [ l] [x] 
       */
      if (x > 0 && ctx->labels[idx - 1])
        labels_above[nl++] = ctx->labels[idx - 1];
      if (y > 0 && ctx->labels[idx - w])
        labels_above[nl++] = ctx->labels[idx - w];
      if (x > 0 && y > 0 && ctx->labels[idx - w - 1])
        labels_above[nl++] = ctx->labels[idx - w - 1];
      if (x + 1 < w && y > 0 && ctx->labels[idx - w + 1])
        labels_above[nl++] = ctx->labels[idx - w + 1];

      if (nl == 0) {
        /* New label */
        ctx->labels[idx] = next_label;
        ctx->regions[next_label].parent = next_label;
        ctx->regions[next_label].min_x = x;
        ctx->regions[next_label].max_x = x;
        ctx->regions[next_label].min_y = y;
        ctx->regions[next_label].max_y = y;
        next_label++;
        if (next_label >= MAX_REGIONS) return;
      } else {
        /* Assign smallest label and record equivalences */
        uint16_t min_label = labels_above[0];
        for (int i = 1; i < nl; i++) {
          if (labels_above[i] < min_label)
            min_label = labels_above[i];
          union_regions(ctx->regions, labels_above[0], labels_above[i]);
        }
        ctx->labels[idx] = min_label;
      }
    }
  }

  /* Step 3: Second pass - resolve equivalences and accumulate stats */
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      uint32_t idx = y * w + x;
      if (!ctx->labels[idx]) continue;

      uint16_t root = find_parent(ctx->regions, ctx->labels[idx]);
      ctx->labels[idx] = root;

      ctx->regions[root].area++;
      ctx->regions[root].sum_x += x;
      ctx->regions[root].sum_y += y;
      ctx->regions[root].sum_intensity += img->buf[idx];

      if (x < ctx->regions[root].min_x) ctx->regions[root].min_x = x;
      if (x > ctx->regions[root].max_x) ctx->regions[root].max_x = x;
      if (y < ctx->regions[root].min_y) ctx->regions[root].min_y = y;
      if (y > ctx->regions[root].max_y) ctx->regions[root].max_y = y;
    }
  }

  /* Step 4: Collect valid blobs (filter by size) */
  for (uint16_t i = 1; i < next_label; i++) {
    if (ctx->regions[i].parent == i && ctx->regions[i].area > 16) {
      if (blobs->count >= PATTERN_MAX_BLOBS) break;

      pattern_blob_t *b = &blobs->blobs[blobs->count];
      cc_region_t *r = &ctx->regions[i];

      b->x = (uint16_t)(r->sum_x / r->area);
      b->y = (uint16_t)(r->sum_y / r->area);
      b->area = r->area;
      b->min_x = r->min_x;
      b->max_x = r->max_x;
      b->min_y = r->min_y;
      b->max_y = r->max_y;
      b->width = r->max_x - r->min_x + 1;
      b->height = r->max_y - r->min_y + 1;
      b->aspect = (b->height > 0) ? (float)b->width / b->height : 1.0f;
      b->mean = (uint8_t)(r->sum_intensity / r->area);

      blobs->count++;
    }
  }
}

void pattern_detect_landing_pad(pattern_ctx_t *ctx,
                                 const vision_image_t *img,
                                 landing_pad_t *pad) {
  pad->found = false;
  pad->confidence = 0;

  /* Adaptive threshold: use local mean over a 32x32 window */
  /* Simplified: use global mean + offset */
  uint32_t sum = 0;
  uint32_t count = (uint32_t)img->w * img->h;
  for (uint32_t i = 0; i < count; i++)
    sum += img->buf[i];
  uint8_t mean = (uint8_t)(sum / count);

  /* Threshold: objects darker than mean - 30 */
  uint8_t threshold = (mean > 35) ? mean - 30 : 10;

  /* Find dark blobs */
  pattern_blobs_t blobs;
  pattern_find_blobs(ctx, img, threshold, &blobs);

  /* Score each blob as a potential landing pad */
  int best_idx = -1;
  float best_score = 0;

  for (uint16_t i = 0; i < blobs.count; i++) {
    pattern_blob_t *b = &blobs.blobs[i];

    /* Size filter */
    if (b->width < MIN_PAD_SIZE || b->height < MIN_PAD_SIZE)
      continue;
    if (b->width > MAX_PAD_SIZE || b->height > MAX_PAD_SIZE)
      continue;

    /* Aspect ratio: roughly square for a landing pad (0.6 - 1.7) */
    if (b->aspect < 0.6f || b->aspect > 1.7f)
      continue;

    /* Compactness: area vs bounding box */
    float fill_ratio = (float)b->area / (float)(b->width * b->height);

    /* Landing pad should fill ~40-90% of its bounding box */
    if (fill_ratio < 0.2f || fill_ratio > 0.95f)
      continue;

    /* Prefer objects near the center of the image */
    float cx = img->w / 2.0f;
    float cy = img->h / 2.0f;
    float dist = sqrtf((b->x - cx) * (b->x - cx) + (b->y - cy) * (b->y - cy));
    float center_score = 1.0f - dist / (sqrtf(cx * cx + cy * cy));

    /* Prefer larger objects (closer = more important for landing) */
    float size_score = (float)(b->width * b->height) / (float)(img->w * img->h);
    if (size_score > 0.5f) size_score = 0.5f;

    /* Combined score */
    float score = fill_ratio * 0.3f + center_score * 0.4f + size_score * 2.0f;

    if (score > best_score) {
      best_score = score;
      best_idx = i;
    }
  }

  if (best_idx >= 0) {
    pattern_blob_t *b = &blobs.blobs[best_idx];
    pad->found = true;
    pad->center_x = b->x;
    pad->center_y = b->y;
    pad->size = (b->width > b->height) ? b->width : b->height;

    /* Confidence: 0-255, based on score */
    pad->confidence = (uint8_t)((best_score > 2.0f) ? 255 : (uint8_t)(best_score * 127));

    /* Distance estimation: distance = (focal * real_size) / image_size */
    if (pad->size > 0) {
      float real_size_mm = ctx->pad_size_mm;
      pad->distance_est = ctx->focal_length_px * real_size_mm
                          / (pad->size * 10.0f);
      /* Convert to cm */
      pad->distance_est /= 10.0f;
    } else {
      pad->distance_est = 0;
    }

    ctx->last_pad = *pad;
    ctx->has_last_pad = true;
  }
}

void pattern_track_landing_pad(pattern_ctx_t *ctx,
                                int32_t flow_x, int32_t flow_y,
                                landing_pad_t *pad) {
  if (!ctx->has_last_pad) return;

  /* Apply flow to translate expected pad position */
  int16_t dx = (int16_t)(flow_x / 10);
  int16_t dy = (int16_t)(flow_y / 10);

  int32_t new_x = (int32_t)ctx->last_pad.center_x + dx;
  int32_t new_y = (int32_t)ctx->last_pad.center_y + dy;

  if (new_x >= 0 && new_x < ctx->image_width &&
      new_y >= 0 && new_y < ctx->image_height) {
    pad->center_x = (uint16_t)new_x;
    pad->center_y = (uint16_t)new_y;
    pad->size = ctx->last_pad.size;
    pad->confidence = ctx->last_pad.confidence / 2;  /* degrade confidence */
    pad->found = true;
  }
}

void pattern_set_pad_size(pattern_ctx_t *ctx, uint16_t size_mm) {
  ctx->pad_size_mm = size_mm;
}

void pattern_detect_home(pattern_ctx_t *ctx,
                          const vision_image_t *img,
                          home_marker_t *home) {
  home->found = false;

  if (!ctx->has_home_snapshot) return;

  /* Sparse tile grid from current frame */
  #define HOME_MAX_TILES 300
  uint16_t px[HOME_MAX_TILES], py[HOME_MAX_TILES];
  int16_t dx[HOME_MAX_TILES], dy[HOME_MAX_TILES];
  uint32_t err[HOME_MAX_TILES];

  uint16_t nt = 0;
  uint8_t ts = 8, sr = 60;
  for (uint16_t y = ts + sr; y + ts + sr <= img->h; y += 24)
    for (uint16_t x = ts + sr; x + ts + sr <= img->w; x += 24)
      if (nt < HOME_MAX_TILES) { px[nt] = x; py[nt] = y; nt++; }

  if (nt < 4) return;

  /* Match current frame against stored snapshot */
  image_sad_block_many(img, &ctx->home_snapshot, px, py, dx, dy, err, nt, ts, sr);

  /* Histogram with 4px bins for outlier rejection */
  #define HOME_HIST_BINS 256
  int16_t hist_dx[HOME_HIST_BINS], hist_dy[HOME_HIST_BINS];
  uint16_t hist_w[HOME_HIST_BINS];
  uint16_t hist_n = 0;

  for (uint16_t i = 0; i < nt; i++) {
    if (err[i] == UINT32_MAX) continue;
    /* dx,dy are in subpixel*10: divide by 10 to get pixels */
    int16_t bx = (dx[i] / 10) & ~3;  /* round down to nearest 4 */
    int16_t by = (dy[i] / 10) & ~3;

    bool found = false;
    for (uint16_t j = 0; j < hist_n; j++) {
      if (hist_dx[j] == bx && hist_dy[j] == by) {
        hist_w[j]++;
        found = true;
        break;
      }
    }
    if (!found && hist_n < HOME_HIST_BINS) {
      hist_dx[hist_n] = bx;
      hist_dy[hist_n] = by;
      hist_w[hist_n] = 1;
      hist_n++;
    }
  }

  /* Find dominant bin */
  uint16_t best_idx = 0, best_w = 0;
  for (uint16_t i = 0; i < hist_n; i++) {
    if (hist_w[i] > best_w) { best_w = hist_w[i]; best_idx = i; }
  }

  if (best_w < nt / 8) return;  /* insufficient consensus */

  int16_t dom_dx = hist_dx[best_idx];
  int16_t dom_dy = hist_dy[best_idx];

  /* Refine: average all tiles in the dominant bin */
  int64_t sum_dx = 0, sum_dy = 0;
  uint16_t count = 0;
  for (uint16_t i = 0; i < nt; i++) {
    if (err[i] == UINT32_MAX) continue;
    int16_t bx = (dx[i] / 10) & ~3;
    int16_t by = (dy[i] / 10) & ~3;
    if (bx == dom_dx && by == dom_dy) {
      sum_dx += dx[i];
      sum_dy += dy[i];
      count++;
    }
  }
  if (count == 0) return;

  float avg_dx = (float)sum_dx / (count * 10.0f);  /* pixels */
  float avg_dy = (float)sum_dy / (count * 10.0f);

  /* Convert pixel displacement to angle and distance.
   *
   * SAD matching gives how far each tile shifted between snapshot and now.
   * Camera moved opposite to scene displacement in image:
   *   camera_body_forward ~= -dy  (image -y = body forward)
   *   camera_body_right   ~=  dx  (image +x = body right)
   *
   * Home is opposite of camera movement.
   */
  float px_disp = sqrtf(avg_dx * avg_dx + avg_dy * avg_dy);
  if (px_disp < 1.0f) return;  /* too close to zero */

  /* Distance estimate using pinhole: dist = px_disp * altitude / focal */
  float altitude_cm = 100.0f;
  float dist_cm = px_disp * altitude_cm / ctx->focal_length_px;

  /* Angle to home: atan2(-dx, dy) in body frame */
  float angle_rad = atan2f(-avg_dx, avg_dy);
  int angle_cdeg = (int)(angle_rad * 18000.0f / 3.14159265f);  /* centidegrees */

  home->found = true;
  home->angle = (int16_t)angle_cdeg;
  home->distance = (int16_t)(dist_cm < 32767.0f ? dist_cm : 32767.0f);
  home->confidence = (uint8_t)(best_w * 255 / nt);
}

void pattern_snapshot_home(pattern_ctx_t *ctx, const vision_image_t *img) {
  if (!ctx->has_home_snapshot) {
    image_create(&ctx->home_snapshot, img->w, img->h, VISION_IMAGE_GRAYSCALE);
    ctx->has_home_snapshot = true;
  }
  image_copy(img, &ctx->home_snapshot);
}
