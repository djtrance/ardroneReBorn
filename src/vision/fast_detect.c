#include "fast_detect.h"
#include <stdlib.h>
#include <string.h>

/* Bresenham circle of radius 3: offsets from center (x,y) */
static const int8_t fast_circle_x[16] = {
  0, 1, 2, 3, 3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1
};
static const int8_t fast_circle_y[16] = {
  3, 3, 2, 1, 0, -1, -2, -3, -3, -3, -2, -1, 0, 1, 2, 3
};

/* FAST-9 detection using the segment test.
 * There are 16 pixels on the circle. For FAST-9, we need 9 contiguous
 * pixels that are all brighter or all darker.
 *
 * Optimization: test positions 0, 4, 8, 12 (4 cardinal points) first.
 * If none of these passes the threshold, skip non-maximum.
 */
uint16_t fast_detect(const vision_image_t *img, uint8_t threshold,
                     fast_corners_t *corners) {
  corners->count = 0;

  uint16_t w = img->w;
  uint16_t h = img->h;

  /* Must have 3-pixel border on each side for the circle */
  if (w < 7 || h < 7) return 0;

  for (uint16_t y = 3; y < h - 3; y++) {
    for (uint16_t x = 3; x < w - 3; x++) {
      if (corners->count >= FAST_MAX_CORNERS) break;

      uint8_t center = img->buf[y * img->stride + x];
      uint16_t cb = center + threshold;
      uint16_t cd = center > threshold ? center - threshold : 0;

      /* Quick check: at least 3 of 4 cardinal pixels must pass */
      uint8_t p0 = img->buf[(y + 3) * img->stride + x];        /* north */
      uint8_t p4 = img->buf[y * img->stride + (x + 3)];        /* east */
      uint8_t p8 = img->buf[(y - 3) * img->stride + x];        /* south */
      uint8_t p12 = img->buf[y * img->stride + (x - 3)];       /* west */

      bool brighter = false;
      bool darker = false;

      /* Check if enough cardinal points are bright or dark */
      uint8_t bright_cnt = 0, dark_cnt = 0;
      bright_cnt += (p0 >= cb) + (p4 >= cb) + (p8 >= cb) + (p12 >= cb);
      dark_cnt  += (p0 <= cd) + (p4 <= cd) + (p8 <= cd) + (p12 <= cd);

      if (bright_cnt >= 3) brighter = true;
      if (dark_cnt >= 3) darker = true;
      if (!brighter && !darker) continue;

      /* Full segment test */
      bool is_corner = false;

      if (brighter) {
        /* Check for 9 contiguous bright pixels */
        for (int s = 0; s < 16 && !is_corner; s++) {
          uint8_t count = 0;
          for (int i = 0; i < 16; i++) {
            int idx = (s + i) % 16;
            int px = (int)x + fast_circle_x[idx];
            int py = (int)y + fast_circle_y[idx];
            if (img->buf[py * img->stride + px] >= cb) {
              count++;
              if (count >= 9) { is_corner = true; break; }
            } else {
              count = 0;
            }
          }
        }
      }

      if (!is_corner && darker) {
        /* Check for 9 contiguous dark pixels */
        for (int s = 0; s < 16 && !is_corner; s++) {
          uint8_t count = 0;
          for (int i = 0; i < 16; i++) {
            int idx = (s + i) % 16;
            int px = (int)x + fast_circle_x[idx];
            int py = (int)y + fast_circle_y[idx];
            if (img->buf[py * img->stride + px] <= cd) {
              count++;
              if (count >= 9) { is_corner = true; break; }
            } else {
              count = 0;
            }
          }
        }
      }

      if (is_corner) {
        fast_corner_t *c = &corners->corners[corners->count];
        c->x = x;
        c->y = y;
        c->score = fast_corner_score(img, x, y, threshold);
        corners->count++;
      }
    }
  }

  return corners->count;
}

uint32_t fast_corner_score(const vision_image_t *img, uint16_t x, uint16_t y,
                           uint8_t threshold) {
  (void)threshold;
  uint8_t center = img->buf[y * img->stride + x];
  uint32_t score_brighter = 0;
  uint32_t score_darker = 0;

  for (int i = 0; i < 16; i++) {
    int px = (int)x + fast_circle_x[i];
    int py = (int)y + fast_circle_y[i];
    int diff = (int)img->buf[py * img->stride + px] - (int)center;

    if (diff > 0) score_brighter += (uint32_t)diff;
    else          score_darker  += (uint32_t)(-diff);
  }

  return score_brighter > score_darker ? score_brighter : score_darker;
}

/* Corner score comparison for sorting (descending) */
static int corner_cmp_desc(const void *a, const void *b) {
  const fast_corner_t *ca = (const fast_corner_t*)a;
  const fast_corner_t *cb = (const fast_corner_t*)b;
  if (ca->score > cb->score) return -1;
  if (ca->score < cb->score) return 1;
  return 0;
}

void fast_nonmax_suppression(fast_corners_t *corners, uint8_t min_distance,
                             uint16_t max_corners) {
  if (corners->count == 0) return;

  /* Sort by score descending */
  qsort(corners->corners, corners->count, sizeof(fast_corner_t),
        corner_cmp_desc);

  /* Greedy suppression: keep highest-scoring corner, remove nearby */
  fast_corner_t kept[FAST_MAX_CORNERS];
  uint16_t kept_count = 0;
  bool removed[FAST_MAX_CORNERS];
  memset(removed, 0, sizeof(removed));

  uint16_t md2 = (uint16_t)min_distance * min_distance;

  for (uint16_t i = 0; i < corners->count && kept_count < max_corners; i++) {
    if (removed[i]) continue;

    kept[kept_count++] = corners->corners[i];

    /* Remove all corners within min_distance of this one */
    for (uint16_t j = i + 1; j < corners->count; j++) {
      if (removed[j]) continue;
      int16_t dx = (int16_t)corners->corners[i].x - (int16_t)corners->corners[j].x;
      int16_t dy = (int16_t)corners->corners[i].y - (int16_t)corners->corners[j].y;
      if ((uint16_t)(dx * dx + dy * dy) <= md2) {
        removed[j] = true;
      }
    }
  }

  /* Copy kept corners back */
  memcpy(corners->corners, kept, kept_count * sizeof(fast_corner_t));
  corners->count = kept_count;
}
