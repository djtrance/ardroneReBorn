#include "image.h"
#include <stdlib.h>
#include <string.h>

void image_init(vision_image_t *img, uint16_t w, uint16_t h, vision_image_type_t type) {
  img->type = type;
  img->w = w;
  img->h = h;
  switch (type) {
    case VISION_IMAGE_GRAYSCALE:
      img->bytes_per_pixel = 1;
      break;
    case VISION_IMAGE_UYVY:
      img->bytes_per_pixel = 2; /* 4 bytes per 2 pixels = 2 bpp */
      break;
    case VISION_IMAGE_YUV420:
      img->bytes_per_pixel = 1; /* planar, handled specially */
      break;
  }
  img->stride = w * img->bytes_per_pixel;
  if (type == VISION_IMAGE_YUV420) {
    img->stride = w; /* planar Y plane */
  }
  img->buf = NULL;
  img->buf_size = 0;
}

void image_destroy(vision_image_t *img) {
  if (img->buf) {
    free(img->buf);
    img->buf = NULL;
  }
  img->buf_size = 0;
}

void image_zero(vision_image_t *img) {
  if (img->buf) {
    memset(img->buf, 0, img->buf_size);
  }
}

bool image_create(vision_image_t *img, uint16_t w, uint16_t h, vision_image_type_t type) {
  image_init(img, w, h, type);
  switch (type) {
    case VISION_IMAGE_GRAYSCALE:
      img->buf_size = (uint32_t)h * w;
      break;
    case VISION_IMAGE_UYVY:
      img->buf_size = (uint32_t)h * w * 2;
      img->stride = w * 2;
      break;
    case VISION_IMAGE_YUV420:
      img->buf_size = (uint32_t)h * w * 3 / 2;
      img->stride = w;
      break;
  }
  img->buf = (uint8_t*)malloc(img->buf_size);
  if (!img->buf) {
    img->buf_size = 0;
    return false;
  }
  return true;
}

void image_copy(const vision_image_t *src, vision_image_t *dst) {
  if (src->buf && dst->buf && src->buf_size <= dst->buf_size) {
    memcpy(dst->buf, src->buf, src->buf_size);
    dst->w = src->w;
    dst->h = src->h;
    dst->stride = src->stride;
    dst->type = src->type;
    dst->bytes_per_pixel = src->bytes_per_pixel;
  }
}

void image_uyvy_to_grayscale(const vision_image_t *src, vision_image_t *dst) {
  if (!src->buf || !dst->buf) return;
  if (dst->w < src->w || dst->h < src->h) return;

  uint16_t w = src->w;
  uint16_t h = src->h;
  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x += 2) {
      uint8_t y0 = src->buf[y * src->stride + x * 2 + 1];
      uint8_t y1 = src->buf[y * src->stride + (x + 1) * 2 + 1];
      dst->buf[y * dst->stride + x] = y0;
      dst->buf[y * dst->stride + x + 1] = y1;
    }
  }
}

void image_uyvy_extract_y(const vision_image_t *src, vision_image_t *dst) {
  image_uyvy_to_grayscale(src, dst);
}

void image_uyvy_downsample(const vision_image_t *src, vision_image_t *dst, uint8_t factor) {
  if (!src->buf || !dst->buf) return;
  if (dst->w < src->w / factor || dst->h < src->h / factor) return;

  uint16_t dst_w = src->w / factor;
  uint16_t dst_h = src->h / factor;

  for (uint16_t dy = 0; dy < dst_h; dy++) {
    for (uint16_t dx = 0; dx < dst_w; dx++) {
      uint32_t sum = 0;
      for (uint8_t fy = 0; fy < factor; fy++) {
        for (uint8_t fx = 0; fx < factor; fx++) {
          uint16_t sx = dx * factor + fx;
          uint16_t sy = dy * factor + fy;
          sum += image_get_y(src, sx, sy);
        }
      }
      dst->buf[dy * dst->stride + dx] = (uint8_t)(sum / (factor * factor));
    }
  }
}

uint32_t image_sad_block(const vision_image_t *img, uint16_t x, uint16_t y,
                          const vision_image_t *prev, uint16_t px, uint16_t py,
                          uint8_t block_size) {
  uint32_t sad = 0;
  for (uint8_t fy = 0; fy < block_size; fy++) {
    for (uint8_t fx = 0; fx < block_size; fx++) {
      int16_t diff = (int16_t)image_get_pixel(img, x + fx, y + fy)
                   - (int16_t)image_get_pixel(prev, px + fx, py + fy);
      sad += (uint32_t)(diff < 0 ? -diff : diff);
    }
  }
  return sad;
}

typedef struct {
  int16_t dx;
  int16_t dy;
  uint32_t sad;
} match_result_t;

static bool search_best_match(const vision_image_t *img, const vision_image_t *prev,
                               uint16_t tx, uint16_t ty, uint8_t tile_size,
                               uint8_t search_range, int16_t *out_dx, int16_t *out_dy,
                               uint32_t *out_sad) {
  uint32_t best_sad = UINT32_MAX;
  int16_t best_dx = 0, best_dy = 0;
  bool found = false;

  int16_t sr = search_range;

  for (int16_t dy = -sr; dy <= sr; dy++) {
    for (int16_t dx = -sr; dx <= sr; dx++) {
      int32_t px = (int32_t)tx + dx;
      int32_t py = (int32_t)ty + dy;

      if (px < 0 || py < 0 || px + tile_size > prev->w || py + tile_size > prev->h)
        continue;

      uint32_t sad = image_sad_block(img, tx, ty, prev, (uint16_t)px, (uint16_t)py, tile_size);
      if (sad < best_sad) {
        best_sad = sad;
        best_dx = dx;
        best_dy = dy;
        found = true;
      }
    }
  }

  if (found) {
    *out_dx = best_dx;
    *out_dy = best_dy;
    *out_sad = best_sad;
  }
  return found;
}

static void subpixel_refine(const vision_image_t *img, const vision_image_t *prev,
                             uint16_t tx, uint16_t ty, uint8_t tile_size,
                             int16_t *dx, int16_t *dy) {
  uint32_t sad_center = image_sad_block(img, tx, ty, prev,
    (uint16_t)((int32_t)tx + *dx), (uint16_t)((int32_t)ty + *dy), tile_size);

  uint32_t sad_xn = image_sad_block(img, tx, ty, prev,
    (uint16_t)((int32_t)tx + *dx - 1), (uint16_t)((int32_t)ty + *dy), tile_size);
  uint32_t sad_xp = image_sad_block(img, tx, ty, prev,
    (uint16_t)((int32_t)tx + *dx + 1), (uint16_t)((int32_t)ty + *dy), tile_size);

  uint32_t sad_yn = image_sad_block(img, tx, ty, prev,
    (uint16_t)((int32_t)tx + *dx), (uint16_t)((int32_t)ty + *dy - 1), tile_size);
  uint32_t sad_yp = image_sad_block(img, tx, ty, prev,
    (uint16_t)((int32_t)tx + *dx), (uint16_t)((int32_t)ty + *dy + 1), tile_size);

  int32_t numer_x = (int32_t)(sad_xn - sad_xp);
  int32_t denom_x = (int32_t)(sad_xn + sad_xp - 2 * sad_center);
  if (denom_x != 0) {
    *dx = (int16_t)(*dx * 10 + (numer_x * 10 / denom_x) / 2);
  } else {
    *dx = *dx * 10;
  }

  int32_t numer_y = (int32_t)(sad_yn - sad_yp);
  int32_t denom_y = (int32_t)(sad_yn + sad_yp - 2 * sad_center);
  if (denom_y != 0) {
    *dy = (int16_t)(*dy * 10 + (numer_y * 10 / denom_y) / 2);
  } else {
    *dy = *dy * 10;
  }
}

void image_sad_block_many(const vision_image_t *img, const vision_image_t *prev,
                           uint16_t *x_positions, uint16_t *y_positions,
                           int16_t *out_dx, int16_t *out_dy, uint32_t *out_sad,
                           uint16_t num_tiles, uint8_t tile_size, uint8_t search_range) {
  for (uint16_t i = 0; i < num_tiles; i++) {
    int16_t dx, dy;
    uint32_t sad;
    bool found = search_best_match(img, prev, x_positions[i], y_positions[i],
                                    tile_size, search_range, &dx, &dy, &sad);
    if (found) {
      subpixel_refine(img, prev, x_positions[i], y_positions[i], tile_size, &dx, &dy);
      out_dx[i] = dx;
      out_dy[i] = dy;
      out_sad[i] = sad;
    } else {
      out_dx[i] = 0;
      out_dy[i] = 0;
      out_sad[i] = UINT32_MAX;
    }
  }
}

uint32_t image_mean(const vision_image_t *img) {
  if (!img->buf) return 0;
  uint32_t sum = 0;
  uint32_t count = (uint32_t)img->w * img->h;
  for (uint32_t i = 0; i < count; i++) {
    sum += img->buf[i];
  }
  return sum / count;
}

uint32_t image_variance(const vision_image_t *img) {
  if (!img->buf) return 0;
  uint32_t mean = image_mean(img);
  uint32_t sum_sq = 0;
  uint32_t count = (uint32_t)img->w * img->h;
  for (uint32_t i = 0; i < count; i++) {
    int32_t diff = (int32_t)img->buf[i] - (int32_t)mean;
    sum_sq += (uint32_t)(diff * diff);
  }
  return sum_sq / count;
}

void image_draw_flow(vision_image_t *img, int32_t flow_x, int32_t flow_y, uint8_t subpixel_factor) {
  if (img->type != VISION_IMAGE_GRAYSCALE) return;

  uint16_t cx = img->w / 2;
  uint16_t cy = img->h / 2;
  int16_t ex = cx + (int16_t)(flow_x / (int32_t)subpixel_factor);
  int16_t ey = cy + (int16_t)(flow_y / (int32_t)subpixel_factor);

  for (int16_t t = 0; t <= 10; t++) {
    int16_t lx = cx + (ex - cx) * t / 10;
    int16_t ly = cy + (ey - cy) * t / 10;
    if (lx >= 0 && lx < img->w && ly >= 0 && ly < img->h)
      img->buf[ly * img->stride + lx] = 255;
  }
}

void image_draw_crosshair(vision_image_t *img, uint16_t x, uint16_t y, uint8_t color) {
  if (img->type != VISION_IMAGE_GRAYSCALE) return;
  for (int16_t dx = -5; dx <= 5; dx++) {
    int16_t px = (int16_t)x + dx;
    if (px >= 0 && px < img->w) img->buf[y * img->stride + px] = color;
  }
  for (int16_t dy = -5; dy <= 5; dy++) {
    int16_t py = (int16_t)y + dy;
    if (py >= 0 && py < img->h) img->buf[py * img->stride + x] = color;
  }
}
