#ifndef VISION_IMAGE_H
#define VISION_IMAGE_H

#include "types.h"
#include <stddef.h>

/* Lifecycle */
void image_init(vision_image_t *img, uint16_t w, uint16_t h, vision_image_type_t type);
void image_destroy(vision_image_t *img);
void image_zero(vision_image_t *img);
bool image_create(vision_image_t *img, uint16_t w, uint16_t h, vision_image_type_t type);

/* Copy and convert */
void image_copy(const vision_image_t *src, vision_image_t *dst);
void image_uyvy_to_grayscale(const vision_image_t *src, vision_image_t *dst);
void image_uyvy_downsample(const vision_image_t *src, vision_image_t *dst, uint8_t factor);
void image_uyvy_extract_y(const vision_image_t *src, vision_image_t *dst);

/* Pixel access (portable, stride-aware) */
static inline uint8_t* image_pixel_ptr(const vision_image_t *img, uint16_t x, uint16_t y) {
  return img->buf + y * img->stride + x * img->bytes_per_pixel;
}

static inline uint8_t image_get_y(const vision_image_t *img, uint16_t x, uint16_t y) {
  /* For UYVY: Y is at offset 1 within each 2-pixel macro-pixel */
  if (img->type == VISION_IMAGE_UYVY) {
    return img->buf[y * img->stride + (x / 2) * 4 + 1 + (x % 2) * 2];
  }
  return img->buf[y * img->stride + x];
}

static inline uint8_t image_get_pixel(const vision_image_t *img, uint16_t x, uint16_t y) {
  /* Grayscale: direct access */
  if (img->type == VISION_IMAGE_GRAYSCALE) {
    return img->buf[y * img->stride + x];
  }
  /* UYVY: return Y (luma) */
  return image_get_y(img, x, y);
}

/* SAD computation */
uint32_t image_sad_block(const vision_image_t *img, uint16_t x, uint16_t y,
                          const vision_image_t *prev, uint16_t px, uint16_t py,
                          uint8_t block_size);

void image_sad_block_many(const vision_image_t *img, const vision_image_t *prev,
                           uint16_t *x_positions, uint16_t *y_positions,
                           int16_t *out_dx, int16_t *out_dy, uint32_t *out_sad,
                           uint16_t num_tiles, uint8_t tile_size, uint8_t search_range);

/* Statistics */
uint32_t image_mean(const vision_image_t *img);
uint32_t image_variance(const vision_image_t *img);

/* Drawing (for visualization) */
void image_draw_flow(vision_image_t *img, int32_t flow_x, int32_t flow_y, uint8_t subpixel_factor);
void image_draw_crosshair(vision_image_t *img, uint16_t x, uint16_t y, uint8_t color);

#endif /* VISION_IMAGE_H */
