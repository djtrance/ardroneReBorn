#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include <SDL2/SDL.h>

#include "types.h"
#include "image.h"
#include "flow_stage1.h"
#include "flow_stage2.h"
#include "rotation.h"
#include "pattern.h"
#include "obstacle.h"
#include "line_detect.h"

#define W 320
#define H 240
#define SCALE 3
#define WIN_W (W * SCALE)
#define WIN_H (H * SCALE)
#define SUBP 10

/* 8x8 bitmap font (ASCII 32-126), from Linux kernel font_8x8 */
static const uint8_t font8x8[95][8] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
  {0x18,0x18,0x18,0x18,0x00,0x00,0x18,0x00},
  {0x6c,0x6c,0x6c,0x00,0x00,0x00,0x00,0x00},
  {0x6c,0x6c,0xfe,0x6c,0xfe,0x6c,0x6c,0x00},
  {0x18,0x3e,0x60,0x3c,0x06,0x7c,0x18,0x00},
  {0x00,0x66,0xac,0xd8,0x36,0x6a,0xcc,0x00},
  {0x38,0x6c,0x38,0x76,0xdc,0xcc,0x76,0x00},
  {0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x00},
  {0x0c,0x18,0x30,0x30,0x30,0x18,0x0c,0x00},
  {0x30,0x18,0x0c,0x0c,0x0c,0x18,0x30,0x00},
  {0x00,0x66,0x3c,0xff,0x3c,0x66,0x00,0x00},
  {0x00,0x18,0x18,0x7e,0x18,0x18,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30},
  {0x00,0x00,0x00,0x7e,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00},
  {0x06,0x0c,0x18,0x30,0x60,0xc0,0x80,0x00},
  {0x3c,0x66,0x6e,0x7e,0x76,0x66,0x3c,0x00},
  {0x18,0x38,0x18,0x18,0x18,0x18,0x7e,0x00},
  {0x3c,0x66,0x06,0x1c,0x30,0x60,0x7e,0x00},
  {0x3c,0x66,0x06,0x1c,0x06,0x66,0x3c,0x00},
  {0x1c,0x3c,0x6c,0xcc,0xfe,0x0c,0x1e,0x00},
  {0x7e,0x60,0x7c,0x06,0x06,0x66,0x3c,0x00},
  {0x1c,0x30,0x60,0x7c,0x66,0x66,0x3c,0x00},
  {0x7e,0x06,0x0c,0x18,0x30,0x30,0x30,0x00},
  {0x3c,0x66,0x66,0x3c,0x66,0x66,0x3c,0x00},
  {0x3c,0x66,0x66,0x3e,0x06,0x0c,0x38,0x00},
  {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00},
  {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x30},
  {0x0c,0x18,0x30,0x60,0x30,0x18,0x0c,0x00},
  {0x00,0x00,0x7e,0x00,0x7e,0x00,0x00,0x00},
  {0x30,0x18,0x0c,0x06,0x0c,0x18,0x30,0x00},
  {0x3c,0x66,0x06,0x1c,0x18,0x00,0x18,0x00},
  {0x3c,0x66,0x6e,0x6e,0x6e,0x60,0x3c,0x00},
  {0x18,0x3c,0x66,0x66,0x7e,0x66,0x66,0x00},
  {0x7c,0x66,0x66,0x7c,0x66,0x66,0x7c,0x00},
  {0x1c,0x30,0x60,0x60,0x60,0x30,0x1c,0x00},
  {0x78,0x6c,0x66,0x66,0x66,0x6c,0x78,0x00},
  {0x7e,0x60,0x60,0x7c,0x60,0x60,0x7e,0x00},
  {0x7e,0x60,0x60,0x7c,0x60,0x60,0x60,0x00},
  {0x3c,0x66,0x60,0x6e,0x66,0x66,0x3c,0x00},
  {0x66,0x66,0x66,0x7e,0x66,0x66,0x66,0x00},
  {0x7e,0x18,0x18,0x18,0x18,0x18,0x7e,0x00},
  {0x06,0x06,0x06,0x06,0x66,0x66,0x3c,0x00},
  {0x66,0x6c,0x78,0x70,0x78,0x6c,0x66,0x00},
  {0x60,0x60,0x60,0x60,0x60,0x60,0x7e,0x00},
  {0xc6,0xee,0xfe,0xd6,0xc6,0xc6,0xc6,0x00},
  {0x66,0x76,0x7e,0x7e,0x6e,0x66,0x66,0x00},
  {0x3c,0x66,0x66,0x66,0x66,0x66,0x3c,0x00},
  {0x7c,0x66,0x66,0x7c,0x60,0x60,0x60,0x00},
  {0x3c,0x66,0x66,0x66,0x6e,0x3c,0x07,0x00},
  {0x7c,0x66,0x66,0x7c,0x78,0x6c,0x66,0x00},
  {0x3c,0x66,0x60,0x3c,0x06,0x66,0x3c,0x00},
  {0x7e,0x18,0x18,0x18,0x18,0x18,0x18,0x00},
  {0x66,0x66,0x66,0x66,0x66,0x66,0x3c,0x00},
  {0x66,0x66,0x66,0x66,0x66,0x3c,0x18,0x00},
  {0xc6,0xc6,0xc6,0xd6,0xfe,0xee,0xc6,0x00},
  {0x66,0x66,0x3c,0x18,0x3c,0x66,0x66,0x00},
  {0x66,0x66,0x66,0x3c,0x18,0x18,0x18,0x00},
  {0x7e,0x06,0x0c,0x18,0x30,0x60,0x7e,0x00},
  {0x3c,0x30,0x30,0x30,0x30,0x30,0x3c,0x00},
  {0xc0,0x60,0x30,0x18,0x0c,0x06,0x02,0x00},
  {0x3c,0x0c,0x0c,0x0c,0x0c,0x0c,0x3c,0x00},
  {0x18,0x3c,0x66,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff},
  {0x18,0x18,0x0c,0x00,0x00,0x00,0x00,0x00},
  {0x00,0x00,0x3c,0x06,0x3e,0x66,0x3e,0x00},
  {0x60,0x60,0x7c,0x66,0x66,0x66,0x7c,0x00},
  {0x00,0x00,0x3c,0x60,0x60,0x60,0x3c,0x00},
  {0x06,0x06,0x3e,0x66,0x66,0x66,0x3e,0x00},
  {0x00,0x00,0x3c,0x66,0x7e,0x60,0x3c,0x00},
  {0x1c,0x30,0x7c,0x30,0x30,0x30,0x7e,0x00},
  {0x00,0x00,0x3e,0x66,0x66,0x3e,0x06,0x7c},
  {0x60,0x60,0x7c,0x66,0x66,0x66,0x66,0x00},
  {0x18,0x00,0x38,0x18,0x18,0x18,0x3c,0x00},
  {0x18,0x00,0x38,0x18,0x18,0x18,0x18,0x70},
  {0x60,0x60,0x66,0x6c,0x78,0x6c,0x66,0x00},
  {0x38,0x18,0x18,0x18,0x18,0x18,0x3c,0x00},
  {0x00,0x00,0xcc,0xfe,0xd6,0xc6,0xc6,0x00},
  {0x00,0x00,0x7c,0x66,0x66,0x66,0x66,0x00},
  {0x00,0x00,0x3c,0x66,0x66,0x66,0x3c,0x00},
  {0x00,0x00,0x7c,0x66,0x66,0x7c,0x60,0x60},
  {0x00,0x00,0x3e,0x66,0x66,0x3e,0x06,0x06},
  {0x00,0x00,0x7c,0x66,0x60,0x60,0x60,0x00},
  {0x00,0x00,0x3e,0x60,0x3c,0x06,0x7c,0x00},
  {0x30,0x30,0x7c,0x30,0x30,0x30,0x1c,0x00},
  {0x00,0x00,0x66,0x66,0x66,0x66,0x3e,0x00},
  {0x00,0x00,0x66,0x66,0x66,0x3c,0x18,0x00},
  {0x00,0x00,0xc6,0xc6,0xd6,0xfe,0x6c,0x00},
  {0x00,0x00,0x66,0x3c,0x18,0x3c,0x66,0x00},
  {0x00,0x00,0x66,0x66,0x66,0x3e,0x06,0x7c},
  {0x00,0x00,0x7e,0x0c,0x18,0x30,0x7e,0x00},
  {0x0e,0x18,0x18,0x70,0x18,0x18,0x0e,0x00},
  {0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00},
  {0x70,0x18,0x18,0x0e,0x18,0x18,0x70,0x00},
  {0x76,0xdc,0x00,0x00,0x00,0x00,0x00,0x00},
};

typedef struct {
  uint8_t r, g, b, a;
} color_t;

static const color_t COLOR_WHITE  = {255,255,255,255};
static const color_t COLOR_BLACK  = {0,0,0,255};
static const color_t COLOR_GREEN  = {0,255,0,255};
static const color_t COLOR_YELLOW = {255,255,0,255};
static const color_t COLOR_RED    = {255,0,0,255};
static const color_t COLOR_CYAN   = {0,255,255,255};
static const color_t COLOR_GRAY    = {128,128,128,255};
static const color_t COLOR_MAGENTA = {255,0,255,255};

/* Render state */
typedef struct {
  uint32_t pixels[W * H];
} canvas_t;

static void canvas_clear(canvas_t *c, uint8_t gray) {
  uint32_t v = 0xFF000000 | (gray << 16) | (gray << 8) | gray;
  for (int i = 0; i < W * H; i++)
    c->pixels[i] = v;
}

static void canvas_put_pixel(canvas_t *c, int x, int y, color_t col) {
  if (x < 0 || x >= W || y < 0 || y >= H) return;
  uint32_t a = col.a;
  uint32_t v = (a << 24) | (col.r << 16) | (col.g << 8) | col.b;
  if (a == 255) {
    c->pixels[y * W + x] = v;
  } else if (a > 0) {
    uint32_t bg = c->pixels[y * W + x];
    uint8_t br = (bg >> 16) & 0xFF, bg2 = (bg >> 8) & 0xFF, bb = bg & 0xFF;
    uint8_t r = (col.r * a + br * (255 - a)) / 255;
    uint8_t g = (col.g * a + bg2 * (255 - a)) / 255;
    uint8_t b = (col.b * a + bb * (255 - a)) / 255;
    c->pixels[y * W + x] = 0xFF000000 | (r << 16) | (g << 8) | b;
  }
}

static void canvas_draw_char(canvas_t *c, int x, int y, char ch, color_t fg, color_t bg) {
  if (ch < 32 || ch > 126) ch = '?';
  int idx = ch - 32;
  for (int row = 0; row < 8; row++) {
    uint8_t bits = font8x8[idx][row];
    for (int col = 0; col < 8; col++) {
      if (bits & (1 << (7 - col)))
        canvas_put_pixel(c, x + col, y + row, fg);
      else if (bg.a > 0)
        canvas_put_pixel(c, x + col, y + row, bg);
    }
  }
}

static void canvas_draw_string(canvas_t *c, int x, int y, const char *s,
                                color_t fg, color_t bg) {
  while (*s) {
    canvas_draw_char(c, x, y, *s, fg, bg);
    x += 8;
    s++;
  }
}

static void canvas_draw_line(canvas_t *c, int x0, int y0, int x1, int y1, color_t col) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  while (1) {
    canvas_put_pixel(c, x0, y0, col);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

static void canvas_draw_arrow(canvas_t *c, int x0, int y0, int x1, int y1, color_t col) {
  canvas_draw_line(c, x0, y0, x1, y1, col);
  if (x0 == x1 && y0 == y1) return;
  float dx = x1 - x0, dy = y1 - y0;
  float len = sqrtf(dx * dx + dy * dy);
  if (len < 1.0f) return;
  float ux = dx / len, uy = dy / len;
  float px = -uy * 3.0f, py = ux * 3.0f;
  int ax = (int)(x1 - ux * 6 + px + 0.5f);
  int ay = (int)(y1 - uy * 6 + py + 0.5f);
  int bx = (int)(x1 - ux * 6 - px + 0.5f);
  int by = (int)(y1 - uy * 6 - py + 0.5f);
  canvas_draw_line(c, x1, y1, ax, ay, col);
  canvas_draw_line(c, x1, y1, bx, by, col);
}

static void canvas_draw_circle(canvas_t *c, int cx, int cy, int r, color_t col) {
  int x = r, y = 0, err = 0;
  while (x >= y) {
    canvas_put_pixel(c, cx + x, cy + y, col);
    canvas_put_pixel(c, cx + y, cy + x, col);
    canvas_put_pixel(c, cx - y, cy + x, col);
    canvas_put_pixel(c, cx - x, cy + y, col);
    canvas_put_pixel(c, cx - x, cy - y, col);
    canvas_put_pixel(c, cx - y, cy - x, col);
    canvas_put_pixel(c, cx + y, cy - x, col);
    canvas_put_pixel(c, cx + x, cy - y, col);
    if (err <= 0) { y += 1; err += 2*y + 1; }
    if (err > 0) { x -= 1; err -= 2*x + 1; }
  }
}

static void canvas_draw_rect(canvas_t *c, int x, int y, int w, int h, color_t col) {
  canvas_draw_line(c, x, y, x + w, y, col);
  canvas_draw_line(c, x + w, y, x + w, y + h, col);
  canvas_draw_line(c, x + w, y + h, x, y + h, col);
  canvas_draw_line(c, x, y + h, x, y, col);
}

static void canvas_blit_grayscale(canvas_t *c, const uint8_t *gray, int stride) {
  for (int y = 0; y < H; y++) {
    for (int x = 0; x < W; x++) {
      uint8_t v = gray[y * stride + x];
      c->pixels[y * W + x] = 0xFF000000 | (v << 16) | (v << 8) | v;
    }
  }
}

/* Synthetic sequence generator */
#define ROT_BASE_W 1024
#define ROT_BASE_H 1024

typedef enum {
  SCENE_RANDOM = 0,
  SCENE_WALL,        /* centered rectangle expanding/contracting */
  SCENE_CORRIDOR,    /* perspective corridor with center line */
  SCENE_ASYMMETRY,   /* wall on left half, open on right */
} scene_type_t;

static const char *scene_names[] = {
  "random", "wall", "corridor", "asymmetry"
};

static scene_type_t scene_from_name(const char *name) {
  for (int i = 0; i < 4; i++)
    if (!strcmp(name, scene_names[i])) return (scene_type_t)i;
  return SCENE_RANDOM;
}

typedef struct {
  scene_type_t scene;
  uint8_t  base_tile[H][W];  /* small base for translation (repeating) */
  uint8_t *rot_base;          /* large base for rotation/descent (1024x1024) */
  bool     use_yaw;
  bool     use_descent;
  int frame;
  int flow_x;     /* pixels/frame * SUBP (translation mode) */
  int flow_y;
  float yaw_per_frame;   /* deg/frame (rotation mode) */
  float descent_rate;    /* scale change per frame (+ = descent, - = ascent) */
  bool show_pad;
  uint16_t pad_cx, pad_cy, pad_size;
} seq_state_t;

static void seq_init(seq_state_t *s, scene_type_t scene, int flow_x, int flow_y, float yaw, float descent, bool pad) {
  s->scene = scene;
  s->frame = 0;
  s->flow_x = flow_x;
  s->flow_y = flow_y;
  s->yaw_per_frame = yaw;
  s->use_yaw = (fabsf(yaw) > 0.001f);
  s->descent_rate = descent;
  s->use_descent = (fabsf(descent) > 1e-8f);
  s->show_pad = pad;
  s->pad_cx = 160;
  s->pad_cy = 120;
  s->pad_size = 40;

  /* Small repeating base (320x240) for translation mode */
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      s->base_tile[y][x] = 160 + (rand() % 64);

  /* Large base (1024x1024) for rotation/descent mode */
  s->rot_base = (uint8_t*)malloc(ROT_BASE_W * ROT_BASE_H);
  if (!s->rot_base) return;
  for (int y = 0; y < ROT_BASE_H; y++)
    for (int x = 0; x < ROT_BASE_W; x++)
      s->rot_base[y * ROT_BASE_W + x] = 160 + (rand() % 64);
}

static void seq_gen_frame(seq_state_t *s, vision_image_t *img, int frame) {
  s->frame = frame;

  /* Always render base textured image first (coherent motion from rot_base) */
  if (s->use_yaw || s->use_descent) {
    /* Rotation/descent mode: bilinear interpolate from large base */
    float angle = frame * s->yaw_per_frame * 3.14159265f / 180.0f;
    float cos_a = cosf(angle), sin_a = sinf(angle);
    float scale = 1.0f + frame * s->descent_rate;
    if (scale < 0.1f) scale = 0.1f;
    float cx = ROT_BASE_W / 2.0f, cy = ROT_BASE_H / 2.0f;

    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        /* Back-project with scale + rotation */
        float ox = (x - W/2.0f) / scale;
        float oy = (y - H/2.0f) / scale;
        float sx = cx + ox * cos_a - oy * sin_a;
        float sy = cy + ox * sin_a + oy * cos_a;

        int ix = (int)sx, iy = (int)sy;
        if (ix < 0 || ix >= ROT_BASE_W - 1 || iy < 0 || iy >= ROT_BASE_H - 1) {
          img->buf[y * img->stride + x] = 80;  /* border fill */
          continue;
        }
        float fx = sx - ix, fy = sy - iy;
        uint8_t *p = &s->rot_base[iy * ROT_BASE_W + ix];
        float v = (1.0f - fy) * ((1.0f - fx) * p[0] + fx * p[1])
                + fy * ((1.0f - fx) * p[ROT_BASE_W] + fx * p[ROT_BASE_W + 1]);
        img->buf[y * img->stride + x] = (uint8_t)v;
      }
    }
  } else {
    /* Translation mode: shift base with wrap-around */
    int dx = s->flow_x * frame / SUBP;
    int dy = s->flow_y * frame / SUBP;

    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        int sx = (x + dx) % W;
        int sy = (y + dy) % H;
        if (sx < 0) sx += W;
        if (sy < 0) sy += H;
        uint8_t v = s->base_tile[sy][sx];
        img->buf[y * img->stride + x] = v;
      }
    }
  }

  /* Scene overlays: modify the base textured image with structure */
  if (s->scene == SCENE_WALL) {
    float scale = 1.0f + frame * s->descent_rate;
    if (scale < 0.1f) scale = 0.1f;
    float wall_w = W * 0.5f / scale;
    float wall_h = H * 0.5f / scale;
    int wl = (int)((W - wall_w) / 2), wt = (int)((H - wall_h) / 2);
    int wr = (int)((W + wall_w) / 2), wb = (int)((H + wall_h) / 2);
    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        uint8_t v = img->buf[y * img->stride + x];
        bool inside = (x >= wl && x < wr && y >= wt && y < wb);
        img->buf[y * img->stride + x] = inside ? (128 + v/2) : (v/3);
      }
    }
  }

  if (s->scene == SCENE_CORRIDOR) {
    float dx = s->flow_x * frame / (float)SUBP;
    float dx_shift = fmodf(dx * 0.5f + W, W);
    int line_x = W / 2 + (int)dx_shift;
    for (int y = 0; y < H; y++) {
      float ny = (float)y / H;
      int corridor_w = (int)(W * (0.3f + 0.3f * ny));
      int cl = (W - corridor_w) / 2; if (cl < 0) cl = 0;
      int cr = (W + corridor_w) / 2; if (cr > W) cr = W;
      for (int x = 0; x < W; x++) {
        uint8_t v = img->buf[y * img->stride + x];
        if (x >= cl && x < cr)
          img->buf[y * img->stride + x] = 180 + v/4;  /* bright path */
        else
          img->buf[y * img->stride + x] = v/4;        /* dark wall */
      }
    }
    int vl = line_x;
    if (vl >= 0 && vl < W)
      for (int y = 0; y < H; y++)
        img->buf[y * img->stride + vl] = 255;
  }

  if (s->scene == SCENE_ASYMMETRY) {
    float scale = 1.0f + frame * s->descent_rate;
    if (scale < 0.1f) scale = 0.1f;
    int wall_divider = (int)(W * 0.5f / scale);
    if (wall_divider < 10) wall_divider = 10;
    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        uint8_t v = img->buf[y * img->stride + x];
        if (x < wall_divider)
          img->buf[y * img->stride + x] = 160 + v/2;  /* wall texture */
        else
          img->buf[y * img->stride + x] = v/3;        /* open space */
      }
    }
  }

  /* Landing pad overlay (always visible, same coordinates in output) */
  if (s->show_pad) {
    for (int y = 0; y < H; y++) {
      for (int x = 0; x < W; x++) {
        int16_t dx = (int16_t)x - (int16_t)s->pad_cx;
        int16_t dy = (int16_t)y - (int16_t)s->pad_cy;
        if (dx*dx + dy*dy <= (int32_t)(s->pad_size/2)*(s->pad_size/2))
          img->buf[y * img->stride + x] = 30;
      }
    }
  }
}

typedef struct {
  bool stage1 : 1;
  bool stage2 : 1;
  bool stage3 : 1;
  bool stage4 : 1;
  bool lines  : 1;
} overlay_flags_t;

int main(int argc, char **argv) {
  int flow_x = 20, flow_y = 10;
  float yaw_per_frame = 0.0f;
  float descent_rate = 0.0f;
  float approach_rate = 0.0f;
  bool show_pad = true;
  bool use_obstacle = false;
  bool use_lines = false;
  scene_type_t scene = SCENE_RANDOM;
  int start_frame = 0;

  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
      printf("Usage: visual_sim [options]\n");
      printf("  --flow-x <px>    Translation flow X (subpixel*10, default 20)\n");
      printf("  --flow-y <px>    Translation flow Y (subpixel*10, default 10)\n");
      printf("  --yaw <deg/f>    Rotation (degrees/frame, default 0 = translation)\n");
      printf("  --descent <r>    Scale change/frame (+ = descent/expand, default 0)\n");
      printf("  --approach <r>   Scale change/frame for looming (alias for --descent)\n");
      printf("  --obstacle       Enable obstacle detection HUD\n");
      printf("  --lines          Enable line/curve detection HUD\n");
      printf("  --scene <name>   Scene: random, wall, corridor, asymmetry (default random)\n");
      printf("  --no-pad         Disable landing pad overlay\n");
      printf("  --frame <n>      Start frame (default 0)\n");
      return 0;
    }
    if (!strcmp(argv[i], "--flow-x") && i+1 < argc) flow_x = atoi(argv[++i]);
    if (!strcmp(argv[i], "--flow-y") && i+1 < argc) flow_y = atoi(argv[++i]);
    if (!strcmp(argv[i], "--yaw") && i+1 < argc) yaw_per_frame = atof(argv[++i]);
    if (!strcmp(argv[i], "--descent") && i+1 < argc) descent_rate = atof(argv[++i]);
    if (!strcmp(argv[i], "--approach") && i+1 < argc) approach_rate = atof(argv[++i]);
    if (!strcmp(argv[i], "--obstacle")) use_obstacle = true;
    if (!strcmp(argv[i], "--lines")) use_lines = true;
    if (!strcmp(argv[i], "--scene") && i+1 < argc) scene = scene_from_name(argv[++i]);
    if (!strcmp(argv[i], "--no-pad")) show_pad = false;
    if (!strcmp(argv[i], "--frame") && i+1 < argc) start_frame = atoi(argv[++i]);
  }

  if (approach_rate != 0.0f && descent_rate == 0.0f)
    descent_rate = approach_rate;

  srand(time(NULL));

  /* Initialize vision pipeline */
  flow_stage1_config_t s1cfg;
  s1cfg.tile_size = 8;
  s1cfg.search_range = 6;
  s1cfg.image_width = W;
  s1cfg.image_height = H;
  s1cfg.subsample = 1;
  s1cfg.min_quality = 0;

  flow_stage2_config_t s2cfg;
  s2cfg.max_corners = 64;
  s2cfg.fast_threshold = 30;
  s2cfg.fast_min_distance = 8;
  s2cfg.lk_window_size = 5;
  s2cfg.lk_pyramid_levels = 2;
  s2cfg.lk_max_iterations = 10;
  s2cfg.lk_subpixel_factor = 10;

  flow_stage1_t *s1 = flow_stage1_create(&s1cfg);
  flow_stage2_t *s2 = flow_stage2_create(&s2cfg, W, H);
  rotation_ctx_t *rot = rotation_create(W, H);
  pattern_ctx_t *pat = pattern_create(W, H);
  obstacle_t *obs = use_obstacle ? obstacle_create(W, H) : NULL;

  vision_image_t frame;
  image_create(&frame, W, H, VISION_IMAGE_GRAYSCALE);

  seq_state_t seq;
  seq_init(&seq, scene, flow_x, flow_y, yaw_per_frame, descent_rate, show_pad);

  /* SDL2 init */
  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
    return 1;
  }

  SDL_Window *win = SDL_CreateWindow("Parrot Vision Pipeline Simulator",
    SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, WIN_W, WIN_H,
    SDL_WINDOW_SHOWN | SDL_WINDOW_ALWAYS_ON_TOP);
  if (!win) { fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError()); return 1; }

  SDL_Renderer *ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
  SDL_Texture *tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_ARGB8888,
    SDL_TEXTUREACCESS_STREAMING, W, H);

  canvas_t cv;
  vision_result_t result;
  overlay_flags_t ov = {true, true, true, true, false};
  bool playing = true;
  int frame_num = start_frame;
  uint32_t fps = 30;
  uint32_t tick = SDL_GetTicks();
  bool running = true;

  /* Sparse grid for rotation estimation (~200 tiles across full image) */
  #define ROT_TILES 256
  uint16_t rot_px[ROT_TILES], rot_py[ROT_TILES];   /* tile top-left */
  uint16_t rot_cx[ROT_TILES], rot_cy[ROT_TILES];   /* tile centers */
  int16_t  rot_dx[ROT_TILES], rot_dy[ROT_TILES];
  uint32_t rot_sad[ROT_TILES];
  int32_t  rot_fx[ROT_TILES], rot_fy[ROT_TILES];
  uint16_t n_rot = 0;
  {
    uint8_t ts = 8, sr = 6;
    /* step=24 gives ~12x9=108 tiles; step=16 gives ~17x13=221 tiles. Use step=16. */
    int step = 16, x0 = sr + 20, y0 = sr + 20;
    for (uint16_t y = y0; y + ts + sr + 20 <= H; y += step)
      for (uint16_t x = x0; x + ts + sr + 20 <= W; x += step)
        if (n_rot < ROT_TILES) {
          rot_px[n_rot] = x;
          rot_py[n_rot] = y;
          rot_cx[n_rot] = x + ts/2;
          rot_cy[n_rot] = y + ts/2;
          n_rot++;
        }
  }

  /* Previous frame for rotation SAD (separate from flow_stage1's internal prev) */
  vision_image_t rot_prev;
  image_create(&rot_prev, W, H, VISION_IMAGE_GRAYSCALE);
  seq_gen_frame(&seq, &rot_prev, 0);

  while (running) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT) running = false;
      else if (e.type == SDL_KEYDOWN) {
        switch (e.key.keysym.sym) {
          case SDLK_ESCAPE: case SDLK_q: running = false; break;
          case SDLK_SPACE: playing = !playing; break;
          case SDLK_RIGHT: if (frame_num < 10000) frame_num++; break;
          case SDLK_LEFT: if (frame_num > 0) frame_num--; break;
          case SDLK_r: frame_num = 0; rotation_reset(rot); break;
          case SDLK_1: ov.stage1 = !ov.stage1; break;
          case SDLK_2: ov.stage2 = !ov.stage2; break;
          case SDLK_3: ov.stage3 = !ov.stage3; break;
          case SDLK_4: ov.stage4 = !ov.stage4; break;
          case SDLK_5: ov.lines = !ov.lines; break;
          case SDLK_h: pattern_snapshot_home(pat, &frame); break;
          default: break;
        }
      }
    }

    if (playing) {
      uint32_t now = SDL_GetTicks();
      if (now - tick >= 1000u / fps) {
        frame_num++;
        tick = now;
      }
    }
    if (frame_num < 0) frame_num = 0;

    /* Generate current frame */
    seq_gen_frame(&seq, &frame, frame_num);

    /* === STAGE 1: exact library code as on drone === */
    memset(&result, 0, sizeof(result));
    flow_stage1_process(s1, &frame, &result);

    /* === STAGE 3: rotation from sparse wide-grid SAD === */
    if (frame_num > 0) {
      image_sad_block_many(&frame, &rot_prev, rot_px, rot_py,
                            rot_dx, rot_dy, rot_sad,
                            n_rot, 8, 6);

      for (uint16_t i = 0; i < n_rot; i++) {
        rot_fx[i] = (int32_t)rot_dx[i] * SUBP;
        rot_fy[i] = (int32_t)rot_dy[i] * SUBP;
      }
    } else {
      for (uint16_t i = 0; i < n_rot; i++) {
        rot_dx[i] = 0; rot_dy[i] = 0;
        rot_fx[i] = 0; rot_fy[i] = 0;
        rot_sad[i] = 0;
      }
    }

    uint8_t rot_qual = 0;
    int32_t yaw_mdeg = 0;
    if (frame_num > 0 && n_rot > 1) {
      yaw_mdeg = rotation_estimate(rot, rot_fx, rot_fy,
                                    rot_cx, rot_cy, n_rot, &rot_qual);
    }

    /* Stage 2: multi-scale SAD (every 5 frames) */
    static float alt_est = 300.0f;
    static int32_t s2_divergence = 0;
    static uint8_t s2_corners = 0;
    static int s2_last_frame = -1;

    if (frame_num % 5 == 0 && frame_num != s2_last_frame) {
      s2_last_frame = frame_num;
      flow_stage2_process(s2, &frame, &result);
      s2_divergence = result.divergence;
      s2_corners = result.corner_cnt;

      /* Altitude from divergence (pinhole: z ∝ 1/scale, divergence = -Δscale/scale * 1000).
       * divergence is measured over the 5-frame interval. */
      if (s2_corners > 0 && abs(s2_divergence) > 2) {
        float exp_ratio = -(float)s2_divergence / 1000.0f;  /* expansion over 5 frames */
        alt_est = alt_est / (1.0f + exp_ratio);
        if (alt_est < 10.0f) alt_est = 10.0f;
        if (alt_est > 1000.0f) alt_est = 1000.0f;
      }
    }

    /* Stage 4: landing pad detection */
    landing_pad_t pad = {false};
    home_marker_t home_mk = {false};
    if (ov.stage4) {
      pattern_detect_landing_pad(pat, &frame, &pad);
      pattern_detect_home(pat, &frame, &home_mk);
    }

    /* === OBSTACLE DETECTION === */
    obstacle_result_t obst_result;
    if (use_obstacle && frame_num > 0) {
      obstacle_process(obs, &frame, &rot_prev, &obst_result);
    } else if (use_obstacle) {
      memset(&obst_result, 0, sizeof(obst_result));
    }

    /* === LINE DETECTION === */
    line_result_t line_result;
    line_config_t *line_cfg = NULL;
    if (use_lines) {
      line_cfg = line_default_config();
      line_cfg->hough_threshold = 30;
      line_cfg->hough_min_line_len = 20;
      line_cfg->sobel_threshold = 25;
      line_detect(&frame, line_cfg, &line_result);
    }

    /* === RENDER === */
    canvas_clear(&cv, 0);
    canvas_blit_grayscale(&cv, frame.buf, frame.stride);

    /* Stage 1 per-tile display (from flow_stage1_get_tile_flows) */
    if (ov.stage1) {
      const uint16_t *s1_pos_x, *s1_pos_y;
      const int16_t *s1_fx, *s1_fy;
      const uint32_t *s1_sad;
      uint16_t n_s1 = flow_stage1_get_tile_flows(s1,
          &s1_pos_x, &s1_pos_y, &s1_fx, &s1_fy, &s1_sad);

      int skip = n_s1 > 32 ? n_s1 / 32 : 1;
      for (uint16_t i = 0; i < n_s1; i += skip) {
        int px = s1_pos_x[i] + 4, py = s1_pos_y[i] + 4;
        int fx = s1_fx[i] / SUBP, fy = s1_fy[i] / SUBP;
        if (fx == 0 && fy == 0) {
          canvas_put_pixel(&cv, px, py, COLOR_GREEN);
          continue;
        }
        int ex = px + fx, ey = py + fy;
        if (ex < 0 || ex >= W) ex = px + (fx > 0 ? W-1-px : -px);
        if (ey < 0 || ey >= H) ey = py + (fy > 0 ? H-1-py : -py);
        uint32_t err = s1_sad[i];
        color_t col = err < 500 ? COLOR_GREEN :
                      err < 2000 ? COLOR_YELLOW : COLOR_RED;
        canvas_draw_arrow(&cv, px, py, ex, ey, col);
      }

      /* Overall flow arrow at center-bottom */
      int afx = result.flow_x_fast / SUBP;
      int afy = result.flow_y_fast / SUBP;
      if (afx != 0 || afy != 0) {
        canvas_draw_arrow(&cv, W/2, H - 40, W/2 + afx, H - 40 + afy, COLOR_WHITE);
      }
    }

    /* Stage 2: robust flow */
    if (ov.stage2 && result.corner_cnt > 0) {
      color_t col = result.quality_robust > 200 ? COLOR_CYAN :
                    result.quality_robust > 100 ? COLOR_YELLOW : COLOR_RED;
      int fx = result.flow_x_robust / SUBP;
      int fy = result.flow_y_robust / SUBP;
      canvas_draw_arrow(&cv, W/2, H/2, W/2 + fx, H/2 + fy, col);
    }

    /* Stage 3: rotation display */
    if (ov.stage3) {
      static int32_t cumulative_yaw = 0;
      cumulative_yaw += yaw_mdeg;

      char buf[64];
      snprintf(buf, 64, "Yaw: %+4d mdeg/f  qual=%u", yaw_mdeg, rot_qual);
      canvas_draw_string(&cv, W - 164, 4, buf, COLOR_CYAN, COLOR_BLACK);

      /* Horizontal balance bar */
      int bar_cx = W - 80, bar_cy = H - 20;
      int bar_half = 32, bar_w = 4;
      color_t bar_col = abs(yaw_mdeg) < 300 ? COLOR_GREEN :
                        abs(yaw_mdeg) < 1500 ? COLOR_YELLOW : COLOR_RED;
      canvas_draw_line(&cv, bar_cx, bar_cy - 4, bar_cx, bar_cy + 4, COLOR_CYAN);
      int bar_len = yaw_mdeg / 20;
      if (bar_len > bar_half) bar_len = bar_half;
      if (bar_len < -bar_half) bar_len = -bar_half;
      if (bar_len > 0) {
        for (int w = 0; w < bar_w; w++)
          canvas_draw_line(&cv, bar_cx, bar_cy - bar_w + w,
                                bar_cx + bar_len, bar_cy - bar_w + w, bar_col);
      } else if (bar_len < 0) {
        for (int w = 0; w < bar_w; w++)
          canvas_draw_line(&cv, bar_cx + bar_len, bar_cy - bar_w + w,
                                bar_cx, bar_cy - bar_w + w, bar_col);
      }

      /* Cumulative yaw needle */
      int needle_cx = W - 16, needle_cy = H - 20, needle_r = 8;
      canvas_draw_circle(&cv, needle_cx, needle_cy, needle_r, COLOR_CYAN);
      float cum_rad = cumulative_yaw * 3.14159f / 180000.0f;
      int nx = needle_cx + (int)(sinf(cum_rad) * needle_r);
      int ny = needle_cy - (int)(cosf(cum_rad) * needle_r);
      canvas_draw_line(&cv, needle_cx, needle_cy, nx, ny, COLOR_CYAN);
      int nty = needle_cy - needle_r - 2;
      canvas_draw_line(&cv, needle_cx - 2, nty, needle_cx + 2, nty, COLOR_CYAN);
    }

    /* Stage 4: landing pad overlay */
    if (ov.stage4 && pad.found) {
      canvas_draw_circle(&cv, pad.center_x, pad.center_y, pad.size/2, COLOR_GREEN);
      canvas_draw_circle(&cv, pad.center_x, pad.center_y, 3, COLOR_RED);
      canvas_draw_rect(&cv, pad.center_x - pad.size/2, pad.center_y - pad.size/2,
                        pad.size, pad.size, COLOR_YELLOW);
      char buf[64];
      snprintf(buf, 64, "PAD(%3u,%3u) sz=%u conf=%u dist=%.0fcm",
               pad.center_x, pad.center_y, pad.size, pad.confidence,
               pad.distance_est);
      canvas_draw_string(&cv, 4, H - 24, buf, COLOR_GREEN, COLOR_BLACK);
    }

    /* Home marker overlay */
    if (ov.stage4 && home_mk.found) {
      float arad = home_mk.angle * 3.14159265f / 18000.0f;
      int sx = W/2, sy = H/2;
      int len = 40;
      int ex = sx + (int)(sinf(arad) * len);
      int ey = sy - (int)(cosf(arad) * len);
      canvas_draw_arrow(&cv, sx, sy, ex, ey, COLOR_MAGENTA);
      char buf[64];
      snprintf(buf, 64, "HOME %dcm dir=%+ddeg conf=%u",
               home_mk.distance, home_mk.angle / 100, home_mk.confidence);
      canvas_draw_string(&cv, 4, H - 12, buf, COLOR_MAGENTA, COLOR_BLACK);
    }

    /* Obstacle HUD overlay */
    if (use_obstacle) {
      /* Vertical line overlay */
      if (obst_result.line_found) {
        canvas_draw_line(&cv, obst_result.line_x, 0,
                          obst_result.line_x, H - 1, COLOR_YELLOW);
      }

      /* Looming bar (top center) */
      int bar_w = 100, bar_h = 8;
      int bar_x = (W - bar_w) / 2;
      int bar_y = 0;
      int filled = (obst_result.looming * bar_w) / 255;
      color_t loom_col = obst_result.looming < 80 ? COLOR_GREEN :
                          obst_result.looming < 160 ? COLOR_YELLOW : COLOR_RED;
      canvas_draw_rect(&cv, bar_x, bar_y, bar_w, bar_h, COLOR_GRAY);
      for (int i = 0; i < filled && i < bar_w; i++)
        canvas_draw_line(&cv, bar_x + i, bar_y, bar_x + i, bar_y + bar_h - 1, loom_col);

      /* Asymmetry indicator (small triangle/arrow near top-center, below bar) */
      int asym_cx = W / 2, asym_cy = bar_y + bar_h + 6;
      int asym_mag = abs(obst_result.asymmetry);
      if (asym_mag > 0) {
        int asym_len = (asym_mag * 20) / 128;
        if (asym_len < 2) asym_len = 2;
        int dir = obst_result.asymmetry < 0 ? -1 : 1;  /* negative = obstacle on left */
        color_t asym_col = asym_mag < 40 ? COLOR_YELLOW : COLOR_RED;
        int ax = asym_cx + dir * asym_len;
        canvas_draw_arrow(&cv, asym_cx, asym_cy, ax, asym_cy, asym_col);
        canvas_put_pixel(&cv, asym_cx, asym_cy, COLOR_WHITE);
      }

      /* Overlay text */
      char buf2[80];
      snprintf(buf2, sizeof(buf2), "LOOM=%u ASYM=%+d CONF=%u",
               obst_result.looming, obst_result.asymmetry, obst_result.confidence);
      canvas_draw_string(&cv, (W - 8 * 24) / 2, bar_y + bar_h + 12,
                          buf2, COLOR_WHITE, COLOR_BLACK);

      if (obst_result.line_found) {
        snprintf(buf2, sizeof(buf2), "LINE x=%u a=%+d str=%u",
                 obst_result.line_x, obst_result.line_angle, obst_result.line_strength);
        canvas_draw_string(&cv, 4, H - 60, buf2, COLOR_YELLOW, COLOR_BLACK);
      }
    }

    /* Line detection overlay */
    if (use_lines && ov.lines && frame_num > 0) {
      /* Draw detected lines */
      for (uint8_t i = 0; i < line_result.num_lines; i++) {
        const detected_line_t *l = &line_result.lines[i];
        /* Color by length: long=green, medium=yellow, short=red */
        color_t col = l->length > 80 ? COLOR_GREEN :
                      l->length > 40 ? COLOR_YELLOW : COLOR_RED;
        canvas_draw_line(&cv, l->x1, l->y1, l->x2, l->y2, col);

        /* Draw theta/rho label at midpoint */
        int mx = (l->x1 + l->x2) / 2;
        int my = (l->y1 + l->y2) / 2;
        if (i < 5) { /* limit labels to avoid clutter */
          char buf[48];
          snprintf(buf, sizeof(buf), "%d", l->votes);
          canvas_draw_string(&cv, mx + 2, my - 8, buf, col, COLOR_BLACK);
        }
      }

      /* Draw detected circles */
      for (uint8_t i = 0; i < line_result.num_circles; i++) {
        const detected_circle_t *c = &line_result.circles[i];
        canvas_draw_circle(&cv, (int)c->cx, (int)c->cy, (int)c->radius, COLOR_MAGENTA);
        canvas_draw_circle(&cv, (int)c->cx, (int)c->cy, 2, COLOR_MAGENTA);
      }

      /* Lane info */
      if (line_result.lane_valid) {
        /* Draw lane center line */
        canvas_draw_line(&cv, line_result.lane_center_x, 0,
                          line_result.lane_center_x, H - 1, COLOR_CYAN);
        char buf[80];
        snprintf(buf, sizeof(buf), "LANE off=%+d head=%.1fdeg",
                 line_result.lane_offset_x,
                 line_result.lane_heading * 180.0f / 3.14159f);
        canvas_draw_string(&cv, 4, H - 48, buf, COLOR_CYAN, COLOR_BLACK);
      }

      /* Line count + edge toggle info */
      {
        char buf[80];
        snprintf(buf, sizeof(buf), "LINES=%u CIRC=%u",
                 line_result.num_lines, line_result.num_circles);
        canvas_draw_string(&cv, W - 8*16, H - 36, buf, COLOR_WHITE, COLOR_BLACK);
      }

      /* Edge image overlay (toggle with 'e') */
      if (line_result.edge_image) {
        /* Draw edge pixels as dim green overlay */
        for (int y = 0; y < H; y++) {
          for (int x = 0; x < W; x++) {
            if (line_result.edge_image->buf[y * W + x] > 0) {
              canvas_put_pixel(&cv, x, y, (color_t){0, 80, 0, 80});
            }
          }
        }
        free(line_result.edge_image->buf);
        free(line_result.edge_image);
      }
    }

    /* HUD */
    {
      char buf[256];
      snprintf(buf, sizeof(buf),
               "Frame %d  Flow(%+d,%+d) qual=%u  alt=%.0fcm  %s%s%s%s%s",
               frame_num,
               result.flow_x_fast / SUBP, result.flow_y_fast / SUBP,
               result.quality_fast, (double)alt_est,
               ov.stage1 ? "[1] " : "",
               ov.stage2 ? "[2] " : "",
               ov.stage3 ? "[3] " : "",
               use_obstacle ? "[O] " : "",
               use_lines ? "[L] " : "");
      canvas_draw_string(&cv, 4, 4, buf, COLOR_WHITE, COLOR_BLACK);

      if (seq.use_yaw) {
        snprintf(buf, sizeof(buf), "  Yaw=%.1f deg/f  S2(%+d,%+d) div=%d  pad=%s  home=%s",
                 (double)seq.yaw_per_frame,
                 result.flow_x_robust / SUBP, result.flow_y_robust / SUBP,
                 result.divergence,
                 pad.found ? "YES" : "no",
                 home_mk.found ? "YES" : "no");
      } else if (seq.use_descent) {
        snprintf(buf, sizeof(buf), "  Descent=%.5f/f  S2(%+d,%+d) div=%d  pad=%s  home=%s",
                 (double)seq.descent_rate,
                 result.flow_x_robust / SUBP, result.flow_y_robust / SUBP,
                 result.divergence,
                 pad.found ? "YES" : "no",
                 home_mk.found ? "YES" : "no");
      } else {
        snprintf(buf, sizeof(buf), "  S2(%+d,%+d) div=%d  pad=%s home=%s gt=(%+d,%+d)",
                 result.flow_x_robust / SUBP, result.flow_y_robust / SUBP,
                 result.divergence,
                 pad.found ? "YES" : "no",
                 home_mk.found ? "YES" : "no",
                 flow_x / SUBP, flow_y / SUBP);
      }
      canvas_draw_string(&cv, 4, 12, buf, COLOR_WHITE, COLOR_BLACK);
    }

    canvas_draw_string(&cv, 4, H - 36,
      "SPC:play R:reset 1-5:toggle <-/->:step h:home", COLOR_GRAY, COLOR_BLACK);

    SDL_UpdateTexture(tex, NULL, cv.pixels, W * 4);
    SDL_RenderClear(ren);
    SDL_Rect dst = {0, 0, WIN_W, WIN_H};
    SDL_RenderCopy(ren, tex, NULL, &dst);
    SDL_RenderPresent(ren);

    /* Store current frame as rotation SAD previous */
    image_copy(&frame, &rot_prev);

    if (!playing) SDL_Delay(16);
  }

  /* Cleanup */
  flow_stage1_destroy(s1);
  flow_stage2_destroy(s2);
  rotation_destroy(rot);
  pattern_destroy(pat);
  obstacle_destroy(obs);
  free(seq.rot_base);
  image_destroy(&frame);
  image_destroy(&rot_prev);
  SDL_DestroyTexture(tex);
  SDL_DestroyRenderer(ren);
  SDL_DestroyWindow(win);
  SDL_Quit();
  return 0;
}
