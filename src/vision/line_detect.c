#include "line_detect.h"
#include "image.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

line_config_t* line_default_config(void) {
  static line_config_t cfg = {
    .sobel_threshold = 30,
    .canny_low = 0,
    .canny_high = 0,
    .suppress_radius = 2,
    .hough_rho = 1,
    .hough_theta_deg = 1,
    .hough_threshold = 40,
    .hough_min_line_len = 30,
    .hough_max_line_gap = 10,
    .circle_threshold = 30,
    .circle_min_radius = 10,
    .circle_max_radius = 100,
    .roi_x = 0, .roi_y = 0,
    .roi_w = 0, .roi_h = 0,
    .draw_edges = false,
    .draw_lines = false,
  };
  return &cfg;
}

void line_config_set_roi(line_config_t *cfg, uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  cfg->roi_x = x;
  cfg->roi_y = y;
  cfg->roi_w = w;
  cfg->roi_h = h;
}

/* ================================================================== */
/*  Sobel edge detection                                              */
/* ================================================================== */

void line_sobel(const vision_image_t *img, vision_image_t *magnitude, vision_image_t *direction) {
  uint16_t w = img->w, h = img->h;
  uint16_t mw = magnitude->w, mh = magnitude->h;
  (void)mh;

  memset(magnitude->buf, 0, mw * magnitude->stride);
  if (direction) memset(direction->buf, 0, mw * direction->stride);

  for (uint16_t y = 1; y < h - 1; y++) {
    for (uint16_t x = 1; x < w - 1; x++) {
      /* Sobel kernels */
      int gx = -image_get_pixel(img, x-1, y-1) - 2*image_get_pixel(img, x-1, y) - image_get_pixel(img, x-1, y+1)
               +image_get_pixel(img, x+1, y-1) + 2*image_get_pixel(img, x+1, y) + image_get_pixel(img, x+1, y+1);
      int gy = -image_get_pixel(img, x-1, y-1) - 2*image_get_pixel(img, x, y-1) - image_get_pixel(img, x+1, y-1)
               +image_get_pixel(img, x-1, y+1) + 2*image_get_pixel(img, x, y+1) + image_get_pixel(img, x+1, y+1);

      float mag = sqrtf((float)(gx*gx + gy*gy));
      uint8_t val = (mag > 255.0f) ? 255 : (uint8_t)mag;
      magnitude->buf[y * mw + x] = val;

      if (direction) {
        float angle = atan2f((float)gy, (float)gx);
        if (angle < 0) angle += (float)M_PI;
        direction->buf[y * mw + x] = (uint8_t)(angle * 255.0f / (float)M_PI);
      }
    }
  }
}

/* ================================================================== */
/*  Non-max suppression                                               */
/* ================================================================== */

void line_nonmax_suppress(const vision_image_t *magnitude, vision_image_t *suppressed,
                           uint8_t radius) {
  uint16_t w = magnitude->w, h = magnitude->h;
  memset(suppressed->buf, 0, suppressed->w * suppressed->stride);

  for (uint16_t y = radius; y < h - radius; y++) {
    for (uint16_t x = radius; x < w - radius; x++) {
      uint8_t val = magnitude->buf[y * w + x];
      if (val == 0) continue;

      bool is_max = true;
      for (int dy = -radius; dy <= radius && is_max; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
          if (dx == 0 && dy == 0) continue;
          if (magnitude->buf[(y+dy)*w + (x+dx)] > val) {
            is_max = false;
            break;
          }
        }
      }
      if (is_max) suppressed->buf[y * w + x] = val;
    }
  }
}

/* ================================================================== */
/*  Canny edge detection                                              */
/* ================================================================== */

void line_canny(const vision_image_t *img, uint8_t low, uint8_t high, vision_image_t *edges) {
  uint16_t w = img->w, h = img->h;

  /* Temporary buffers for magnitude and direction */
  vision_image_t mag, dir_tmp;
  mag.type = VISION_IMAGE_GRAYSCALE;
  mag.w = w; mag.h = h; mag.stride = w; mag.bytes_per_pixel = 1;
  mag.buf = (uint8_t*)malloc(w * h);
  dir_tmp.type = VISION_IMAGE_GRAYSCALE;
  dir_tmp.w = w; dir_tmp.h = h; dir_tmp.stride = w; dir_tmp.bytes_per_pixel = 1;
  dir_tmp.buf = (uint8_t*)malloc(w * h);

  line_sobel(img, &mag, &dir_tmp);

  /* Non-max suppression */
  vision_image_t nms;
  nms.type = VISION_IMAGE_GRAYSCALE;
  nms.w = w; nms.h = h; nms.stride = w; nms.bytes_per_pixel = 1;
  nms.buf = (uint8_t*)malloc(w * h);
  line_nonmax_suppress(&mag, &nms, 1);

  /* Double threshold + hysteresis */
  memset(edges->buf, 0, w * h);
  uint8_t *visited = (uint8_t*)calloc(w * h, 1);

  for (uint16_t y = 1; y < h - 1; y++) {
    for (uint16_t x = 1; x < w - 1; x++) {
      uint8_t val = nms.buf[y * w + x];
      if (val >= high && !visited[y*w+x]) {
        /* BFS hysteresis */
        uint16_t stack[1024];
        int sp = 0;
        stack[sp++] = y * w + x;
        visited[y*w+x] = 1;
        edges->buf[y*w+x] = 255;

        while (sp > 0) {
          int idx = stack[--sp];
          int cy = idx / w, cx = idx % w;
          for (int dy = -1; dy <= 1; dy++) {
            for (int dx = -1; dx <= 1; dx++) {
              int nx = cx + dx, ny = cy + dy;
              if (nx >= 0 && nx < (int)w && ny >= 0 && ny < (int)h) {
                uint8_t nval = nms.buf[ny*w+nx];
                if (nval >= low && !visited[ny*w+nx]) {
                  visited[ny*w+nx] = 1;
                  edges->buf[ny*w+nx] = 255;
                  if (sp < 1023) stack[sp++] = ny*w+nx;
                }
              }
            }
          }
        }
      }
    }
  }

  free(mag.buf);
  free(dir_tmp.buf);
  free(nms.buf);
  free(visited);
}

/* ================================================================== */
/*  Hough transform for lines                                         */
/* ================================================================== */

void line_hough(const vision_image_t *edges, const line_config_t *cfg,
                detected_line_t *lines, uint8_t *num_lines) {
  uint16_t w = edges->w, h = edges->h;
  float rho_max = sqrtf((float)(w*w + h*h)) / 2.0f;
  int num_rho = (int)(rho_max / cfg->hough_rho) * 2 + 1;
  int num_theta = 180 / cfg->hough_theta_deg;
  int center_x = w / 2, center_y = h / 2;

  /* Allocate accumulator */
  int *acc = (int*)calloc((size_t)num_rho * num_theta, sizeof(int));

  /* Find edge pixels */
  uint16_t *edge_xs = NULL, *edge_ys = NULL;
  uint32_t num_edges = 0;

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      if (edges->buf[y * w + x] > 0) num_edges++;
    }
  }

  if (num_edges == 0) {
    *num_lines = 0;
    free(acc);
    return;
  }

  edge_xs = (uint16_t*)malloc(num_edges * sizeof(uint16_t));
  edge_ys = (uint16_t*)malloc(num_edges * sizeof(uint16_t));
  num_edges = 0;

  for (uint16_t y = 0; y < h; y++) {
    for (uint16_t x = 0; x < w; x++) {
      if (edges->buf[y * w + x] > 0) {
        edge_xs[num_edges] = x;
        edge_ys[num_edges] = y;
        num_edges++;
      }
    }
  }

  /* Accumulate votes */
  for (uint32_t i = 0; i < num_edges; i++) {
    float xf = (float)edge_xs[i] - center_x;
    float yf = (float)edge_ys[i] - center_y;

    for (int ti = 0; ti < num_theta; ti++) {
      float theta = (float)ti * cfg->hough_theta_deg * (float)M_PI / 180.0f - (float)M_PI / 2.0f;
      float rho = xf * cosf(theta) + yf * sinf(theta);
      int ri = (int)((rho / cfg->hough_rho) + num_rho / 2);
      if (ri >= 0 && ri < num_rho) {
        acc[ri * num_theta + ti]++;
      }
    }
  }

  /* Find peaks */
  *num_lines = 0;
  uint8_t max_lines = LINE_DETECT_MAX_LINES;

  /* Simple peak detection with NMS */
  for (int ri = 1; ri < num_rho - 1 && *num_lines < max_lines; ri++) {
    for (int ti = 1; ti < num_theta - 1 && *num_lines < max_lines; ti++) {
      int val = acc[ri * num_theta + ti];
      if (val < cfg->hough_threshold) continue;

      /* Check if local maximum */
      bool is_max = true;
      for (int dr = -1; dr <= 1 && is_max; dr++) {
        for (int dt = -1; dt <= 1; dt++) {
          if (dr == 0 && dt == 0) continue;
          if (acc[(ri+dr)*num_theta + (ti+dt)] > val) {
            is_max = false;
          }
        }
      }
      if (!is_max) continue;

      /* Found a line */
      float rho = (float)(ri - num_rho/2) * cfg->hough_rho;
      float theta = (float)ti * cfg->hough_theta_deg * (float)M_PI / 180.0f - (float)M_PI / 2.0f;

      detected_line_t *line = &lines[*num_lines];
      line->rho = rho;
      line->theta = theta;
      line->votes = (uint16_t)val;
      line->strength = (val > 255) ? 255 : (uint8_t)val;

      /* Compute endpoints */
      float cos_t = cosf(theta), sin_t = sinf(theta);
      float x0 = center_x + rho * cos_t;
      float y0 = center_y + rho * sin_t;
      float dx = -sin_t * 1000;
      float dy = cos_t * 1000;

      /* Clip to image */
      float t1 = (-x0) / dx;
      float t2 = ((float)w - x0) / dx;
      if (t1 > t2) { float tmp = t1; t1 = t2; t2 = tmp; }
      float t3 = (-y0) / dy;
      float t4 = ((float)h - y0) / dy;
      if (t3 > t4) { float tmp = t3; t3 = t4; t4 = tmp; }
      float tstart = (t1 > t3) ? t1 : t3;
      float tend = (t2 < t4) ? t2 : t4;

      float sx = x0 + tstart * dx;
      float sy = y0 + tstart * dy;
      float ex = x0 + tend * dx;
      float ey = y0 + tend * dy;

      line->x1 = (sx < 0) ? 0 : (sx >= w) ? w-1 : (uint16_t)sx;
      line->y1 = (sy < 0) ? 0 : (sy >= h) ? h-1 : (uint16_t)sy;
      line->x2 = (ex < 0) ? 0 : (ex >= w) ? w-1 : (uint16_t)ex;
      line->y2 = (ey < 0) ? 0 : (ey >= h) ? h-1 : (uint16_t)ey;
      line->length = sqrtf((float)((line->x2-line->x1)*(line->x2-line->x1) +
                                   (line->y2-line->y1)*(line->y2-line->y1)));

      if (line->length >= cfg->hough_min_line_len) {
        (*num_lines)++;
      }
    }
  }

  free(edge_xs);
  free(edge_ys);
  free(acc);
}

/* ================================================================== */
/*  Hough transform for circles                                       */
/* ================================================================== */

void line_hough_circles(const vision_image_t *edges, const line_config_t *cfg,
                         detected_circle_t *circles, uint8_t *num_circles) {
  uint16_t w = edges->w, h = edges->h;
  *num_circles = 0;

  /* Collect edge pixels */
  uint16_t *xs = NULL, *ys = NULL;
  uint32_t nedges = 0;
  for (uint16_t y = 0; y < h; y++)
    for (uint16_t x = 0; x < w; x++)
      if (edges->buf[y*w+x] > 0) nedges++;
  if (nedges == 0) return;

  xs = (uint16_t*)malloc(nedges * sizeof(uint16_t));
  ys = (uint16_t*)malloc(nedges * sizeof(uint16_t));
  nedges = 0;
  for (uint16_t y = 0; y < h; y++)
    for (uint16_t x = 0; x < w; x++)
      if (edges->buf[y*w+x] > 0) { xs[nedges]=x; ys[nedges]=y; nedges++; }

  /* 3D accumulator (cx, cy, r) — quantized */
  uint8_t r_step = 2;
  uint16_t num_r = (cfg->circle_max_radius - cfg->circle_min_radius) / r_step + 1;
  uint8_t *acc3d = (uint8_t*)calloc((size_t)w * h * num_r, 1);

  for (uint32_t i = 0; i < nedges; i++) {
    for (uint16_t r = cfg->circle_min_radius; r <= cfg->circle_max_radius; r += r_step) {
      uint16_t ri = (r - cfg->circle_min_radius) / r_step;
      /* Draw circle in accumulator */
      for (int a = 0; a < 360; a += 4) {
        float rad = (float)a * (float)M_PI / 180.0f;
        int cx = (int)xs[i] + (int)(r * cosf(rad));
        int cy = (int)ys[i] + (int)(r * sinf(rad));
        if (cx >= 0 && cx < (int)w && cy >= 0 && cy < (int)h) {
          size_t idx = (size_t)cy * w * num_r + (size_t)cx * num_r + ri;
          if (acc3d[idx] < 255) acc3d[idx]++;
        }
      }
    }
  }

  /* Find peaks */
  for (uint16_t r = cfg->circle_min_radius; r <= cfg->circle_max_radius && *num_circles < LINE_DETECT_MAX_CIRCLES; r += r_step) {
    uint16_t ri = (r - cfg->circle_min_radius) / r_step;
    for (uint16_t cy = (uint16_t)r; cy < h - r; cy++) {
      for (uint16_t cx = (uint16_t)r; cx < w - r; cx++) {
        size_t idx = (size_t)cy * w * num_r + (size_t)cx * num_r + ri;
        uint8_t val = acc3d[idx];
        if (val < cfg->circle_threshold) continue;

        /* Check local max */
        bool is_max = true;
        for (int dr = -1; dr <= 1 && is_max; dr++) {
          for (int dc = -1; dc <= 1; dc++) {
            size_t ni = (size_t)(cy+dr) * w * num_r + (size_t)(cx+dc) * num_r + ri;
            if (acc3d[ni] > val) is_max = false;
          }
        }
        if (!is_max) continue;

        detected_circle_t *c = &circles[*num_circles];
        c->cx = (float)cx;
        c->cy = (float)cy;
        c->radius = (float)r;
        c->votes = val;
        c->strength = val;
        (*num_circles)++;
      }
    }
  }

  free(xs);
  free(ys);
  free(acc3d);
}

/* ================================================================== */
/*  Curve fitting                                                     */
/* ================================================================== */

bool line_fit_line(const float points[][2], uint16_t n, float *a, float *b, float *score) {
  if (n < 2) { *score = 0; return false; }

  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (uint16_t i = 0; i < n; i++) {
    sx += points[i][0];
    sy += points[i][1];
    sxx += points[i][0] * points[i][0];
    sxy += points[i][0] * points[i][1];
  }
  float nf = (float)n;
  float denom = nf * sxx - sx * sx;
  if (fabsf(denom) < 1e-6f) { *score = 0; return false; }

  *a = (nf * sxy - sx * sy) / denom;
  *b = (sy * (*a) * sx / nf) ? (sy - (*a) * sx) / nf : sy / nf;

  /* Compute R² score */
  float y_mean = sy / nf;
  float ss_tot = 0, ss_res = 0;
  for (uint16_t i = 0; i < n; i++) {
    float y_pred = (*a) * points[i][0] + (*b);
    ss_res += (points[i][1] - y_pred) * (points[i][1] - y_pred);
    ss_tot += (points[i][1] - y_mean) * (points[i][1] - y_mean);
  }
  *score = (ss_tot > 0) ? 1.0f - ss_res / ss_tot : 0;
  if (*score < 0) *score = 0;
  return true;
}

bool line_fit_parabola(const float points[][2], uint16_t n, float *a, float *b, float *c, float *score) {
  if (n < 3) { *score = 0; return false; }

  /* Solve via least squares: y = ax^2 + bx + c */
  float s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
  float sy0 = 0, sy1 = 0, sy2 = 0;
  for (uint16_t i = 0; i < n; i++) {
    float x = points[i][0], y = points[i][1];
    float x2 = x*x, x3 = x2*x, x4 = x3*x;
    s0 += 1; s1 += x; s2 += x2; s3 += x3; s4 += x4;
    sy0 += y; sy1 += x*y; sy2 += x2*y;
  }

  /* 3x3 system: [s0 s1 s2] [c]   [sy0]
                  [s1 s2 s3] [b] = [sy1]
                  [s2 s3 s4] [a]   [sy2] */
  float m[3][4] = {
    {s0, s1, s2, sy0},
    {s1, s2, s3, sy1},
    {s2, s3, s4, sy2}
  };

  /* Gaussian elimination */
  for (int col = 0; col < 3; col++) {
    int pivot = col;
    for (int row = col+1; row < 3; row++)
      if (fabsf(m[row][col]) > fabsf(m[pivot][col])) pivot = row;
    for (int j = 0; j < 4; j++) { float t = m[col][j]; m[col][j] = m[pivot][j]; m[pivot][j] = t; }
    if (fabsf(m[col][col]) < 1e-10f) { *score = 0; return false; }
    for (int row = col+1; row < 3; row++) {
      float f = m[row][col] / m[col][col];
      for (int j = col; j < 4; j++) m[row][j] -= f * m[col][j];
    }
  }

  *a = m[2][3] / m[2][2];
  *b = (m[1][3] - m[1][2] * (*a)) / m[1][1];
  *c = (m[0][3] - m[0][1] * (*b) - m[0][2] * (*a)) / m[0][0];

  /* R² score */
  float y_mean = sy0 / (float)n;
  float ss_tot = 0, ss_res = 0;
  for (uint16_t i = 0; i < n; i++) {
    float y_pred = (*a)*points[i][0]*points[i][0] + (*b)*points[i][0] + (*c);
    ss_res += (points[i][1] - y_pred) * (points[i][1] - y_pred);
    ss_tot += (points[i][1] - y_mean) * (points[i][1] - y_mean);
  }
  *score = (ss_tot > 0) ? 1.0f - ss_res / ss_tot : 0;
  if (*score < 0) *score = 0;
  return true;
}

bool line_fit_circle(const float points[][2], uint16_t n, float *cx, float *cy, float *r, float *score) {
  if (n < 3) { *score = 0; return false; }

  /* Algebraic circle fit: minimize sum((x-cx)^2 + (y-cy)^2 - r^2)^2
   * Linearized: 2*cx*x + 2*cy*y + (r^2 - cx^2 - cy^2) = x^2 + y^2
   * Let a=2*cx, b=2*cy, c=r^2-cx^2-cy^2 → a*x + b*y + c = x^2+y^2 */
  float s0 = 0, s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0;
  float sz0 = 0, sz1 = 0, sz2 = 0;

  for (uint16_t i = 0; i < n; i++) {
    float x = points[i][0], y = points[i][1];
    float x2 = x*x, y2 = y*y;
    s0 += x2; s1 += x*y; s2 += x;
    s3 += y2; s4 += y;
    s5 += 1;
    sz0 += x2*x + y2*x;
    sz1 += x*y2 + y2*y;
    sz2 += x2 + y2;
  }

  float m[3][4] = {
    {s0, s1, s2, sz0},
    {s1, s3, s4, sz1},
    {s2, s4, s5, sz2}
  };

  for (int col = 0; col < 3; col++) {
    int pivot = col;
    for (int row = col+1; row < 3; row++)
      if (fabsf(m[row][col]) > fabsf(m[pivot][col])) pivot = row;
    for (int j = 0; j < 4; j++) { float t = m[col][j]; m[col][j] = m[pivot][j]; m[pivot][j] = t; }
    if (fabsf(m[col][col]) < 1e-10f) { *score = 0; return false; }
    for (int row = col+1; row < 3; row++) {
      float f = m[row][col] / m[col][col];
      for (int j = col; j < 4; j++) m[row][j] -= f * m[col][j];
    }
  }

  float c_val = m[2][3] / m[2][2];
  float b_val = (m[1][3] - m[1][2] * c_val) / m[1][1];
  float a_val = (m[0][3] - m[0][1] * b_val - m[0][2] * c_val) / m[0][0];

  *cx = a_val / 2.0f;
  *cy = b_val / 2.0f;
  *r = sqrtf(c_val + (*cx)*(*cx) + (*cy)*(*cy));

  /* R² score */
  float y_mean = sz2 / (float)n;
  float ss_tot = 0, ss_res = 0;
  for (uint16_t i = 0; i < n; i++) {
    float d = sqrtf((points[i][0]-(*cx))*(points[i][0]-(*cx)) + (points[i][1]-(*cy))*(points[i][1]-(*cy)));
    ss_res += (d - (*r)) * (d - (*r));
    ss_tot += (d - y_mean) * (d - y_mean);
  }
  *score = (ss_tot > 0) ? 1.0f - ss_res / ss_tot : 0;
  if (*score < 0) *score = 0;
  return true;
}

/* ================================================================== */
/*  Lane detection                                                    */
/* ================================================================== */

void line_find_lane(const detected_line_t *lines, uint8_t num_lines,
                     uint16_t image_center_x, uint16_t image_w, line_result_t *result) {
  result->lane_valid = false;
  result->lane_offset_x = 0;
  result->lane_center_x = image_center_x;
  result->lane_heading = 0;

  /* Separate left and right lane lines by angle */
  float left_sum_rho = 0, right_sum_rho = 0;
  int left_count = 0, right_count = 0;

  for (uint8_t i = 0; i < num_lines; i++) {
    const detected_line_t *l = &lines[i];
    if (l->length < 30) continue;

    /* Lines on left side of image have theta > pi/4,
     * lines on right side have theta < pi/4 (relative to vertical) */
    if (l->x1 < image_center_x && l->x2 < image_center_x) {
      left_sum_rho += l->rho;
      left_count++;
    } else if (l->x1 > image_center_x && l->x2 > image_center_x) {
      right_sum_rho += l->rho;
      right_count++;
    }
  }

  if (left_count > 0 && right_count > 0) {
    float left_rho = left_sum_rho / left_count;
    float right_rho = right_sum_rho / right_count;

    /* Lane center = average of left and right rho */
    float avg_rho = (left_rho + right_rho) / 2.0f;
    result->lane_offset_x = (int16_t)avg_rho;
    result->lane_center_x = image_center_x + (int16_t)avg_rho;

    /* Heading from average theta */
    float left_theta = 0, right_theta = 0;
    for (uint8_t i = 0; i < num_lines; i++) {
      if (lines[i].x1 < image_center_x) left_theta += lines[i].theta;
      else right_theta += lines[i].theta;
    }
    result->lane_heading = ((left_theta/left_count) + (right_theta/right_count)) / 2.0f - (float)M_PI/2.0f;
    result->lane_valid = true;
  } else if (left_count > 0) {
    result->lane_offset_x = (int16_t)(left_sum_rho / left_count);
    result->lane_center_x = image_center_x + result->lane_offset_x;
    result->lane_valid = true;
  } else if (right_count > 0) {
    result->lane_offset_x = (int16_t)(right_sum_rho / right_count);
    result->lane_center_x = image_center_x + result->lane_offset_x;
    result->lane_valid = true;
  }
}

/* ================================================================== */
/*  Main detection entry point                                        */
/* ================================================================== */

void line_detect(const vision_image_t *img, const line_config_t *cfg, line_result_t *result) {
  uint16_t w = img->w, h = img->h;
  memset(result, 0, sizeof(line_result_t));

  /* Create edge image */
  vision_image_t edges;
  edges.type = VISION_IMAGE_GRAYSCALE;
  edges.w = w; edges.h = h; edges.stride = w; edges.bytes_per_pixel = 1;
  edges.buf = (uint8_t*)malloc(w * h);

  if (cfg->canny_low > 0 && cfg->canny_high > 0) {
    line_canny(img, cfg->canny_low, cfg->canny_high, &edges);
  } else {
    /* Raw Sobel thresholding */
    vision_image_t mag;
    mag.type = VISION_IMAGE_GRAYSCALE;
    mag.w = w; mag.h = h; mag.stride = w; mag.bytes_per_pixel = 1;
    mag.buf = (uint8_t*)malloc(w * h);
    line_sobel(img, &mag, NULL);

    memset(edges.buf, 0, w * h);
    for (uint16_t y = 0; y < h; y++)
      for (uint16_t x = 0; x < w; x++)
        edges.buf[y*w+x] = (mag.buf[y*w+x] >= cfg->sobel_threshold) ? 255 : 0;

    free(mag.buf);
  }

  /* Hough lines */
  line_hough(&edges, cfg, result->lines, &result->num_lines);

  /* Hough circles */
  if (cfg->circle_max_radius > cfg->circle_min_radius) {
    line_hough_circles(&edges, cfg, result->circles, &result->num_circles);
  }

  /* Lane detection */
  line_find_lane(result->lines, result->num_lines, w/2, w, result);

  /* Store edge image if requested */
  if (cfg->draw_edges) {
    result->edge_image = (vision_image_t*)malloc(sizeof(vision_image_t));
    memcpy(result->edge_image, &edges, sizeof(vision_image_t));
    result->edge_image->buf = edges.buf; /* transfer ownership */
    edges.buf = NULL;
  }

  free(edges.buf);
}

void line_print(const line_result_t *result) {
  printf("Lines: %d\n", result->num_lines);
  for (uint8_t i = 0; i < result->num_lines; i++) {
    const detected_line_t *l = &result->lines[i];
    printf("  L%d: rho=%.1f theta=%.1f° len=%.0f votes=%d (%d,%d)-(%d,%d)\n",
           i, l->rho, l->theta * 180.0f / (float)M_PI,
           l->length, l->votes, l->x1, l->y1, l->x2, l->y2);
  }

  printf("Circles: %d\n", result->num_circles);
  for (uint8_t i = 0; i < result->num_circles; i++) {
    const detected_circle_t *c = &result->circles[i];
    printf("  C%d: center=(%.1f,%.1f) r=%.1f votes=%d\n",
           i, c->cx, c->cy, c->radius, c->votes);
  }

  if (result->lane_valid) {
    printf("Lane: offset_x=%d heading=%.1f°\n",
           result->lane_offset_x, result->lane_heading * 180.0f / (float)M_PI);
  }
}
