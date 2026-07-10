#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/videodev2.h>
#include "video_capture.h"
#include "image.h"
#include "types.h"
#include "flow_stage1.h"
#include "obstacle.h"

#define W 320
#define H 240
#define SUBP 10
#define FRAME_SIZE (W * H)

static volatile int g_running = 1;

static void handle_signal(int sig) {
  (void)sig;
  g_running = 0;
}

static void uyvy_to_gray(const uint8_t *uyvy, uint8_t *gray, int stride) {
  for (int y = 0; y < H; y++)
    for (int x = 0; x < W; x++)
      gray[y * stride + x] = uyvy[y * W * 2 + (x / 2) * 4 + 1 + (x % 2) * 2];
}

static void sp_set(uint8_t *buf, int stride, int x, int y) {
  if (x >= 0 && x < W && y >= 0 && y < H) buf[y * stride + x] = 255;
}

static void draw_arrow(uint8_t *buf, int stride, int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  while (1) {
    sp_set(buf, stride, x0, y0);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

static void draw_char(uint8_t *buf, int stride, int x, int y, char ch) {
  /* 5x7 font subset for numbers + letters */
  static const uint8_t font[10][5] = {
    {0x7E,0x42,0x42,0x42,0x7E}, /* 0 */
    {0x10,0x30,0x10,0x10,0x7C}, /* 1 */
    {0x7E,0x02,0x7E,0x40,0x7E}, /* 2 */
    {0x7E,0x02,0x7E,0x02,0x7E}, /* 3 */
    {0x42,0x42,0x7E,0x02,0x02}, /* 4 */
    {0x7E,0x40,0x7E,0x02,0x7E}, /* 5 */
    {0x7E,0x40,0x7E,0x42,0x7E}, /* 6 */
    {0x7E,0x02,0x04,0x08,0x08}, /* 7 */
    {0x7E,0x42,0x7E,0x42,0x7E}, /* 8 */
    {0x7E,0x42,0x7E,0x02,0x7E}, /* 9 */
  };
  if (ch < '0' || ch > '9') return;
  int idx = ch - '0';
  for (int row = 0; row < 7 && y + row < H; row++) {
    for (int col = 0; col < 5 && x + col < W; col++) {
      if (font[idx][col] & (1 << (6 - row)))
        buf[(y + row) * stride + x + col] = 255;
    }
  }
}

static void draw_num(uint8_t *buf, int stride, int x, int y, int val) {
  char s[16];
  snprintf(s, sizeof(s), "%d", val);
  int len = strlen(s);
  for (int i = 0; i < len; i++)
    draw_char(buf, stride, x + i * 6, y, s[i]);
}

int main(int argc, char **argv) {
  const char *device = argc > 1 ? argv[1] : VIDEO_CAPTURE_FRONT_DEVICE;
  const char *dest_ip = argc > 2 ? argv[2] : "192.168.1.2";
  int dest_port = argc > 3 ? atoi(argv[3]) : 9090;
  int num_frames = argc > 4 ? atoi(argv[4]) : 0;

  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  /* Open camera */
  video_capture_t vc;
  if (video_capture_open(&vc, device, W, H, V4L2_PIX_FMT_UYVY) != 0) {
    fprintf(stderr, "Failed to open %s\n", device);
    return 1;
  }
  printf("Camera: %dx%d\n", vc.width, vc.height);

  if (video_capture_start(&vc) != 0) {
    fprintf(stderr, "Failed to start capture\n");
    video_capture_close(&vc);
    return 1;
  }

  /* UDP output socket */
  int out_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (out_fd < 0) { perror("socket"); return 1; }
  struct sockaddr_in out_addr;
  memset(&out_addr, 0, sizeof(out_addr));
  out_addr.sin_family = AF_INET;
  out_addr.sin_port = htons(dest_port);
  out_addr.sin_addr.s_addr = inet_addr(dest_ip);

  /* Init vision modules */
  flow_stage1_config_t s1cfg;
  s1cfg.tile_size = 8;
  s1cfg.search_range = 6;
  s1cfg.image_width = W;
  s1cfg.image_height = H;
  s1cfg.subsample = 1;
  s1cfg.min_quality = 0;
  flow_stage1_t *s1 = flow_stage1_create(&s1cfg);
  obstacle_t *obs = obstacle_create(W, H);

  /* Buffers */
  uint8_t *gray = malloc(FRAME_SIZE);
  uint8_t *prev_gray = malloc(FRAME_SIZE);
  uint8_t *out_buf = malloc(FRAME_SIZE + 64);  /* frame + header space */
  if (!gray || !prev_gray || !out_buf) { fprintf(stderr, "Alloc failed\n"); return 1; }

  vision_image_t frame_img, prev_img;
  image_create(&frame_img, W, H, VISION_IMAGE_GRAYSCALE);
  image_create(&prev_img, W, H, VISION_IMAGE_GRAYSCALE);

  printf("Streaming to %s:%d\n", dest_ip, dest_port);
  printf("%-6s %-8s %-8s %-6s %-6s %-6s %-6s\n",
         "FRAME", "FX", "FY", "QUAL", "LOOM", "ASYM", "CONF");

  int frame = 0;
  bool have_prev = false;

  while (g_running && (num_frames == 0 || frame < num_frames)) {
    uint8_t *data = NULL;
    size_t size = 0;

    int ret = video_capture_frame(&vc, &data, &size);
    if (ret < 0) break;
    if (ret == 1) {
      struct timespec req = {0, 1000};
      nanosleep(&req, NULL);
      continue;
    }

    uyvy_to_gray(data, gray, W);
    video_capture_release_frame(&vc);
    memcpy(frame_img.buf, gray, FRAME_SIZE);

    /* Stage 1 flow */
    vision_result_t vres;
    memset(&vres, 0, sizeof(vres));
    flow_stage1_process(s1, &frame_img, &vres);

    /* Obstacle */
    obstacle_result_t obst;
    memset(&obst, 0, sizeof(obst));
    if (have_prev) obstacle_process(obs, &frame_img, &prev_img, &obst);

    /* Draw overlay on the grayscale frame */
    memcpy(out_buf + 64, gray, FRAME_SIZE);
    uint8_t *ov = out_buf + 64;

    /* Flow arrow at center */
    int fx = vres.flow_x_fast / SUBP;
    int fy = vres.flow_y_fast / SUBP;
    if (fx != 0 || fy != 0)
      draw_arrow(ov, W, W/2, H/2, W/2 + fx, H/2 + fy);

    /* Looming bar at top */
    int bar_h = 6, bar_w = 80;
    int bar_x = (W - bar_w) / 2;
    int filled = (obst.looming * bar_w) / 255;
    for (int y = 1; y <= bar_h; y++)
      for (int x = bar_x; x < bar_x + bar_w; x++)
        ov[y * W + x] = (x - bar_x) < filled ? 255 : 40;

    /* Text info (top-left) */
    draw_num(ov, W, 2, 10, frame);
    draw_num(ov, W, 2, 20, fx);
    draw_num(ov, W, 60, 20, fy);
    draw_num(ov, W, 2, 30, obst.looming);
    draw_num(ov, W, 50, 30, obst.asymmetry);

    /* Draw vertical line if found */
    if (obst.line_found)
      for (int y = 0; y < H; y++)
        ov[y * W + obst.line_x] = 200;

    /* Send metadata + frame as single UDP datagram */
    /* Format: [frame_num:4][flow_x:4][flow_y:4][qual:1][loom:1][asym:1][conf:1]
     *         [line_found:1][line_x:2][line_angle:2][line_strength:1] = 22 bytes header
     *         then 320x240 = 76800 bytes raw grayscale */
    uint32_t *hdr = (uint32_t*)out_buf;
    hdr[0] = htonl(frame);
    hdr[1] = htonl(vres.flow_x_fast);
    hdr[2] = htonl(vres.flow_y_fast);
    out_buf[12] = vres.quality_fast;
    out_buf[13] = obst.looming;
    out_buf[14] = (uint8_t)(int8_t)obst.asymmetry;
    out_buf[15] = obst.confidence;
    out_buf[16] = obst.line_found ? 1 : 0;
    out_buf[17] = obst.line_x & 0xFF;
    out_buf[18] = (obst.line_x >> 8) & 0xFF;
    out_buf[19] = obst.line_angle & 0xFF;
    out_buf[20] = (obst.line_angle >> 8) & 0xFF;
    out_buf[21] = obst.line_strength;

    sendto(out_fd, out_buf, 64 + FRAME_SIZE, 0,
           (struct sockaddr*)&out_addr, sizeof(out_addr));

    if (frame % 10 == 0) {
      printf("%-6d %+8d %+8d %-6u %-6u %+6d %-6u\n",
             frame, vres.flow_x_fast, vres.flow_y_fast,
             vres.quality_fast, obst.looming, obst.asymmetry, obst.confidence);
    }

    /* Swap frames */
    vision_image_t tmp = prev_img;
    prev_img = frame_img;
    frame_img = tmp;
    uint8_t *tmp_b = prev_gray;
    prev_gray = gray;
    gray = tmp_b;
    have_prev = true;
    frame++;
  }

  printf("\nSent %d frames to %s:%d\n", frame, dest_ip, dest_port);

  free(gray); free(prev_gray); free(out_buf);
  image_destroy(&frame_img); image_destroy(&prev_img);
  obstacle_destroy(obs); flow_stage1_destroy(s1);
  video_capture_stop(&vc); video_capture_close(&vc);
  close(out_fd);
  return 0;
}
