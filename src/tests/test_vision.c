#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <linux/videodev2.h>
#include "video_capture.h"
#include "image.h"
#include "types.h"
#include "flow_stage1.h"
#include "obstacle.h"

#define W 320
#define H 240

static volatile int g_running = 1;

static void handle_signal(int sig) {
  (void)sig;
  g_running = 0;
}

static void uyvy_to_gray(const uint8_t *uyvy, uint8_t *gray,
                          int width, int height, int gray_stride) {
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      /* UYVY: U0 Y0 V0 Y1 — Y at offset 1 and 3 of each 4-byte pair */
      int uyvy_idx = y * width * 2 + (x / 2) * 4;
      gray[y * gray_stride + x] = uyvy[uyvy_idx + 1 + (x % 2) * 2];
    }
  }
}

int main(int argc, char **argv) {
  const char *device = argc > 1 ? argv[1] : VIDEO_CAPTURE_FRONT_DEVICE;
  int num_frames = argc > 2 ? atoi(argv[2]) : 300;

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

  vision_image_t gray, prev_gray;
  image_create(&gray, W, H, VISION_IMAGE_GRAYSCALE);
  image_create(&prev_gray, W, H, VISION_IMAGE_GRAYSCALE);

  /* Allocate conversion buffer */
  uint8_t *gray_buf = malloc(W * H);
  uint8_t *gray_prev_buf = malloc(W * H);
  if (!gray_buf || !gray_prev_buf) {
    fprintf(stderr, "Alloc failed\n");
    return 1;
  }

  printf("\n%-4s %-10s %-10s %-6s %-8s %-8s %-8s\n",
         "fr", "flow_x", "flow_y", "qual",
         "loom", "asym", "conf");
  printf("---- ---------- ---------- ------ -------- -------- --------\n");

  int frame = 0;
  bool have_prev = false;

  while (g_running && frame < num_frames) {
    uint8_t *data = NULL;
    size_t size = 0;

    int ret = video_capture_frame(&vc, &data, &size);
    if (ret < 0) break;
    if (ret == 1) {
      struct timespec req = {0, 1000};
      nanosleep(&req, NULL);
      continue;
    }

    /* Convert UYVY to grayscale */
    uyvy_to_gray(data, gray_buf, vc.width, vc.height, vc.width);
    video_capture_release_frame(&vc);

    /* Copy to vision_image_t */
    memcpy(gray.buf, gray_buf, W * H);

    /* Stage 1: optical flow */
    vision_result_t result;
    memset(&result, 0, sizeof(result));
    flow_stage1_process(s1, &gray, &result);

    /* Obstacle detection */
    obstacle_result_t obst;
    memset(&obst, 0, sizeof(obst));
    if (have_prev) {
      obstacle_process(obs, &gray, &prev_gray, &obst);
    }

    if (frame % 10 == 0 || frame == num_frames - 1) {
      printf("%-4d %+10d %+10d %-6u %-8u %+8d %-8u\n",
             frame,
             result.flow_x_fast, result.flow_y_fast,
             result.quality_fast,
             obst.looming, obst.asymmetry, obst.confidence);
    }

    /* Swap frames */
    vision_image_t tmp = prev_gray;
    prev_gray = gray;
    gray = tmp;
    /* Swap buffers */
    uint8_t *tmp_buf = gray_prev_buf;
    gray_prev_buf = gray_buf;
    gray_buf = tmp_buf;
    have_prev = true;
    frame++;
  }

  printf("\nCaptured %d frames\n", frame);

  /* Cleanup */
  free(gray_buf);
  free(gray_prev_buf);
  image_destroy(&gray);
  image_destroy(&prev_gray);
  obstacle_destroy(obs);
  flow_stage1_destroy(s1);
  video_capture_stop(&vc);
  video_capture_close(&vc);
  return 0;
}
