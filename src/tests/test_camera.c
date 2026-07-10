#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <linux/videodev2.h>
#include "video_capture.h"

static volatile int g_running = 1;

static void handle_signal(int sig) {
  (void)sig;
  g_running = 0;
}

int main(int argc, char **argv) {
  const char *device = argc > 1 ? argv[1] : VIDEO_CAPTURE_FRONT_DEVICE;
  int req_w = argc > 2 ? atoi(argv[2]) : 320;
  int req_h = argc > 3 ? atoi(argv[3]) : 240;
  int num_frames = argc > 4 ? atoi(argv[4]) : 100;

  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);

  video_capture_t vc;
  if (video_capture_open(&vc, device, req_w, req_h, V4L2_PIX_FMT_UYVY) != 0) {
    fprintf(stderr, "Failed to open %s\n", device);
    return 1;
  }

  printf("Camera: %s\n", device);
  printf("Requested: %dx%d\n", req_w, req_h);
  printf("Actual:    %dx%d\n", vc.width, vc.height);
  printf("Format:    %c%c%c%c\n",
         (char)(vc.format & 0xFF),
         (char)((vc.format >> 8) & 0xFF),
         (char)((vc.format >> 16) & 0xFF),
         (char)((vc.format >> 24) & 0xFF));
  printf("Frame size: %zu bytes\n", vc.frame_size);

  if (video_capture_start(&vc) != 0) {
    fprintf(stderr, "Failed to start capture\n");
    video_capture_close(&vc);
    return 1;
  }

  printf("\nCapturing %d frames...\n", num_frames);
  uint32_t total_us = 0;

  for (int i = 0; i < num_frames && g_running; i++) {
    uint8_t *data = NULL;
    size_t size = 0;

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    int ret = video_capture_frame(&vc, &data, &size);
    if (ret < 0) {
      fprintf(stderr, "Capture error at frame %d\n", i);
      break;
    }
    if (ret == 1) {
      struct timespec req = {0, 1000000};
      nanosleep(&req, NULL);
      i--;
      continue;
    }

    clock_gettime(CLOCK_MONOTONIC, &t1);
    uint32_t us = (t1.tv_sec - t0.tv_sec) * 1000000 +
                  (t1.tv_nsec - t0.tv_nsec) / 1000;
    total_us += us;

    /* Compute mean brightness of first few pixels as a sanity check */
    uint32_t sum = 0;
    for (int j = 0; j < 100 && j < (int)size; j++) sum += data[j];

    if (i % 10 == 0 || i == num_frames - 1) {
      printf("Frame %4d: %zu bytes, avg_brightness=%u, capture_time=%u us\n",
             i, size, sum / 100, us);
    }

    video_capture_release_frame(&vc);
  }

  uint32_t avg_us = num_frames > 0 ? total_us / num_frames : 0;
  printf("\nStats: %d frames, avg %u us/frame (%.1f FPS)\n",
         num_frames, avg_us, avg_us > 0 ? 1000000.0f / avg_us : 0);

  video_capture_stop(&vc);
  video_capture_close(&vc);
  return 0;
}
