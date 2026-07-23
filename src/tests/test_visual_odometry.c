#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include "../vision/types.h"
#include "../vision/visual_odometry.h"
#include "../vision/fast_detect.h"
#include "../vision/lk_flow.h"
#include "../video_capture.h"

static volatile int running = 1;

static void sighandler(int sig) {
    (void)sig;
    running = 0;
}

static uint32_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

static void print_usage(const char *prog) {
    printf("Usage: %s [options]\n", prog);
    printf("\n");
    printf("Visual Odometry test for AR.Drone 2.0\n");
    printf("\n");
    printf("Options:\n");
    printf("  -d <device>   Camera device (default: /dev/video0)\n");
    printf("  -w <width>    Capture width (default: 320)\n");
    printf("  -e <height>   Capture height (default: 240)\n");
    printf("  -n <frames>   Number of frames (0=infinite, default: 0)\n");
    printf("  -a <alt_m>    Altitude for scale recovery (default: 1.0)\n");
    printf("  -v            Verbose output\n");
    printf("  -t            Test mode (synthetic motion)\n");
    printf("  -h            This help\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s                        # Default 320x240, infinite\n", prog);
    printf("  %s -w 640 -e 480          # Higher resolution\n", prog);
    printf("  %s -a 2.0                 # Assume 2m altitude\n", prog);
    printf("  %s -t                     # Synthetic test\n", prog);
}

/* ================================================================== */
/*  Synthetic test: generate frames with known translation             */
/* ================================================================== */

static void generate_test_frame(uint8_t *buf, uint16_t w, uint16_t h,
                                float shift_x, float shift_y) {
    /* Create a textured pattern (checkerboard + gradient) */
    for (uint16_t y = 0; y < h; y++) {
        for (uint16_t x = 0; x < w; x++) {
            /* Shifted coordinates */
            float sx = (float)x + shift_x;
            float sy = (float)y + shift_y;

            /* Checkerboard */
            int cx = ((int)(sx / 20)) & 1;
            int cy = ((int)(sy / 20)) & 1;
            uint8_t val = (cx ^ cy) ? 180 : 80;

            /* Add gradient */
            val += (uint8_t)((float)x / w * 40);

            buf[y * w + x] = val;
        }
    }
}

/* ================================================================== */
/*  Main                                                               */
/* ================================================================== */

int main(int argc, char *argv[]) {
    const char *device = "/dev/video0";
    uint16_t width = 320;
    uint16_t height = 240;
    int max_frames = 0;
    float altitude_m = 1.0f;
    int verbose = 0;
    int test_mode = 0;

    int opt;
    while ((opt = getopt(argc, argv, "d:w:e:n:a:vth")) != -1) {
        switch (opt) {
            case 'd': device = optarg; break;
            case 'w': width = atoi(optarg); break;
            case 'e': height = atoi(optarg); break;
            case 'n': max_frames = atoi(optarg); break;
            case 'a': altitude_m = atof(optarg); break;
            case 'v': verbose = 1; break;
            case 't': test_mode = 1; break;
            case 'h': print_usage(argv[0]); return 0;
            default:  print_usage(argv[0]); return 1;
        }
    }

    signal(SIGINT, sighandler);
    signal(SIGTERM, sighandler);

    printf("=== Visual Odometry Test ===\n");
    printf("Device: %s, Resolution: %dx%d\n", device, width, height);
    printf("Altitude: %.1fm, Test mode: %s\n", altitude_m, test_mode ? "yes" : "no");

    /* --- Initialize VO --- */
    vo_config_t cfg;
    vo_default_config(&cfg);
    cfg.use_barometer = (altitude_m > 0.1f);

    vo_context_t *vo = vo_create(&cfg, width, height);
    if (!vo) {
        fprintf(stderr, "Failed to create VO context\n");
        return 1;
    }
    printf("VO context created\n");

    /* --- Camera capture (or synthetic) --- */
    video_capture_t cap;
    uint8_t *frame_buf = NULL;
    int camera_fd = -1;

    if (!test_mode) {
        printf("Opening camera %s...\n", device);
        if (video_capture_open(&cap, device, width, height, 0) < 0) {
            fprintf(stderr, "Failed to open camera, switching to test mode\n");
            test_mode = 1;
        } else {
            camera_fd = cap.fd;
            if (video_capture_start(&cap) < 0) {
                fprintf(stderr, "Failed to start capture, switching to test mode\n");
                test_mode = 1;
                video_capture_close(&cap);
            } else {
                printf("Camera opened (fd=%d)\n", camera_fd);
            }
        }
    }

    if (test_mode) {
        frame_buf = malloc(width * height);
        if (!frame_buf) {
            fprintf(stderr, "Failed to allocate frame buffer\n");
            return 1;
        }
    } else {
        frame_buf = malloc(width * height);
        if (!frame_buf) {
            fprintf(stderr, "Failed to allocate frame buffer\n");
            return 1;
        }
    }

    /* --- Main loop --- */
    printf("\nRunning visual odometry...\n");
    printf("Press Ctrl+C to stop\n\n");

    uint32_t start = now_ms();
    uint32_t last_log = 0;
    int frame_count = 0;
    uint32_t prev_time = start;

    /* For synthetic test: simulate forward motion */
    float synth_shift_x = 0;
    float synth_shift_y = 0;

    while (running && (max_frames == 0 || frame_count < max_frames)) {
        uint32_t loop_start = now_ms();

        /* Get frame */
        vision_image_t frame;
        frame.type = VISION_IMAGE_GRAYSCALE;
        frame.w = width;
        frame.h = height;
        frame.stride = width;
        frame.bytes_per_pixel = 1;

        if (test_mode) {
            /* Generate synthetic frame with translation */
            synth_shift_x += 2.0f;  /* 2 pixels per frame to the right */
            synth_shift_y += 0.5f;  /* 0.5 pixels per frame down */
            generate_test_frame(frame_buf, width, height, synth_shift_x, synth_shift_y);
            frame.buf = frame_buf;
        } else {
            /* Capture from camera */
            uint8_t *data = NULL;
            size_t data_size = 0;
            if (video_capture_frame(&cap, &data, &data_size) < 0 || !data) {
                fprintf(stderr, "Frame capture failed\n");
                continue;
            }
            /* Copy and convert UYVY to grayscale */
            /* For now, use Y channel (every 2nd byte of UYVY) */
            for (uint16_t y = 0; y < height; y++) {
                for (uint16_t x = 0; x < width; x++) {
                    frame_buf[y * width + x] = data[(y * width + x) * 2 + 1];
                }
            }
            frame.buf = frame_buf;
            video_capture_release_frame(&cap);
        }

        frame.timestamp_us = loop_start * 1000;  /* monotonic */

        /* Compute dt */
        uint32_t dt_ms = loop_start - prev_time;
        prev_time = loop_start;

        /* Process frame */
        vo_motion_t motion;
        vo_position_t pos;
        memset(&motion, 0, sizeof(motion));
        memset(&pos, 0, sizeof(pos));

        if (vo_process(vo, &frame, altitude_m, 1.0f, dt_ms, &motion, &pos) < 0) {
            fprintf(stderr, "VO processing failed\n");
            continue;
        }

        frame_count++;

        /* Log every 500ms */
        if (loop_start - last_log >= 500) {
            uint32_t elapsed = loop_start - start;
            printf("[%5u.%03u] frame=%d "
                   "pos[%.3f,%.3f,%.3f] "
                   "vel[%.3f,%.3f,%.3f] "
                   "matches=%d inliers=%d (%.0f%%) "
                   "att[%.1f,%.1f,%.1f]deg\n",
                   elapsed / 1000, elapsed % 1000,
                   frame_count,
                   pos.x, pos.y, pos.z,
                   pos.vx, pos.vy, pos.vz,
                   motion.num_matches, motion.num_inliers,
                   motion.inlier_ratio * 100.0f,
                   pos.roll * 180.0f / M_PI,
                   pos.pitch * 180.0f / M_PI,
                   pos.yaw * 180.0f / M_PI);

            if (verbose && motion.valid) {
                printf("       R=[%.3f %.3f %.3f; %.3f %.3f %.3f; %.3f %.3f %.3f]\n",
                       motion.R[0][0], motion.R[0][1], motion.R[0][2],
                       motion.R[1][0], motion.R[1][1], motion.R[1][2],
                       motion.R[2][0], motion.R[2][1], motion.R[2][2]);
                printf("       t=[%.3f, %.3f, %.3f]\n",
                       motion.t[0], motion.t[1], motion.t[2]);
            }

            last_log = loop_start;
        }

        /* Target 30fps */
        uint32_t elapsed_loop = now_ms() - loop_start;
        if (elapsed_loop < 33) {
            usleep((33 - elapsed_loop) * 1000);
        }
    }

    /* --- Cleanup --- */
    printf("\n=== Results ===\n");
    printf("Frames processed: %d\n", frame_count);
    vo_position_t final_pos;
    vo_get_position(vo, &final_pos);
    printf("Final position: [%.3f, %.3f, %.3f] m\n",
           final_pos.x, final_pos.y, final_pos.z);
    printf("Final velocity: [%.3f, %.3f, %.3f] m/s\n",
           final_pos.vx, final_pos.vy, final_pos.vz);

    if (frame_buf) free(frame_buf);
    if (!test_mode) {
        video_capture_stop(&cap);
        video_capture_close(&cap);
    }
    vo_destroy(vo);

    printf("Done.\n");
    return 0;
}
