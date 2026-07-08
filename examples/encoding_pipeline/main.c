/*
 * encoding_pipeline — Full onboard H.264 encoding pipeline:
 *   V4L2 capture → UYVY → NV12 → DSP H.264 encode → file output
 *
 * Cross-compile:
 *   cd build && make
 *
 * Deploy:
 *   cd tools && ./deploy.sh ../build/bin/drone_encoder 192.168.1.1
 *
 * Run on drone:
 *   telnet 192.168.1.1
 *   killall program.elf
 *   mount --bind /data/video/opt/arm /opt/arm
 *   mount --bind /data/video/opt/arm/lib/dsp /lib/dsp
 *   export DSP_PATH=/opt/arm/tidsp-binaries-23.i3.8/
 *   /data/video/drone_encoder
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#include "video_capture.h"
#include "h264_init.h"
#include "h264_encode.h"

static volatile int running = 1;

void handle_signal(int sig) {
    (void)sig;
    printf("\nShutting down...\n");
    running = 0;
}

static const char *OUTPUT_FILE = "/data/video/output.h264";

int main(int argc, char **argv) {
    const char *device = VIDEO_CAPTURE_FRONT_DEVICE;
    int width = 1280;
    int height = 720;
    int bitrate = 2000;  // kbps
    int fps = 30;
    int duration = 10;   // seconds

    if (argc > 1) device = argv[1];
    if (argc > 2) duration = atoi(argv[2]);
    if (argc > 3) bitrate = atoi(argv[3]);

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    printf("=== AR.Drone 2.0 H.264 Encoding Pipeline ===\n");
    printf("Device:    %s\n", device);
    printf("Output:    %s\n", OUTPUT_FILE);
    printf("Resolution: %dx%d\n", width, height);
    printf("Bitrate:   %d kbps\n", bitrate);
    printf("Duration:  %d seconds\n", duration);

    // Step 1: Initialize DSP / H.264 hardware
    printf("\n[1] Initializing DSP...\n");
    if (dsp_full_init() < 0) {
        fprintf(stderr, "DSP init failed. Is vision framework installed?\n");
        fprintf(stderr, "Run this from the drone after setting up DSP:\n");
        fprintf(stderr, "  mount --bind /data/video/opt/arm /opt/arm\n");
        fprintf(stderr, "  mount --bind /data/video/opt/arm/lib/dsp /lib/dsp\n");
        fprintf(stderr, "  export DSP_PATH=/opt/arm/tidsp-binaries-23.i3.8/\n");
        return 1;
    }

    // Step 2: Open video capture
    printf("\n[2] Opening camera %s...\n", device);
    video_capture_t vc;
    if (video_capture_open(&vc, device, width, height, V4L2_PIX_FMT_UYVY) < 0) {
        fprintf(stderr, "Failed to open camera\n");
        return 1;
    }
    printf("Camera opened: %dx%d, format UYVY\n", vc.width, vc.height);

    // Step 3: Open H.264 encoder
    printf("\n[3] Opening H.264 encoder...\n");
    h264_encoder_t enc;
    if (h264_encoder_open(&enc, vc.width, vc.height, bitrate, fps) < 0) {
        fprintf(stderr, "Failed to open encoder\n");
        video_capture_close(&vc);
        return 1;
    }

    // Allocate conversion buffer (NV12)
    int nv12_size = vc.width * vc.height * 3 / 2;
    uint8_t *nv12_buf = malloc(nv12_size);
    if (!nv12_buf) {
        fprintf(stderr, "Failed to allocate NV12 buffer\n");
        h264_encoder_close(&enc);
        video_capture_close(&vc);
        return 1;
    }

    // Allocate H.264 output buffer
    int h264_buf_size = vc.width * vc.height;
    uint8_t *h264_buf = malloc(h264_buf_size);
    if (!h264_buf) {
        fprintf(stderr, "Failed to allocate H.264 buffer\n");
        free(nv12_buf);
        h264_encoder_close(&enc);
        video_capture_close(&vc);
        return 1;
    }

    // Step 4: Start capture
    printf("\n[4] Starting video capture...\n");
    remove(OUTPUT_FILE);
    if (video_capture_start(&vc) < 0) {
        fprintf(stderr, "Failed to start capture\n");
        free(h264_buf);
        free(nv12_buf);
        h264_encoder_close(&enc);
        video_capture_close(&vc);
        return 1;
    }

    // Step 5: Encoding loop
    printf("\n[5] Encoding for %d seconds...\n", duration);
    int total_frames = duration * fps;
    int encoded_frames = 0;

    for (int i = 0; i < total_frames && running; i++) {
        uint8_t *frame_data;
        size_t frame_size;

        int ret = video_capture_frame(&vc, &frame_data, &frame_size);
        if (ret < 0) {
            fprintf(stderr, "Capture error\n");
            break;
        }
        if (ret > 0) {
            // No frame ready yet
            usleep(1000);
            continue;
        }

        // Convert UYVY → NV12
        uyvy_to_nv12(frame_data, nv12_buf, vc.width, vc.height);

        // Release V4L2 buffer
        video_capture_release_frame(&vc);

        // Encode via DSP
        int h264_size = 0;
        if (h264_encoder_encode(&enc, nv12_buf, h264_buf, &h264_size) == 0
            && h264_size > 0) {
            // Write to file (with Annex B start code)
            h264_write_bitstream(OUTPUT_FILE, h264_buf, h264_size, 0);

            if (encoded_frames % 30 == 0) {
                printf("  Frame %4d: %d bytes H.264\n", encoded_frames, h264_size);
            }
            encoded_frames++;
        }
    }

    printf("\n[6] Done. %d frames encoded.\n", encoded_frames);
    printf("Output: %s\n", OUTPUT_FILE);

    // Verify output
    FILE *fp = fopen(OUTPUT_FILE, "rb");
    if (fp) {
        fseek(fp, 0, SEEK_END);
        long out_size = ftell(fp);
        fclose(fp);
        printf("File size: %ld bytes (%.1f KB)\n", out_size, out_size / 1024.0);
    }

    // Cleanup
    free(h264_buf);
    free(nv12_buf);
    h264_encoder_close(&enc);
    video_capture_close(&vc);

    printf("Pipeline complete.\n");
    return 0;
}
