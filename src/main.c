#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <linux/videodev2.h>

#include "h264_init.h"
#include "h264_encode.h"
#include "video_capture.h"

#define RTP_PT_H264 96
#define RTP_CLOCK_RATE 90000
#define RTP_MTU 1400
#define RTP_SSRC 0xDEADBEEF

typedef struct __attribute__((packed)) {
    uint8_t v_p_x_cc;
    uint8_t m_pt;
    uint16_t seq;
    uint32_t ts;
    uint32_t ssrc;
} rtp_header_t;

static volatile int g_running = 1;

static char *output_file = NULL;
static char *stream_host = NULL;
static int stream_port = 0;
static int target_bitrate = 2000000;
static int target_fps = 30;
static int target_duration = 0;
static int target_width = 1280;
static int target_height = 720;
static char *video_device = VIDEO_CAPTURE_FRONT_DEVICE;

static void handle_signal(int sig) {
    (void)sig;
    g_running = 0;
}

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options] <mode> <output>\n"
        "\n"
        "Modes:\n"
        "  record <file.h264>    Record H.264 video to file\n"
        "  stream <host:port>    Stream H.264 over RTP/UDP\n"
        "\n"
        "Options:\n"
        "  -b <bps>     Bitrate in bits/sec (default: 2000000)\n"
        "  -f <fps>     Target framerate (default: 30)\n"
        "  -d <sec>     Duration limit, 0=infinite (default: 0)\n"
        "  -w <px>      Capture width (default: 1280)\n"
        "  -h <px>      Capture height (default: 720)\n"
        "  -c <dev>     Video device (default: /dev/video0)\n",
        prog);
}

static int parse_args(int argc, char **argv) {
    int opt;
    while ((opt = getopt(argc, argv, "b:f:d:w:h:c:")) != -1) {
        switch (opt) {
            case 'b': target_bitrate = atoi(optarg); break;
            case 'f': target_fps = atoi(optarg); break;
            case 'd': target_duration = atoi(optarg); break;
            case 'w': target_width = atoi(optarg); break;
            case 'h': target_height = atoi(optarg); break;
            case 'c': video_device = optarg; break;
            default: print_usage(argv[0]); return -1;
        }
    }

    if (optind >= argc) {
        fprintf(stderr, "Missing mode argument\n");
        print_usage(argv[0]);
        return -1;
    }

    const char *mode = argv[optind++];

    if (strcmp(mode, "record") == 0) {
        if (optind >= argc) {
            fprintf(stderr, "Missing output filename for record mode\n");
            return -1;
        }
        output_file = argv[optind];
    } else if (strcmp(mode, "stream") == 0) {
        if (optind >= argc) {
            fprintf(stderr, "Missing host:port for stream mode\n");
            return -1;
        }
        char *colon = strchr(argv[optind], ':');
        if (!colon) {
            fprintf(stderr, "Invalid host:port format\n");
            return -1;
        }
        *colon = '\0';
        stream_host = argv[optind];
        stream_port = atoi(colon + 1);
        if (stream_port <= 0 || stream_port > 65535) {
            fprintf(stderr, "Invalid port\n");
            return -1;
        }
    } else {
        fprintf(stderr, "Unknown mode: %s\n", mode);
        print_usage(argv[0]);
        return -1;
    }

    return 0;
}

static int open_rtp_socket(void) {
    if (!stream_host) return -1;

    struct addrinfo hints, *res;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    char port_str[16];
    snprintf(port_str, sizeof(port_str), "%d", stream_port);

    int ret = getaddrinfo(stream_host, port_str, &hints, &res);
    if (ret != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(ret));
        return -1;
    }

    int fd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (fd < 0) {
        perror("socket");
        freeaddrinfo(res);
        return -1;
    }

    if (connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
        perror("connect");
        close(fd);
        freeaddrinfo(res);
        return -1;
    }

    freeaddrinfo(res);
    return fd;
}

static int send_rtp(int fd, rtp_header_t *hdr,
                    const uint8_t *nal, int nal_size) {
    struct iovec iov[2];
    iov[0].iov_base = hdr;
    iov[0].iov_len = sizeof(rtp_header_t);
    iov[1].iov_base = (void*)nal;
    iov[1].iov_len = nal_size;

    struct msghdr msg;
    memset(&msg, 0, sizeof(msg));
    msg.msg_iov = iov;
    msg.msg_iovlen = 2;

    return sendmsg(fd, &msg, 0);
}

static int find_nal_units(const uint8_t *data, int size,
                          int (*callback)(const uint8_t *nal, int nal_size,
                                          int nal_type, void *ctx),
                          void *ctx) {
    int i = 0;

    if (size < 4)
        return 0;

    while (i < size - 3) {
        if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 1)
            break;
        if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1)
            break;
        i++;
    }

    if (i >= size - 3) {
        if (size > 0) {
            int nal_type = data[0] & 0x1F;
            return callback(data, size, nal_type, ctx);
        }
        return 0;
    }

    while (i < size - 3) {
        int start_code_len;
        if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 1) {
            start_code_len = 3;
        } else if (data[i] == 0 && data[i+1] == 0 && data[i+2] == 0 && data[i+3] == 1) {
            start_code_len = 4;
        } else {
            i++;
            continue;
        }

        int nal_start = i + start_code_len;
        int j = nal_start;
        int next_start = size;
        while (j < size - 3) {
            if ((data[j] == 0 && data[j+1] == 0 && data[j+2] == 1) ||
                (data[j] == 0 && data[j+1] == 0 && data[j+2] == 0 && data[j+3] == 1)) {
                next_start = j;
                break;
            }
            j++;
        }

        int nal_size = next_start - nal_start;
        if (nal_size > 0) {
            int nal_type = data[nal_start] & 0x1F;
            int ret = callback(data + nal_start, nal_size, nal_type, ctx);
            if (ret != 0) return ret;
        }

        i = next_start;
    }

    return 0;
}

typedef struct {
    int fd;
    rtp_header_t hdr;
    uint16_t seq;
    uint32_t ts;
} rtp_stream_ctx_t;

static int rtp_send_callback(const uint8_t *nal, int nal_size,
                              int nal_type, void *ctx) {
    rtp_stream_ctx_t *sc = (rtp_stream_ctx_t*)ctx;
    uint8_t nri = nal[0] & 0x60;

    if (nal_size <= RTP_MTU) {
        sc->hdr.seq = htons(sc->seq++);
        sc->hdr.ts = htonl(sc->ts);
        sc->hdr.m_pt = 0x80 | RTP_PT_H264; // M=0

        send_rtp(sc->fd, &sc->hdr, nal, nal_size);
    } else {
        int offset = 1;
        int remaining = nal_size - 1;
        int frag = 0;

        while (remaining > 0) {
            uint8_t fu_indicator = nri | 28;
            uint8_t fu_header;

            if (remaining <= RTP_MTU - 2) {
                fu_header = 0x40 | nal_type;
            } else if (frag == 0) {
                fu_header = 0x80 | nal_type;
            } else {
                fu_header = nal_type;
            }

            int frag_size = remaining;
            if (frag_size > RTP_MTU - 2)
                frag_size = RTP_MTU - 2;

            uint8_t frag_buf[2 + frag_size];
            frag_buf[0] = fu_indicator;
            frag_buf[1] = fu_header;
            memcpy(frag_buf + 2, nal + offset, frag_size);

            sc->hdr.seq = htons(sc->seq++);
            sc->hdr.ts = htonl(sc->ts);
            sc->hdr.m_pt = (remaining <= RTP_MTU - 2) ? 0x80 | RTP_PT_H264 : 0x80 | RTP_PT_H264;

            send_rtp(sc->fd, &sc->hdr, frag_buf, 2 + frag_size);

            offset += frag_size;
            remaining -= frag_size;
            frag++;
        }
    }

    return 0;
}

static int record_callback(const uint8_t *nal, int nal_size,
                            int nal_type, void *ctx) {
    (void)nal_type;
    FILE *fp = (FILE*)ctx;
    if (!fp) return -1;

    static const uint8_t startcode[] = {0x00, 0x00, 0x00, 0x01};
    fwrite(startcode, 1, 4, fp);
    fwrite(nal, 1, nal_size, fp);
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        print_usage(argv[0]);
        return 1;
    }

    if (parse_args(argc, argv) != 0)
        return 1;

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    printf("drone_encoder - Parrot AR.Drone 2.0 H.264 Encoder\n");
    printf("  Resolution: %dx%d\n", target_width, target_height);
    printf("  Bitrate:    %d bps\n", target_bitrate);
    printf("  Framerate:  %d fps\n", target_fps);

    if (output_file) {
        printf("  Output:     %s\n", output_file);
    }
    if (stream_host) {
        printf("  Stream:     %s:%d\n", stream_host, stream_port);
    }

    if (dsp_full_init() != 0) {
        fprintf(stderr, "DSP initialization failed\n");
        return 1;
    }

    video_capture_t vc;
    if (video_capture_open(&vc, video_device,
                           target_width, target_height,
                           V4L2_PIX_FMT_UYVY) != 0) {
        fprintf(stderr, "Failed to open video device: %s\n", video_device);
        return 1;
    }

    int actual_width = vc.width;
    int actual_height = vc.height;
    printf("Camera opened: %dx%d (%s)\n", actual_width, actual_height, video_device);

    h264_encoder_t enc;
    if (h264_encoder_open(&enc, actual_width, actual_height,
                          target_bitrate, target_fps) != 0) {
        fprintf(stderr, "Failed to open H.264 encoder\n");
        video_capture_close(&vc);
        return 1;
    }

    uint8_t *nv12_buf = malloc(actual_width * actual_height * 3 / 2);
    if (!nv12_buf) {
        fprintf(stderr, "Failed to allocate NV12 buffer\n");
        h264_encoder_close(&enc);
        video_capture_close(&vc);
        return 1;
    }

    uint8_t *h264_buf = malloc(actual_width * actual_height);
    if (!h264_buf) {
        fprintf(stderr, "Failed to allocate H.264 output buffer\n");
        free(nv12_buf);
        h264_encoder_close(&enc);
        video_capture_close(&vc);
        return 1;
    }

    FILE *fp = NULL;
    if (output_file) {
        fp = fopen(output_file, "wb");
        if (!fp) {
            perror("fopen");
            free(h264_buf);
            free(nv12_buf);
            h264_encoder_close(&enc);
            video_capture_close(&vc);
            return 1;
        }
    }

    int rtp_fd = -1;
    rtp_stream_ctx_t stream_ctx;
    if (stream_host) {
        rtp_fd = open_rtp_socket();
        if (rtp_fd < 0) {
            fprintf(stderr, "Failed to open RTP socket\n");
            if (fp) fclose(fp);
            free(h264_buf);
            free(nv12_buf);
            h264_encoder_close(&enc);
            video_capture_close(&vc);
            return 1;
        }

        memset(&stream_ctx, 0, sizeof(stream_ctx));
        stream_ctx.fd = rtp_fd;
        stream_ctx.seq = 0;
        stream_ctx.ts = 0;
        stream_ctx.hdr.v_p_x_cc = 0x80;
        stream_ctx.hdr.ssrc = htonl(RTP_SSRC);
    }

    if (video_capture_start(&vc) != 0) {
        fprintf(stderr, "Failed to start video capture\n");
        if (rtp_fd >= 0) close(rtp_fd);
        if (fp) fclose(fp);
        free(h264_buf);
        free(nv12_buf);
        h264_encoder_close(&enc);
        video_capture_close(&vc);
        return 1;
    }

    printf("Encoding started. Press Ctrl+C to stop.\n");

    int frame_count = 0;
    uint64_t start_time = 0;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    start_time = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;

    while (g_running) {
        uint8_t *frame_data = NULL;
        size_t frame_size = 0;

        int ret = video_capture_frame(&vc, &frame_data, &frame_size);
        if (ret < 0) {
            fprintf(stderr, "Capture error\n");
            break;
        }
        if (ret == 1) {
            struct timespec req = {0, 1000000};
            nanosleep(&req, NULL);
            continue;
        }

        uyvy_to_nv12(frame_data, nv12_buf, actual_width, actual_height);
        video_capture_release_frame(&vc);

        int h264_size = 0;
        if (h264_encoder_encode(&enc, nv12_buf, h264_buf, &h264_size) != 0) {
            fprintf(stderr, "Encode error at frame %d\n", frame_count);
            continue;
        }

        if (h264_size > 0) {
            if (fp) {
                find_nal_units(h264_buf, h264_size, record_callback, fp);
            }

            if (rtp_fd >= 0) {
                stream_ctx.ts = (uint64_t)frame_count * RTP_CLOCK_RATE / target_fps;
                find_nal_units(h264_buf, h264_size, rtp_send_callback, &stream_ctx);
            }
        }

        frame_count++;

        if (frame_count % 30 == 0) {
            printf("  Frames: %d\r", frame_count);
            fflush(stdout);
        }

        if (target_duration > 0 && frame_count >= target_fps * target_duration)
            break;
    }

    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t end_time = (uint64_t)ts.tv_sec * 1000000000ULL + ts.tv_nsec;
    double elapsed = (double)(end_time - start_time) / 1e9;
    double actual_fps = elapsed > 0 ? frame_count / elapsed : 0;

    printf("\nEncoding stopped.\n");
    printf("  Frames encoded: %d\n", frame_count);
    printf("  Elapsed time:   %.2f s\n", elapsed);
    printf("  Actual FPS:     %.1f\n", actual_fps);

    video_capture_stop(&vc);

    if (fp) {
        printf("  Output file:    %s\n", output_file);
        fclose(fp);
    }

    free(h264_buf);
    free(nv12_buf);
    h264_encoder_close(&enc);
    video_capture_close(&vc);

    if (rtp_fd >= 0) close(rtp_fd);

    return 0;
}
