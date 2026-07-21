#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <math.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <time.h>

#include "skyproxy.h"
#include "mdns.h"
#include "recovery.h"

static volatile int running = 1;

static int sc_sock = -1;
static int at_sock = -1;
static int nav_sock = -1;
static int vid_out = -1;
static struct sockaddr_in at_addr;
static uint32_t at_seq = 1;
static uint64_t frame_count = 0;
static uint64_t cmd_count = 0;
static uint64_t video_frames = 0;
static uint64_t video_bytes = 0;
static uint8_t *vid_buf = NULL;
static uint8_t *fu_buf = NULL;

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static struct {
    uint64_t last_navdata_ms;
    uint64_t last_sc_cmd_ms;
    struct sockaddr_in sc_addr;
    int sc_connected;
    int gps_fix;
} g_shared;

static recovery_t g_recovery;

typedef struct __attribute__((packed)) {
    uint16_t tag;
    uint16_t size;
    uint32_t ctrl_state;
    uint32_t battery_percentage;
    uint32_t theta;
    uint32_t phi;
    uint32_t psi;
    int32_t altitude;
    int32_t velocity[3];
} navdata_demo_t;

static uint64_t gettime_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + (uint64_t)ts.tv_nsec / 1000000;
}

static void handle_sigint(int sig) {
    (void)sig;
    running = 0;
}

static int udp_bind(int port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return -1; }
    int opt = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    struct timeval tv = { .tv_sec = 0, .tv_usec = 100000 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(fd); return -1;
    }
    return fd;
}

static int udp_connect(const char *ip, int port) {
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) return -1;
    memset(&at_addr, 0, sizeof(at_addr));
    at_addr.sin_family = AF_INET;
    at_addr.sin_port = htons(port);
    at_addr.sin_addr.s_addr = inet_addr(ip);
    return fd;
}

static void at_send(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (n > 0)
        sendto(at_sock, buf, n, 0, (struct sockaddr*)&at_addr, sizeof(at_addr));
}

static void at_pcmd(const joystick_t *joy) {
    union { float f; int i; } r = { .f = joy->roll };
    union { float f; int i; } p = { .f = joy->pitch };
    union { float f; int i; } y = { .f = joy->yaw };
    union { float f; int i; } g = { .f = joy->gaz };
    at_send("AT*PCMD=%d,1,%d,%d,%d,%d\r", at_seq++, r.i, p.i, g.i, y.i);
}

static void at_pcmd_zero(void) {
    at_send("AT*PCMD=%d,1,0,0,0,0\r", at_seq++);
}

static void at_takeoff(void) {
    at_send("AT*REF=%d,290718208\r", at_seq++);
}

static void at_land(void) {
    at_send("AT*REF=%d,290717696\r", at_seq++);
}

static void at_emergency(void) {
    at_send("AT*REF=%d,290717952\r", at_seq++);
}

static void at_comwdg(void) {
    at_send("AT*COMWDG=%d\r", at_seq++);
}

static void at_config(const char *key, const char *val) {
    at_send("AT*CONFIG=%d,\"%s\",\"%s\"\r", at_seq++, key, val);
}

typedef struct __attribute__((packed)) {
    uint16_t tag;
    uint16_t size;
    double   lat;
    double   lon;
    double   elevation;
    double   hdop;
    int32_t  data_available;
    uint8_t  unk_0[8];
    double   lat0;
    double   lon0;
    double   lat_fuse;
    double   lon_fuse;
    uint32_t gps_state;
    uint8_t  unk_1[40];
    double   vdop;
    double   pdop;
    float    speed;
    uint32_t last_frame_timestamp;
    float    degree;
    float    degree_mag;
    uint8_t  unk_2[16];
    struct {
        uint8_t sat;
        uint8_t cn0;
    } channels[12];
    int32_t  gps_plugged;
    uint8_t  unk_3[108];
    double   gps_time;
    uint16_t week;
    uint8_t  gps_fix;
    uint8_t  num_satellites;
    uint8_t  unk_4[24];
    double   ned_vel_c0;
    double   ned_vel_c1;
    double   ned_vel_c2;
    double   pos_accur_c0;
    double   pos_accur_c1;
    double   pos_accur_c2;
    float    speed_acur;
    float    time_acur;
    uint8_t  unk_5[72];
    float    temperature;
    float    pressure;
} navdata_gps_t;

static void send_telemetry(const telemetry_t *tel) {
    pthread_mutex_lock(&g_lock);
    int connected = g_shared.sc_connected;
    struct sockaddr_in dest;
    memset(&dest, 0, sizeof(dest));
    dest.sin_family = AF_INET;
    dest.sin_port = htons(SC_TELEM_PORT);
    dest.sin_addr.s_addr = g_shared.sc_addr.sin_addr.s_addr;
    pthread_mutex_unlock(&g_lock);

    if (!connected) return;
    char buf[512];
    int n;
    if (tel->gps_fix)
        n = snprintf(buf, sizeof(buf),
            "{\"bat\":%d,\"alt\":%.1f,"
            "\"vx\":%.2f,\"vy\":%.2f,\"vz\":%.2f,"
            "\"phi\":%.2f,\"theta\":%.2f,\"psi\":%.2f"
            ",\"lat\":%.6f,\"lon\":%.6f,"
            "\"gps_alt\":%.1f,\"gps_spd\":%.1f,"
            "\"gps_brg\":%.1f,\"sat\":%d}\n",
            tel->battery, tel->altitude,
            tel->vx, tel->vy, tel->vz,
            tel->phi, tel->theta, tel->psi,
            tel->lat, tel->lon,
            tel->gps_alt, tel->gps_speed,
            tel->gps_bearing, tel->satellites);
    else
        n = snprintf(buf, sizeof(buf),
            "{\"bat\":%d,\"alt\":%.1f,"
            "\"vx\":%.2f,\"vy\":%.2f,\"vz\":%.2f,"
            "\"phi\":%.2f,\"theta\":%.2f,\"psi\":%.2f}\n",
            tel->battery, tel->altitude,
            tel->vx, tel->vy, tel->vz,
            tel->phi, tel->theta, tel->psi);

    sendto(vid_out, buf, n, 0, (struct sockaddr*)&dest, sizeof(dest));
}

static int try_parse_arsdk3(const uint8_t *data, int len, joystick_t *joy) {
    if (len < 4) return -1;
    const arsdk3_cmd_t *cmd = (const arsdk3_cmd_t*)data;
    if (cmd->project != ARDRONE3_PROJECT || cmd->cls != PILOTING_CLASS)
        return -1;

    int hdr_size = sizeof(arsdk3_cmd_t);
    int args_len = len - hdr_size;
    const uint8_t *args = data + hdr_size;

    switch (cmd->cmd) {
    case 0x00: {
        if (args_len < 9) return -1;
        const pcmd_args_t *pcmd = (const pcmd_args_t*)args;
        if (pcmd->flag) {
            joy->roll  = pcmd->roll  / 100.0f;
            joy->pitch = pcmd->pitch / 100.0f;
            joy->yaw   = pcmd->yaw   / 100.0f;
            joy->gaz   = pcmd->gaz   / 100.0f;
        } else {
            joy->roll = joy->pitch = joy->yaw = joy->gaz = 0.0f;
        }
        return 0;
    }
    case 0x01: at_takeoff();  return 1;
    case 0x02: at_land();     return 1;
    case 0x03: at_emergency(); return 1;
    case 0x04:
        at_config("control:flight_without_shell", "TRUE");
        return 1;
    default:
        return -1;
    }
}

static int try_parse_raw_pcmd(const uint8_t *data, int len, joystick_t *joy) {
    if (len < 9) return -1;
    const pcmd_args_t *pcmd = (const pcmd_args_t*)data;
    if (pcmd->flag > 1) return -1;
    if (pcmd->flag) {
        joy->roll  = pcmd->roll  / 100.0f;
        joy->pitch = pcmd->pitch / 100.0f;
        joy->yaw   = pcmd->yaw   / 100.0f;
        joy->gaz   = pcmd->gaz   / 100.0f;
    } else {
        joy->roll = joy->pitch = joy->yaw = joy->gaz = 0.0f;
    }
    return 0;
}

static void *command_thread(void *arg) {
    (void)arg;
    uint8_t buf[2048];
    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    fd_set fds;
    struct timeval tv;
    joystick_t joy;
    int ret_pcmd = -1;

    while (running) {
        FD_ZERO(&fds);
        FD_SET(sc_sock, &fds);
        tv.tv_sec = 0;
        tv.tv_usec = 50000;
        int ret = select(sc_sock + 1, &fds, NULL, NULL, &tv);
        if (ret < 0) continue;
        if (ret == 0) continue;

        from_len = sizeof(from);
        int n = recvfrom(sc_sock, buf, sizeof(buf), 0,
                         (struct sockaddr*)&from, &from_len);
        if (n <= 0) continue;

        uint64_t now = gettime_ms();

        pthread_mutex_lock(&g_lock);
        if (!g_shared.sc_connected ||
            memcmp(&from.sin_addr, &g_shared.sc_addr.sin_addr,
                   sizeof(struct in_addr)) != 0) {
            g_shared.sc_addr = from;
            g_shared.sc_connected = 1;
            printf("Paired with Skycontroller: %s:%d\n",
                   inet_ntoa(from.sin_addr), ntohs(from.sin_port));
            joystick_t zero_joy;
            memset(&zero_joy, 0, sizeof(zero_joy));
            at_pcmd(&zero_joy);
        }
        g_shared.last_sc_cmd_ms = now;
        pthread_mutex_unlock(&g_lock);

        memset(&joy, 0, sizeof(joy));
        ret_pcmd = -1;
        frame_count++;

        if (buf[0] == ARDRONE3_PROJECT)
            ret_pcmd = try_parse_arsdk3(buf, n, &joy);
        if (ret_pcmd < 0)
            ret_pcmd = try_parse_raw_pcmd(buf, n, &joy);

        if (ret_pcmd == 0) {
            at_pcmd(&joy);
            cmd_count++;
        }
    }
    return NULL;
}

static void *navdata_thread(void *arg) {
    (void)arg;
    uint8_t buf[NAVDATA_BUF_SIZE];
    telemetry_t tel;
    memset(&tel, 0, sizeof(tel));

    telemetry_t prev_tel;
    memset(&prev_tel, 0, sizeof(prev_tel));
    uint64_t last_telem_send = 0;

    struct sockaddr_in from;
    socklen_t from_len = sizeof(from);
    fd_set fds;
    struct timeval tv;

    while (running) {
        FD_ZERO(&fds);
        FD_SET(nav_sock, &fds);
        tv.tv_sec = 0;
        tv.tv_usec = 50000;

        int ret = select(nav_sock + 1, &fds, NULL, NULL, &tv);
        uint64_t now = gettime_ms();

        at_comwdg();

        if (ret <= 0) continue;

        from_len = sizeof(from);
        int n = recvfrom(nav_sock, buf, sizeof(buf), 0,
                         (struct sockaddr*)&from, &from_len);
        if (n <= 0) continue;

        now = gettime_ms();
        pthread_mutex_lock(&g_lock);
        g_shared.last_navdata_ms = now;
        pthread_mutex_unlock(&g_lock);

        if (n < 16) continue;
        int offset = 16;
        while (offset + 4 < n) {
            int tag  = buf[offset] | (buf[offset+1] << 8);
            int size = buf[offset+2] | (buf[offset+3] << 8);
            if (tag == 0 && (unsigned int)size >= 44) {
                navdata_demo_t *d = (navdata_demo_t*)&buf[offset];
                tel.battery  = d->battery_percentage;
                tel.altitude = d->altitude / 1000.0f;
                tel.vx       = d->velocity[0] / 1000.0f;
                tel.vy       = d->velocity[1] / 1000.0f;
                tel.vz       = d->velocity[2] / 1000.0f;
                tel.phi      = d->phi   / 1000.0f;
                tel.theta    = d->theta / 1000.0f;
                tel.psi      = d->psi   / 1000.0f;
            }
            if (tag == 27 && (unsigned int)size >= sizeof(navdata_gps_t) - 200) {
                navdata_gps_t *g = (navdata_gps_t*)&buf[offset];
                if (g->gps_plugged && g->gps_fix > 0) {
                    tel.gps_fix    = g->gps_fix;
                    tel.lat        = g->lat;
                    tel.lon        = g->lon;
                    tel.gps_alt    = g->elevation;
                    tel.gps_speed  = g->speed;
                    tel.gps_bearing = g->degree;
                    tel.satellites = g->num_satellites;
                    pthread_mutex_lock(&g_lock);
                    g_shared.gps_fix = g->gps_fix;
                    pthread_mutex_unlock(&g_lock);
                } else {
                    tel.gps_fix = 0;
                }
            }
            offset += size;
            if (offset % 4) offset += 4 - (offset % 4);
        }

        if (now - last_telem_send > 100) {
            if (memcmp(&tel, &prev_tel, sizeof(tel)) != 0) {
                send_telemetry(&tel);
                prev_tel = tel;
                last_telem_send = now;
            }
        }
    }
    return NULL;
}

static int find_all_nal(const uint8_t *data, int len, int *offsets, int max_nal) {
    int count = 0;
    int i = 0;
    while (i < len - 3 && count < max_nal) {
        if (data[i] == 0 && data[i+1] == 0) {
            if (data[i+2] == 0 && data[i+3] == 1) {
                offsets[count++] = i;
                i += 4;
                continue;
            }
            if (data[i+2] == 1) {
                offsets[count++] = i;
                i += 3;
                continue;
            }
        }
        i++;
    }
    return count;
}

static void send_video_data(const uint8_t *data, int len, int flush) {
    static uint8_t *batch = NULL;
    static int batch_len = 0;
    if (!batch) batch = vid_buf;

    if (!data || len == 0) {
        if (flush && batch_len > 0) {
            pthread_mutex_lock(&g_lock);
            int connected = g_shared.sc_connected;
            struct sockaddr_in dest;
            memset(&dest, 0, sizeof(dest));
            dest.sin_family = AF_INET;
            dest.sin_port = htons(VID_DST_PORT);
            dest.sin_addr.s_addr = g_shared.sc_addr.sin_addr.s_addr;
            pthread_mutex_unlock(&g_lock);
            if (connected) {
                sendto(vid_out, batch, batch_len, 0,
                       (struct sockaddr*)&dest, sizeof(dest));
                video_bytes += batch_len;
            }
            batch_len = 0;
        }
        return;
    }

    if (batch_len + len > VIDEO_BUF_SIZE) {
        send_video_data(NULL, 0, 1);
    }
    memcpy(batch + batch_len, data, len);
    batch_len += len;
    video_frames++;
    (void)flush;
}

static int tcp_connect(const char *ip, int port) {
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip);
    struct timeval tv = { .tv_sec = 2, .tv_usec = 0 };
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

typedef struct {
    int fd;
    int type;
    int port;
} video_src_t;

static void *video_relay_thread(void *arg) {
    (void)arg;
    uint8_t sps[MAX_SPS_PPS], pps[MAX_SPS_PPS];
    int sps_len = 0, pps_len = 0;
    fd_set fds;
    struct timeval tv;

    vid_out = socket(AF_INET, SOCK_DGRAM, 0);
    if (vid_out < 0) { perror("vid_out socket"); return NULL; }
    int sndbuf = 262144;
    setsockopt(vid_out, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    vid_buf = malloc(VIDEO_BUF_SIZE);
    fu_buf = malloc(VIDEO_BUF_SIZE);
    if (!vid_buf || !fu_buf) {
        fprintf(stderr, "Video relay: OOM\n");
        free(vid_buf); free(fu_buf);
        if (vid_out >= 0) close(vid_out);
        return NULL;
    }

    video_src_t srcs[MAX_VID_SRCS];
    int n_srcs = 0;

    int tcp_fd = tcp_connect("127.0.0.1", VID_TCP_PORT);
    if (tcp_fd < 0) tcp_fd = tcp_connect("192.168.1.1", VID_TCP_PORT);
    if (tcp_fd >= 0) {
        srcs[n_srcs++] = (video_src_t){ .fd = tcp_fd, .type = 0, .port = VID_TCP_PORT };
        printf("Video relay: TCP %d connected\n", VID_TCP_PORT);
    }

    int udp_fd = udp_bind(VID_UDP_PORT);
    if (udp_fd >= 0)
        srcs[n_srcs++] = (video_src_t){ .fd = udp_fd, .type = 1, .port = VID_UDP_PORT };

    int rtp_fd = udp_bind(VID_RTP_PORT);
    if (rtp_fd >= 0)
        srcs[n_srcs++] = (video_src_t){ .fd = rtp_fd, .type = 2, .port = VID_RTP_PORT };

    if (n_srcs == 0) {
        printf("Video relay: no video sources available\n");
        close(vid_out); vid_out = -1;
        free(vid_buf); free(fu_buf);
        return NULL;
    }
    printf("Video relay: %d source(s) active\n", n_srcs);

    uint8_t *buf = malloc(VIDEO_BUF_SIZE);
    if (!buf) {
        fprintf(stderr, "Video relay: OOM for buf\n");
        close(vid_out);
        free(vid_buf); free(fu_buf);
        return NULL;
    }

    int fu_len = 0, fu_type = 0;
    int nal_offsets[256];

    while (running) {
        FD_ZERO(&fds);
        int max_fd = -1;
        for (int i = 0; i < n_srcs; i++) {
            if (srcs[i].fd >= 0) {
                FD_SET(srcs[i].fd, &fds);
                if (srcs[i].fd > max_fd) max_fd = srcs[i].fd;
            }
        }
        if (max_fd < 0) break;
        tv.tv_sec = 0; tv.tv_usec = 100000;

        int ret = select(max_fd + 1, &fds, NULL, NULL, &tv);
        if (ret <= 0) continue;

        int src_idx = -1;
        for (int i = 0; i < n_srcs; i++) {
            if (srcs[i].fd >= 0 && FD_ISSET(srcs[i].fd, &fds)) {
                src_idx = i; break;
            }
        }
        if (src_idx < 0) continue;

        int n = -1;
        if (srcs[src_idx].type == 0) {
            n = read(srcs[src_idx].fd, buf, VIDEO_BUF_SIZE);
        } else {
            struct sockaddr_in from;
            socklen_t from_len = sizeof(from);
            n = recvfrom(srcs[src_idx].fd, buf, VIDEO_BUF_SIZE, 0,
                         (struct sockaddr*)&from, &from_len);
        }
        if (n <= 0) {
            if (srcs[src_idx].type == 0) {
                printf("Video relay: TCP disconnected\n");
                close(srcs[src_idx].fd);
                srcs[src_idx].fd = -1;
            }
            continue;
        }

        pthread_mutex_lock(&g_lock);
        int connected = g_shared.sc_connected;
        pthread_mutex_unlock(&g_lock);
        if (!connected) continue;

        int is_rtp = (n > 12 && (buf[0] & 0xC0) == 0x80);

        if (is_rtp) {
            int rtp_hdr = 12;
            int pt = buf[1] & 0x7F;
            if (pt != 96 && pt != 97 && pt != 98) continue;
            uint8_t nal_type = buf[rtp_hdr] & 0x1F;

            if (nal_type == 28) {
                uint8_t fu_ind = buf[rtp_hdr];
                uint8_t fu_hdr = buf[rtp_hdr + 1];
                int start = (fu_hdr >> 7) & 1;
                int end   = (fu_hdr >> 6) & 1;
                uint8_t original_nal_type = fu_hdr & 0x1F;
                uint8_t nal_header = (fu_ind & 0xE0) | original_nal_type;
                int payload = rtp_hdr + 2;
                int payload_len = n - payload;

                if (start) {
                    fu_len = 0;
                    fu_type = original_nal_type;
                    if (payload_len >= VIDEO_BUF_SIZE - 1) payload_len = VIDEO_BUF_SIZE - 2;
                    fu_buf[fu_len++] = nal_header;
                    memcpy(fu_buf + fu_len, buf + payload, payload_len);
                    fu_len += payload_len;
                } else if (fu_len > 0 && fu_type == original_nal_type) {
                    int space = VIDEO_BUF_SIZE - fu_len;
                    int copy = payload_len < space ? payload_len : space;
                    memcpy(fu_buf + fu_len, buf + payload, copy);
                    fu_len += copy;
                }

                if (end && fu_len > 0) {
                    uint8_t sc4[] = {0,0,0,1};
                    if (original_nal_type == 5 && sps_len > 0 && pps_len > 0) {
                        send_video_data(sc4, 4, 0);
                        send_video_data(sps, sps_len, 0);
                        send_video_data(sc4, 4, 0);
                        send_video_data(pps, pps_len, 0);
                    }
                    send_video_data(sc4, 4, 0);
                    send_video_data(fu_buf, fu_len, 0);
                    send_video_data(NULL, 0, 1);
                    fu_len = 0;
                }
            } else if (nal_type < 24) {
                uint8_t *nal = buf + rtp_hdr;
                int nal_len = n - rtp_hdr;
                if (nal_type == 7) {
                    sps_len = (nal_len < MAX_SPS_PPS) ? nal_len : MAX_SPS_PPS;
                    memcpy(sps, nal, sps_len);
                } else if (nal_type == 8) {
                    pps_len = (nal_len < MAX_SPS_PPS) ? nal_len : MAX_SPS_PPS;
                    memcpy(pps, nal, pps_len);
                }
                uint8_t sc4[] = {0,0,0,1};
                if (nal_type == 5) {
                    if (sps_len > 0 && pps_len > 0) {
                        send_video_data(sc4, 4, 0);
                        send_video_data(sps, sps_len, 0);
                        send_video_data(sc4, 4, 0);
                        send_video_data(pps, pps_len, 0);
                    }
                }
                send_video_data(sc4, 4, 0);
                send_video_data(nal, nal_len, 0);
                send_video_data(NULL, 0, 1);
            }
        } else {
            int n_nal = find_all_nal(buf, n, nal_offsets, 256);
            if (n_nal == 0) continue;

            for (int i = 0; i < n_nal; i++) {
                int nal_start = nal_offsets[i];
                int sc_len = (i + 1 < n_nal) ? nal_offsets[i+1] - nal_start : n - nal_start;
                int nal_type = buf[nal_start + (buf[nal_start+2] == 1 ? 3 : 4)] & 0x1F;

                if (nal_type == 7) {
                    sps_len = (sc_len < MAX_SPS_PPS) ? sc_len : MAX_SPS_PPS;
                    memcpy(sps, buf + nal_start, sps_len);
                } else if (nal_type == 8) {
                    pps_len = (sc_len < MAX_SPS_PPS) ? sc_len : MAX_SPS_PPS;
                    memcpy(pps, buf + nal_start, pps_len);
                }

                if (nal_type == 5) {
                    if (sps_len > 0 && pps_len > 0) {
                        static const uint8_t sc[] = {0,0,0,1};
                        send_video_data(sc, 4, 0);
                        send_video_data(sps, sps_len, 0);
                        send_video_data(sc, 4, 0);
                        send_video_data(pps, pps_len, 0);
                    }
                }
                send_video_data(buf + nal_start, sc_len, 0);
            }
            send_video_data(NULL, 0, 1);
        }
    }

    free(buf);
    free(vid_buf); vid_buf = NULL;
    free(fu_buf); fu_buf = NULL;
    for (int i = 0; i < n_srcs; i++)
        if (srcs[i].fd >= 0) close(srcs[i].fd);
    if (vid_out >= 0) close(vid_out);
    return NULL;
}

static void *recovery_task(void *arg) {
    (void)arg;
    recovery_init(&g_recovery);
    uint64_t last_comwdg = 0;

    while (running) {
        uint64_t now = gettime_ms();

        pthread_mutex_lock(&g_lock);
        recovery_update(&g_recovery, now,
            g_shared.last_navdata_ms > 0,
            g_shared.last_sc_cmd_ms > 0,
            g_shared.gps_fix);
        recovery_state_t state = recovery_tick(&g_recovery, now);
        pthread_mutex_unlock(&g_lock);

        int drone_connected = g_recovery.last_navdata_ms > 0 &&
            (now - g_recovery.last_navdata_ms) < 5000;

        if (drone_connected && now - last_comwdg > 50) {
            at_comwdg();
            last_comwdg = now;
        }

        switch (state) {
        case RECOVERY_HOVER:
            at_pcmd_zero();
            break;
        case RECOVERY_LAND:
            at_land();
            break;
        case RECOVERY_DRONE_LOST:
            at_pcmd_zero();
            break;
        default:
            break;
        }

        usleep(50000);
    }
    return NULL;
}

static void print_usage(const char *name) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "Skycontroller 2 <-> AR.Drone 2.0 proxy\n\n"
        "Options:\n"
        "  -p <port>     Skycontroller command port (default: %d)\n"
        "  -t <port>     Telemetry port (default: %d)\n"
        "  -d <ip>       Drone IP (default: 192.168.1.1)\n"
        "  -v            Verbose\n"
        "  -h            This help\n",
        name, SC_PORT, SC_TELEM_PORT);
}

int main(int argc, char **argv) {
    int sc_port = SC_PORT;
    char drone_ip[16] = "192.168.1.1";
    int verbose = 0;
    int opt;

    while ((opt = getopt(argc, argv, "p:d:vh")) != -1) {
        switch (opt) {
        case 'p': sc_port = atoi(optarg); break;
        case 'd': strncpy(drone_ip, optarg, 15); drone_ip[15] = '\0'; break;
        case 'v': verbose = 1; break;
        case 'h': print_usage(argv[0]); return 0;
        default:  print_usage(argv[0]); return 1;
        }
    }

    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    memset(&g_shared, 0, sizeof(g_shared));
    recovery_init(&g_recovery);

    printf("=== Skycontroller 2 <-> AR.Drone Proxy ===\n");
    printf("Drone IP: %s\n", drone_ip);
    printf("Listening for Skycontroller on port %d\n", sc_port);
    printf("Video relay: TCP %d / UDP %d / RTP %d -> %d\n",
           VID_TCP_PORT, VID_UDP_PORT, VID_RTP_PORT, VID_DST_PORT);
    printf("Telemetry on port %d\n", SC_TELEM_PORT);

    sc_sock = udp_bind(sc_port);
    if (sc_sock < 0) return 1;

    at_sock = udp_connect(drone_ip, AT_PORT);
    if (at_sock < 0) { close(sc_sock); return 1; }

    nav_sock = udp_bind(NAVDATA_PORT);
    if (nav_sock < 0) { close(sc_sock); close(at_sock); return 1; }

    at_config("general:navdata_demo", "FALSE");
    at_config("general:navdata_options", "777060865");
    at_config("gps:latitude", "0.0");
    at_config("gps:longitude", "0.0");
    at_config("gps:altitude", "0.0");
    at_config("control:flight_without_shell", "TRUE");
    at_seq = 10;
    at_comwdg();
    usleep(50000);

    pthread_t cmd_thr, nav_thr, vid_thr, mdns_thr, rec_thr;
    pthread_create(&cmd_thr, NULL, command_thread, NULL);
    pthread_create(&nav_thr, NULL, navdata_thread, NULL);
    pthread_create(&vid_thr, NULL, video_relay_thread, NULL);
    pthread_create(&mdns_thr, NULL, mdns_thread, NULL);
    pthread_create(&rec_thr, NULL, recovery_task, NULL);

    printf("\nProxy running. Connect Skycontroller 2 to drone WiFi\n");
    printf("(SSID: ardrone2_xxxx) and launch FreeFlight Pro.\n\n");

    while (running) {
        usleep(500000);
        if (verbose && g_shared.sc_connected)
            printf("\rCMDs: %llu | Video frames: %llu (%llu KB) | %s",
                   (unsigned long long)cmd_count,
                   (unsigned long long)video_frames,
                   (unsigned long long)(video_bytes / 1024),
                   recovery_state_name(g_recovery.state));
    }

    printf("\n\nShutting down...\n");
    joystick_t zero_joy;
    memset(&zero_joy, 0, sizeof(zero_joy));
    at_pcmd(&zero_joy);
    running = 0;
    mdns_stop();
    pthread_join(cmd_thr, NULL);
    pthread_join(nav_thr, NULL);
    pthread_join(vid_thr, NULL);
    pthread_join(mdns_thr, NULL);
    pthread_join(rec_thr, NULL);
    close(sc_sock);
    close(at_sock);
    close(nav_sock);

    printf("Stats: %llu commands, %llu video frames, %llu KB sent\n",
           (unsigned long long)cmd_count,
           (unsigned long long)video_frames,
           (unsigned long long)(video_bytes / 1024));
    return 0;
}
