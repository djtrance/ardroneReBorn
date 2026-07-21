/*
 * wfb_tx.c -- Minimal wifibroadcast transmitter for AR.Drone 2.0
 *
 * Reads H.264 video from a UDP source (RTP or raw byte stream)
 * and transmits it over a raw WiFi interface in monitor mode.
 *
 * This implements the original wifibroadcast protocol (befinitiv),
 * NOT wfb-ng (which requires Python/Go on the drone).
 *
 * Architecture:
 *   H.264 encoder (DSP) --> RTP/UDP --> wfb_tx --> raw WiFi
 *
 * Usage on drone:
 *   # Put WiFi card in monitor mode
 *   iw dev wlan0 set monitor otherbss fcsfail
 *   ifconfig wlan0 up
 *   iwconfig wlan0 channel 13
 *
 *   # Start transmitter
 *   ./wfb_tx -i wlan0 -p 5602 -b 8 -r 4
 *
 *   # Start H.264 encoder (drone_encoder or GStreamer)
 *   drone_encoder stream 127.0.0.1:5602 &
 *   # OR: gst-launch ... ! udpsink host=127.0.0.1 port=5602
 *
 * Packet format (befinitiv wifibroadcast):
 *   IEEE 802.11 data frame + radiotap header:
 *     - Data packets: fragment of FEC block
 *     - FEC blocks: K data + N parity packets (Reed-Solomon)
 *
 * Build:
 *   arm-none-linux-gnueabi-gcc -marm -march=armv7-a -mtune=cortex-a8 \
 *       -mfloat-abi=softfp -Os -std=gnu99 \
 *       wfb_tx.c -o wfb_tx \
 *       -static-libgcc -lm -lrt \
 *       -Wl,--dynamic-linker=/lib/ld-linux.so.3 -Wl,-rpath,/lib
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <linux/wireless.h>

/* ---------------------------------------------------------------
 * Configuration
 * --------------------------------------------------------------- */
#define MAX_PAYLOAD_SIZE       1466
#define MAX_PACKET_POOL        512
#define DEFAULT_UDP_PORT       5602
#define DEFAULT_FEC_K           8
#define DEFAULT_FEC_N           12
#define DEFAULT_BLOCK_TIMEOUT   10  /* ms */

/* ---------------------------------------------------------------
 * Radiotap + 802.11 header for wifibroadcast frames
 * --------------------------------------------------------------- */
static const uint8_t wfb_radiotap_header[] = {
    0x00, 0x00,             /* radiotap version */
    0x0f, 0x00,             /* header length = 15 */
    0x04, 0x0c, 0x00, 0x00, /* present flags: rate + tx flags */
    0x00,                   /* data rate (filled in) */
    0x00,                   /* pad */
    0x00, 0x00, 0x00, 0x00, /* pad, not used */
    0x08, 0x00              /* tx flags: no ACK */
};

/* 802.11 data frame header (to DS, broadcast) */
static const uint8_t wfb_80211_header[] = {
    0x08, 0x01,             /* frame control: data, from DS */
    0x00, 0x00,             /* duration */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff,  /* addr1: broadcast */
    0x57, 0x42, 0x00, 0x00, 0x00, 0x01,  /* addr2: wifibroadcast TX MAC */
    0x57, 0x42, 0x00, 0x00, 0x00, 0x01,  /* addr3: BSSID */
    0x00, 0x00              /* seq control */
};

#define WFB_HEADER_SIZE   (sizeof(wfb_radiotap_header) + sizeof(wfb_80211_header))
#define WFB_PACKET_SIZE    (WFB_HEADER_SIZE + MAX_PAYLOAD_SIZE)

/* ---------------------------------------------------------------
 * FEC (Reed-Solomon via Vandermonde - simplified)
 *
 * In production, use libfec or zfec. This is a minimal
 * implementation for testing.
 * --------------------------------------------------------------- */
struct fec_block {
    uint8_t *packets[MAX_PACKET_POOL];
    int     sizes[MAX_PACKET_POOL];
    int     k;       /* data packets */
    int     n;       /* total packets (k data + n-k parity) */
    int     count;   /* packets collected so far */
    uint32_t block_id;
    struct timespec deadline;
};

static struct fec_block g_block;
static int g_block_timeout_ms = DEFAULT_BLOCK_TIMEOUT;

static void fec_block_init(struct fec_block *b, int k, int n) {
    b->k = k;
    b->n = n;
    b->count = 0;
    b->block_id = 0;
    memset(b->packets, 0, sizeof(b->packets));
    memset(b->sizes, 0, sizeof(b->sizes));
}

static int fec_block_add(struct fec_block *b, const uint8_t *data, int len) {
    if (b->count >= b->n) return -1;
    b->packets[b->count] = malloc(len);
    if (!b->packets[b->count]) return -1;
    memcpy(b->packets[b->count], data, len);
    b->sizes[b->count] = len;
    b->count++;
    return b->count - 1;
}

static void fec_block_flush(struct fec_block *b) {
    for (int i = 0; i < b->count; i++) {
        free(b->packets[i]);
        b->packets[i] = NULL;
    }
    b->count = 0;
    b->block_id++;
}

/* ---------------------------------------------------------------
 * Raw WiFi injection
 * --------------------------------------------------------------- */
struct wfb_tx {
    int sd;
    struct sockaddr_ll sll;
    int rate_mbps;  /* data rate for radiotap */
    uint8_t packet_buf[WFB_PACKET_SIZE];
};

static int wfb_tx_open(struct wfb_tx *tx, const char *iface, int rate) {
    tx->sd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (tx->sd < 0) {
        perror("socket");
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, iface, IFNAMSIZ-1);
    if (ioctl(tx->sd, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl SIOCGIFINDEX");
        close(tx->sd);
        return -1;
    }

    memset(&tx->sll, 0, sizeof(tx->sll));
    tx->sll.sll_family = AF_PACKET;
    tx->sll.sll_ifindex = ifr.ifr_ifindex;
    tx->sll.sll_protocol = htons(ETH_P_ALL);

    tx->rate_mbps = rate;

    /* Build packet template */
    memcpy(tx->packet_buf, wfb_radiotap_header, sizeof(wfb_radiotap_header));
    memcpy(tx->packet_buf + sizeof(wfb_radiotap_header),
           wfb_80211_header, sizeof(wfb_80211_header));

    /* Set radiotap rate */
    /* rate index: 2 = 1Mbps, 4 = 2Mbps, 11 = 5.5Mbps, 22 = 11Mbps,
     * 12 = 6Mbps, 18 = 9Mbps, 24 = 12Mbps, 36 = 18Mbps,
     * 48 = 24Mbps, 72 = 36Mbps, 96 = 48Mbps, 108 = 54Mbps */
    uint8_t rate_idx;
    if (rate <= 1) rate_idx = 2;
    else if (rate <= 2) rate_idx = 4;
    else if (rate <= 5) rate_idx = 11;
    else if (rate <= 6) rate_idx = 12;
    else if (rate <= 9) rate_idx = 18;
    else if (rate <= 12) rate_idx = 24;
    else if (rate <= 18) rate_idx = 36;
    else if (rate <= 24) rate_idx = 48;
    else rate_idx = 108;

    tx->packet_buf[8] = rate_idx;

    printf("  TX opened: %s, rate=%d Mbps\n", iface, rate);
    return 0;
}

static int wfb_tx_send(struct wfb_tx *tx, const uint8_t *payload,
                        int payload_len, int flags)
{
    int offset = WFB_HEADER_SIZE;
    if (offset + payload_len > (int)sizeof(tx->packet_buf)) {
        payload_len = sizeof(tx->packet_buf) - offset;
    }

    memcpy(tx->packet_buf + offset, payload, payload_len);

    /* Update sequence number in 802.11 header (seq_num << 4) */
    static uint16_t seq = 0;
    int header_80211_offset = sizeof(wfb_radiotap_header);
    tx->packet_buf[header_80211_offset + 22] = (seq >> 4) & 0xff;
    tx->packet_buf[header_80211_offset + 23] = ((seq & 0xf) << 4) | 0;
    seq++;

    int total_len = offset + payload_len;
    int rc = sendto(tx->sd, tx->packet_buf, total_len, 0,
                    (struct sockaddr*)&tx->sll, sizeof(tx->sll));
    if (rc < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        perror("sendto");
    }
    return rc;
}

static void wfb_tx_close(struct wfb_tx *tx) {
    if (tx->sd >= 0) close(tx->sd);
}

/* ---------------------------------------------------------------
 * UDP receiver
 * --------------------------------------------------------------- */
struct udp_rx {
    int sd;
    struct sockaddr_in local;
};

static int udp_rx_open(struct udp_rx *rx, int port) {
    rx->sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (rx->sd < 0) {
        perror("socket");
        return -1;
    }

    int reuse = 1;
    setsockopt(rx->sd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    memset(&rx->local, 0, sizeof(rx->local));
    rx->local.sin_family = AF_INET;
    rx->local.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    rx->local.sin_port = htons(port);

    if (bind(rx->sd, (struct sockaddr*)&rx->local, sizeof(rx->local)) < 0) {
        perror("bind");
        close(rx->sd);
        return -1;
    }

    printf("UDP RX on 127.0.0.1:%d\n", port);
    return 0;
}

static int udp_rx_recv(struct udp_rx *rx, uint8_t *buf, int bufsize) {
    struct sockaddr_in from;
    socklen_t fromlen = sizeof(from);
    int rc = recvfrom(rx->sd, buf, bufsize, 0,
                      (struct sockaddr*)&from, &fromlen);
    return rc;
}

static void udp_rx_close(struct udp_rx *rx) {
    if (rx->sd >= 0) close(rx->sd);
}

/* ---------------------------------------------------------------
 * Global state
 * --------------------------------------------------------------- */
static volatile int g_running = 1;
static void handle_signal(int sig) {
    (void)sig; g_running = 0;
}

/* ---------------------------------------------------------------
 * Stats
 * --------------------------------------------------------------- */
struct stats {
    unsigned long packets_in;
    unsigned long packets_tx;
    unsigned long bytes_tx;
    unsigned long fec_blocks;
    unsigned long errors;
    struct timeval start;
};

static void print_stats(struct stats *s) {
    struct timeval now;
    gettimeofday(&now, NULL);
    double elapsed = (now.tv_sec - s->start.tv_sec) +
                     (now.tv_usec - s->start.tv_usec) / 1e6;
    if (elapsed < 0.1) elapsed = 0.1;

    printf("\r  IN:%lu TX:%lu BLK:%lu ERR:%lu "
           "%.1f pkt/s %.1f kbps    ",
           s->packets_in, s->packets_tx, s->fec_blocks, s->errors,
           s->packets_tx / elapsed,
           (s->bytes_tx * 8.0) / (elapsed * 1000.0));
    fflush(stdout);
}

/* ---------------------------------------------------------------
 * Main
 * --------------------------------------------------------------- */
int main(int argc, char **argv) {
    const char *iface = "wlan0";
    int udp_port = DEFAULT_UDP_PORT;
    int fec_k = DEFAULT_FEC_K;
    int fec_n = DEFAULT_FEC_N;
    int bitrate_mbps = 12;

    int opt;
    while ((opt = getopt(argc, argv, "i:p:b:k:n:t:h")) != -1) {
        switch (opt) {
        case 'i': iface = optarg; break;
        case 'p': udp_port = atoi(optarg); break;
        case 'b': bitrate_mbps = atoi(optarg); break;
        case 'k': fec_k = atoi(optarg); break;
        case 'n': fec_n = atoi(optarg); break;
        case 't': g_block_timeout_ms = atoi(optarg); break;
        case 'h':
        default:
            printf("Usage: %s [options]\n", argv[0]);
            printf("  -i <iface>   WiFi interface (default: wlan0)\n");
            printf("  -p <port>    UDP input port (default: %d)\n", DEFAULT_UDP_PORT);
            printf("  -b <mbps>    TX bitrate (default: 12)\n");
            printf("  -k <k>       FEC data packets (default: %d)\n", DEFAULT_FEC_K);
            printf("  -n <n>       FEC total packets (default: %d)\n", DEFAULT_FEC_N);
            printf("  -t <ms>      FEC block timeout (default: %d)\n", DEFAULT_BLOCK_TIMEOUT);
            return 1;
        }
    }

    signal(SIGINT, handle_signal);

    printf("=== WFB TX Bridge ===\n");
    printf("  Interface: %s\n", iface);
    printf("  UDP port:  %d\n", udp_port);
    printf("  FEC:       %d/%d\n", fec_k, fec_n);
    printf("  Rate:      %d Mbps\n", bitrate_mbps);
    printf("  MTU:       %d bytes\n", MAX_PAYLOAD_SIZE);
    printf("\n");

    /* Open interfaces */
    struct wfb_tx tx;
    if (wfb_tx_open(&tx, iface, bitrate_mbps) < 0) {
        fprintf(stderr, "Failed to open TX interface\n");
        fprintf(stderr, "Make sure the interface exists and is in monitor mode:\n");
        fprintf(stderr, "  iw dev %s set monitor otherbss fcsfail\n", iface);
        fprintf(stderr, "  ifconfig %s up\n", iface);
        return 1;
    }

    struct udp_rx rx;
    if (udp_rx_open(&rx, udp_port) < 0) {
        wfb_tx_close(&tx);
        return 1;
    }

    /* FEC init */
    fec_block_init(&g_block, fec_k, fec_n);

    struct stats s = {0};
    gettimeofday(&s.start, NULL);

    uint8_t buf[MAX_PAYLOAD_SIZE + 64];
    int64_t block_start_ms = 0;

    printf("Running... (Ctrl-C to stop)\n");
    printf("  Legend: IN=UDP packets, TX=WiFi packets sent, "
           "BLK=FEC blocks, ERR=errors\n");

    while (g_running) {
        int rc = udp_rx_recv(&rx, buf, sizeof(buf));
        if (rc < 0) {
            if (errno == EINTR) continue;
            s.errors++;
            continue;
        }
        if (rc == 0) continue;

        s.packets_in++;

        /* Truncate to max payload */
        int payload_len = rc;
        if (payload_len > MAX_PAYLOAD_SIZE)
            payload_len = MAX_PAYLOAD_SIZE;

        /* Add to FEC block */
        int idx = fec_block_add(&g_block, buf, payload_len);
        if (idx < 0) {
            /* Block full, flush */
            s.errors++;
            fec_block_flush(&g_block);
            continue;
        }

        /* Statistics for deadline tracking */
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int64_t now_ms = (int64_t)now.tv_sec * 1000 +
                         now.tv_nsec / 1000000;

        if (block_start_ms == 0)
            block_start_ms = now_ms;

        /* Transmit the packet immediately */
        rc = wfb_tx_send(&tx, buf, payload_len, 0);
        if (rc > 0) {
            s.packets_tx++;
            s.bytes_tx += rc;
        }

        /* If FEC block is full or timeout reached, send parity */
        int fec_block_full = (g_block.count >= g_block.k);
        int fec_timeout = (now_ms - block_start_ms) > g_block_timeout_ms;

        if ((fec_block_full || fec_timeout) && g_block.count > 0) {
            /* In a minimal implementation, we just send duplicates
             * as parity. Production should use actual RS encoding. */
            int parity_count = g_block.n - g_block.count;
            if (parity_count > 0 && parity_count <= g_block.count) {
                for (int p = 0; p < parity_count; p++) {
                    /* Send a copy of the first data packet as parity */
                    int pi = p % g_block.count;
                    rc = wfb_tx_send(&tx, g_block.packets[pi],
                                     g_block.sizes[pi], 0);
                    if (rc > 0) {
                        s.packets_tx++;
                        s.bytes_tx += rc;
                    }
                }
            }
            s.fec_blocks++;
            fec_block_flush(&g_block);
            block_start_ms = 0;
        }

        /* Periodic stats */
        if (s.packets_in % 100 == 0)
            print_stats(&s);
    }

    print_stats(&s);
    printf("\n\n=== Stopped ===\n");
    printf("Packets in: %lu, TX: %lu, FEC blocks: %lu\n",
           s.packets_in, s.packets_tx, s.fec_blocks);

    fec_block_flush(&g_block);
    udp_rx_close(&rx);
    wfb_tx_close(&tx);

    return 0;
}
