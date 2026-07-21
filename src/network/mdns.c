#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdint.h>
#include <errno.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <pthread.h>

#include "mdns.h"
#include "skyproxy.h"

#define MDNS_PORT     5353
#define MDNS_GROUP    "224.0.0.251"
#define MDNS_TTL      120
#define BEBOP_PORT    44444

static int mdns_sock = -1;
static volatile int mdns_running = 0;
static uint32_t drone_ip;

static void pkt_put16(uint8_t *buf, int *pos, uint16_t v) {
    buf[(*pos)++] = (v >> 8) & 0xFF;
    buf[(*pos)++] = v & 0xFF;
}

static void pkt_put32(uint8_t *buf, int *pos, uint32_t v) {
    buf[(*pos)++] = (v >> 24) & 0xFF;
    buf[(*pos)++] = (v >> 16) & 0xFF;
    buf[(*pos)++] = (v >> 8) & 0xFF;
    buf[(*pos)++] = v & 0xFF;
}

static void pkt_put_bytes(uint8_t *buf, int *pos, const uint8_t *data, int len) {
    memcpy(buf + *pos, data, len);
    *pos += len;
}

static int pkt_put_name_uc(uint8_t *buf, int *pos, const uint8_t *name) {
    int start = *pos;
    memcpy(buf + start, name, strlen((const char*)name) + 1);
    *pos += strlen((const char*)name) + 1;
    return start;
}

static void pkt_put_ptr(uint8_t *buf, int *pos, int offset) {
    buf[(*pos)++] = 0xC0 | ((offset >> 8) & 0x0F);
    buf[(*pos)++] = offset & 0xFF;
}

static int dns_name_len(const uint8_t *name) {
    return strlen((const char*)name) + 1;
}

static int build_ptr_response(uint8_t *resp, uint16_t id, const uint8_t *name_svc) {
    int pos = 0;
    uint8_t header[12];
    memset(header, 0, sizeof(header));
    pkt_put16(header, &(int){0}, id);
    pkt_put16(header, &(int){2}, 0x8400);
    pkt_put16(header, &(int){4}, 0);
    pkt_put16(header, &(int){6}, 4);
    pkt_put16(header, &(int){8}, 0);
    pkt_put16(header, &(int){10}, 0);
    pkt_put_bytes(resp, &pos, header, 12);

    int name_len = dns_name_len(name_svc);
    int inst_rdlen = 1 + 11 + 2;

    int off_svc_in_resp = pos;
    pkt_put_name_uc(resp, &pos, name_svc);
    pkt_put16(resp, &pos, 12);
    pkt_put16(resp, &pos, 0x8001);
    pkt_put32(resp, &pos, MDNS_TTL);
    pkt_put16(resp, &pos, inst_rdlen);
    int rdata_start = pos;
    resp[pos++] = 11;
    pkt_put_bytes(resp, &pos, (const uint8_t*)"Bebop-Reborn", 11);
    pkt_put_ptr(resp, &pos, off_svc_in_resp);

    int off_inst = rdata_start;
    int off_local = off_svc_in_resp + name_len - 7;

    pkt_put_ptr(resp, &pos, off_inst);
    pkt_put16(resp, &pos, 33);
    pkt_put16(resp, &pos, 0x8001);
    pkt_put32(resp, &pos, MDNS_TTL);
    pkt_put16(resp, &pos, 2 + 2 + 2 + 15 + 2);
    pkt_put16(resp, &pos, 0);
    pkt_put16(resp, &pos, 0);
    pkt_put16(resp, &pos, BEBOP_PORT);
    resp[pos++] = 14;
    pkt_put_bytes(resp, &pos, (const uint8_t*)"ardrone-reborn", 14);
    pkt_put_ptr(resp, &pos, off_local);

    int off_host_chk = pos - (15 + 2);

    pkt_put_ptr(resp, &pos, off_inst);
    pkt_put16(resp, &pos, 16);
    pkt_put16(resp, &pos, 0x8001);
    pkt_put32(resp, &pos, MDNS_TTL);
    static const uint8_t txt_data[] = {
        9,'p','r','o','d','u','c','t','=','1',
        16,'s','e','r','i','a','l','=','A','R','D','R','O','N','E','0','0','0','1',
        13,'s','o','f','t','w','a','r','e','=','4','.','0','.','8',
    };
    int txt_len = sizeof(txt_data);
    pkt_put16(resp, &pos, txt_len);
    pkt_put_bytes(resp, &pos, txt_data, txt_len);

    pkt_put_ptr(resp, &pos, off_host_chk);
    pkt_put16(resp, &pos, 1);
    pkt_put16(resp, &pos, 0x8001);
    pkt_put32(resp, &pos, MDNS_TTL);
    pkt_put16(resp, &pos, 4);
    pkt_put32(resp, &pos, drone_ip);

    return pos;
}

static int build_short_response(uint8_t *resp, uint16_t id, const uint8_t *name_svc, int off_svc, uint16_t qtype) {
    int pos = 0;
    uint8_t header[12];
    memset(header, 0, sizeof(header));
    pkt_put16(header, &(int){0}, id);
    pkt_put16(header, &(int){2}, 0x8400);
    pkt_put16(header, &(int){4}, 0);
    pkt_put16(header, &(int){6}, 1);
    pkt_put16(header, &(int){8}, 0);
    pkt_put16(header, &(int){10}, 1);
    pkt_put_bytes(resp, &pos, header, 12);

    int name_len = dns_name_len(name_svc);
    int off_name = pos;
    pkt_put_name_uc(resp, &pos, name_svc);
    int off_local = off_name + name_len - 7;

    if (qtype == 33) {
        pkt_put_ptr(resp, &pos, off_name);
        pkt_put16(resp, &pos, 33);
        pkt_put16(resp, &pos, 0x8001);
        pkt_put32(resp, &pos, MDNS_TTL);
        pkt_put16(resp, &pos, 2 + 2 + 2 + 15 + 2);
        pkt_put16(resp, &pos, 0);
        pkt_put16(resp, &pos, 0);
        pkt_put16(resp, &pos, BEBOP_PORT);
        int host_start = pos;
        resp[pos++] = 14;
        pkt_put_bytes(resp, &pos, (const uint8_t*)"ardrone-reborn", 14);
        pkt_put_ptr(resp, &pos, off_local);

        pkt_put_ptr(resp, &pos, host_start);
        pkt_put16(resp, &pos, 1);
        pkt_put16(resp, &pos, 0x8001);
        pkt_put32(resp, &pos, MDNS_TTL);
        pkt_put16(resp, &pos, 4);
        pkt_put32(resp, &pos, drone_ip);
    } else if (qtype == 1) {
        pkt_put_ptr(resp, &pos, off_name);
        pkt_put16(resp, &pos, 1);
        pkt_put16(resp, &pos, 0x8001);
        pkt_put32(resp, &pos, MDNS_TTL);
        pkt_put16(resp, &pos, 4);
        pkt_put32(resp, &pos, drone_ip);
    }

    return pos;
}

static int compare_dns_name(const uint8_t *pkt, int len, int offset, const uint8_t *name) {
    int p = offset;
    int n = 0;
    while (p < len) {
        uint8_t clen = pkt[p];
        if (clen == 0) {
            if (name[n] == 0) return p + 1;
            return -1;
        }
        if (clen & 0xC0) {
            int ptr = ((clen & 0x3F) << 8) | pkt[p + 1];
            return compare_dns_name(pkt, len, ptr, name + n);
        }
        if (name[n] != clen) return -1;
        if (n + 1 + clen > len || p + 1 + clen > len) return -1;
        if (memcmp(pkt + p + 1, name + n + 1, clen) != 0) return -1;
        p += 1 + clen;
        n += 1 + clen;
    }
    return -1;
}

int mdns_init(void) {
    return 0;
}

void *mdns_thread(void *arg) {
    (void)arg;
    mdns_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (mdns_sock < 0) {
        perror("mdns socket");
        return NULL;
    }

    int opt = 1;
    setsockopt(mdns_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(MDNS_PORT);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (bind(mdns_sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("mdns bind");
        close(mdns_sock);
        mdns_sock = -1;
        return NULL;
    }

    struct ip_mreq mreq;
    memset(&mreq, 0, sizeof(mreq));
    mreq.imr_multiaddr.s_addr = inet_addr(MDNS_GROUP);
    mreq.imr_interface.s_addr = INADDR_ANY;

    if (setsockopt(mdns_sock, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0) {
        perror("mdns add membership");
        close(mdns_sock);
        mdns_sock = -1;
        return NULL;
    }

    struct timeval tv = { .tv_sec = 0, .tv_usec = 250000 };
    setsockopt(mdns_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    drone_ip = inet_addr(DRONE_IP);

    printf("mDNS responder active on %s:%d\n", MDNS_GROUP, MDNS_PORT);
    mdns_running = 1;

    uint8_t buf[1500];
    uint8_t resp[1500];

    static const uint8_t NAME_SVC_PTR[] = {
        10, '_','a','r','d','r','o','n','e','3',
         4, '_','t','c','p',
         5, 'l','o','c','a','l',
         0
    };

    static const uint8_t NAME_HOST_PTR[] = {
        14, 'a','r','d','r','o','n','e','-','r','e','b','o','r','n',
         5, 'l','o','c','a','l',
         0
    };

    struct sockaddr_in from;
    socklen_t from_len;

    while (mdns_running) {
        from_len = sizeof(from);
        int n = recvfrom(mdns_sock, buf, sizeof(buf), 0,
                         (struct sockaddr*)&from, &from_len);
        if (n <= 0) continue;

        if (n < 12) continue;

        uint16_t flags = (buf[2] << 8) | buf[3];
        if (flags & 0x8000) continue;

        uint16_t qdcount = (buf[4] << 8) | buf[5];
        if (qdcount == 0) continue;

        uint16_t id = (buf[0] << 8) | buf[1];

        int offset = 12;
        int rlen = 0;
        for (int q = 0; q < qdcount; q++) {
            if (offset >= n) break;

            int name_end = -1;
            int p = offset;
            while (p < n) {
                uint8_t clen = buf[p];
                if (clen == 0) { p++; break; }
                if (clen & 0xC0) { p += 2; break; }
                p += 1 + clen;
            }
            name_end = p;

            if (name_end + 4 > n) break;

            uint16_t qtype = (buf[name_end] << 8) | buf[name_end + 1];

            if (qtype == 12) {
                if (compare_dns_name(buf, n, offset, NAME_SVC_PTR) >= 0)
                    rlen = build_ptr_response(resp, id, NAME_SVC_PTR);
            } else if (qtype == 33) {
                rlen = build_short_response(resp, id, NAME_SVC_PTR, 12, 33);
            } else if (qtype == 1) {
                if (compare_dns_name(buf, n, offset, NAME_HOST_PTR) >= 0)
                    rlen = build_short_response(resp, id, NAME_HOST_PTR, 12, 1);
            }

            offset = name_end + 4;
        }

        if (rlen > 0) {
            struct sockaddr_in to;
            memset(&to, 0, sizeof(to));
            to.sin_family = AF_INET;
            to.sin_port = htons(MDNS_PORT);
            to.sin_addr.s_addr = inet_addr(MDNS_GROUP);
            sendto(mdns_sock, resp, rlen, 0, (struct sockaddr*)&to, sizeof(to));
        }
    }

    close(mdns_sock);
    mdns_sock = -1;
    return NULL;
}

void mdns_stop(void) {
    mdns_running = 0;
}
