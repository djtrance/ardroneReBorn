#include "modem.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/sockios.h>

modem_state_t g_modem;

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

/* --- Serial AT port --- */

static int at_set_baud(int fd, int baud) {
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) < 0) return -1;

    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 5;  /* 500ms timeout */

    speed_t speed;
    switch (baud) {
    case 9600:     speed = B9600;     break;
    case 115200:   speed = B115200;   break;
    case 460800:   speed = B460800;   break;
    case 921600:   speed = B921600;   break;
    default:       speed = B115200;
    }

    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    return tcsetattr(fd, TCSANOW, &tio);
}

static int at_open(const char *device, int baud) {
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "MODEM: Cannot open %s: %s\n", device, strerror(errno));
        return -1;
    }
    if (at_set_baud(fd, baud) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

int modem_at_command(const char *cmd, char *response, int resp_size, int timeout_ms) {
    if (g_modem.at_fd < 0) return -1;

    /* Flush pending data */
    tcflush(g_modem.at_fd, TCIOFLUSH);

    /* Send command with \r\n */
    char full_cmd[128];
    snprintf(full_cmd, sizeof(full_cmd), "%s\r\n", cmd);

    if (write(g_modem.at_fd, full_cmd, strlen(full_cmd)) < 0) {
        return -1;
    }

    /* Read response */
    uint64_t deadline = now_ms() + timeout_ms;
    int pos = 0;

    while (now_ms() < deadline && pos < resp_size - 1) {
        int n = read(g_modem.at_fd, response + pos, resp_size - 1 - pos);
        if (n > 0) {
            pos += n;
            response[pos] = 0;
            /* Check for OK or ERROR at end */
            if (strstr(response, "\r\nOK\r\n") || strstr(response, "\r\nERROR\r\n"))
                break;
        } else if (n < 0 && errno != EAGAIN) {
            break;
        } else {
            usleep(50000); /* 50ms */
        }
    }

    if (pos > 0 && response[pos-1] == '\n')
        response[pos-1] = 0;
    if (pos > 0 && response[pos-1] == '\r')
        response[pos-1] = 0;

    return strstr(response, "OK") ? 0 : -1;
}

/* --- Detection --- */

modem_type_t modem_detect(void) {
    FILE *fp = popen("lsusb 2>/dev/null || cat /sys/kernel/debug/usb/devices 2>/dev/null", "r");
    if (!fp) return MODEM_TYPE_UNKNOWN;

    char buf[256];
    modem_type_t detected = MODEM_TYPE_UNKNOWN;

    while (fgets(buf, sizeof(buf), fp)) {
        /* Check for known modem vendor IDs */
        if (strstr(buf, "12d1")) {      /* Huawei */
            if (strstr(buf, "14db") ||  /* E3372 */
                strstr(buf, "1c05") ||  /* E3372 (alternate) */
                strstr(buf, "1c1e") ||  /* E3372h */
                strstr(buf, "14fe")) {  /* E3276 */
                detected = MODEM_TYPE_CDC_ETHER;
                g_modem.type = detected;
                strcpy(g_modem.network.operator_name, "Huawei");
            }
        } else if (strstr(buf, "19d2")) { /* ZTE */
            detected = MODEM_TYPE_CDC_NCM;
            g_modem.type = detected;
            strcpy(g_modem.network.operator_name, "ZTE");
        } else if (strstr(buf, "2c7c")) { /* Quectel */
            detected = MODEM_TYPE_QMI_WWAN;
            g_modem.type = detected;
            strcpy(g_modem.network.operator_name, "Quectel");
        } else if (strstr(buf, "0bdb")) { /* Sierra */
            detected = MODEM_TYPE_QMI_WWAN;
            strcpy(g_modem.network.operator_name, "Sierra");
        } else if (strstr(buf, "0bb4")) { /* Qualcomm RNDIS */
            detected = MODEM_TYPE_RNDIS;
        } else if (strstr(buf, "04e8")) { /* Samsung RNDIS */
            detected = MODEM_TYPE_RNDIS;
        }
    }
    pclose(fp);

    if (detected) {
        printf("MODEM: Detected %s type modem\n", g_modem.network.operator_name);
    } else {
        printf("MODEM: No known modem detected\n");
    }
    return detected;
}

/* --- Network interface --- */

static int interface_up(const char *ifname) {
    struct ifreq ifr;
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0) return -1;

    strncpy(ifr.ifr_name, ifname, IFNAMSIZ-1);
    if (ioctl(sd, SIOCGIFFLAGS, &ifr) < 0) {
        close(sd);
        return -1;
    }
    ifr.ifr_flags |= IFF_UP;
    int rc = ioctl(sd, SIOCSIFFLAGS, &ifr);
    close(sd);
    return rc;
}

static int interface_get_ip(const char *ifname, char *ip, int size) {
    struct ifreq ifr;
    int sd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sd < 0) return -1;

    strncpy(ifr.ifr_name, ifname, IFNAMSIZ-1);
    ifr.ifr_addr.sa_family = AF_INET;

    if (ioctl(sd, SIOCGIFADDR, &ifr) < 0) {
        close(sd);
        return -1;
    }

    struct sockaddr_in *addr = (struct sockaddr_in *)&ifr.ifr_addr;
    strncpy(ip, inet_ntoa(addr->sin_addr), size-1);
    ip[size-1] = 0;
    close(sd);
    return 0;
}

int modem_get_ip(char *ip, int size) {
    const char *ifaces[] = {"eth1", "eth0", "usb0", "wwan0", NULL};
    for (int i = 0; ifaces[i]; i++) {
        if (interface_get_ip(ifaces[i], ip, size) == 0) {
            if (strcmp(ip, "0.0.0.0") != 0) {
                return 0;
            }
        }
    }
    return -1;
}

/* --- Signal quality via AT --- */

int modem_update_signal(modem_signal_t *sig) {
    char resp[256];
    if (modem_at_command("AT+CSQ", resp, sizeof(resp), 3000) < 0)
        return -1;

    /* +CSQ: <rssi>,<ber> */
    int rssi_idx = 0, ber = 0;
    sscanf(resp, "+CSQ: %d,%d", &rssi_idx, &ber);

    /* Convert RSSI index to dBm (3GPP TS 27.007) */
    if (rssi_idx == 0)       sig->rssi = -113;
    else if (rssi_idx == 1)  sig->rssi = -111;
    else if (rssi_idx == 31) sig->rssi = -51;
    else if (rssi_idx == 99) sig->rssi = -999; /* not known */
    else                     sig->rssi = -113 + (rssi_idx * 2);

    sig->ber = ber;
    sig->rsrp = 0;  /* Not available via CSQ */
    sig->rsrq = 0;
    sig->sinr = 0;
    return 0;
}

/* --- Network info via AT --- */

int modem_update_network(modem_network_t *net) {
    char resp[512];

    /* Get operator */
    if (modem_at_command("AT+COPS?", resp, sizeof(resp), 5000) == 0) {
        /* +COPS: <mode>,<format>,<oper>,... */
        char *p = strstr(resp, "\"");
        if (p) {
            p++;
            char *end = strchr(p, '\"');
            if (end) {
                int len = (int)(end - p);
                if (len > 31) len = 31;
                strncpy(net->operator_name, p, len);
                net->operator_name[len] = 0;
            }
        }
    }

    /* IMEI */
    if (modem_at_command("AT+GSN", resp, sizeof(resp), 3000) == 0) {
        sscanf(resp, "%15s", net->imei);
    }

    /* IP address */
    modem_get_ip(net->ip_address, sizeof(net->ip_address));
    if (strlen(net->ip_address) > 0) {
        g_modem.has_ip = true;
    }
    return 0;
}

/* --- Connect / Disconnect --- */

int modem_connect(void) {
    if (g_modem.status == MODEM_STATUS_CONNECTED)
        return 0;

    g_modem.status = MODEM_STATUS_CONNECTING;
    printf("MODEM: Connecting to cellular network...\n");

    /* Try to find the right AT port */
    const char *at_devices[] = {
        "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
        "/dev/ttyACM0", "/dev/ttyACM1",
        NULL
    };

    for (int i = 0; at_devices[i]; i++) {
        g_modem.at_fd = at_open(at_devices[i], MODEM_AT_BAUD);
        if (g_modem.at_fd >= 0) {
            char resp[64];
            if (modem_at_command("AT", resp, sizeof(resp), 2000) == 0) {
                printf("MODEM: AT port at %s\n", at_devices[i]);
                break;
            }
            close(g_modem.at_fd);
            g_modem.at_fd = -1;
        }
    }

    if (g_modem.at_fd < 0) {
        fprintf(stderr, "MODEM: No AT command port found\n");
        g_modem.status = MODEM_STATUS_ERROR;
        return -1;
    }

    /* Set full functionality */
    modem_at_command("AT+CFUN=1", NULL, 0, 3000);

    /* Attach to packet domain (auto for most modems) */
    modem_at_command("AT+CGATT=1", NULL, 0, 10000);

    /* Set APN if known */
    if (strlen(g_modem.network.apn) > 0) {
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"",
                 g_modem.network.apn);
        modem_at_command(cmd, NULL, 0, 5000);
    }

    /* Try DHCP on each possible interface */
    const char *ifaces[] = {"eth1", "eth0", "usb0", "wwan0", NULL};
    for (int i = 0; ifaces[i]; i++) {
        interface_up(ifaces[i]);
        /* Check if interface gets IP */
        char ip[16];
        usleep(2000000); /* wait 2s for DHCP */
        if (interface_get_ip(ifaces[i], ip, sizeof(ip)) == 0 && strcmp(ip, "0.0.0.0") != 0) {
            printf("MODEM: Got IP %s on %s\n", ip, ifaces[i]);
            g_modem.has_ip = true;
            break;
        }
    }

    if (g_modem.has_ip) {
        g_modem.status = MODEM_STATUS_CONNECTED;
        g_modem.connected_us = now_ms();
        printf("MODEM: Connected\n");
        return 0;
    } else {
        /* If busybox udhcpc is available, try that too */
        FILE *fp = popen("which udhcpc 2>/dev/null", "r");
        char path[64];
        if (fp && fgets(path, sizeof(path), fp)) {
            pclose(fp);
            path[strcspn(path, "\n")] = 0;
            if (strlen(path) > 0) {
                printf("MODEM: Trying DHCP via udhcpc...\n");
                char cmd[128];
                for (int i = 0; ifaces[i]; i++) {
                    snprintf(cmd, sizeof(cmd),
                             "udhcpc -i %s -n -q -t 5 2>/dev/null", ifaces[i]);
                    int rc = system(cmd);
                    if (rc == 0) {
                        g_modem.has_ip = true;
                        g_modem.status = MODEM_STATUS_CONNECTED;
                        printf("MODEM: DHCP succeeded on %s\n", ifaces[i]);
                        return 0;
                    }
                }
            }
        }

        fprintf(stderr, "MODEM: Could not get IP address\n");
        g_modem.status = MODEM_STATUS_ERROR;
        return -1;
    }
}

int modem_disconnect(void) {
    if (g_modem.at_fd >= 0) {
        modem_at_command("AT+CGATT=0", NULL, 0, 5000);
        close(g_modem.at_fd);
        g_modem.at_fd = -1;
    }
    g_modem.status = MODEM_STATUS_DISCONNECTED;
    g_modem.has_ip = false;
    printf("MODEM: Disconnected\n");
    return 0;
}

bool modem_ping_internet(void) {
    /* Try ping to Google DNS */
    int rc = system("ping -c 1 -W 3 8.8.8.8 >/dev/null 2>&1 || ping -c 1 -W 3 1.1.1.1 >/dev/null 2>&1");
    return (rc == 0);
}

int modem_restart(void) {
    printf("MODEM: Restarting...\n");
    modem_disconnect();
    sleep(3);
    modem_init();
    return modem_connect();
}

void modem_update(void) {
    /* Periodic signal update */
    static uint64_t last_sig_update = 0;
    uint64_t now = now_ms();

    if (now - last_sig_update > 30000) {  /* every 30s */
        if (g_modem.status == MODEM_STATUS_CONNECTED) {
            modem_update_signal(&g_modem.signal);
            modem_update_network(&g_modem.network);
        }
        last_sig_update = now;
    }
}

int modem_init(void) {
    memset(&g_modem, 0, sizeof(g_modem));
    g_modem.status = MODEM_STATUS_DISCONNECTED;
    g_modem.at_fd = -1;

    modem_type_t type = modem_detect();
    if (type == MODEM_TYPE_UNKNOWN) {
        printf("MODEM: No cellular modem detected. "
               "Plug in Huawei/ZTE/Quectel USB modem.\n");
        return -1;
    }

    return 0;
}

void modem_shutdown(void) {
    modem_disconnect();
    printf("MODEM: Shutdown\n");
}
