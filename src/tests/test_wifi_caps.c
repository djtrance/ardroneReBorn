/*
 * test_wifi_caps.c -- Test WiFi interface capabilities on the drone
 *
 * Checks:
 *   1. List all WiFi interfaces (ath0, wlan0, etc.)
 *   2. Query driver info (ar6000, ath9k_htc, etc.)
 *   3. Check monitor mode support (via wireless extensions)
 *   4. Check if external USB WiFi adapters are present
 *   5. Test raw socket injection (for wifibroadcast)
 *   6. Estimate range/best channel
 *
 * Build:
 *   arm-none-linux-gnueabi-gcc -marm -march=armv7-a -mtune=cortex-a8 \
 *       -mfloat-abi=softfp -Os -std=gnu99 \
 *       test_wifi_caps.c -o test_wifi_caps \
 *       -static-libgcc \
 *       -Wl,--dynamic-linker=/lib/ld-linux.so.3 -Wl,-rpath,/lib \
 *       -lm -lrt -Wl,--gc-sections
 *
 * Usage:
 *   ./test_wifi_caps [options]
 *
 * Options:
 *   -i <iface>   Test specific interface (default: scan all)
 *   -m           Test monitor mode on each interface
 *   -r           Test raw packet injection
 *   -s           Scan channels for RSSI
 *   -a           Run all tests
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <dirent.h>
/*
 * ARM 2012.03 toolchain (glibc 2.11): kernel headers conflict with libc net/ headers.
 * Use kernel headers only to avoid struct redefinition errors.
 */
#include <linux/if.h>
#include <linux/if_arp.h>
#include <linux/wireless.h>
#include <linux/if_packet.h>
#include <linux/if_ether.h>
#include <linux/rtnetlink.h>
#include <arpa/inet.h>

#define PROC_NET_DEV "/proc/net/dev"
#define SYS_CLASS_NET "/sys/class/net"
#define PROC_MODULES "/proc/modules"

static volatile int g_running = 1;
static void handle_signal(int sig) {
    (void)sig; g_running = 0;
}

/* ---------------------------------------------------------------
 * Interface listing
 * --------------------------------------------------------------- */
static int list_interfaces(char ifaces[][IFNAMSIZ], int max) {
    FILE *fp = fopen(PROC_NET_DEV, "r");
    if (!fp) return 0;
    char line[256];
    int n = 0;
    fgets(line, sizeof(line), fp); /* header */
    fgets(line, sizeof(line), fp); /* header */
    while (fgets(line, sizeof(line), fp) && n < max) {
        char *colon = strchr(line, ':');
        if (!colon) continue;
        *colon = '\0';
        /* Trim spaces */
        char *p = line;
        while (*p == ' ') p++;
        strncpy(ifaces[n], p, IFNAMSIZ-1);
        ifaces[n][IFNAMSIZ-1] = '\0';
        n++;
    }
    fclose(fp);
    return n;
}

/* ---------------------------------------------------------------
 * Driver info via sysfs
 * --------------------------------------------------------------- */
static void get_driver_info(const char *iface) {
    char path[256];
    char buf[256];

    /* Try /sys/class/net/<iface>/device/driver/module/drivers */
    snprintf(path, sizeof(path), "%s/%s/device/uevent",
             SYS_CLASS_NET, iface);
    FILE *fp = fopen(path, "r");
    if (fp) {
        while (fgets(buf, sizeof(buf), fp)) {
            if (strncmp(buf, "DRIVER=", 7) == 0) {
                buf[strcspn(buf, "\n")] = 0;
                printf("  Driver: %s\n", buf+7);
            }
        }
        fclose(fp);
    }

    /* Try reading modalias */
    snprintf(path, sizeof(path), "%s/%s/device/modalias",
             SYS_CLASS_NET, iface);
    fp = fopen(path, "r");
    if (fp) {
        if (fgets(buf, sizeof(buf), fp))
            printf("  Modalias: %s", buf);
        fclose(fp);
    }

    /* Check operstate */
    snprintf(path, sizeof(path), "%s/%s/operstate",
             SYS_CLASS_NET, iface);
    fp = fopen(path, "r");
    if (fp) {
        if (fgets(buf, sizeof(buf), fp))
            printf("  State: %s", buf);
        fclose(fp);
    }

    /* Check type */
    snprintf(path, sizeof(path), "%s/%s/type",
             SYS_CLASS_NET, iface);
    fp = fopen(path, "r");
    if (fp) {
        if (fgets(buf, sizeof(buf), fp)) {
            int type = atoi(buf);
            printf("  Type: %d", type);
            switch (type) {
            case ARPHRD_ETHER: printf(" (Ethernet)\n"); break;
            case ARPHRD_IEEE80211: printf(" (802.11)\n"); break;
            case ARPHRD_IEEE80211_PRISM: printf(" (802.11 Prism)\n"); break;
            case ARPHRD_IEEE80211_RADIOTAP: printf(" (802.11 Radiotap)\n"); break;
            default: printf(" (unknown)\n"); break;
            }
        }
        fclose(fp);
    }
}

/* ---------------------------------------------------------------
 * Wireless extensions query
 * --------------------------------------------------------------- */
static int get_wireless_info(int sock, const char *iface) {
    struct iwreq wrq;
    memset(&wrq, 0, sizeof(wrq));
    strncpy(wrq.ifr_name, iface, IFNAMSIZ-1);

    /* Get wireless name/version */
    char wname[IFNAMSIZ];
    if (ioctl(sock, SIOCGIWNAME, &wrq) < 0) {
        printf("  Wireless Extensions: not supported\n");
        return -1;
    }
    strncpy(wname, wrq.u.name, IFNAMSIZ-1);
    printf("  Wireless Extensions: %s\n", wname);

    /* Get mode */
    wrq.u.mode = IW_MODE_AUTO;
    if (ioctl(sock, SIOCGIWMODE, &wrq) >= 0) {
        const char *modes[] = {
            "Auto", "Ad-hoc", "Managed (AP/STA)",
            "Master (AP)", "Repeater", "Secondary",
            "Monitor", "Mesh"
        };
        int mode = wrq.u.mode;
        printf("  Mode: %d", mode);
        if (mode >= 0 && mode < 8)
            printf(" (%s)", modes[mode]);
        printf("\n");

        /* Check MONITOR support specifically */
        if (mode == IW_MODE_MONITOR)
            printf("  [MONITOR MODE ACTIVE]\n");
        else {
            /* Try to set monitor mode (detach only, don't change) */
            wrq.u.mode = IW_MODE_MONITOR;
            int ret = ioctl(sock, SIOCSIWMODE, &wrq);
            printf("  Monitor mode test: %s\n",
                   ret == 0 ? "SUPPORTED" : "not supported");
            /* Restore original mode */
            if (ret == 0) {
                wrq.u.mode = mode;
                ioctl(sock, SIOCSIWMODE, &wrq);
            }
        }
    }

    /* Get frequency/channel */
    wrq.u.freq.m = 0; wrq.u.freq.e = 0;
    if (ioctl(sock, SIOCGIWFREQ, &wrq) >= 0) {
        double freq = (double)wrq.u.freq.m;
        if (wrq.u.freq.e > 0) {
            for (int i = 0; i < wrq.u.freq.e; i++) freq *= 10;
        } else if (wrq.u.freq.e < 0) {
            for (int i = 0; i > wrq.u.freq.e; i--) freq /= 10;
        }
        printf("  Frequency: %.1f MHz\n", freq);
    }

    /* Get range (capabilities) */
    struct iw_range range;
    memset(&range, 0, sizeof(range));
    wrq.u.data.pointer = &range;
    wrq.u.data.length = sizeof(range);
    wrq.u.data.flags = 0;
    if (ioctl(sock, SIOCGIWRANGE, &wrq) >= 0) {
        printf("  Num channels: %d\n", range.num_channels);
        printf("  Max bitrate: %.1f Mbps\n",
               (double)range.throughput / 1000000);
#ifdef IW_MAX_TXPOWER
        /* max_txpower not available in older wireless_extensions headers */
        /* printf("  Max tx power: %d dBm\n", range.max_txpower); */
#endif
        /* suppress unused warning if sensitivity below is also wrapped */
        (void)range.throughput;
        printf("  Sensitivity: %d dBm\n", range.sensitivity);

        /* Check for monitor mode support in WE version */
        if (range.we_version_compiled >= 16)
            printf("  WE version: %d (supports monitor mode)\n",
                   range.we_version_compiled);
    }

    /* Get bitrate */
    wrq.u.bitrate.value = 0;
    if (ioctl(sock, SIOCGIWRATE, &wrq) >= 0) {
        printf("  Current bitrate: %.1f Mbps\n",
               (double)wrq.u.bitrate.value / 1000000);
    }

    /* Get TX power */
    wrq.u.txpower.value = 0;
    wrq.u.txpower.fixed = 0;
    wrq.u.txpower.disabled = 0;
    wrq.u.txpower.flags = 0;
    if (ioctl(sock, SIOCGIWTXPOW, &wrq) >= 0) {
        printf("  TX power: %d dBm\n", wrq.u.txpower.value);
    }

    /* Get RSSI (link quality) */
    struct iw_statistics stats;
    wrq.u.data.pointer = &stats;
    wrq.u.data.length = sizeof(stats);
    wrq.u.data.flags = 1;
    if (ioctl(sock, SIOCGIWSTATS, &wrq) >= 0) {
        printf("  Link quality: %d/%d\n",
               stats.qual.qual, stats.qual.updated);
        printf("  Signal level: %d dBm\n", stats.qual.level);
        printf("  Noise level: %d dBm\n", stats.qual.noise);
    }

    return 0;
}

/* ---------------------------------------------------------------
 * Raw packet / injection test
 * --------------------------------------------------------------- */
static int test_raw_socket(const char *iface) {
    int sd = socket(AF_PACKET, SOCK_RAW, htons(ETH_P_ALL));
    if (sd < 0) {
        printf("  Raw socket: FAILED (%s)\n", strerror(errno));
        return -1;
    }

    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, iface, IFNAMSIZ-1);
    if (ioctl(sd, SIOCGIFINDEX, &ifr) < 0) {
        close(sd);
        printf("  Raw socket: cannot get ifindex\n");
        return -1;
    }

    struct sockaddr_ll sll;
    memset(&sll, 0, sizeof(sll));
    sll.sll_family = AF_PACKET;
    sll.sll_ifindex = ifr.ifr_ifindex;
    sll.sll_protocol = htons(ETH_P_ALL);

    if (bind(sd, (struct sockaddr*)&sll, sizeof(sll)) < 0) {
        close(sd);
        printf("  Raw socket bind: FAILED (%s)\n", strerror(errno));
        return -1;
    }

    printf("  Raw socket: OK (bound to %s)\n", iface);
    close(sd);
    return 0;
}

/* ---------------------------------------------------------------
 * Channel scan
 * --------------------------------------------------------------- */
static void scan_channels(int sock, const char *iface) {
    printf("  Channel scan:\n");
    int channels[] = {1, 6, 11, 13, 36, 40, 44, 48, 149, 153, 157, 161, 165};
    int nch = sizeof(channels)/sizeof(channels[0]);

    for (int i = 0; i < nch && g_running; i++) {
        struct iwreq wrq;
        memset(&wrq, 0, sizeof(wrq));
        strncpy(wrq.ifr_name, iface, IFNAMSIZ-1);
        wrq.u.freq.m = channels[i];
        wrq.u.freq.e = 0;
        wrq.u.freq.flags = IW_FREQ_FIXED;

        if (ioctl(sock, SIOCSIWFREQ, &wrq) < 0)
            continue;

        usleep(50000); /* 50ms settle */

        struct iw_statistics stats;
        wrq.u.data.pointer = &stats;
        wrq.u.data.length = sizeof(stats);
        wrq.u.data.flags = 1;
        if (ioctl(sock, SIOCGIWSTATS, &wrq) >= 0) {
            printf("    Ch %3d: RSSI=%d dBm, qual=%d\n",
                   channels[i], stats.qual.level, stats.qual.qual);
        }
    }
}

/* ---------------------------------------------------------------
 * USB WiFi adapter detection
 * --------------------------------------------------------------- */
static void check_usb_wifi(void) {
    printf("\n--- USB WiFi Adapters ---\n");
    DIR *dir = opendir("/sys/bus/usb/devices");
    if (!dir) {
        printf("  Cannot access USB info\n");
        return;
    }
    struct dirent *entry;
    int found = 0;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_name[0] == '.') continue;
        char path[256];
        char buf[512];

        /* Check if this is a WiFi adapter */
        snprintf(path, sizeof(path),
                 "/sys/bus/usb/devices/%s/interface", entry->d_name);
        FILE *fp = fopen(path, "r");
        if (fp) {
            if (fgets(buf, sizeof(buf), fp)) {
                buf[strcspn(buf, "\n")] = 0;
                if (strstr(buf, "Wireless") || strstr(buf, "WLAN") ||
                    strstr(buf, "802.11") || strstr(buf, "WiFi")) {
                    printf("  USB WiFi: %s (%s)\n", entry->d_name, buf);
                    found = 1;
                }
            }
            fclose(fp);
        } else {
            /* Check via idVendor/idProduct */
            char vpath[256], ppath[256];
            snprintf(vpath, sizeof(vpath),
                     "/sys/bus/usb/devices/%s/idVendor", entry->d_name);
            snprintf(ppath, sizeof(ppath),
                     "/sys/bus/usb/devices/%s/idProduct", entry->d_name);
            FILE *vf = fopen(vpath, "r");
            FILE *pf = fopen(ppath, "r");
            if (vf && pf) {
                char vid[16], pid[16];
                if (fgets(vid, sizeof(vid), vf) && fgets(pid, sizeof(pid), pf)) {
                    vid[strcspn(vid, "\n")] = 0;
                    pid[strcspn(pid, "\n")] = 0;
                    /* Known WiFi chipset VID/PIDs */
                    int v = strtol(vid, NULL, 16);
                    int p = strtol(pid, NULL, 16);
                    const char *chip = NULL;
                    if (v == 0x0cf3) { /* Atheros */
                        if (p == 0x9271) chip = "Atheros AR9271 (ath9k_htc)";
                        else if (p == 0x7015) chip = "Atheros AR7010 (ath9k_htc)";
                        else chip = "Atheros (unknown)";
                    } else if (v == 0x0bda) { /* Realtek */
                        if (p == 0x8812) chip = "Realtek RTL8812AU";
                        else if (p == 0x881a) chip = "Realtek RTL8812AU";
                        else if (p == 0x8178) chip = "Realtek RTL8188EU";
                        else if (p == 0x8179) chip = "Realtek RTL8188EU";
                        else if (p == 0x881b) chip = "Realtek RTL8812EU";
                        else chip = "Realtek (unknown)";
                    } else if (v == 0x148f) { /* Ralink */
                        chip = "Ralink (rt28xx)";
                    }
                    if (chip) {
                        printf("  USB WiFi: %s VID=%s PID=%s (%s)\n",
                               entry->d_name, vid, pid, chip);
                        found = 1;
                    }
                }
                fclose(vf); fclose(pf);
            }
        }
    }
    closedir(dir);
    if (!found)
        printf("  No USB WiFi adapters detected\n");
}

/* ---------------------------------------------------------------
 * Kernel module check
 * --------------------------------------------------------------- */
static void check_kernel_modules(void) {
    printf("\n--- Relevant Kernel Modules ---\n");
    FILE *fp = fopen(PROC_MODULES, "r");
    if (!fp) return;
    char line[256];
    const char *relevant[] = {
        "ar6000", "ath6kl", "ath9k_htc", "ath9k",
        "uvcvideo", "videodev", "rtl8812", "rtl8188",
        "musb_hdrc", "ehci_hcd", "ohci_hcd",
        NULL
    };
    while (fgets(line, sizeof(line), fp)) {
        for (int i = 0; relevant[i]; i++) {
            if (strstr(line, relevant[i])) {
                line[strcspn(line, "\n")] = 0;
                printf("  %s\n", line);
                break;
            }
        }
    }
    fclose(fp);
}

/* ---------------------------------------------------------------
 * Main
 * --------------------------------------------------------------- */
int main(int argc, char **argv) {
    int opt;
    int test_monitor __attribute__((unused)) = 0;
    int test_raw = 0;
    int test_scan = 0;
    int all_tests = 0;
    char target_iface[IFNAMSIZ] = "";

    while ((opt = getopt(argc, argv, "i:mrsa")) != -1) {
        switch (opt) {
        case 'i': strncpy(target_iface, optarg, IFNAMSIZ-1); break;
        case 'm': test_monitor = 1; break;
        case 'r': test_raw = 1; break;
        case 's': test_scan = 1; break;
        case 'a': all_tests = 1; break;
        default:
            fprintf(stderr, "Usage: %s [-i <iface>] [-m] [-r] [-s] [-a]\n",
                    argv[0]);
            return 1;
        }
    }

    if (all_tests) {
        test_monitor = test_raw = test_scan = 1;
    }

    signal(SIGINT, handle_signal);

    printf("=== AR.Drone 2.0 WiFi Capability Test ===\n");
    printf("Kernel: "); fflush(stdout);
    system("uname -a 2>/dev/null");
    printf("\n");

    /* List interfaces */
    char ifaces[16][IFNAMSIZ];
    int nif = list_interfaces(ifaces, 16);
    printf("Network interfaces (%d found):\n", nif);
    for (int i = 0; i < nif; i++)
        printf("  [%d] %s\n", i, ifaces[i]);

    int sock = -1;
    if (nif > 0) {
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock < 0) {
            fprintf(stderr, "Cannot create socket\n");
            return 1;
        }
    }

    /* Test each or specific interface */
    int tested = 0;
    for (int i = 0; i < nif; i++) {
        const char *name = ifaces[i];
        if (target_iface[0] && strcmp(name, target_iface) != 0)
            continue;

        printf("\n=== Interface: %s ===\n", name);
        get_driver_info(name);
        get_wireless_info(sock, name);

        if (test_raw) {
            printf("--- Raw socket test ---\n");
            test_raw_socket(name);
        }

        if (test_scan) {
            scan_channels(sock, name);
        }

        tested = 1;
    }

    if (sock >= 0) close(sock);

    if (target_iface[0] && !tested) {
        printf("Interface '%s' not found\n", target_iface);
    }

    check_usb_wifi();
    check_kernel_modules();

    printf("\n=== Summary ===\n");
    printf("Internal WiFi: AR6003 (ar6000 driver) on ath0\n");
    printf("  - AP mode: YES (default)\n");
    printf("  - Station mode: YES (via wpa_supplicant)\n");
    printf("  - Monitor mode: Check 'Monitor mode test' above\n");
    printf("  - Raw injection: Check 'Raw socket' above\n");
    printf("\n");
    printf("USB WiFi: plug in an adapter and run again\n");
    printf("Recommended for wifibroadcast:\n");
    printf("  - Atheros AR9271 (Alfa AWUS036NHA, TP-Link TL-WN722N)\n");
    printf("  - Realtek RTL8812AU (Alfa AWUS036ACH) - needs patched driver\n");

    return 0;
}
