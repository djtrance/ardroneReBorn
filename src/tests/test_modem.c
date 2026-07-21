/*
 * test_modem.c — Test 4G/5G USB modem detection, AT commands, and connectivity
 *
 * Usage:
 *   ./test_modem                # Auto-detect and test modem
 *   ./test_modem info           # Show modem info only
 *   ./test_modem connect        # Connect to cellular network
 *   ./test_modem signal         # Show signal quality only
 *   ./test_modem at "AT+CSQ"    # Send arbitrary AT command
 *
 * Compiles with soft-float toolchain for on-drone use.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>

#include "modem.h"

static volatile int g_running = 1;

static void handle_signal(int sig) {
    (void)sig;
    g_running = 0;
    printf("\nSignal received, shutting down...\n");
}

static const char *modem_type_str(modem_type_t t) {
    switch (t) {
    case MODEM_TYPE_CDC_ETHER:   return "CDC Ethernet (Huawei-style)";
    case MODEM_TYPE_CDC_NCM:     return "CDC NCM (ZTE-style)";
    case MODEM_TYPE_QMI_WWAN:    return "QMI WWAN (Quectel/Sierra)";
    case MODEM_TYPE_RNDIS:       return "RNDIS (USB tethering)";
    case MODEM_TYPE_SERIAL_ONLY: return "Serial AT only";
    default:                     return "Unknown";
    }
}

static void signal_to_str(modem_signal_t *sig, char *buf, int size) {
    if (sig->rssi == -999) {
        snprintf(buf, size, "N/A");
    } else {
        snprintf(buf, size, "%d dBm", sig->rssi);
    }
}

static void print_modem_info(void) {
    printf("\n=== Modem Information ===\n");
    printf("Type:     %s\n", modem_type_str(g_modem.type));
    printf("Status:   %s\n",
           g_modem.status == MODEM_STATUS_CONNECTED ? "Connected" :
           g_modem.status == MODEM_STATUS_CONNECTING ? "Connecting..." :
           g_modem.status == MODEM_STATUS_ERROR ? "Error" : "Disconnected");
    printf("Operator: %s\n", g_modem.network.operator_name);
    printf("IMEI:     %s\n", g_modem.network.imei);
    printf("IP:       %s\n", g_modem.network.ip_address);
    printf("APN:      %s\n", g_modem.network.apn);

    if (g_modem.signal.rssi != 0) {
        char sig_str[32];
        signal_to_str(&g_modem.signal, sig_str, sizeof(sig_str));
        printf("Signal:   %s\n", sig_str);
        printf("BER:      %d%%\n", g_modem.signal.ber);
    }
}

int main(int argc, char **argv) {
    signal(SIGINT, handle_signal);

    const char *cmd = (argc > 1) ? argv[1] : "info";

    if (strcmp(cmd, "at") == 0) {
        /* Send AT command directly */
        if (argc < 3) {
            fprintf(stderr, "Usage: %s at \"<command>\"\n", argv[0]);
            return 1;
        }

        if (modem_init() < 0) {
            fprintf(stderr, "No modem detected\n");
            return 1;
        }

        /* Try to open AT port */
        const char *at_devices[] = {
            "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
            "/dev/ttyACM0", "/dev/ttyACM1",
            NULL
        };

        for (int i = 0; at_devices[i]; i++) {
            int fd = open(at_devices[i], O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd >= 0) {
                char resp[256];
                g_modem.at_fd = fd;
                if (modem_at_command(argv[2], resp, sizeof(resp), 5000) == 0) {
                    printf("Response:\n%s\n", resp);
                } else {
                    printf("No response on %s\n", at_devices[i]);
                }
                close(fd);
                g_modem.at_fd = -1;
            }
        }
        return 0;
    }

    printf("=== 4G/5G Modem Test ===\n");
    printf("Plug USB modem, then run.\n\n");

    /* Initialize */
    if (modem_init() < 0) {
        fprintf(stderr, "No compatible USB modem detected.\n");
        fprintf(stderr, "\nCheck lsusb output. Known working modems:\n");
        fprintf(stderr, "  Huawei E3372  (ID 12d1:14db / 12d1:1c05)\n");
        fprintf(stderr, "  Huawei E3276  (ID 12d1:14fe)\n");
        fprintf(stderr, "  ZTE MF833     (ID 19d2:xxxx)\n");
        fprintf(stderr, "  Quectel EC25  (ID 2c7c:0125)\n");
        fprintf(stderr, "\nKernel modules needed:\n");
        fprintf(stderr, "  usbnet.ko, cdc_ether.ko, cdc_acm.ko, cdc_ncm.ko\n");
        return 1;
    }

    print_modem_info();

    if (strcmp(cmd, "info") == 0) {
        modem_shutdown();
        return 0;
    }

    if (strcmp(cmd, "signal") == 0) {
        modem_signal_t sig;
        if (modem_update_signal(&sig) == 0) {
            printf("\n=== Signal Quality ===\n");
            char sig_str[32];
            signal_to_str(&sig, sig_str, sizeof(sig_str));
            printf("RSSI: %s\n", sig_str);
            printf("BER:  %d%%\n", sig.ber);
        } else {
            printf("Could not get signal (AT port may need configuration)\n");
        }
        modem_shutdown();
        return 0;
    }

    if (strcmp(cmd, "connect") == 0) {
        printf("\n=== Connecting to cellular network ===\n");
        if (modem_connect() == 0) {
            printf("Connected!\n");
            printf("IP: %s\n", g_modem.network.ip_address);

            printf("\nPinging internet... ");
            fflush(stdout);
            if (modem_ping_internet()) {
                printf("OK\n");
            } else {
                printf("FAIL (might need routes)\n");
            }

            printf("\nMonitoring signal (30s updates). Ctrl-C to stop.\n");
            while (g_running) {
                modem_update();
                modem_update_signal(&g_modem.signal);
                modem_update_network(&g_modem.network);

                char sig_str[32];
                signal_to_str(&g_modem.signal, sig_str, sizeof(sig_str));
                printf("\r  [%s] Signal: %s | IP: %s | Operator: %s   ",
                       g_modem.network.imei[0] ? g_modem.network.imei : "----",
                       sig_str,
                       g_modem.network.ip_address[0] ? g_modem.network.ip_address : "?.?.?.?",
                       g_modem.network.operator_name);
                fflush(stdout);

                for (int i = 0; i < 30 && g_running; i++)
                    sleep(1);
            }
            printf("\n");
        } else {
            fprintf(stderr, "Connection failed\n");
        }
        modem_shutdown();
        return 0;
    }

    fprintf(stderr, "Usage: %s {info|signal|connect|at <cmd>}\n", argv[0]);
    return 1;
}
