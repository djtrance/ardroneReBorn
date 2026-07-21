#ifndef MODEM_H
#define MODEM_H

#include <stdint.h>
#include <stdbool.h>

/* 4G/5G USB Modem Control Interface
 *
 * Supports:
 *   - Huawei E3372 / E3276 (cdc_ether / cdc_ncm)
 *   - ZTE MF833 (cdc_ncm / cdc_ether)
 *   - Quectel EC25 (QMI + serial AT)
 *   - Generic RNDIS tethering (usb0)
 *   - Generic CDC ECM / NCM modems (ethX)
 */

#define MODEM_AT_DEVICE   "/dev/ttyUSB0"  /* AT command port */
#define MODEM_NET_IFACE   "eth1"          /* network interface */
#define MODEM_AT_BAUD     115200

typedef enum {
    MODEM_TYPE_UNKNOWN = 0,
    MODEM_TYPE_CDC_ETHER,    /* Huawei E3372, E3276 */
    MODEM_TYPE_CDC_NCM,      /* Huawei E3372 (NCM mode), ZTE */
    MODEM_TYPE_QMI_WWAN,     /* Quectel EC25, Sierra MC7455 */
    MODEM_TYPE_RNDIS,        /* Generic USB tethering */
    MODEM_TYPE_SERIAL_ONLY   /* AT commands only, use external router */
} modem_type_t;

typedef enum {
    MODEM_STATUS_DISCONNECTED = 0,
    MODEM_STATUS_CONNECTING,
    MODEM_STATUS_CONNECTED,
    MODEM_STATUS_ERROR
} modem_status_t;

/* Modem signal quality */
typedef struct {
    int rssi;      /* dBm */
    int rsrp;      /* 4G RSRP, dBm */
    int rsrq;      /* 4G RSRQ, dB */
    int sinr;      /* Signal/Interference+Noise Ratio */
    int ber;       /* Bit Error Rate (percent) */
} modem_signal_t;

/* Modem network info */
typedef struct {
    char operator_name[32];   /* e.g. "Movistar", "T-Mobile" */
    char imei[16];            /* IMEI number */
    char imsi[16];            /* IMSI number */
    char ip_address[16];      /* WAN IP address */
    char apn[32];             /* APN in use */
    int  technology;          /* 0=2G, 1=3G, 2=4G, 3=5G */
    int  band;                /* Frequency band number */
} modem_network_t;

typedef struct {
    modem_type_t    type;
    modem_status_t  status;
    modem_signal_t  signal;
    modem_network_t network;
    int             at_fd;        /* AT command serial FD */
    bool            has_ip;       /* got IP from cellular network */
    uint64_t        connected_us; /* timestamp of connection */
} modem_state_t;

/* Global modem state (accessible by test programs) */
extern modem_state_t g_modem;

/* Initialize modem: detect type, open AT port, start connection */
int modem_init(void);

/* Detect modem type by probing USB devices */
modem_type_t modem_detect(void);

/* Send AT command, wait for response */
int modem_at_command(const char *cmd, char *response, int resp_size, int timeout_ms);

/* Connect to cellular network */
int modem_connect(void);

/* Disconnect */
int modem_disconnect(void);

/* Get current signal quality */
int modem_update_signal(modem_signal_t *sig);

/* Get network info */
int modem_update_network(modem_network_t *net);

/* Get interface IP via DHCP */
int modem_get_ip(char *ip, int size);

/* Check if modem has internet connectivity */
bool modem_ping_internet(void);

/* Restart modem (power cycle via AT) */
int modem_restart(void);

/* Periodic update (call in main loop) */
void modem_update(void);

/* Cleanup */
void modem_shutdown(void);

#endif /* MODEM_H */
