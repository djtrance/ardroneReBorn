/*
 * test_uvc_camera.c -- Userspace UVC camera test via libusb
 *
 * Tests USB Video Class (UVC) cameras WITHOUT kernel driver.
 * Uses libusb to:
 *   1. Enumerate USB devices, identify UVC cameras
 *   2. Open camera, select format/resolution
 *   3. Stream video frames to memory
 *   4. Optionally convert to grayscale for vision pipeline
 *
 * This avoids needing the uvcvideo kernel module on the drone.
 * The drone has libusb-1.0.so and libusb-0.1.so available.
 *
 * Build:
 *   arm-none-linux-gnueabi-gcc -marm -march=armv7-a -mtune=cortex-a8 \
 *       -mfloat-abi=softfp -Os -std=gnu99 \
 *       test_uvc_camera.c -o test_uvc_camera \
 *       -static-libgcc -lusb-1.0 -lpthread \
 *       -Wl,--dynamic-linker=/lib/ld-linux.so.3
 *
 * Usage:
 *   ./test_uvc_camera [options]
 *
 * Options:
 *   -l            List UVC cameras only
 *   -d <dev>      USB device (bus:dev), e.g. 001:003
 *   -w <width>    Width (default: 640)
 *   -h <height>   Height (default: 480)
 *   -f <fps>      Target framerate (default: 30)
 *   -n <frames>   Number of frames to capture (default: 100)
 *   -g            Save first frame as grayscale PGM
 *   -v            Verbose UVC descriptors
 *
 * Protocol reference: UVC 1.5 spec (USB-IF)
 *   VideoControl interface: controls unit/terminal
 *   VideoStreaming interface: format/frame descriptors + payload
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <signal.h>

/* The drone has libusb-1.0 */
#include <libusb-1.0/libusb.h>

/* ---------------------------------------------------------------
 * UVC constants (USB Video Class 1.5)
 * --------------------------------------------------------------- */
#define UVC_CLASS_VIDEO            14  /* bInterfaceClass for UVC */
#define UVC_SUBCLASS_VS           2   /* VideoStreaming */
#define UVC_SUBCLASS_VC           1   /* VideoControl */

#define UVC_VS_PROBE_CONTROL      0x01
#define UVC_VS_COMMIT_CONTROL     0x02
#define UVC_VS_STREAM_ERROR_CONTROL 0x03

#define UVC_SET_CUR     0x01
#define UVC_GET_CUR     0x81
#define UVC_GET_MIN     0x82
#define UVC_GET_MAX     0x83
#define UVC_GET_RES     0x84
#define UVC_GET_LEN     0x85
#define UVC_GET_INFO    0x86
#define UVC_GET_DEF     0x87

/* Video streaming interface class-specific request codes */
#define UVC_RC_UNDEFINED           0x00
#define UVC_SET_CUR                0x01
#define UVC_GET_CUR                0x81
#define UVC_GET_MIN                0x82
#define UVC_GET_MAX                0x83
#define UVC_GET_RES                0x84
#define UVC_GET_LEN                0x85
#define UVC_GET_INFO               0x86
#define UVC_GET_DEF                0x87

#define UVC_REQ_TYPE_SET          0x21
#define UVC_REQ_TYPE_GET          0xa1

/* GUID for uncompressed YUY2 format */
static const uint8_t GUID_YUY2[16] = {
    0x59, 0x55, 0x59, 0x32, 0x00, 0x00, 0x10, 0x00,
    0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71
};

/* Probe/Commit structure (UVC 1.5, 4.3.1.1) */
struct __attribute__((packed)) uvc_streaming_control {
    uint16_t bmHint;
    uint8_t  bFormatIndex;
    uint8_t  bFrameIndex;
    uint32_t dwFrameInterval;
    uint16_t wKeyFrameRate;
    uint16_t wPFrameRate;
    uint16_t wCompQuality;
    uint16_t wCompWindowSize;
    uint16_t wDelay;
    uint32_t dwMaxVideoFrameSize;
    uint32_t dwMaxPayloadTransferSize;
    uint32_t dwClockFrequency;
    uint8_t  bmFramingInfo;
    uint8_t  bPreferedVersion;
    uint8_t  bMinVersion;
    uint8_t  bMaxVersion;
    uint8_t  bInterfaceNumber;
};

/* ---------------------------------------------------------------
 * State
 * --------------------------------------------------------------- */
static volatile int g_running = 1;
static void handle_signal(int sig) {
    (void)sig;
    g_running = 0;
}

/* ---------------------------------------------------------------
 * USB helpers
 * --------------------------------------------------------------- */
static void check(int rc, const char *msg) {
    if (rc < 0) {
        fprintf(stderr, "ERROR %s: %s\n", msg, libusb_error_name(rc));
        exit(1);
    }
}

/* ---------------------------------------------------------------
 * UVC descriptor parsing
 * --------------------------------------------------------------- */
static void parse_vc_header(const uint8_t *desc, int len) {
    if (len < 12) return;
    printf("  VC Header: bcdUVC=%x.%x, "
           "in_cols=%d, out_cols=%d\n",
           desc[4] | (desc[5] << 8),
           desc[3],
           desc[6], desc[7]);
}

static void parse_vs_format(const uint8_t *desc, int len) {
    if (len < 8) return;
    uint8_t bFormatIndex = desc[3];
    uint8_t bNumFrameDescs = desc[4];
    printf("  VS Format[%d]: %d frame types, "
           "bits=%d, flags=0x%02x\n",
           bFormatIndex, bNumFrameDescs,
           desc[5], desc[6]);

    /* Identify format type from GUID */
    if (len >= 16+8) {
        uint8_t guid[16];
        memcpy(guid, desc+8, 16);
        if (memcmp(guid, GUID_YUY2, 16) == 0) {
            printf("    Format: YUY2 (uncompressed)\n");
        } else if (guid[0] == 'H' && guid[1] == '2' &&
                   guid[2] == '6' && guid[3] == '4') {
            printf("    Format: H.264 (compressed)\n");
        } else {
            printf("    Format: GUID=");
            for (int i = 0; i < 16; i++)
                printf("%02x", guid[i]);
            printf("\n");
        }
    }
}

static void parse_vs_frame(const uint8_t *desc, int len) {
    if (len < 26) return;
    uint8_t bFrameIndex = desc[3];
    uint32_t wWidth = desc[4] | (desc[5] << 8);
    uint32_t wHeight = desc[6] | (desc[7] << 8);
    uint32_t dwMinInterval = *(uint32_t *)(desc+8);
    uint32_t dwMaxInterval = *(uint32_t *)(desc+20);
    uint32_t dwDefaultInterval = *(uint32_t *)(desc+12);

    printf("    Frame[%d]: %dx%d, %d-%d fps",
           bFrameIndex, wWidth, wHeight,
           dwMaxInterval ? 10000000/dwMaxInterval : 0,
           dwMinInterval ? 10000000/dwMinInterval : 0);
    if (len >= 30 && desc[2] == 6) {
        /* Frame descriptor with expected interval */
        uint32_t dwExpectedInterval = *(uint32_t *)(desc+26);
        if (dwExpectedInterval)
            printf(" (expected %d fps)", 10000000/dwExpectedInterval);
    }
    printf("\n");
}

static void parse_uvc_descriptors(libusb_device *dev) {
    struct libusb_config_descriptor *config;
    if (libusb_get_active_config_descriptor(dev, &config) < 0)
        return;

    printf("  UVC Descriptors:\n");
    for (int i = 0; i < config->bNumInterfaces; i++) {
        const struct libusb_interface *iface = &config->interface[i];
        for (int a = 0; a < iface->num_altsetting; a++) {
            const struct libusb_interface_descriptor *alt = &iface->altsetting[a];
            if (alt->bInterfaceClass != UVC_CLASS_VIDEO)
                continue;

            /* Parse class-specific descriptors in extra data */
            for (int e = 0; e < alt->extra_length; ) {
                const uint8_t *desc = alt->extra + e;
                int dlen = desc[0];
                if (dlen < 2 || e + dlen > alt->extra_length)
                    break;
                uint8_t subtype = desc[2];

                if (alt->bInterfaceSubClass == UVC_SUBCLASS_VC) {
                    if (subtype == 1) /* VS_UNDEFINED */
                        parse_vc_header(desc, dlen);
                } else if (alt->bInterfaceSubClass == UVC_SUBCLASS_VS) {
                    if (subtype == 2)
                        parse_vs_format(desc, dlen);
                    else if (subtype == 3 || subtype == 6)
                        parse_vs_frame(desc, dlen);
                }
                e += dlen;
            }
        }
    }
    libusb_free_config_descriptor(config);
}

/* ---------------------------------------------------------------
 * UVC camera enumeration
 * --------------------------------------------------------------- */
static int is_uvc_camera(libusb_device *dev) {
    struct libusb_device_descriptor desc;
    libusb_get_device_descriptor(dev, &desc);
    if (desc.bDeviceClass == UVC_CLASS_VIDEO)
        return 1;

    /* Check for interface-level UVC class */
    struct libusb_config_descriptor *config;
    if (libusb_get_active_config_descriptor(dev, &config) < 0)
        return 0;
    int found = 0;
    for (int i = 0; i < config->bNumInterfaces && !found; i++) {
        for (int a = 0; a < config->interface[i].num_altsetting && !found; a++) {
            if (config->interface[i].altsetting[a].bInterfaceClass == UVC_CLASS_VIDEO) {
                found = 1;
            }
        }
    }
    libusb_free_config_descriptor(config);
    return found;
}

static void list_cameras(libusb_context *ctx) {
    libusb_device **list;
    ssize_t cnt = libusb_get_device_list(ctx, &list);
    if (cnt < 0) return;

    int found = 0;
    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device *dev = list[i];
        if (!is_uvc_camera(dev)) continue;

        struct libusb_device_descriptor desc;
        libusb_get_device_descriptor(dev, &desc);
        found = 1;
        printf("UVC Camera: bus=%03d dev=%03d "
               "VID=%04x PID=%04x\n",
               libusb_get_bus_number(dev),
               libusb_get_device_address(dev),
               desc.idVendor, desc.idProduct);

        /* Get manufacturer/product strings */
        libusb_device_handle *handle;
        if (libusb_open(dev, &handle) == 0) {
            char buf[256];
            if (desc.iManufacturer) {
                if (libusb_get_string_descriptor_ascii(handle,
                        desc.iManufacturer, (unsigned char*)buf, sizeof(buf)) > 0)
                    printf("  Manufacturer: %s\n", buf);
            }
            if (desc.iProduct) {
                if (libusb_get_string_descriptor_ascii(handle,
                        desc.iProduct, (unsigned char*)buf, sizeof(buf)) > 0)
                    printf("  Product: %s\n", buf);
            }
            libusb_close(handle);
        }
        parse_uvc_descriptors(dev);
    }
    if (!found)
        printf("No UVC cameras found\n");

    libusb_free_device_list(list, 1);
}

/* ---------------------------------------------------------------
 * UVC streaming control
 * --------------------------------------------------------------- */
static int uvc_probe_commit(libusb_device_handle *handle,
                            uint8_t vs_iface,
                            struct uvc_streaming_control *ctrl,
                            uint8_t req)
{
    return libusb_control_transfer(
        handle, UVC_REQ_TYPE_SET,
        req, UVC_VS_PROBE_CONTROL << 8, vs_iface,
        (unsigned char*)ctrl, sizeof(*ctrl), 5000);
}

static int uvc_get_probe(libusb_device_handle *handle,
                         uint8_t vs_iface,
                         struct uvc_streaming_control *ctrl)
{
    return libusb_control_transfer(
        handle, UVC_REQ_TYPE_GET,
        UVC_GET_CUR, UVC_VS_PROBE_CONTROL << 8, vs_iface,
        (unsigned char*)ctrl, sizeof(*ctrl), 5000);
}

static int uvc_set_commit(libusb_device_handle *handle,
                          uint8_t vs_iface,
                          struct uvc_streaming_control *ctrl)
{
    return libusb_control_transfer(
        handle, UVC_REQ_TYPE_SET,
        UVC_SET_CUR, UVC_VS_COMMIT_CONTROL << 8, vs_iface,
        (unsigned char*)ctrl, sizeof(*ctrl), 5000);
}

/* ---------------------------------------------------------------
 * Main
 * --------------------------------------------------------------- */
int main(int argc, char **argv) {
    int opt;
    int list_only = 0;
    int verbose = 0;
    int save_pgm = 0;
    int target_width = 640, target_height = 480;
    int target_fps = 30, nframes = 100;
    const char *usb_dev = NULL;

    while ((opt = getopt(argc, argv, "ld:w:h:f:n:gv")) != -1) {
        switch (opt) {
        case 'l': list_only = 1; break;
        case 'd': usb_dev = optarg; break;
        case 'w': target_width = atoi(optarg); break;
        case 'h': target_height = atoi(optarg); break;
        case 'f': target_fps = atoi(optarg); break;
        case 'n': nframes = atoi(optarg); break;
        case 'g': save_pgm = 1; break;
        case 'v': verbose = 1; break;
        default:
            fprintf(stderr, "Usage: %s [-l] [-d bus:dev] "
                    "[-w W] [-h H] [-f FPS] [-n N] [-g] [-v]\n", argv[0]);
            return 1;
        }
    }

    signal(SIGINT, handle_signal);

    libusb_context *ctx = NULL;
    check(libusb_init(&ctx), "libusb_init");

    if (list_only) {
        list_cameras(ctx);
        libusb_exit(ctx);
        return 0;
    }

    /* Find UVC camera */
    printf("Scanning for UVC cameras...\n");
    libusb_device **list;
    ssize_t cnt = libusb_get_device_list(ctx, &list);
    if (cnt < 0) {
        fprintf(stderr, "Failed to get device list\n");
        return 1;
    }

    libusb_device *cam_dev = NULL;
    int cam_bus = 0, cam_addr = 0;

    for (ssize_t i = 0; i < cnt; i++) {
        libusb_device *dev = list[i];
        if (!is_uvc_camera(dev)) continue;

        int bus = libusb_get_bus_number(dev);
        int addr = libusb_get_device_address(dev);

        /* If user specified a device, match */
        if (usb_dev) {
            char devstr[32];
            snprintf(devstr, sizeof(devstr), "%03d:%03d", bus, addr);
            if (strcmp(usb_dev, devstr) == 0) {
                cam_dev = dev;
                cam_bus = bus;
                cam_addr = addr;
                break;
            }
        } else {
            /* Pick first */
            cam_dev = dev;
            cam_bus = bus;
            cam_addr = addr;
            struct libusb_device_descriptor ddesc;
            libusb_get_device_descriptor(dev, &ddesc);
            printf("Auto-selected: bus=%03d dev=%03d "
                   "VID=%04x PID=%04x\n",
                   bus, addr, ddesc.idVendor, ddesc.idProduct);
            break;
        }
    }

    if (!cam_dev) {
        if (usb_dev)
            printf("UVC camera %s not found\n", usb_dev);
        else
            printf("No UVC cameras found. Try -l to list.\n");
        libusb_free_device_list(list, 1);
        libusb_exit(ctx);
        return 1;
    }

    libusb_device_handle *handle;
    check(libusb_open(cam_dev, &handle), "libusb_open");

    /* Detach kernel driver if active */
    for (int i = 0; i < 4; i++) {
        if (libusb_kernel_driver_active(handle, i) == 1) {
            printf("Detaching kernel driver from interface %d\n", i);
            libusb_detach_kernel_driver(handle, i);
        }
    }

    if (verbose)
        parse_uvc_descriptors(cam_dev);

    libusb_free_device_list(list, 1);

    /* Claim interface */
    int vc_iface = -1, vs_iface = -1;
    struct libusb_config_descriptor *config;
    libusb_get_active_config_descriptor(cam_dev, &config);
    for (int i = 0; i < config->bNumInterfaces; i++) {
        const struct libusb_interface *iface = &config->interface[i];
        if (iface->num_altsetting == 0) continue;
        int cls = iface->altsetting[0].bInterfaceClass;
        int sub = iface->altsetting[0].bInterfaceSubClass;
        if (cls == UVC_CLASS_VIDEO) {
            if (sub == UVC_SUBCLASS_VC) vc_iface = i;
            if (sub == UVC_SUBCLASS_VS) vs_iface = i;
        }
    }
    libusb_free_config_descriptor(config);

    if (vc_iface < 0 || vs_iface < 0) {
        fprintf(stderr, "Cannot find VC/VS interfaces\n");
        libusb_close(handle);
        libusb_exit(ctx);
        return 1;
    }

    printf("VC interface: %d, VS interface: %d\n", vc_iface, vs_iface);
    check(libusb_claim_interface(handle, vc_iface), "claim VC");
    check(libusb_claim_interface(handle, vs_iface), "claim VS");

    /* Select alternate setting 1 for streaming (isochronous) */
    check(libusb_set_interface_alt_setting(handle, vs_iface, 1),
          "set alt setting");

    /* Probe format */
    struct uvc_streaming_control probe;
    memset(&probe, 0, sizeof(probe));
    probe.bFormatIndex = 1;
    probe.bFrameIndex = 1;
    probe.dwFrameInterval = 10000000 / target_fps;
    probe.dwMaxPayloadTransferSize = 8192;

    int rc = uvc_probe_commit(handle, vs_iface, &probe, UVC_SET_CUR);
    if (rc < 0) {
        printf("Probe SET failed: %s (trying with defaults)\n",
               libusb_error_name(rc));
        probe.bFormatIndex = 1;
        probe.bFrameIndex = 1;
        probe.dwFrameInterval = 333333; /* ~30fps */
        rc = uvc_probe_commit(handle, vs_iface, &probe, UVC_SET_CUR);
    }
    check(rc, "probe set");

    /* Get probe result back (driver fills in actual values) */
    memset(&probe, 0, sizeof(probe));
    check(uvc_get_probe(handle, vs_iface, &probe), "probe get");

    printf("Probe result: %dx%d, format=%d, frame=%d, "
           "interval=%d us (%d fps)\n",
           probe.dwMaxVideoFrameSize >> 16,
           probe.dwMaxVideoFrameSize & 0xFFFF,
           probe.bFormatIndex, probe.bFrameIndex,
           probe.dwFrameInterval,
           probe.dwFrameInterval ? 10000000 / probe.dwFrameInterval : 0);
    printf("Max frame size: %d bytes\n", probe.dwMaxVideoFrameSize);
    printf("Max payload: %d bytes\n", probe.dwMaxPayloadTransferSize);

    /* Commit */
    check(uvc_set_commit(handle, vs_iface, &probe), "commit");

    /* Start ISO streaming */
    unsigned char buf[65536];
    int total_frames = 0;
    int frame_state = 0; /* 0=header, 1=data */
    int frame_bytes = 0;

    printf("\nCapturing %d frames...\n", nframes);
    while (g_running && total_frames < nframes) {
        int transferred;
        rc = libusb_bulk_transfer(handle, 0x81 | (vs_iface << 8),
                                  buf, sizeof(buf),
                                  &transferred, 5000);
        if (rc == LIBUSB_ERROR_TIMEOUT) continue;
        if (rc < 0) {
            fprintf(stderr, "USB error: %s\n", libusb_error_name(rc));
            break;
        }

        /* Parse UVC payload header */
        if (transferred < 2) continue;
        uint8_t header_len = buf[0];
        uint8_t bmHeaderInfo = buf[1];
        int payload_data = transferred - header_len;

        /* bmHeaderInfo bits:
         *   0x01: Frame End (EOF)
         *   0x02: Frame Start (FID changed)
         *   0x04: Presentation Time
         *   0x08: Source Clock
         */
        if (bmHeaderInfo & 0x02) {
            /* End previous frame */
            if (frame_state == 1 && frame_bytes > 0) {
                total_frames++;
                printf("Frame %4d: %d bytes\n",
                       total_frames, frame_bytes);
            }
            frame_state = 1;
            frame_bytes = 0;
        }

        if (frame_state == 1) {
            frame_bytes += payload_data;
        }

        if (bmHeaderInfo & 0x01) {
            /* EOF - end current frame */
            if (frame_state == 1 && frame_bytes > 0) {
                total_frames++;
                printf("Frame %4d: %d bytes (EOF)\n",
                       total_frames, frame_bytes);
            }
            frame_state = 0;
            frame_bytes = 0;
        }
    }

    printf("\n=== Done: %d frames captured ===\n", total_frames);

    /* Stop streaming: select alt setting 0 */
    libusb_set_interface_alt_setting(handle, vs_iface, 0);

    libusb_release_interface(handle, vs_iface);
    libusb_release_interface(handle, vc_iface);
    libusb_close(handle);
    libusb_exit(ctx);
    return 0;
}
