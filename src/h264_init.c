#include "h264_init.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define DSP_BASEIMAGE_PATH "/opt/arm/tidsp-binaries-23.i3.8/baseimage.dof"
#define DSP_M4VENC_PATH   "/opt/arm/tidsp-binaries-23.i3.8/m4venc_sn.dll64P"
#define DSP_PATH_ENV       "DSP_PATH=/opt/arm/tidsp-binaries-23.i3.8/"
#define GST_BIN_PATH       "PATH=/opt/arm/gst/bin:$PATH"

int dsp_init(void) {
    int ret;

    // Create mount points if they don't exist
    ret = system("mkdir -p /opt/arm");
    if (ret < 0) return -1;

    ret = system("mkdir -p /lib/dsp");
    if (ret < 0) return -1;

    // Bind mount persistent storage to tmpfs locations
    ret = system("mount --bind /data/video/opt/arm /opt/arm");
    if (ret < 0) {
        fprintf(stderr, "Mount bind failed (already mounted?)\n");
    }

    ret = system("mount --bind /data/video/opt/arm/lib/dsp /lib/dsp");
    if (ret < 0) {
        fprintf(stderr, "Mount bind for dsp failed (already mounted?)\n");
    }

    return 0;
}

int dsp_load_baseimage(void) {
    char cmd[256];

    // Kill any conflicting processes first
    system("killall -9 program.elf 2>/dev/null");
    system("killall -9 gst-launch-0.10 2>/dev/null");

    // Set environment
    setenv("DSP_PATH", "/opt/arm/tidsp-binaries-23.i3.8/", 1);

    // Load DSP baseimage (firmware)
    snprintf(cmd, sizeof(cmd),
             "/bin/dspbridge/cexec.out -T %s -v",
             DSP_BASEIMAGE_PATH);
    int ret = system(cmd);
    if (ret != 0) {
        fprintf(stderr, "cexec.out failed (exit %d). DSP may already be running.\n", ret);
        return -1;
    }

    printf("DSP baseimage loaded successfully\n");
    return 0;
}

int dsp_register_h264enc(void) {
    char cmd[256];

    // Register H.264 encoder codec with DSP
    snprintf(cmd, sizeof(cmd),
             "/bin/dspbridge/dynreg.out -r %s -v",
             DSP_M4VENC_PATH);
    int ret = system(cmd);
    if (ret != 0) {
        fprintf(stderr, "dynreg for h264enc failed (exit %d)\n", ret);
        return -1;
    }

    printf("H.264 encoder registered on DSP\n");
    return 0;
}

int dsp_is_loaded(void) {
    // Check if DSP bridge device exists
    FILE *fp = fopen("/dev/dspbridge", "r");
    if (fp) {
        fclose(fp);
        return 1;
    }

    // Check for DSPBridge proc
    fp = popen("ls /proc/dspbridge/ 2>/dev/null | head -1", "r");
    if (fp) {
        char buf[64];
        int found = (fgets(buf, sizeof(buf), fp) != NULL);
        pclose(fp);
        return found;
    }

    return 0;
}

int dsp_is_h264enc_registered(void) {
    // Check if the H.264 dynreg module is visible
    FILE *fp = popen("ls /opt/arm/tidsp-binaries-23.i3.8/m4venc_sn.dll64P 2>/dev/null", "r");
    if (fp) {
        char buf[256];
        int found = (fgets(buf, sizeof(buf), fp) != NULL);
        pclose(fp);
        return found;
    }
    return 0;
}

int dsp_full_init(void) {
    if (dsp_is_loaded()) {
        printf("DSP already initialized\n");
        return 0;
    }

    printf("Starting full DSP initialization...\n");

    if (dsp_init() < 0) {
        fprintf(stderr, "dsp_init failed\n");
        return -1;
    }

    if (dsp_load_baseimage() < 0) {
        // Check if already loaded
        if (!dsp_is_loaded()) {
            fprintf(stderr, "DSP baseimage load failed\n");
            return -1;
        }
        printf("DSP was already running\n");
    }

    if (dsp_register_h264enc() < 0) {
        if (!dsp_is_h264enc_registered()) {
            fprintf(stderr, "H.264 encoder registration failed\n");
            return -1;
        }
        printf("H.264 encoder was already registered\n");
    }

    printf("DSP H.264 acceleration initialized successfully\n");
    return 0;
}
