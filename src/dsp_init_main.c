#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "h264_init.h"

static void print_usage(const char *prog) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "Initialize DSP and H.264 encoder on AR.Drone 2.0\n"
        "\n"
        "Options:\n"
        "  -q          Quiet mode\n"
        "  -c          Check status only, don't init\n"
        "  -r          Register H.264 encoder only (skip baseimage)\n",
        prog);
}

int main(int argc, char **argv) {
    int quiet = 0;
    int check_only = 0;
    int register_only = 0;

    int opt;
    while ((opt = getopt(argc, argv, "qcr")) != -1) {
        switch (opt) {
            case 'q': quiet = 1; break;
            case 'c': check_only = 1; break;
            case 'r': register_only = 1; break;
            default: print_usage(argv[0]); return 1;
        }
    }

    if (check_only) {
        int loaded = dsp_is_loaded();
        int registered = dsp_is_h264enc_registered();
        if (loaded && registered) {
            if (!quiet) printf("DSP: loaded, H.264 encoder: registered\n");
            return 0;
        }
        if (!quiet) {
            if (!loaded) printf("DSP: NOT loaded\n");
            if (!registered) printf("H.264 encoder: NOT registered\n");
        }
        return (loaded && registered) ? 0 : 1;
    }

    if (register_only) {
        if (dsp_is_loaded()) {
            if (!quiet) printf("DSP is running, registering H.264 encoder...\n");
        } else {
            if (!quiet) printf("DSP not running, loading baseimage first...\n");
            if (dsp_load_baseimage() != 0) {
                fprintf(stderr, "Failed to load DSP baseimage\n");
                return 1;
            }
        }

        if (dsp_register_h264enc() != 0) {
            fprintf(stderr, "Failed to register H.264 encoder\n");
            return 1;
        }

        if (!quiet) printf("H.264 encoder registered successfully\n");
        return 0;
    }

    if (dsp_full_init() != 0) {
        fprintf(stderr, "Full DSP initialization failed\n");
        return 1;
    }

    if (!quiet) {
        printf("\nDSP status:\n");
        printf("  Bridge device: %s\n", dsp_is_loaded() ? "present" : "missing");
        printf("  H.264 codec:   %s\n", dsp_is_h264enc_registered() ? "registered" : "not registered");
    }

    return 0;
}
