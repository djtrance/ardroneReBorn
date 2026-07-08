#include "h264_encode.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#ifdef USE_TI_CODEC_ENGINE
#include <ti/sdo/ce/Engine.h>
#include <ti/sdo/ce/video/videnc.h>
#include <ti/sdo/ce/osal/Memory.h>
#endif

int h264_encoder_open(h264_encoder_t *enc, int width, int height,
                      int bitrate, int fps) {
    if (!enc) return -1;

    memset(enc, 0, sizeof(h264_encoder_t));
    enc->width = width;
    enc->height = height;
    enc->bitrate = bitrate;
    enc->fps = fps;

#ifdef USE_TI_CODEC_ENGINE
    // Open Codec Engine
    Engine_Error errorcode;
    enc->engine = Engine_open("encode", NULL, &errorcode);
    if (!enc->engine) {
        fprintf(stderr, "Failed to open Codec Engine (error %d)\n", errorcode);
        return -1;
    }

    // Configure encoder params
    VIDENC_Params vp;
    memset(&vp, 0, sizeof(vp));
    vp.size = sizeof(vp);
    vp.maxWidth = width;
    vp.maxHeight = height;
    vp.rateControlPreset = IVIDEO_NONE;

    enc->encoder = VIDENC_create(enc->engine, "h264enc", &vp);
    if (!enc->encoder) {
        fprintf(stderr, "Failed to create H.264 encoder\n");
        Engine_close(enc->engine);
        return -1;
    }

    // Set dynamic params
    VIDENC_DynamicParams dp;
    memset(&dp, 0, sizeof(dp));
    dp.size = sizeof(dp);
    dp.inputWidth = width;
    dp.inputHeight = height;
    dp.targetBitRate = bitrate;
    dp.refFrameRate = fps * 1000;
    dp.targetFrameRate = fps * 1000;

    // Allocate input buffer (NV12: width*height*1.5)
    int input_size = width * height * 3 / 2;
    enc->input_buffer = Memory_contigAlloc(input_size, 128);
    if (!enc->input_buffer) {
        fprintf(stderr, "Failed to allocate contiguous input buffer\n");
        VIDENC_delete(enc->encoder);
        Engine_close(enc->engine);
        return -1;
    }

    // Allocate output buffer
    int output_size = width * height;  // worst-case
    enc->output_buffer = Memory_contigAlloc(output_size, 128);
    if (!enc->output_buffer) {
        fprintf(stderr, "Failed to allocate contiguous output buffer\n");
        Memory_contigFree(enc->input_buffer, input_size);
        VIDENC_delete(enc->encoder);
        Engine_close(enc->engine);
        return -1;
    }

    enc->input_size = input_size;
    enc->output_size = output_size;
    enc->initialized = 1;
#else
    // Stub: encoder initialized without TI Codec Engine
    enc->input_size = width * height * 3 / 2;
    enc->output_size = width * height;
    enc->initialized = 1;
#endif

    printf("H.264 encoder opened: %dx%d @ %d fps, %d kbps\n",
           width, height, fps, bitrate);
    return 0;
}

int h264_encoder_encode(h264_encoder_t *enc,
                        const uint8_t *nv12_input,
                        uint8_t *h264_output,
                        int *h264_size) {
    if (!enc || !enc->initialized) return -1;
    if (!nv12_input || !h264_output || !h264_size) return -1;

#ifdef USE_TI_CODEC_ENGINE
    XDM_BufDesc inBuf, outBuf;
    IVIDENC_InArgs inArgs;
    IVIDENC_OutArgs outArgs;

    // Copy input to contiguous buffer
    memcpy(enc->input_buffer, nv12_input, enc->input_size);

    // Setup buffer descriptors
    XDAS_Int8 *inBufPtrs[XDM_MAX_IO_BUFFERS] = { enc->input_buffer };
    XDAS_Int32 inBufSizes[XDM_MAX_IO_BUFFERS] = { enc->input_size };
    XDAS_Int8 *outBufPtrs[XDM_MAX_IO_BUFFERS] = { enc->output_buffer };
    XDAS_Int32 outBufSizes[XDM_MAX_IO_BUFFERS] = { enc->output_size };

    inBuf.numBufs = 1;
    inBuf.bufs = inBufPtrs;
    inBuf.bufSizes = inBufSizes;

    outBuf.numBufs = 1;
    outBuf.bufs = outBufPtrs;
    outBuf.bufSizes = outBufSizes;

    memset(&inArgs, 0, sizeof(inArgs));
    inArgs.size = sizeof(inArgs);

    memset(&outArgs, 0, sizeof(outArgs));
    outArgs.size = sizeof(outArgs);

    // Process through DSP
    Int32 status = VIDENC_process(enc->encoder, &inBuf, &outBuf,
                                  &inArgs, &outArgs);
    if (status != VIDENC_EOK) {
        fprintf(stderr, "VIDENC_process failed with status %d\n", (int)status);
        return -1;
    }

    *h264_size = outArgs.bytesGenerated;
    memcpy(h264_output, enc->output_buffer, *h264_size);
#else
    // Stub: return empty H.264 data (placeholder)
    // In real usage, this calls the TI Codec Engine
    *h264_size = 0;
#endif

    return 0;
}

int h264_encoder_set_bitrate(h264_encoder_t *enc, int bitrate) {
    if (!enc || !enc->initialized) return -1;
    enc->bitrate = bitrate;

#ifdef USE_TI_CODEC_ENGINE
    VIDENC_DynamicParams dp;
    VIDENC_Status status;
    memset(&dp, 0, sizeof(dp));
    dp.size = sizeof(dp);
    dp.targetBitRate = bitrate;

    VIDENC_control(enc->encoder, XDM_SETPARAMS, &dp, &status);
#endif

    return 0;
}

int h264_encoder_set_framerate(h264_encoder_t *enc, int fps) {
    if (!enc || !enc->initialized) return -1;
    enc->fps = fps;

#ifdef USE_TI_CODEC_ENGINE
    VIDENC_DynamicParams dp;
    VIDENC_Status status;
    memset(&dp, 0, sizeof(dp));
    dp.size = sizeof(dp);
    dp.refFrameRate = fps * 1000;
    dp.targetFrameRate = fps * 1000;

    VIDENC_control(enc->encoder, XDM_SETPARAMS, &dp, &status);
#endif

    return 0;
}

int h264_encoder_request_idr(h264_encoder_t *enc) {
    if (!enc || !enc->initialized) return -1;

#ifdef USE_TI_CODEC_ENGINE
    VIDENC_DynamicParams dp;
    VIDENC_Status status;
    memset(&dp, 0, sizeof(dp));
    dp.size = sizeof(dp);
    dp.forceIFrame = 1;

    VIDENC_control(enc->encoder, XDM_SETPARAMS, &dp, &status);
#endif

    return 0;
}

void h264_encoder_close(h264_encoder_t *enc) {
    if (!enc) return;

#ifdef USE_TI_CODEC_ENGINE
    int input_size = enc->input_size;
    int output_size = enc->output_size;

    if (enc->encoder) {
        VIDENC_delete(enc->encoder);
    }
    if (enc->engine) {
        Engine_close(enc->engine);
    }
    if (enc->input_buffer) {
        Memory_contigFree(enc->input_buffer, input_size);
    }
    if (enc->output_buffer) {
        Memory_contigFree(enc->output_buffer, output_size);
    }
#endif

    memset(enc, 0, sizeof(h264_encoder_t));
}

// Utility: write raw H.264 bitstream to file
int h264_write_bitstream(const char *filename,
                         const uint8_t *data, int size,
                         int is_keyframe) {
    if (!filename || !data || size <= 0) return -1;

    FILE *fp = fopen(filename, "ab");
    if (!fp) return -1;

    // Write H.264 Annex B start code
    if (is_keyframe) {
        // SPS/PPS before keyframe (would need encoder to provide these)
        // Simplified: assume caller prepends them
    }

    static const uint8_t startcode[] = {0x00, 0x00, 0x00, 0x01};
    fwrite(startcode, 1, 4, fp);
    fwrite(data, 1, size, fp);

    fclose(fp);
    return 0;
}
