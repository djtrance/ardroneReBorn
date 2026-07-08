#ifndef H264_ENCODE_H
#define H264_ENCODE_H

#include <stdint.h>

#ifdef USE_TI_CODEC_ENGINE
#include <ti/sdo/ce/Engine.h>
#include <ti/sdo/ce/video/videnc.h>
#endif

typedef struct {
    int width;
    int height;
    int bitrate;
    int fps;
    int initialized;

    int input_size;
    int output_size;

#ifdef USE_TI_CODEC_ENGINE
    Engine_Handle engine;
    VIDENC_Handle encoder;
    void *input_buffer;
    void *output_buffer;
#endif
} h264_encoder_t;

int h264_encoder_open(h264_encoder_t *enc, int width, int height,
                      int bitrate, int fps);
int h264_encoder_encode(h264_encoder_t *enc,
                        const uint8_t *nv12_input,
                        uint8_t *h264_output,
                        int *h264_size);
int h264_encoder_set_bitrate(h264_encoder_t *enc, int bitrate);
int h264_encoder_set_framerate(h264_encoder_t *enc, int fps);
int h264_encoder_request_idr(h264_encoder_t *enc);
void h264_encoder_close(h264_encoder_t *enc);

int h264_write_bitstream(const char *filename,
                         const uint8_t *data, int size,
                         int is_keyframe);

#endif
