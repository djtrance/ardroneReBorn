#ifndef H264_INIT_H
#define H264_INIT_H

int dsp_init(void);
int dsp_load_baseimage(void);
int dsp_register_h264enc(void);
int dsp_is_loaded(void);
int dsp_is_h264enc_registered(void);
int dsp_full_init(void);

#endif
