#ifndef VIDEO_CAPTURE_H
#define VIDEO_CAPTURE_H

#include <stdint.h>
#include <sys/mman.h>

#define VIDEO_CAPTURE_NUM_BUFFERS 4
#define VIDEO_CAPTURE_FRONT_DEVICE "/dev/video0"
#define VIDEO_CAPTURE_BOTTOM_DEVICE "/dev/video1"

typedef struct {
    void *start;
    size_t length;
} video_buffer_t;

typedef struct {
    int fd;
    int width;
    int height;
    uint32_t format;
    size_t frame_size;
    int bytesperline;
    int nbufs;
    int streaming;
    int last_index;
    video_buffer_t buffers[VIDEO_CAPTURE_NUM_BUFFERS];
} video_capture_t;

int video_capture_open(video_capture_t *vc, const char *device,
                       int width, int height, uint32_t format);
int video_capture_start(video_capture_t *vc);
int video_capture_frame(video_capture_t *vc, uint8_t **data, size_t *size);
int video_capture_release_frame(video_capture_t *vc);
int video_capture_stop(video_capture_t *vc);
void video_capture_close(video_capture_t *vc);

void uyvy_to_nv12(const uint8_t *uyvy, uint8_t *nv12,
                  int width, int height);

#endif
