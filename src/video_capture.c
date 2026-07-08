#include "video_capture.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

int video_capture_open(video_capture_t *vc, const char *device,
                       int width, int height, uint32_t format) {
    if (!vc || !device) return -1;

    memset(vc, 0, sizeof(video_capture_t));

    vc->fd = open(device, O_RDWR);
    if (vc->fd < 0) {
        perror("open video device");
        return -1;
    }

    // Query device capabilities
    struct v4l2_capability cap;
    if (ioctl(vc->fd, VIDIOC_QUERYCAP, &cap) < 0) {
        perror("VIDIOC_QUERYCAP");
        close(vc->fd);
        return -1;
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
        fprintf(stderr, "Device is not a capture device\n");
        close(vc->fd);
        return -1;
    }

    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
        fprintf(stderr, "Device does not support streaming\n");
        close(vc->fd);
        return -1;
    }

    // Set format
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = format;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(vc->fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("VIDIOC_S_FMT");
        close(vc->fd);
        return -1;
    }

    vc->width = fmt.fmt.pix.width;
    vc->height = fmt.fmt.pix.height;
    vc->format = fmt.fmt.pix.pixelformat;
    vc->frame_size = fmt.fmt.pix.sizeimage;
    vc->bytesperline = fmt.fmt.pix.bytesperline;

    // Request buffers
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = VIDEO_CAPTURE_NUM_BUFFERS;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(vc->fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("VIDIOC_REQBUFS");
        close(vc->fd);
        return -1;
    }

    vc->nbufs = req.count;
    if (vc->nbufs < 2) {
        fprintf(stderr, "Insufficient buffer memory\n");
        close(vc->fd);
        return -1;
    }

    // Map buffers
    for (int i = 0; i < vc->nbufs; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(vc->fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("VIDIOC_QUERYBUF");
            video_capture_close(vc);
            return -1;
        }

        vc->buffers[i].length = buf.length;
        vc->buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                     MAP_SHARED, vc->fd, buf.m.offset);
        if (vc->buffers[i].start == MAP_FAILED) {
            perror("mmap");
            video_capture_close(vc);
            return -1;
        }
    }

    return 0;
}

int video_capture_start(video_capture_t *vc) {
    if (!vc || vc->fd < 0) return -1;

    // Queue all buffers
    for (int i = 0; i < vc->nbufs; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(vc->fd, VIDIOC_QBUF, &buf) < 0) {
            perror("VIDIOC_QBUF");
            return -1;
        }
    }

    // Start streaming
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(vc->fd, VIDIOC_STREAMON, &type) < 0) {
        perror("VIDIOC_STREAMON");
        return -1;
    }

    vc->streaming = 1;
    return 0;
}

int video_capture_frame(video_capture_t *vc, uint8_t **data, size_t *size) {
    if (!vc || !vc->streaming) return -1;

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(vc->fd, VIDIOC_DQBUF, &buf) < 0) {
        if (errno == EAGAIN) return 1; // no frame ready
        perror("VIDIOC_DQBUF");
        return -1;
    }

    vc->last_index = buf.index;

    if (data) *data = vc->buffers[buf.index].start;
    if (size) *size = buf.bytesused;

    return 0;
}

int video_capture_release_frame(video_capture_t *vc) {
    if (!vc || vc->last_index < 0) return -1;

    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = vc->last_index;

    if (ioctl(vc->fd, VIDIOC_QBUF, &buf) < 0) {
        perror("VIDIOC_QBUF");
        return -1;
    }

    vc->last_index = -1;
    return 0;
}

int video_capture_stop(video_capture_t *vc) {
    if (!vc || vc->fd < 0) return -1;

    if (vc->streaming) {
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(vc->fd, VIDIOC_STREAMOFF, &type);
        vc->streaming = 0;
    }

    return 0;
}

void video_capture_close(video_capture_t *vc) {
    if (!vc) return;

    video_capture_stop(vc);

    for (int i = 0; i < vc->nbufs; i++) {
        if (vc->buffers[i].start != MAP_FAILED) {
            munmap(vc->buffers[i].start, vc->buffers[i].length);
        }
    }

    if (vc->fd >= 0) {
        close(vc->fd);
    }

    memset(vc, 0, sizeof(video_capture_t));
}

void uyvy_to_nv12(const uint8_t *uyvy, uint8_t *nv12,
                  int width, int height) {
    int frame_size = width * height;
    uint8_t *y_plane = nv12;
    uint8_t *uv_plane = nv12 + frame_size;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x += 2) {
            int uyvy_idx = (y * width + x) * 2;
            int nv12_y_idx = y * width + x;
            int nv12_uv_idx = (y / 2) * width + x;

            y_plane[nv12_y_idx]     = uyvy[uyvy_idx + 1];
            y_plane[nv12_y_idx + 1] = uyvy[uyvy_idx + 3];

            if (y % 2 == 0) {
                uv_plane[nv12_uv_idx]     = uyvy[uyvy_idx + 2];
                uv_plane[nv12_uv_idx + 1] = uyvy[uyvy_idx];
            }
        }
    }
}
