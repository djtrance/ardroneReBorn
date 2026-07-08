# Formatos de Video y Códecs

## Formatos Soportados por el AR.Drone 2.0

### Salida nativa del dron (streaming a ground station)

| Formato | Códec | Container | Transporte | Resolución |
|---|---|---|---|---|
| H.264 Baseline | H.264/AVC | MP4 fragmentado | RTP/UDP puerto 5000 | 1280x720 @ 30fps |
| H.264 (inferior) | H.264/AVC | MP4 fragmentado | RTP/UDP puerto 5000 | 320x240 @ 60fps |

### Captura raw (para procesamiento onboard)

| Formato | Descripción | Device |
|---|---|---|
| YUV422 (UYVY) | Formato raw de la cámara frontal | `/dev/video0` |
| YUV420 (NV12) | Formato raw de la cámara inferior | `/dev/video1` |

## Pipeline de Video Onboard

### Captura (V4L2)

```c
// Formato típico de captura V4L2
struct v4l2_format fmt;
fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
fmt.fmt.pix.width = 1280;
fmt.fmt.pix.height = 720;
fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;  // YUV422
fmt.fmt.pix.field = V4L2_FIELD_NONE;
```

### Encoding (DSP H.264)

El DSP C64x+ codifica desde YUV420 (NV12). Si la entrada es UYVY (YUV422), debe convertirse a NV12 antes de enviar al DSP.

### Formatos Intermedios

| Etapa | Formato | Tamaño (720p) |
|---|---|---|
| Captura V4L2 | UYVY (YUV422) | 1280x720x2 = 1,843,200 bytes |
| Conversión CSC | NV12 (YUV420) | 1280x720x1.5 = 1,382,400 bytes |
| Bitstream H.264 | H.264 comprimido | Variable (~25-100 KB/frame) |

## Ejemplo de GStreamer Pipeline (host-side)

### Recibir video del dron (RTP)

```bash
gst-launch-0.10 udpsrc port=5000 caps="application/x-rtp, media=video, clock-rate=90000, encoding-name=H264" ! \
    rtph264depay ! ffdec_h264 ! autovideosink
```

### Pipeline Onboard (codificar raw a H.264)

```
v4l2src device=/dev/video0 ! \
    video/x-raw-yuv, width=1280, height=720, format=UYVY ! \
    ffmpegcolorspace ! video/x-raw-yuv, format=NV12 ! \
    tidspv10enc_h264 ! video/x-h264 ! \
    filesink location=/data/video/output.h264
```

## Referencias

- V4L2: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/
- H.264/AVC: ITU-T Rec. H.264 | ISO/IEC 14496-10
- TI DVSDK H.264 Encoder: documentación del códec m4venc
