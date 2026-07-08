# FFmpeg con soporte para OMAP3530

## Descripción General

FFmpeg puede compilarse para el AR.Drone 2.0 con soporte para:
- Decodificación H.264 (ARM software)
- Encoding H.264 usando el DSP (vía TI Codec Engine)
- Conversión de formatos (colorspace, escalado)
- Muxing/demuxing MP4, TS, AVI

## Compilación Cruzada para AR.Drone 2.0

### Requisitos

- Toolchain: `arm-none-linux-gnueabi-`
- TI DVSDK Codec Engine headers
- Librerías ARM del sysroot del dron

### Configuración

```bash
git clone git://source.ffmpeg.org/ffmpeg.git ffmpeg-omap
cd ffmpeg-omap

export CC=arm-none-linux-gnueabi-gcc
export CXX=arm-none-linux-gnueabi-g++
export AR=arm-none-linux-gnueabi-ar
export RANLIB=arm-none-linux-gnueabi-ranlib

./configure \
    --arch=arm \
    --target-os=linux \
    --cross-prefix=arm-none-linux-gnueabi- \
    --enable-cross-compile \
    --prefix=/opt/arm/ffmpeg \
    --enable-gpl \
    --enable-libx264 \
    --enable-decoder=h264 \
    --enable-encoder=libx264 \
    --disable-doc \
    --disable-programs
```

### DSP H.264 Encoding vía FFmpeg

FFmpeg puede integrarse con el DSP usando el módulo `libavcodec/h264_omap.c` o mediante un patch externo. La implementación típica:

```c
// El encoder H.264 del DSP se accede mediante:
// 1. Capturar frame raw (V4L2)
// 2. Convertir a NV12
// 3. Llamar al Codec Engine (DSP)
// 4. Obtener bitstream H.264
```

## Limitaciones

- FFmpeg con librerías completas pesa ~5-8MB — limitado por espacio en /data
- El códec x264 en ARM es lento (sin NEON en OMAP3530) — no recomendado para tiempo real
- La aceleración DSP requiere parches específicos del fabricante
- Es preferible usar GStreamer con plugins TI para pipeline fluido

## Alternativa Recomendada

En lugar de FFmpeg para encoding, usar **GStreamer 0.10** con:
- `tidspv10enc_h264` — plugin TI para encoding DSP
- `tidspv10dec_h264` — plugin TI para decoding DSP
- Pipeline directo GStreamer → DSP → salida

## Referencias

- Código fuente FFmpeg: https://ffmpeg.org/
- TI DVSDK H.264 encoder patches (en repos upstream)
- `docs/gstreamer_plugins.md` para pipelines GStreamer
