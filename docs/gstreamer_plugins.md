# GStreamer con Plugins de Aceleración

## Descripción General

GStreamer 0.10 es el framework multimedia principal disponible en el AR.Drone 2.0. Los plugins TI DSP (`tidspv10enc_h264`, `tidspv10dec_h264`) permiten codificar/decodificar H.264 usando el DSP C64x+.

## Versión Importante

**GStreamer 0.10** — NO 1.x. Las APIs difieren significativamente:
- Prefijos de función: `gst_` (0.10) vs `gst_` con pads renamed
- Cabeceras: `<gst/gst.h>` (0.10)
- Plugins: `.so` en `/opt/arm/gst/lib/gstreamer-0.10/`

## Plugins TI DSP Disponibles

| Plugin | Función | Elemento GStreamer |
|---|---|---|
| tidspv10enc_h264 | H.264 encoder (DSP) | `tidspv10enc_h264` |
| tidspv10dec_h264 | H.264 decoder (DSP) | `tidspv10dec_h264` |
| tidspv10enc_mpeg4 | MPEG-4 encoder (DSP) | `tidspv10enc_mpeg4` |

## Pipeline Típico (Onboard)

### Capturar y codificar a archivo H.264

```bash
gst-launch-0.10 v4l2src device=/dev/video0 ! \
    video/x-raw-yuv,width=1280,height=720,format=UYVY ! \
    ffmpegcolorspace ! video/x-raw-yuv,format=NV12 ! \
    tidspv10enc_h264 ! \
    filesink location=/data/video/output.h264
```

### Capturar y codificar a RTP (streaming a ground station)

```bash
gst-launch-0.10 -v v4l2src device=/dev/video0 ! \
    video/x-raw-yuv,width=1280,height=720 ! \
    ffmpegcolorspace ! video/x-raw-yuv,format=NV12 ! \
    tidspv10enc_h264 ! \
    rtph264pay ! udpsink host=192.168.1.2 port=5000
```

## Crear un Plugin GStreamer Personalizado

Template: https://github.com/flixr/ardrone2_gstreamer (script `create_new_plugin.py`)

```bash
# En el host, clonar el template
git clone https://github.com/flixr/ardrone2_gstreamer.git
cd ardrone2_gstreamer
python create_new_plugin.py MiPlugin

# Compilar cruzado
cd gst_MiPlugin_plugin
# Editar Makefile para toolchain ARM
make CC=arm-none-linux-gnueabi-gcc
```

Estructura del plugin generado:
```
gst_MiPlugin_plugin/
├── gst_MiPlugin_plugin.c   — interfaz GStreamer (caps, pads, settables)
├── gst_MiPlugin_plugin.h
├── MiPlugin_code.c          — lógica de negocio
├── MiPlugin_code.h
└── Makefile
```

## Elementos GStreamer Clave

| Elemento | Propósito |
|---|---|
| `v4l2src` | Captura de video desde cámara |
| `ffmpegcolorspace` | Conversión de espacio de color |
| `capssetter` | Forzar capacidades específicas |
| `tidspv10enc_h264` | Encoding H.264 vía DSP |
| `rtph264pay` | Payload H.264 a RTP |
| `udpsink` | Envío UDP |
| `filesink` | Escritura a archivo |
| `queue` | Buffer entre elementos |

## Referencias

- https://github.com/flixr/ardrone2_gstreamer — template y scripts
- GStreamer 0.10 API docs: https://gstreamer.freedesktop.org/documentation/0.10/
- TI GStreamer plugin source (parte del DVSDK)
