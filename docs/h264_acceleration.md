# Aceleración H.264 por Hardware

## Descripción General

El TI OMAP3530 incluye un DSP C64x+ que ejecuta un códec de video H.264 mediante el framework **Codec Engine** de TI. La codificación se delega al DSP mientras el ARM Cortex-A8 maneja el resto del pipeline de video.

## Pipeline de Codificación

```
Cámara (raw Bayer)
    │
    ▼
ISP OMAP3530 (procesa a YUV422)
    │
    ▼
ARM Cortex-A8 (captura frame v4l2)
    │
    ▼
ARM → DSPLink → DSP C64x+
    │
    ▼
DSP: m4venc_sn.dll64P (encode H.264)
    │
    ▼
DSP → DSPLink → ARM (bitstream H.264)
    │
    ▼
ARM: guarda/envía H.264
```

## Códec H.264 — m4venc_sn.dll64P

| Parámetro | Valor |
|---|---|
| Perfil | Baseline Profile |
| Nivel | 3.1 (720p @ 30fps) |
| Resoluciones | 160x120 a 1280x720 |
| Bitrate | Personalizable (ej: 1-8 Mbps) |
| GOP | Configurable (IDR period) |
| Rate control | CBR, VBR |
| Entropy | CAVLC |

## Codec Engine — API

```c
#include <ti/sdo/ce/engine.h>
#include <ti/sdo/ce/video/videnc.h>

// Abrir engine
Engine_Handle engine = Engine_open("encode", NULL, NULL);

// Crear encoder
VIDENC_Handle encoder = VIDENC_create(engine, "h264enc", &params);

// Procesar frame
VIDENC_process(encoder, inBuf, outBuf);

// Destruir
VIDENC_delete(encoder);
Engine_close(engine);
```

## Limitaciones

- **1 sesión de encoding** — No hay multitarea en el DSP
- Codificar ocupa ~85% del DSP — no hay capacidad para otra tarea DSP simultánea
- Latencia ARM→DSP→ARM: ~16-33ms por frame (720p)
- El ARM debe preparar buffers en memoria compartida
- El DSP no puede acceder directamente a dispositivos de E/S

## Inicialización del DSP

```bash
# Cargar BaseImage (firmware DSP)
/bin/dspbridge/cexec.out -T $DSP_PATH/baseimage.dof -v

# Registrar códec H.264
/bin/dspbridge/dynreg.out -r $DSP_PATH/m4venc_sn.dll64P -v
```

Referencia: `src/h264_init.c` en este repositorio.
