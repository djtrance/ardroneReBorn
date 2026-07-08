# SDK de TI OMAP3530 (DVSDK)

## Descripción General

El Digital Video Software Development Kit (DVSDK) de TI es el conjunto de librerías, códecs y herramientas para aceleración multimedia en el OMAP3530. Incluye el Codec Engine, los códecs DSP y el framework de integración.

## Componentes del DVSDK

| Componente | Descripción |
|---|---|
| **Codec Engine** | Framework de comunicación ARM ↔ DSP |
| **DSP/BIOS** | Sistema operativo del DSP |
| **DSPLink** | Capa de transporte entre ARM y DSP |
| **m4venc** | Códec H.264 encoder para DSP |
| **m4vdec** | Códec H.264 decoder para DSP |
| **CE (Codec Engine) Tools** | Herramientas de configuración de CE |
| **CMEM** | Módulo kernel para memoria compartida |

## Codec Engine API

### Abrir Engine

```c
#include <ti/sdo/ce/Engine.h>

Engine_Handle engine;
Engine_Error errorcode;

engine = Engine_open("encode", NULL, &errorcode);
if (engine == NULL) {
    // error
}
```

### Crear Códec

```c
#include <ti/sdo/ce/video/videnc.h>

VIDENC_Params params = VIDENC_Params_DEFAULT;
VIDENC_DynamicParams dynParams = VIDENC_DynamicParams_DEFAULT;
VIDENC_Handle encoder;

encoder = VIDENC_create(engine, "h264enc", &params);
if (encoder == NULL) {
    // error
}
```

### Procesar Frame

```c
XDM_BufDesc inBuf, outBuf;
XDM1_BufDesc inArgs, outArgs;
VIDENC_Status status;

// Configurar buffers de entrada (YUV) y salida (H.264)
inBuf.numBufs = 1;
inBuf.descs[0].buf = input_yuv_buffer;
inBuf.descs[0].bufSize = input_size;

outBuf.numBufs = 1;
outBuf.descs[0].buf = output_h264_buffer;
outBuf.descs[0].bufSize = output_size;

// Procesar
status = VIDENC_process(encoder, &inBuf, &outBuf, &inArgs, &outArgs);
if (status != VIDENC_EOK) {
    // error
}

// bytesProduced contiene el tamaño del bitstream
int h264_size = outArgs.bytesProduced;
```

### Cerrar

```c
VIDENC_delete(encoder);
Engine_close(engine);
```

## Archivos DSP en el Dron

Los binarios DSP están en `/opt/arm/tidsp-binaries-23.i3.8/` (después de mount bind):

| Archivo | Propósito |
|---|---|
| `baseimage.dof` | Firmware DSP/BIOS (DSP OS) |
| `m4venc_sn.dll64P` | H.264 encoder (dynreg) |
| `m4vdec_sn.dll64P` | H.264 decoder (dynreg) |
| `imgdec_copy.dll64P` | Imagen decoder |
| `jpgenc_sn.dll64P` | JPEG encoder |

## Memoria Compartida (CMEM)

```c
#include <sys/ioctl.h>
#include <ti/sdo/ce/osal/Memory.h>

// Asignar memoria compartida ARM-DSP
void *shared_buf = Memory_contigAlloc(size, Memory_DEFAULTALIGNMENT);
// El DSP puede acceder a esta memoria físicamente contigua

Memory_contigFree(shared_buf, size);
```

## Server de Códecs (DSP Server)

El DSP ejecuta un "server" que maneja múltiples codecs. En el AR.Drone 2.0, el server es `baseimage.dof`. Los códecs se registran dinámicamente con `dynreg`.

## Limitaciones

- La licencia del DVSDK requiere acuerdo con TI
- El código del DVSDK no se redistribuye en este repositorio
- Las cabeceras de CE necesarias para compilar están en `/opt/arm/gst/include/` del dron
- Version DVSDK: 3.10 (corresponde al firmware AR.Drone 2.0)

## Referencias

- TI DVSDK Documentation: SPRUEC6 (Codec Engine User's Guide)
- TI DVSDK Release Notes (versión 3.10)
- OMAP3530 TRM: SPRUF98
- Código de inicialización: `src/h264_init.c` en este repositorio
- Código de encoding: `src/h264_encode.c` en este repositorio
