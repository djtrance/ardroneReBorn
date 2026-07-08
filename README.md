# parrotFramework

Framework de codificación de video H.264 onboard para **Parrot AR.Drone 2.0** con procesador **TI OMAP3530**.

Este repositorio contiene el código fuente, documentación y herramientas necesarias para desarrollar aplicaciones de codificación de video H.264 utilizando aceleración por hardware (DSP C64x+) directamente a bordo del dron.

> **Importante**: Todo el código se ejecuta **en el dron**, no en una estación de tierra. La compilación cruzada desde un host (macOS/Linux) es obligatoria.

## Estructura del Repositorio

```
parrotFramework/
├── README.md            # Este archivo
├── AGENTS.md            # Instrucciones para asistentes IA
├── docs/                # Documentación técnica detallada
│   ├── ardrone2_specs.md
│   ├── omap3530.md
│   ├── h264_acceleration.md
│   ├── communication_protocols.md
│   ├── video_formats.md
│   ├── ffmpeg_omap.md
│   ├── gstreamer_plugins.md
│   ├── v4l2.md
│   ├── toolchain.md
│   └── ti_sdk.md
├── src/                 # Código fuente C
│   ├── connection.c/h
│   ├── video_capture.c/h
│   ├── h264_init.c/h
│   ├── h264_encode.c/h
│   └── gst_plugin.c/h
├── tools/               # Herramientas de desarrollo (host-side)
│   ├── deploy.sh
│   └── ardrone2_helper.py
├── build/               # Scripts de compilación cruzada
│   ├── Dockerfile          # <-- Entorno Docker cross-compilation
│   ├── docker-build.sh
│   ├── docker-run.sh
│   ├── Makefile
│   ├── Makefile.include
│   └── install_deps.sh
└── examples/            # Ejemplos funcionales
    ├── basic_connection/
    └── encoding_pipeline/
```

## Requisitos

- **Host**: macOS o Linux (con Docker)
- **Docker**: Para la toolchain ARM (funciona en macOS y Linux por igual)
- **Dron**: Parrot AR.Drone 2.0 con firmware actualizado
- **Conexión**: WiFi al dron (192.168.1.1)

## Inicio Rápido (Docker — macOS/Linux)

```bash
# 1. Construir imagen Docker (solo la primera vez)
cd build
./docker-build.sh

# 2. Compilar dentro del contenedor
./docker-run.sh
# Dentro del contenedor:
make

# 3. Salir del contenedor y subir al dron
exit
cd ../tools
./deploy.sh ../build/bin/drone_encoder 192.168.1.1

# 4. Conectar por telnet y ejecutar
telnet 192.168.1.1
# En el dron:
killall program.elf
chmod 777 /data/video/drone_encoder
/data/video/drone_encoder
```

## Recursos Externos

- https://github.com/flixr/ardrone2_gstreamer — GStreamer framework para AR.Drone 2
- https://github.com/flixr/paparazzi — Autopilot Paparazzi (código de referencia para el dron)
- https://github.com/tudelft/toolchains — Toolchains ARM para productos Parrot
- https://github.com/tudelft/drone_vision — Procesamiento de visión para AR.Drone
