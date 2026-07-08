# AR.Drone 2.0 — Especificaciones Técnicas

## Hardware

| Componente | Especificación |
|---|---|
| Procesador | TI OMAP3630 (OMAP3530 family) @ 1GHz |
| DSP | TMS320C64x+ @ 430MHz (DSP/BIOS) |
| RAM | 512MB DDR2 @ 400MHz |
| Almacenamiento | 4GB NAND flash (aprox. 1.2GB libres) |
| WiFi | 802.11b/g/n (Wi-Fi direct, access point mode) |
| Cámaras | Frontal 720p @ 30fps + inferior 320x240 @ 60fps |
| Sensores | IMU 6-axis, ultrasonido, cámara vertical |
| Batería | 3S LiPo 11.1V 1500mAh (aprox. 12 min vuelo) |

## Software

| Componente | Versión |
|---|---|
| Kernel Linux | 2.6.32 (ARMv7) |
| Toolchain | arm-none-linux-gnueabi (gcc 4.4.5/4.6.x) |
| GStreamer | 0.10.x (NO 1.x) |
| DSP | DVSDK 3.10 (DSP/BIOS 5.41) |
| Códec H.264 | m4venc_sn.dll64P (DSP) |

## Conexiones de Red

- El dron crea su propia red WiFi como access point
- **IP por defecto**: 192.168.1.1
- **Telnet**: puerto 23 (shell remota como root)
- **FTP**: puerto 21 (subida de archivos)
- **Video RTP**: UDP puerto 5000 (desde el dron a ground station)
- **AT Command**: UDP puerto 5556 (ground station al dron, control)
- **NavData**: UDP puerto 5554 (dron a ground station, telemetría)

## Firmware y Particiones

- `/firmware/version.txt` — versión del firmware
- `/data/` — almacenamiento persistente (configuraciones, logs)
- `/data/video/` — directorio para código del usuario y frameworks
- `/opt/` — tmpfs (se pierde al reiniciar — usar mount --bind desde /data/video)
- `/bin/` — utilidades del sistema
- `/lib/dsp/` — librerías DSP (después de mount bind)

## Modos de Video

- Cámara frontal: 1280x720 (720p) raw o H.264
- Cámara inferior: 320x240 raw o H.264
- Streaming: H.264 + AAC en MP4 container sobre RTP
- El dron puede enviar video codificado O hacer encoding onboard

## Referencias

- [AR.Drone 2.0 Developer Guide](https://developer.parrot.com/)
- AR.Drone 2.0 SDK (protocolo AT Command)
- https://github.com/flixr/paparazzi (implementación de referencia)
