=== parrotFramework Deploy Bundle ===
Subir con: tools/deploy-all.sh [drone-ip]

Estructura:
  bin/       - 13 binarios ARM softfp (compatibles con drone kernel 2.6.16)
  libs/      - 9 librerías ARM (libusb-1.0, libjpeg, plugins mjpg-streamer)
  www/       - Web interface para mjpg-streamer
  kmodules/  - uvcvideo.ko para cámara USB
  dsp/       - DSP firmware (vacío - subir tidsp-binaries manualmente)
  drone-boot.sh - Script de inicio automático

Después de subir, en el drone (telnet):
  export LD_LIBRARY_PATH=/data/video/libs:$LD_LIBRARY_PATH
  killall program.elf
  /data/video/bin/test_camera /dev/video3 320 240 10
