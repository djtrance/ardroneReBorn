# Encoding Pipeline Example

Full onboard H.264 encoding pipeline:

```
Camera (V4L2) → UYVY → NV12 → DSP (H.264) → output.h264
```

## Prerequisites

- AR.Drone 2.0 with vision framework installed (see `tools/ardrone2_helper.py installvision`)
- Drone booted and accessible at 192.168.1.1

## Build

```bash
cd build
make
```

## Deploy

```bash
cd tools
./deploy.sh ../build/bin/drone_encoder 192.168.1.1
```

## Run on Drone

```bash
telnet 192.168.1.1

# Stop native program
killall program.elf

# Set up DSP environment
mount --bind /data/video/opt/arm /opt/arm
mount --bind /data/video/opt/arm/lib/dsp /lib/dsp
export PATH=/opt/arm/gst/bin:$PATH
export DSP_PATH=/opt/arm/tidsp-binaries-23.i3.8/
/bin/dspbridge/cexec.out -T $DSP_PATH/baseimage.dof -v
/bin/dspbridge/dynreg.out -r $DSP_PATH/m4venc_sn.dll64P -v

# Run encoder (10 seconds at 2000kbps)
/data/video/drone_encoder /dev/video0 10 2000
```

## Check Output

After the program exits:

```bash
ls -la /data/video/output.h264
# Download to host via FTP to verify:
# ftp 192.168.1.1
# get /data/video/output.h264
```

## View on Host

```bash
# macOS
ffplay output.h264
# or use VLC
```

## Key Files

- `main.c` — full encoding pipeline
- `../../src/video_capture.c` — V4L2 camera capture
- `../../src/h264_init.c` — DSP initialization
- `../../src/h264_encode.c` — H.264 DSP encoding
