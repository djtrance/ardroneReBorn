# AGENTS.md

## Project

H.264 onboard video encoding framework for Parrot AR.Drone 2.0 (TI OMAP3530). Code runs ON the drone, not on a ground station.

## Key Architecture

- **Target**: ARM Cortex-A8 (TI OMAP3530 @ 600MHz), DSP C64x+ for H.264 encode
- **OS**: AR.Drone 2.0 runs a custom Linux; telnet (port 23) + FTP for deployment
- **Default IP**: `192.168.1.1` (drone WiFi access point)
- **GStreamer 0.10** (not 1.x) is the media framework available on the drone
- **DSP acceleration**: TI DVSDK (DSP/BIOS), baseimage.dof + dynreg for codec engine
- **Cross-compilation is mandatory** — no build tools on the drone itself

## Directory Layout

```
parrotFramework/
├── README.md           # main documentation (Spanish/English)
├── AGENTS.md           # this file
├── docs/               # technical specs (OMAP3530, H.264, protocols)
├── src/                # C source code
│   ├── main.c              # standalone drone_encoder binary
│   ├── dsp_init_main.c     # standalone DSP init utility
│   ├── gst_plugin.c/.h     # GStreamer 0.10 filter element
│   ├── h264_encode.c/.h    # H.264 encoder abstraction (TI CE 3.24)
│   ├── h264_init.c/.h      # DSP firmware loader (bind mounts, cexec, dynreg)
│   ├── video_capture.c/.h  # V4L2 camera capture (UYVY → NV12)
│   └── connection.c/.h     # AR.Drone AT command + navdata protocol
├── tools/              # host-side dev tools (deploy scripts, helpers)
├── build/              # Makefiles, cross-compilation toolchain config
│   ├── Dockerfile
│   ├── docker-build.sh
│   ├── docker-run.sh
│   ├── Makefile
│   ├── Makefile.include
│   └── sysroot/ti/packages/  # TI Codec Engine + XDAIS headers (ti/sdo/ce/, ti/xdais/)
└── examples/           # functional examples (connection, encoding pipeline)
```

## Cross-Compilation Toolchain

- **Toolchain**: `parrot-tools-linuxgnutools-2016.02-linaro` (amd64)
  - Install path: `/opt/arm-2016.02-linaro/`
  - Prefix: `arm-linux-gnueabihf-` (hard-float ABI, Cortex-A8 NEON)
  - Source: https://github.com/tudelft/toolchains (archived but packages still available)
  - Install: `dpkg -i parrot-tools-linuxgnutools-*.deb`
- **Older alternative**: `parrot-tools-linuxgnutools-2012.03` (i386, `arm-none-linux-gnueabi-` prefix)
- **Sysroot**: AR.Drone 2.0 libraries must match the drone's firmware version

## Docker (recommended for macOS)

```bash
# Build image (once)
cd build && ./docker-build.sh

# Compile inside container
./docker-run.sh
# Inside container:
make          # standalone binary
make plugin   # GStreamer plugin
make dsp_init # DSP init only
```

- Dockerfile: `build/Dockerfile` (Ubuntu 22.04 amd64 + toolchain + GStreamer 0.10 + V4L2; `--platform=linux/amd64` for Apple Silicon)
- Build context is the project root (so `COPY build/sysroot/ti/packages` works inside the Dockerfile)
- Mounts project directory at `/workspace`
- Uses `--network host` so you can FTP/telnet to drone from container

## DSP/H.264 Acceleration Init (on-drone command sequence)

```bash
# Option A: use our dsp_init binary
/data/video/dsp_init

# Option B: manual sequence
mount --bind /data/video/opt/arm /opt/arm
mount --bind /data/video/opt/arm/lib/dsp /lib/dsp
export PATH=/opt/arm/gst/bin:$PATH
export DSP_PATH=/opt/arm/tidsp-binaries-23.i3.8/
/bin/dspbridge/cexec.out -T $DSP_PATH/baseimage.dof -v
/bin/dspbridge/dynreg.out -r $DSP_PATH/m4venc_sn.dll64P -v
```

- DSP binaries path: `/opt/arm/tidsp-binaries-23.i3.8/`
- GStreamer plugins path: `/opt/arm/gst/lib/gstreamer-0.10/`

## Deployment Workflow

1. Cross-compile on host with ARM toolchain
2. Upload via FTP to `/data/video/` on drone
3. Telnet (`telnet 192.168.1.1`) to execute/configure
4. The `ardrone2.py` helper (from flixr/ardrone2_gstreamer) automates upload, install, start

## Video Protocol

- Drone sends H.264 video over RTP via UDP port 5000 (to ground station)
- For **onboard encoding**: capture raw video from camera, encode via DSP H.264, optionally save or re-stream
- Camera resolution: 1280x720 (720p) @ 30fps front camera; 320x240 bottom camera

## Reference Upstream Repos

- https://github.com/flixr/ardrone2_gstreamer — GStreamer plugin framework for AR.Drone 2
- https://github.com/flixr/paparazzi — Paparazzi UAV autopilot with AR.Drone 2 support
- https://github.com/flixr/paparazzi-portability-support — cross-platform build scripts
- https://github.com/tudelft/toolchains — Parrot ARM toolchain .deb packages
- https://github.com/tudelft/drone_vision — vision processing for AR.Drone
- https://github.com/flixr/opencv_bebop — cross-compiled OpenCV for Parrot drones

## TI Codec Engine Headers

- Downloaded from TI processor-sdk-mirror: `codec_engine_3_24_00_08,lite.tar.gz` + `xdais_7_24_00_04.tar.gz`
- Stored in `build/sysroot/ti/packages/` (mirrors the SDK `packages/` layout so includes are `<ti/sdo/ce/Engine.h>` etc.)
- A minimal `std.h` (XDCtools type compatibility shim: `Void`, `UInt32`, `String`, `Ptr`, `Bool`, etc.) lives alongside at `build/sysroot/ti/packages/std.h` and is force-included via `-include` in `CFLAGS`.
- Enabled by `-DUSE_TI_CODEC_ENGINE` in `Makefile.include`.
- TI Codec Engine symbols (`Engine_open`, `VIDENC_create`, `Memory_contigAlloc`, etc.) appear as **undefined** in the plugin .so and standalone binary, resolved at runtime on the drone via libti_ce.so.
- Baked into Docker image at `/opt/tisysroot/ti/packages/`. The `TI_SYSROOT` env var (set in Dockerfile) takes precedence over the Makefile default.

## Build Targets

| Target | Description |
|--------|-------------|
| `make all` | Build `bin/drone_encoder` — standalone binary with TI CE support |
| `make plugin` | Build `bin/libgstparrot_enc.so` — GStreamer 0.10 plugin with TI CE |
| `make dsp_init` | Build `bin/dsp_init` — DSP firmware loader only |
| `make clean` | Remove object files |

### Standalone Binary (`drone_encoder`)

```
Usage: drone_encoder [options] <mode> <output>

Modes:
  record <file.h264>    Record H.264 video to file (Annex B)
  stream <host:port>    Stream H.264 over RTP/UDP (RFC 3984, FU-A)

Options:
  -b <bps>     Bitrate in bits/sec (default: 2000000)
  -f <fps>     Target framerate (default: 30)
  -d <sec>     Duration limit, 0=infinite (default: 0)
  -w <px>      Capture width (default: 1280)
  -h <px>      Capture height (default: 720)
  -c <dev>     Video device (default: /dev/video0)
```

Capabilities:
- DSP auto-init via `dsp_full_init()` (bind mounts + cexec + dynreg)
- V4L2 capture with UYVY → NV12 conversion
- TI H.264 DSP encode via Codec Engine
- Record: H.264 Annex B byte stream with `0x00000001` start codes
- Stream: RTP/UDP with single NAL and FU-A fragmentation (RFC 3984)
- Graceful SIGINT handling with frame stats

### DSP Init (`dsp_init`)

```
Usage: dsp_init [options]

Options:
  -q    Quiet mode
  -c    Check status only, don't init
  -r    Register H.264 encoder only (skip baseimage)
```

### GStreamer Plugin (`libgstparrot_enc.so`)

GStreamer 0.10 filter element `parrotenc`:
- Input caps: `video/x-raw-yuv, format=(fourcc)NV12, width=[160,1280], height=[120,720], framerate=[1/1,30/1]`
- Output caps: `video/x-h264, width=[160,1280], height=[120,720], framerate=[1/1,30/1]`
- Properties: `bitrate` (100-8000 kbps), `fps` (1-30)

```bash
# Example pipeline
gst-launch-0.10 v4l2src device=/dev/video0 ! \
  ffmpegcolorspace ! video/x-raw-yuv,format=NV12 ! \
  parrotenc bitrate=2000 ! \
  filesink location=test.h264
```

## Features Gained vs. Vanilla TI Pipeline

| Feature | drone_encoder / GStreamer plugin | gst-launch with TI dspv10enc_h264 |
|---------|----------------------------------|-----------------------------------|
| **DSP auto-init** | `dsp_full_init()` in `main.c` | Manual SSH: mount + cexec + dynreg |
| **Dynamic bitrate** | `h264_encoder_set_bitrate()` at runtime | Must restart pipeline |
| **Forced IDR** | `h264_encoder_request_idr()` | Must restart pipeline |
| **Dynamic framerate** | `h264_encoder_set_framerate()` at runtime | Must restart pipeline |
| **Standalone mode** | `drone_encoder` binary, no GStreamer | Requires `gst-launch-0.10` |
| **RTP streaming** | Built-in with FU-A fragmentation | Requires external pipeline |
| **Record to file** | Built-in, Annex B format | Requires external pipeline |
| **Camera → encode** | Single process, lower latency | GStreamer pipeline overhead |
| **Buffer access** | Direct NV12 input / H.264 output buffers | Abstracted through GStreamer |
| **SEI/metadata injection** | Buffer-level access in encode loop | Requires custom GStreamer element |

## Known Issues

- Code uses XDM 0.9 compatibility (`XDM_INCLUDE_DOT9_SUPPORT`), supported by header include order.
- All three binaries (drone_encoder, dsp_init, libgstparrot_enc.so) carry undefined TI CE symbols (`Engine_open`, `VIDENC_create`, etc.) resolved at runtime. The drone must have `libti_ce.so` in its library path.

## Critical Gotchas

- **GStreamer 0.10**, not 1.x — plugin API differs, check version in includes
- Must `killall program.elf` before starting custom code (native drone program conflicts)
- DSP can only run ONE encode session; no multitasking on DSP side
- `/data` is persistent storage; `/opt` is tmpfs — mount bind from `/data/video/opt` to `/opt`
- Always `chmod 777` uploaded binaries before execution
