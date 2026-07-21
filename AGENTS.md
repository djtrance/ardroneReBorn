# AGENTS.md

## Project

Transform the AR.Drone 2.0 (TI OMAP3530, 2012) into a contemporary autonomous
UAV matching DJI Spark capabilities and beyond — with GPS/visual navigation,
obstacle avoidance, 4G/wifibroadcast long-range video, and military-grade
autonomous features. All code runs ON the drone, not on a ground station.

### Target Feature Set

| Feature | DJI Spark | AR.Drone 2 stock | With Upgrades |
|---------|-----------|-------------------|---------------|
| GPS positioning | GPS+GLONASS | ❌ None | ✅ USB GPS (u-blox 8) |
| GPS Return-to-Home | ✅ | ❌ None | ✅ GPS RTH (coded) |
| Vision Return-to-Home | ✅ | ❌ None | ✅ Visual odometry |
| Obstacle avoidance | Forward only | ❌ None | ✅ Vision + sensors |
| Optical flow | ✅ Downward | ❌ None | ✅ Downward camera |
| 720p H.264 video | ❌ 1080p | ✅ 720p | ✅ DSP-encoded H.264 |
| Gesture control | ✅ | ❌ | ✅ Vision-based |
| Follow me | ✅ | ❌ | ✅ GPS + vision |
| Waypoint navigation | ✅ | ❌ | ✅ GPS waypoints |
| Geofence | ✅ | ❌ | ✅ Software geofence |
| WiFi range | 2km (enhanced) | ~200m | ✅ 5km+ (wifibroadcast) |
| 4G remote pilot | ❌ | ❌ | ✅ USB 4G modem |
| Drone detection | ❌ | ❌ | ✅ RTL-SDR + vision |
| FPV head tracking | ❌ | ❌ | ✅ Arduino IMU bridge |
| Autonomous takeoff/land | ✅ | ❌ | ✅ Vision + altitude |
| Swarm coordination | ❌ | ❌ | ⚪ Via 4G mesh |
| ADS-B airspace | ❌ | ❌ | ⚪ RTL-SDR 1090MHz |

## Key Architecture

- **Target**: ARM Cortex-A8 (TI OMAP3530 @ 600MHz), DSP C64x+ for H.264 encode
- **OS**: AR.Drone 2.0 runs a custom Linux; telnet (port 23) + FTP for deployment
- **Default IP**: `192.168.1.1` (drone WiFi access point)
- **GStreamer 0.10** (not 1.x) is the media framework available on the drone
- **DSP acceleration**: TI DVSDK (DSP/BIOS), baseimage.dof + dynreg for codec engine
- **Cross-compilation is mandatory** — no build tools on the drone itself

## Quick Start

## Prerequisites
- Docker (>=20.10) with `--platform=linux/amd64` support for Apple Silicon.
- Host ARM toolchain installed: `parrot-tools-linuxgnutools-2016.02-linaro` (or the older 2012.03 soft‑float version if you need non‑static binaries).
- `arm-linux-gnueabihf-gcc` in PATH for building the DSP binaries.

## Build and Deploy Workflow
1. **Build Docker image** (once):
   ```bash
   cd build && ./docker-build.sh
   ```
2. **Enter container**:
   ```bash
   ./docker-run.sh
   ```
3. **Compile**:
   - All binaries: `make`
   - GStreamer plugin only: `make plugin`
   - DSP init utility: `make dsp_init`
4. **Run tests**:
   ```bash
   make tests
   ```
5. **Deploy to drone** (replace `<drone_ip>`):
   ```bash
   tools/deploy.sh build/bin/<binary> <drone_ip>
   ```
6. **Start binary on drone** (via telnet):
   ```bash
   telnet <drone_ip>
   /data/video/<binary> [args]
   ```

## Important Notes
- The drone runs Linux 2.6.32; binaries must be dynamically linked and **not static**.
- `libti_ce.so` must be present on the drone for DSP encoding to work.
- `/opt/arm` and `/lib/dsp` are bind‑mounted at boot; ensure the firmware blobs exist under `/data/video/opt/arm`.
- For debugging, set `export ARDRONE_LOG=1` on the drone before launching.
- All test binaries are built with the soft‑float toolchain and are located in `build/bin/`.
- The project uses GStreamer 0.10; ensure the drone has the correct version of libgstreamer.

---


```
parrotFramework/
├── README.md                  # main documentation (Spanish/English)
├── AGENTS.md                  # this file
├── docs/                      # technical specs (OMAP3530, H.264, protocols)
│   ├── wfb-integration.md     # wifibroadcast + UVC camera design
│   └── hardware-upgrades.md   # comprehensive hardware upgrade matrix
├── src/                       # C source code
│   ├── main.c                 # standalone drone_encoder binary
│   ├── dsp_init_main.c        # standalone DSP init utility
│   ├── gst_plugin.c/.h        # GStreamer 0.10 filter element
│   ├── h264_encode.c/.h       # H.264 encoder abstraction (TI CE 3.24)
│   ├── h264_init.c/.h         # DSP firmware loader (mounts, cexec, dynreg)
│   ├── video_capture.c/.h     # V4L2 camera capture (UYVY → NV12)
│   ├── connection.c/.h        # AR.Drone AT command + navdata protocol
│   ├── vision/                # Computer vision pipeline
│   │   ├── image.h                # grayscale image struct
│   │   ├── pattern.h              # feature/pattern detection
│   │   ├── flow_stage1.h          # SAD block-matching optical flow
│   │   ├── obstacle.h             # looming, asymmetry, vert line
│   ├── navigation/            # GPS + position estimation
│   │   ├── gps.c/.h               # NMEA parser, Haversine, RTH
│   ├── network/               # Communication
│   │   ├── modem.c/.h             # 4G/5G modem AT + connection control
│   ├── detection/             # External threats/airspace
│   │   (future: RTL-SDR ADS-B, audio drone detection)
│   └── tests/                 # On-drone test programs
│       ├── test_minimal.c         # toolchain verification
│       ├── test_connection.c      # AT/navdata
│       ├── test_camera.c          # V4L2 capture
│       ├── test_vision.c          # full vision pipeline
│       ├── test_stream.c          # vision + UDP to ground station
│       ├── test_wifi_caps.c       # WiFi monitor mode / raw socket test
│       ├── test_uvc_camera.c      # USB UVC via libusb
│       ├── test_gps.c             # GPS NMEA parser test
│       └── test_modem.c           # 4G/5G modem AT + signal test
├── tools/                  # host-side dev tools
│   ├── deploy.sh               # FTP upload to drone
│   ├── drone-boot.sh           # module loading + service startup
│   ├── video_receiver.c        # SDL ground station with HUD
│   ├── linux-kernel/           # kernel module build system
│   │   └── build_modules.sh    # cross-compile ALL kernel .ko modules
│   ├── wfb-bridge/             # wifibroadcast transmitter
│   │   ├── wfb_tx.c                # raw socket WiFi injection
│   │   └── Makefile                # ARM cross-compile
│   └── arduino-bridge/         # I2C/GPIO sensor bridge FW
│       └── arduino_bridge.ino      # Arduino sketch (I2C+GPIO protocol)
├── build/                  # Makefiles, toolchain config
│   ├── Dockerfile, docker-build.sh, docker-run.sh
│   ├── Makefile, Makefile.include
│   └── sysroot/ti/packages/  # TI Codec Engine + XDAIS headers
└── examples/               # functional examples
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

## On-Drone Test Programs

Build with cross-compiler (via Docker):

```bash
cd build && ./docker-run.sh
make tests    # builds all test programs
```

Three test programs in `build/bin/`:

| Program | Source | What it tests | Run on drone |
|---------|--------|---------------|--------------|
| `test_connection` | `src/tests/test_connection.c` | AT commands + navdata (battery, altitude, angles) | `./test_connection [ip]` |
| `test_camera` | `src/tests/test_camera.c` | V4L2 capture (resolution, FPS, brightness) | `./test_camera [dev] [w] [h] [frames]` |
| `test_vision` | `src/tests/test_vision.c` | Camera + Stage 1 flow + obstacle detection | `./test_vision [dev] [frames]` |
| `test_stream` | `src/tests/test_stream.c` | Camera + flow + obstacle + UDP stream to ground station | `./test_stream [dev] [host-ip] [port] [frames]` |
| `test_wifi_caps` | `src/tests/test_wifi_caps.c` | WiFi monitor mode + raw socket + channel scan | `./test_wifi_caps -a` |
| `test_uvc_camera` | `src/tests/test_uvc_camera.c` | USB UVC camera enumeration + streaming | `./test_uvc_camera -n 50` |
| `test_gps` | `src/tests/test_gps.c` | GPS NMEA parser, Haversine, RTH functions | `./test_gps [dev] [baud]` |
| `test_modem` | `src/tests/test_modem.c` | 4G/5G modem detection, AT, signal, connect | `./test_modem info` |

Deploy any binary with:

```bash
cd tools && ./deploy.sh ../build/bin/test_vision 192.168.1.1
```

Then telnet to drone and run:

```bash
telnet 192.168.1.1
/data/video/test_vision /dev/video0 300
```

### test_connection

```
Usage: test_connection [drone-ip]
```

Connects to the drone, sends AT*CONFIG to enable navdata, and prints battery%, altitude, velocity, and heading every navdata packet.

Output:
```
batt%  alt(cm)  vx       vy       vz       heading
68     52       0        0        0        -1
68     53       0        1        0        -1
```

### test_camera

```
Usage: test_camera [device] [width] [height] [frames]
```

Opens the V4L2 camera device, captures N frames at the requested resolution, and prints per-frame capture time and average FPS.

Output:
```
Camera: /dev/video0
Actual: 320x240
Frame   0: 307200 bytes, avg_brightness=112, capture_time=15234 us
...
Stats: 100 frames, avg 15210 us/frame (65.7 FPS)
```

### test_vision

```
Usage: test_vision [device] [frames]
```

Full vision pipeline test: captures grayscale frames, runs Stage 1 optical flow (SAD block matching) and obstacle detection (looming + asymmetry + vertical line). Outputs frame-by-frame results:

Output:
```
Camera: 320x240

fr   flow_x     flow_y     qual   loom    asym     conf
---- ---------- ---------- ------ ------- -------- -------
0    +12        -3         200    0       0        0
10   +10        -2         212    15      0        45
...
```

### test_stream

```
Usage: test_stream [device] [host-ip] [port] [frames]
```

Full vision pipeline with live video streaming to ground station: captures grayscale frames, runs Stage 1 optical flow and obstacle detection, draws overlay (flow arrow, looming bar, asymmetry indicator, vertical line), and sends frames over UDP to a ground station.

The host receiver (`tools/video_receiver`) displays the video with the same overlays:

```bash
# Terminal 1 (host): start receiver
make -C tools/simulator video_receiver && ./tools/simulator/video_receiver 9090

# Terminal 2 (telnet to drone): start stream
/data/video/test_stream /dev/video0 192.168.1.2 9090 300
```

Output on drone:
```
Streaming to 192.168.1.2:9090
FRAME  FX       FY       QUAL   LOOM   ASYM   CONF
0         +12       -3      200     0      0      0
10        +10       -2      212     15     0     45
```

## Critical Gotchas

- **GStreamer 0.10**, not 1.x — plugin API differs, check version in includes
- Must `killall program.elf` before starting custom code (native drone program conflicts)
- DSP can only run ONE encode session; no multitasking on DSP side
- `/data` is persistent storage; `/opt` is tmpfs — mount bind from `/data/video/opt` to `/opt`
- Always `chmod 777` uploaded binaries before execution

## Drone System Information (AR.Drone 2.0)

### OS & Kernel
- Linux 2.6.32.9-g980dab2 (custom Parrot build)
- Build: `/home/stephane/.ardrone/linux/ardrone2_ARDrone2_Version_20130102/`
- BusyBox v1.14.0

### Toolchain & Libraries
- glibc 2.11.1 (soft-float ABI, `/lib/ld-linux.so.3` linker)
- C++: libstdc++ 6.0.14 (from `libstdc++.so.6.0.14`)
- gcc used by kernel: Sourcery G++ Lite 2010.09-50
- libc files: `libc-2.11.1.so`, `ld-2.11.1.so`, `libm-2.11.1.so`, `libpthread-2.11.1.so`, `librt-2.11.1.so`, `libdl-2.11.1.so`, `libnss_dns-2.11.1.so`, `libnss_files-2.11.1.so`
- Other libs: `libiw.so.29` (wireless), `libusb-1.0.so`, `libusb-0.1.so`, `libz.so.1.2.3`, `libexif.so.12.3.2`, `libproc-3.2.8.so`

### Devices
- **Video**: 7 camera devices `/dev/video0`–`/dev/video6` (via omap3_isp)
- **I2C**: 3 buses (`/dev/i2c-1`, `/dev/i2c-2`, `/dev/i2c-3`)
- **DSP**: `/dev/DspBridge` (character device 247)
- **USB**: 1 x usb-ohci, SD card reader on musb_hdrc
- No `dsp`, `fb`, or `ttyUSB` by default

### Kernel Modules
- `ov7670` — front camera sensor driver
- `soc1040` — bottom camera sensor driver
- `omap3_isp` — OMAP3 Image Signal Processor
- `ar6000` — Atheros AR6003 WiFi

### Storage
- `/data` — persistent flash (FAT, approx 4 GB `mmcblk0`)
- `/opt` — tmpfs (bind-mounted from `/data/video/opt/arm` at boot)
- NAND: 5 MTD partitions (mtd0–mtd4) with UBIFS (ubi0–ubi2)

### Important Build Notes

#### "FATAL: kernel too old" Fix
This error occurs when a statically-linked binary built with a modern glibc (≥2.31, Ubuntu 22.04's `arm-linux-gnueabi`) runs on the drone's 2.6.32.9 kernel. The glibc checks `uname()` at startup and refuses to run if the kernel is older than its `--enable-kernel` target (typically 3.2.0 for modern distros).

**Solution**: Use the **Parrot 2012.03 soft-float toolchain** (`arm-none-linux-gnueabi-`, kernel min 2.6.16) with **dynamic linking** (not static — static causes segfaults on drone):

```makefile
SOFT_CC = /opt/arm-2012.03/bin/arm-none-linux-gnueabi-gcc
SOFT_CFLAGS = -marm -march=armv7-a -mtune=cortex-a8 -mfpu=neon \
              -mfloat-abi=softfp -Os -std=gnu99
SOFT_LDFLAGS = -static-libgcc -lm -lrt \
              -Wl,--dynamic-linker=/lib/ld-linux.so.3 -Wl,-rpath,/lib
```

Produces: `ELF 32-bit LSB executable, ARM, version 1 (SYSV), dynamically linked (uses shared libs), for GNU/Linux 2.6.16` — confirmed working on drone.

Key required shared libs (all on drone): `libm.so.6`, `librt.so.1`, `libc.so.6`, `libgcc_s.so.1` (from `-static-libgcc`).

**NEVER use `-static` for on-drone binaries** — all 22 research repos confirmed this causes segfault.

#### Segfault Isolation
If a test binary segfaults on the drone, start with `test_minimal`:
```
Usage: test_minimal
```
It only prints "Hello from drone!" and creates a UDP socket. If this works, the issue is in the test program code, not the toolchain.

#### Test Programs vs. GStreamer Plugin
- On-drone test programs (`test_connection`, `test_camera`, etc.) use the **soft-float** toolchain with **dynamic linking** (not static).
- The H.264 encoder pipeline (`drone_encoder`, `libgstparrot_enc.so`) uses the **hard-float** toolchain (Linaro 2016.02) with TI Codec Engine undefined symbols resolved at runtime.

### Known Quirks
- `/lib/dsp/` does NOT exist on the drone by default — DSP libraries must be uploaded as part of `/data/video/opt/arm/lib/dsp/`
- `/opt/arm/tidsp-binaries*` does NOT exist by default — DSP firmware must be uploaded and bind-mounted
- `/opt/arm/gst/` does NOT exist by default — GStreamer ARM binaries must be uploaded
- USB camera detected (`Sonix Technology Co., Ltd. ID 0c45:6366`) but no UVC driver in kernel — won't appear as `/dev/video*`

### USB Serial / GPS / Modem Quirks
- No `usbserial.ko` in stock kernel — must be cross-compiled and insmod'd
- GPS (via PL2303/CP210x/FTDI) → `/dev/ttyUSB0` after insmod `usbserial.ko` + `pl2303.ko` (or cp210x/ftdi_sio)
- 4G modem (Huawei E3372) → `eth1` after insmod `usbnet.ko` + `cdc_ether.ko`
- Some 4G modems may present as `cdc_ncm` or `qmi_wwan` — compile both
- Modem AT commands on `/dev/ttyUSB0` (or `/dev/ttyACM0`) after `cdc_acm.ko`
- Parrot NMEA GPS Flight Recorder (ID 19CF:3000) uses `cp210x.ko` driver
- u-blox NEO-6M/8M via USB-serial → PL2303 or CP210x

### Hardware Upgrade Paths (see docs/hardware-upgrades.md)
- **Budget** ($55): u-blox NEO-6M GPS + TP-Link WN722N + Arduino Nano
- **Mid** ($175): u-blox NEO-M8N + Alfa AWUS036NHA + ESP32 + TFmini LiDAR
- **Premium** ($500): u-blox SAM-M8Q + Alfa AWUS036ACH + 4G modem + RPLidar + FLIR
