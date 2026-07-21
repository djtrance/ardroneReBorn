# UVC Camera + wifibroadcast Integration for AR.Drone 2.0

## Overview

Two independent upgrades that do NOT modify the drone's firmware permanently:

1. **USB UVC camera** via userspace libusb (no kernel module required)
2. **Long-range WiFi video** via external USB adapter + wifibroadcast protocol

---

## 1. UVC Camera (USB Video Class)

### Background

The drone has `libusb-1.0.so` and `libusb-0.1.so` available. The front camera
(OmniVision OV7725) connects via ISP, not USB. The USB port can host a second
camera (e.g. Logitech C920, Microsoft LifeCam) for better quality or different
viewpoint.

### Approach A: Userspace (Recommended)

Write a libusb application that:
1. Enumerates USB devices, finds UVC interface
2. Detaches kernel driver if any
3. Sends UVC Probe/Commit to select format (YUY2/MJPEG)
4. Reads isochronous frames
5. Converts to grayscale/NV12 for vision pipeline

**Files**: `src/tests/test_uvc_camera.c`

### Approach B: Kernel Module (if needed)

Download Parrot's GPL kernel source and compile `uvcvideo.ko`:

```bash
cd tools/linux-kernel && ./build_modules.sh uvcvideo
```

Deploy and insmod:
```bash
deploy modules/uvcvideo.ko 192.168.1.1
insmod /data/video/uvcvideo.ko
```

The drone's kernel has `musb_hdrc` USB controller. Known issue: some OMAP3530
boards fail USB descriptor reads. If this happens, the USB camera needs
external power (powered USB hub).

**Files**: `tools/linux-kernel/build_modules.sh`

---

## 2. wifibroadcast / wfb-ng Integration

### Background

Standard WiFi requires ACK packets within ~200m. wifibroadcast uses monitor
mode + raw packets to bypass this, achieving km-range with directional antennas.

The protocol hierarchy:
```
H.264 video → RTP/UDP → FEC enc → raw 802.11 frame → WiFi radio
```

### AR6003 Internal WiFi

The drone's `ar6000` driver uses wireless extensions (not nl80211). Monitor mode
support depends on firmware version:

| Feature | ar6000 stock | ath6kl (if compiled) |
|---------|-------------|---------------------|
| AP mode | YES | YES |
| STA mode | YES | YES |
| Monitor mode | Unknown | YES (nl80211) |
| Raw injection | Unknown | YES |

The test program `test_wifi_caps` checks these capabilities at runtime.

### External USB WiFi (Recommended for wifibroadcast)

Plug a supported USB WiFi adapter into the drone's USB port:

| Chipset | Adapter | Driver | wfb-ng support |
|---------|---------|--------|---------------|
| Atheros AR9271 | Alfa AWUS036NHA, TP-Link TL-WN722N | `ath9k_htc` | Original wifibroadcast |
| Realtek RTL8812AU | Alfa AWUS036ACH | `rtl8812au` (patched) | wfb-ng preferred |
| Realtek RTL8812EU | BL-M8812EU2 module | `rtl8812eu` (patched) | wfb-ng recommended |

Compile drivers:
```bash
cd tools/linux-kernel && ./build_modules.sh ath9k_htc
```

### Architecture: On-drone

```
┌──────────────────┐     UDP      ┌──────────────┐   raw 802.11   ┌─────────┐
│  drone_encoder   │ ──────────►  │  wfb_tx      │ ─────────────► │ USB WiFi│
│  (H.264 DSP)     │ port 5602    │  (C program) │                │ (monitor│
│  or GStreamer    │              │  FEC + inject│                │  mode)  │
└──────────────────┘              └──────────────┘                └─────────┘
```

### Architecture: Ground Station

```
┌─────────┐   raw 802.11  ┌──────────────┐   UDP      ┌──────────────────┐
│ USB WiFi │ ◄─────────────│  wfb_rx       │ ──────────►│  QGroundControl  │
│ (monitor │               │  (host Linux) │ port 5600  │  or GStreamer    │
│  mode)   │               │  FEC + deint  │            │  + display       │
└─────────┘                └──────────────┘            └──────────────────┘
```

### Files

| File | Description |
|------|-------------|
| `tools/wfb-bridge/wfb_tx.c` | Minimal wifibroadcast TX for drone |
| `tools/wfb-bridge/Makefile` | Build for ARM toolchain |
| `tools/linux-kernel/build_modules.sh` | Kernel module cross-compiler |
| `src/tests/test_wifi_caps.c` | WiFi capability test |
| `src/tests/test_uvc_camera.c` | UVC camera test via libusb |

---

## 3. Procedure: Testing on the Drone

### Step 1: Build all test programs
```bash
cd build && make clean && make tests
```

### Step 2: Deploy and test basic connectivity
```bash
cd tools && ./deploy.sh ../build/bin/test_minimal 192.168.1.1
# telnet: /data/video/test_minimal
```

### Step 3: Test WiFi capabilities
```bash
cd tools && ./deploy.sh ../build/bin/test_wifi_caps 192.168.1.1
# telnet: /data/video/test_wifi_caps -a
```

### Step 4: Test UVC camera (plug USB camera first)
```bash
cd tools && ./deploy.sh ../build/bin/test_uvc_camera 192.168.1.1
# telnet: /data/video/test_uvc_camera -l
# telnet: /data/video/test_uvc_camera -n 50
```

### Step 5: wifibroadcast (requires USB WiFi adapter)
```bash
# On drone (telnet):
insmod /data/video/ath9k_htc.ko    # if needed
iw dev wlan0 set monitor otherbss fcsfail
ifconfig wlan0 up
iwconfig wlan0 channel 149

# Start transmitter:
/data/video/wfb_tx -i wlan0 -p 5602 -b 12

# On another terminal, start drone_encoder streaming:
/data/video/drone_encoder stream 127.0.0.1:5602
```

---

## 4. Non-Permanent Design Philosophy

All changes are loaded at runtime and revert on reboot:

| Component | Permanent? | Revert method |
|-----------|-----------|---------------|
| `test_*` binaries | No (in /data) | Delete files |
| Kernel modules (.ko) | No (insmod) | `rmmod` or reboot |
| WiFi monitor mode | No (iwconfig) | Reboot restores AP |
| USB camera | No (hotplug) | Unplug camera |

The only persistence is in `/data/video/` where uploaded files live. Nothing
modifies `/lib/`, `/bin/`, or kernel flash partitions.
