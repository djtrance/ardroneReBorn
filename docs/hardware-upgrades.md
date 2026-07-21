# AR.Drone 2.0 — Hardware Upgrade & Capability Matrix

## Project Vision

> Transform the AR.Drone 2.0 from a 2012 toy quadcopter into a contemporary
> autonomous UAV matching (and exceeding) DJI Spark capabilities, with
> military-grade features, without permanent firmware modification.

### Target Feature Set

| Feature | DJI Spark | AR.Drone 2 stock | With Upgrades |
|---------|-----------|-------------------|---------------|
| GPS positioning | GPS+GLONASS | ❌ None | ✅ USB GPS (u-blox 8) |
| GPS RTH | ✅ | ❌ None | ✅ GPS RTH |
| Vision RTH | ✅ | ❌ None | ✅ Visual odometry + optical flow |
| Obstacle avoidance | Forward only | ❌ None | ✅ Forward + downward (vision) |
| Optical flow | ✅ Downward | ❌ None | ✅ Downward camera |
| 1080p video | ✅ | ❌ 720p only | ✅ 720p H.264 (DSP) + recording |
| 12MP photo | ✅ | ❌ None | ✅ Frame capture |
| Gesture control | ✅ | ❌ None | ✅ Custom gesture via vision |
| Follow me | ✅ | ❌ None | ✅ Via GPS + vision tracking |
| Waypoint nav | ✅ | ❌ None | ✅ GPS waypoints |
| Geofence | ✅ | ❌ None | ✅ Software geofence |
| 30min flight | ❌ 16min | ✅ 18min | ✅ Same (battery-limited) |
| WiFi range | 2km (enhanced) | ~200m | ✅ 5km+ (wifibroadcast) |
| 4G remote pilot | ❌ | ❌ | ✅ Via USB 4G modem |
| Drone detection | ❌ | ❌ | ✅ RTL-SDR + vision |
| ADS-B airspace | ❌ | ❌ | ✅ RTL-SDR (1090MHz) |
| FPV head tracking | ❌ | ❌ | ✅ USB serial + Arduino |

---

## 1. GPS Receiver

### Options

| Device | Chipset | Interface | Driver | Accuracy | Price | Feasibility |
|--------|---------|-----------|--------|----------|-------|-------------|
| **u-blox NEO-6M** | u-blox 6 | UART→USB (CP210x) | `cp210x.ko` | 2.5m | $10-15 | ✅ Easy |
| **u-blox NEO-7M** | u-blox 7 | UART→USB (CP210x) | `cp210x.ko` | 2.5m | $15-20 | ✅ Easy |
| **u-blox NEO-M8N** | u-blox 8 | UART→USB (CP210x) | `cp210x.ko` | 1.5m GPS+GLONASS | $25-35 | ✅ Easy |
| **u-blox SAM-M8Q** | u-blox 8 | UART→USB (CP210x) | `cp210x.ko` | 1.5m GPS+GLONASS | $35-45 | ✅ Easy |
| **BN-880** | u-blox NEO-8M + compass | UART→USB | `cp210x.ko` | 1.5m | $20-30 | ✅ Easy |
| **GlobalSat BU-353S4** | SiRF Star IV | USB (PL2303) | `pl2303.ko` | 5m | $40 | ✅ Easy |
| **Parrot Flight Recorder** | CP210x | USB | `cp210x.ko` ID 19CF:3000 | 5-10m | $80 | ⚠️ Weak GPS |
| **Adafruit Ultimate GPS** | MTK3339 | UART→USB (FTDI) | `ftdi_sio.ko` | 3m | $40 | ✅ Easy |

**Recommended**: u-blox NEO-M8N or BN-880 (GPS+GLONASS+compass)

### Implementation

```bash
# 1. Build modules
./build_modules.sh usb-gps

# 2. Deploy to drone
cd tools && ./deploy.sh ../tools/linux-kernel/modules/cp210x.ko 192.168.1.1
cd tools && ./deploy.sh ../tools/linux-kernel/modules/usbserial.ko 192.168.1.1

# 3. On drone (telnet):
insmod /data/video/usbserial.ko
insmod /data/video/cp210x.ko
# Plug in GPS → should appear as /dev/ttyUSB0
cat /dev/ttyUSB0  # should show NMEA sentences ($GPGGA, etc.)
```

**GPS NMEA Parser**: `src/navigation/gps.c` (see below)

---

## 2. Long-Range Video (5km+)

### Options

| Device | Chipset | Type | Driver | Range | Price |
|--------|---------|------|--------|-------|-------|
| **Alfa AWUS036NHA** | Atheros AR9271 | USB WiFi | `ath9k_htc.ko` | 2-5km+ (directional antenna) | $45 |
| **TP-Link TL-WN722N v1** | Atheros AR9271 | USB WiFi | `ath9k_htc.ko` | 500m-2km | $15 |
| **Alfa AWUS036ACH** | Realtek RTL8812AU | USB WiFi AC | external driver | 3-8km (5.8GHz) | $60 |
| **Huawei E3372** | HiSilicon | 4G LTE | `cdc_ether.ko` | Unlimited (cellular) | $60 |
| **ZTE MF833** | Qualcomm | 4G LTE | `cdc_ncm.ko` | Unlimited (cellular) | $40 |
| **Quectel EC25** | Qualcomm | 4G LTE (QMI) | `qmi_wwan.ko` | Unlimited (cellular) | $80 |

**Recommended for range**: Alfa AWUS036NHA + wifibroadcast
**Recommended for cellular**: Huawei E3372 (plugs directly, auto-config)

### Implementation (wifibroadcast)

```bash
# 1. Build ath9k_htc module
./build_modules.sh usb-wifi

# 2. Deploy
cd tools && ./deploy.sh ../tools/linux-kernel/modules/ath9k_htc.ko 192.168.1.1
cd tools && ./deploy.sh ../tools/linux-kernel/modules/ath9k_hw.ko 192.168.1.1
cd tools && ./deploy.sh ../tools/linux-kernel/modules/ath9k_common.ko 192.168.1.1
cd tools && ./deploy.sh ../tools/linux-kernel/modules/ath.ko 192.168.1.1
cd tools && ./deploy.sh ../tools/linux-kernel/modules/cfg80211.ko 192.168.1.1
cd tools && ./deploy.sh ../tools/linux-kernel/modules/mac80211.ko 192.168.1.1

# 3. On drone (telnet):
insmod /data/video/cfg80211.ko
insmod /data/video/mac80211.ko
insmod /data/video/ath.ko
insmod /data/video/ath9k_hw.ko
insmod /data/video/ath9k_common.ko
insmod /data/video/ath9k_htc.ko
# Wait for wlan0 to appear
iw dev wlan0 set monitor otherbss fcsfail
ifconfig wlan0 up
iwconfig wlan0 channel 149  # 5.8GHz or 13/6 for 2.4GHz

# 4. Start wfb bridge
/data/video/wfb_tx -i wlan0 -p 5602 -b 12
```

### Implementation (4G LTE)

```bash
# 1. Build networking modules
./build_modules.sh usb-4g

# 2. Deploy
cd tools && ./deploy.sh ../tools/linux-kernel/modules/usbnet.ko 192.168.1.1
cd tools && ./deploy.sh ../tools/linux-kernel/modules/cdc_ether.ko 192.168.1.1
cd tools && ./deploy.sh ../tools/linux-kernel/modules/cdc_acm.ko 192.168.1.1

# 3. On drone:
insmod /data/video/usbnet.ko
insmod /data/video/cdc_ether.ko
insmod /data/video/cdc_acm.ko
# Plug in Huawei E3372 → eth1 appears
udhcpc -i eth1  # Get IP from cellular network
# Now drone has internet → can be controlled from anywhere
```

---

## 3. External USB WiFi (for AR6003 replacement or dual WiFi)

| Chipset | Adapter | 2.6.32 support | Monitor mode | wifibroadcast | Kernel module |
|---------|---------|---------------|-------------|---------------|--------------|
| AR9271 | Alfa AWUS036NHA, TP-Link WN722N v1 | ✅ | ✅ | ✅ | `ath9k_htc.ko` |
| AR9002U | Various | ✅ | ✅ | ✅ | `ar9170usb.ko` |
| ZD1211 | Various | ✅ | ✅ | ✅ | `zd1211rw.ko` |
| RTL8187 | Alfa AWUS036H | ✅ | ✅ | ✅ | `rtl8187.ko` |
| RTL8192CU | Various | ⚠️ Needs backport | ✅ | ✅ | `rtl8192cu.ko` |
| RTL8812AU | Alfa AWUS036ACH | ❌ External driver | ✅ | ✅ | Needs `rtl8812au` from GitHub |

---

## 4. USB Storage (for video recording, logging)

| Device | Driver | Notes |
|--------|--------|-------|
| Any USB flash drive | `usb-storage.ko` | FAT32 formatted, auto-mount |
| SD card via USB reader | `usb-storage.ko` | Same as flash |
| SSD via USB-SATA | `usb-storage.ko` | Power concern: external power |

```bash
./build_modules.sh usb-storage
# Then on drone:
insmod /data/video/usb-storage.ko
# Device at /dev/sda1
mkdir -p /mnt/usb
mount -t vfat /dev/sda1 /mnt/usb
```

**Use case**: Record H.264 directly to USB drive:
```bash
/data/video/drone_encoder -w 1280 -h 720 -b 2000000 record /mnt/usb/flight_001.h264
```

---

## 5. Manual Control (Gamepad / Joystick)

| Controller | Driver | Use Case |
|-----------|--------|----------|
| Xbox 360 Wireless | `xpad.ko` | Manual flight control |
| PS3/PS4 DualShock | `hid_sony.ko` | Manual flight control + FPV |
| Logitech F710 | `hid_logitech.ko` | Manual flight control |

```bash
./build_modules.sh usb-hid
# On drone:
insmod /data/video/usbhid.ko
insmod /data/video/hid.ko
insmod /data/video/hid_logitech.ko
insmod /data/video/joydev.ko
insmod /data/video/evdev.ko
# Then read from /dev/input/js0
```

---

## 6. Additional Video Cameras (Stereo / Wide-angle)

| Camera | Type | Driver | Use |
|--------|------|--------|-----|
| Logitech C920 | UVC H.264 | `uvcvideo.ko` | HD FPV, stereo pair |
| Microsoft LifeCam Studio | UVC | `uvcvideo.ko` | HD video (wide angle) |
| ELP 720p USB | UVC | `uvcvideo.ko` | Wide-angle FPV |
| OV5640 USB module | UVC | `uvcvideo.ko` | 5MP stills |

```bash
./build_modules.sh usb-video
# On drone:
insmod /data/video/videodev.ko
insmod /data/video/v4l2_common.ko
insmod /data/video/uvcvideo.ko
# → /dev/video7 (or next available)
```

---

## 7. Compass / Heading

### Options

| Approach | Accuracy | Feasibility | Cost |
|----------|----------|-------------|------|
| Use drone's internal HMC5883L (via I2C) | 6° | ✅ Already present on I2C bus 3 | Free |
| External HMC5883L via Arduino→USB bridge | 2° | ✅ Via serial bridge | $15 |
| BN-880 GPS (built-in compass, QMC5883L) | 2° | ✅ Via USB serial | $25 |
| PX4Flow optical flow + sonar | 1° | ⚠️ Needs USB serial | $150 |

The drone already has a 3-axis magnetometer (HMC5883L) on I2C bus 3, accessible
at `/dev/i2c-3`. We can read it with userspace I2C.

---

## 8. Obstacle Avoidance Sensors

### Vision-based (already implemented)
- Optical flow (SAD block matching) → `src/vision/flow.c`
- Looming detection → `src/vision/obstacle.c`
- Asymmetry detection → `src/vision/obstacle.c`
- Vertical line detection → `src/vision/obstacle.c`

### External sensors (hardware upgrade)

| Sensor | Type | Connection | Driver | Range | Use |
|--------|------|-----------|--------|-------|-----|
| HC-SR04 | Ultrasonic | GPIO via Arduino | Serial bridge | 2-400cm | Low-alt hold |
| MaxBotix MB1240 | I2C sonar | I2C via bus 3 | Userspace I2C | 20-500cm | Obstacle avoidance |
| TFmini | LiDAR | UART→USB | `cp210x.ko` | 30cm-12m | Precision alt + avoid |
| TF-Luna | LiDAR | UART→USB | `cp210x.ko` | 20cm-8m | Low-cost obstacle |
| RPLidar A1 | 360° LiDAR | UART→USB | `cp210x.ko` | 0.15-12m | SLAM mapping |
| VL53L1X | ToF laser | I2C via Arduino | Serial bridge | 0-4m | Close obstacle |

### Arduino/ESP32 Serial Bridge (for GPIO + I2C sensors)

Connect an Arduino Nano (5V) or ESP32 connected via USB serial to the drone.
The Arduino runs a simple I2C/GPIO bridge protocol:

```
USB ←→ Serial ←→ Arduino ←→ I2C sensors, GPIO sensors
```

Example protocol (over USB serial at 115200 baud):
```
> I2C:0x77,0xAA<CR>     # Read I2C device 0x77 register 0xAA
< 0x42<CR>               # Response
> GPIO:12,1<CR>          # Set GPIO 12 HIGH
> GPIO:13,0<CR>          # Set GPIO 13 LOW
> GPIO:14,R<CR>          # Read GPIO 14
```

---

## 9. Drone Detection

### Via RTL-SDR (ADS-B)
```bash
| Component | Device | Cost | Feasibility |
|-----------|--------|------|-------------|
| RTL-SDR v3 | R820T2 + RTL2832U | $25 | Requires backported DVB driver |
| ADS-B antenna | 1090 MHz 1/4 wave | $10 | DIY |
```

**Note**: RTL2832U support was added in kernel 3.12+. For 2.6.32, use userspace
librtlsdr via libusb (already on drone: `libusb-1.0.so`).

### Via vision (already implemented)
- Moving object detection via optical flow anomalies
- Known drone visual signatures via template matching

### Via audio spectrum analysis
- DJI and other drones emit distinctive motor/propeller harmonics
- Microphone → FFT → drone signature detection
- Requires USB microphone + USB audio driver + FFT library

---

## 10. Military-Equivalent Features

| Feature | Implementation | Status |
|---------|---------------|--------|
| **Beyond Visual Range (BVR) control** | 4G cellular + wifibroadcast | 🚧 Build needed |
| **Autonomous waypoint nav** | GPS + waypoint list | 🚧 Build needed |
| **Return to home (GPS)** | GPS + last known home position | 🚧 Build needed |
| **Return to home (no GPS)** | Visual odometry + flow integration | ⚪ Planned |
| **Geofence (hard + soft)** | GPS perimeter + altitude limits | 🚧 Build needed |
| **Target tracking** | Vision-based object tracking | 🚧 Build needed |
| **Follow me** | GPS beacon + vision | 🚧 Build needed |
| **Orbit / POI** | Circle around GPS point + camera track | ⚪ Planned |
| **FPV head tracking** | Arduino IMU → USB → drone AT commands | 🚧 Build needed |
| **Automatic takeoff/land** | Vision + altitude triggers | 🚧 Build needed |
| **Drone interceptor** | Detection + pursuit via vision/GPS | ⚪ Planned |
| **Swarm coordination** | Multiple drones via 4G/mesh | ⚪ Future |
| **Encrypted telemetry** | AES-256 over wifibroadcast/4G | 🚧 Build needed |
| **Mission recording** | GPS + video + telemetry replay | 🚧 Build needed |
| **Thermal camera** | FLIR Lepton via SPI→Arduino→USB | ⚪ Future |
| **Secure erase** | Shred onboard data on tamper | ⚪ Future |

---

## 11. Drone-to-Drone Mesh Network

Using 4G LTE + wifibroadcast as redundant links:

```
Drone A ──4G──► Internet ──4G──► Drone B
    │                                    │
    └────wifibroadcast 5.8GHz───────────┘
```

Multiple drones can form a mesh for:
- Extended range (relay)
- Swarm coordination
- Search-and-rescue patterns
- Perimeter surveillance

---

## 12. Complete Upgrade Paths

### Budget Build (~$100)
| Item | Price |
|------|-------|
| u-blox NEO-6M GPS | $12 |
| TP-Link TL-WN722N v1 | $15 |
| Arduino Nano clone | $5 |
| HC-SR04 sonar x2 | $6 |
| USB OTG cable | $3 |
| Various wires/connectors | $5 |
| 16GB USB flash drive | $8 |
| **Total** | **~$54** |

### Mid-Range Build (~$200)
| Item | Price |
|------|-------|
| u-blox NEO-M8N GPS | $30 |
| Alfa AWUS036NHA | $45 |
| ESP32 dev board | $8 |
| TFmini LiDAR | $25 |
| TF-Luna LiDAR | $18 |
| Logitech C920 | $40 |
| Vibration dampening | $10 |
| **Total** | **~$176** |

### Premium Build (~$400)
| Item | Price |
|------|-------|
| u-blox SAM-M8Q GPS | $45 |
| Alfa AWUS036ACH | $60 |
| Huawei E3372 4G | $60 |
| RPLidar A1 | $100 |
| FLIR Lepton 2.5 | $200 |
| ESP32-S3 | $12 |
| 128GB USB 3.0 drive | $20 |
| **Total** | **~$497** |

---

## 13. Software Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     drone_encoder (main)                      │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌─────────────┐  │
│  │ Camera   │→ │ H.264    │→ │ Stream   │→ │ WFB/4G      │  │
│  │ V4L2/UVC │  │ DSP Enc  │  │ RTP/FU-A │  │ Network     │  │
│  └──────────┘  └──────────┘  └──────────┘  └─────────────┘  │
│                                                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                    │
│  │ GPS NMEA │→ │ Position │→ │ Nav      │                    │
│  │ Parser   │  │ Estimator│  │ Controller                    │
│  └──────────┘  └──────────┘  └──────────┘                    │
│                                                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                    │
│  │ Flow     │→ │ Obstacle │→ │ Avoid    │                    │
│  │ Optical  │  │ Detection│  │ Steering │                    │
│  └──────────┘  └──────────┘  └──────────┘                    │
│                                                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                    │
│  │ Sensors  │→ │ Serial   │→ │ Arduino  │                    │
│  │ Bridge   │  │ Protocol │  │ (I2C/GPIO)                    │
│  └──────────┘  └──────────┘  └──────────┘                    │
│                                                               │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐                    │
│  │ SDR      │→ │ ADS-B    │→ │ Airspace │                    │
│  │ (libusb) │  │ Decoder  │  │ Map      │                    │
│  └──────────┘  └──────────┘  └──────────┘                    │
└──────────────────────────────────────────────────────────────┘
```

---

## 14. Next Steps (Build Order)

1. ✅ **Build kernel module system** → `build_modules.sh` with all categories
2. ⬜ **USB serial modules** → GPS, telemetry, Arduino bridge
3. ⬜ **USB networking modules** → 4G/5G modems, Ethernet
4. ⬜ **USB WiFi modules** → ath9k_htc, rtl8187
5. ⬜ **USB storage modules** → Flash drive for recording
6. ⬜ **USB HID modules** → Joystick control
7. ⬜ **USB audio modules** → Audio alerts, FPV audio
8. ⬜ **USB video modules** → UVC cameras, stereo vision
9. ⬜ **Onboard GPS NMEA parser** → `src/navigation/gps.{c,h}`
10. ⬜ **Onboard GPS RTH controller** → `src/navigation/rth.{c,h}`
11. ⬜ **Waypoint following** → `src/navigation/waypoint.{c,h}`
12. ⬜ **Geofence** → `src/navigation/geofence.{c,h}`
13. ⬜ **4G modem interface** → `src/network/modem.{c,h}`
14. ⬜ **Arduino serial bridge** → `tools/arduino-bridge/`
15. ⬜ **Target tracking** → `src/vision/tracker.{c,h}`
16. ⬜ **RTL-SDR integration** → `src/detection/adsb.{c,h}`
17. ⬜ **Encrypted telemetry** → AES via libsodium over wifibroadcast
18. ⬜ **Ground station upgrade** → Enhanced video_receiver with GPS map + HUD
