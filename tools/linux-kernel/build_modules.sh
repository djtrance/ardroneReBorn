#!/usr/bin/env bash
# build_modules.sh — Compile ALL kernel modules for AR.Drone 2.0
set -euo pipefail

# ============================================================================
# MODULE CATEGORIES — enables "partial" builds:
#   ./build_modules.sh all       — everything
#   ./build_modules.sh usb-serial — only USB serial drivers
#   ./build_modules.sh usb-net   — only USB networking drivers
#   ./build_modules.sh usb-wifi  — only USB WiFi drivers
#   ./build_modules.sh usb-storage
#   ./build_modules.sh usb-hid
#   ./build_modules.sh usb-audio
#   ./build_modules.sh usb-gps    — all GPS-relevant serial + CDC ACM
#   ./build_modules.sh usb-4g    — all 4G/5G-relevant modules
#   ./build_modules.sh sensors   — I2C sensor drivers (barometer, etc.)
#   ./build_modules.sh clean     — remove build artifacts
# ============================================================================

TOOLCHAIN_PREFIX="${CROSS_COMPILE:-arm-none-linux-gnueabi-}"
CC="${TOOLCHAIN_PREFIX}gcc"
WORKDIR="$(cd "$(dirname "$0")" && pwd)/_build"
OUTDIR="$(cd "$(dirname "$0")" && pwd)/modules"
KERNEL_DIR="$WORKDIR/linux-2.6.32"

# Parrot GPL kernel source
KERNEL_SOURCE_URL="https://github.com/parrot-opensource/ardrone2-opensource/raw/master/sources/linux-2.6.32.tar.gz"
# Alternative: vanilla 2.6.32.9 (less Parrot-specific drivers)
VANILLA_KERNEL_URL="https://www.kernel.org/pub/linux/kernel/v2.6/linux-2.6.32.9.tar.gz"

mkdir -p "$OUTDIR"

# ============================================================================
# Helper functions
# ============================================================================
check_toolchain() {
    if ! $CC --version >/dev/null 2>&1; then
        echo "ERROR: Toolchain not found: $CC"
        echo "Set CROSS_COMPILE or install from:"
        echo "  https://github.com/parrot-opensource/toolchains"
        exit 1
    fi
    echo "Toolchain: $($CC --version | head -1)"
}

download_kernel_source() {
    if [ -f "$KERNEL_DIR/Makefile" ]; then
        echo "Kernel source already at $KERNEL_DIR"
        return 0
    fi

    mkdir -p "$WORKDIR"
    echo "=== Downloading Parrot AR.Drone 2.0 kernel source ==="
    echo "  URL: $KERNEL_SOURCE_URL"
    if wget -q --timeout=60 "$KERNEL_SOURCE_URL" -O "$WORKDIR/linux-2.6.32.tar.gz"; then
        echo "Extracting..."
        tar -xzf "$WORKDIR/linux-2.6.32.tar.gz" -C "$WORKDIR"
        # Handle tarball naming (might not be linux-2.6.32 internally)
        local srcdir
        srcdir=$(tar -tzf "$WORKDIR/linux-2.6.32.tar.gz" 2>/dev/null | head -1 | cut -d/ -f1)
        if [ -n "$srcdir" ] && [ "$srcdir" != "linux-2.6.32" ] && [ -d "$WORKDIR/$srcdir" ]; then
            mv "$WORKDIR/$srcdir" "$KERNEL_DIR"
        fi
    else
        echo "WARNING: Parrot GPL source download failed, trying vanilla 2.6.32.9"
        sleep 2
        wget -q --timeout=60 "$VANILLA_KERNEL_URL" -O "$WORKDIR/linux-2.6.32.tar.gz"
        tar -xzf "$WORKDIR/linux-2.6.32.tar.gz" -C "$WORKDIR"
        local srcdir
        srcdir=$(tar -tzf "$WORKDIR/linux-2.6.32.tar.gz" 2>/dev/null | head -1 | cut -d/ -f1)
        if [ -n "$srcdir" ] && [ "$srcdir" != "linux-2.6.32" ] && [ -d "$WORKDIR/$srcdir" ]; then
            mv "$WORKDIR/$srcdir" "$KERNEL_DIR"
        fi
    fi

    if [ ! -f "$KERNEL_DIR/Makefile" ]; then
        echo "FATAL: Failed to get kernel source"
        exit 1
    fi

    # Set extraversion to match drone's kernel
    sed -i 's/^EXTRAVERSION =.*$/EXTRAVERSION = .9-g980dab2/' "$KERNEL_DIR/Makefile" 2>/dev/null || true
    echo "Kernel source ready at $KERNEL_DIR"
}

apply_config() {
    local config_file="$KERNEL_DIR/.config"
    if [ -f "$config_file" ]; then
        echo "Config already exists at $config_file"
        return 0
    fi

    echo "=== Creating kernel config with all modules enabled ==="

    # Try Parrot original config first
    local config_urls=(
        "https://raw.githubusercontent.com/parrot-opensource/ardrone2-opensource/master/sources/linux-2.6.32.config"
        "https://raw.githubusercontent.com/RICLAMER/ArDrone2-4G-Kernel/master/kernel.config"
    )

    for url in "${config_urls[@]}"; do
        if wget -q --timeout=10 "$url" -O "$WORKDIR/.config.download" 2>/dev/null; then
            echo "Using config from: $url"
            cp "$WORKDIR/.config.download" "$config_file"
            break
        fi
    done

    if [ ! -f "$config_file" ]; then
        echo "WARNING: Could not download config, generating default"
        cd "$KERNEL_DIR"
        make ARCH=arm CROSS_COMPILE="$TOOLCHAIN_PREFIX" omap2plus_defconfig 2>&1 | tail -5 || true
    fi

    # ========================================================================
    # ENABLE ALL MODULE CATEGORIES — edit this section to customize
    # ========================================================================
    cd "$KERNEL_DIR"

    # --- USB Serial (GPS, modems, telemetry, Arduino) ---
    echo "Enabling USB serial drivers..."
    scripts/config --module CONFIG_USB_SERIAL
    scripts/config --module CONFIG_USB_SERIAL_GENERIC
    scripts/config --module CONFIG_USB_SERIAL_PL2303
    scripts/config --module CONFIG_USB_SERIAL_FTDI_SIO
    scripts/config --module CONFIG_USB_SERIAL_CP210X
    scripts/config --module CONFIG_USB_SERIAL_CH341
    scripts/config --module CONFIG_USB_SERIAL_OPTION
    scripts/config --module CONFIG_USB_SERIAL_QUALCOMM
    scripts/config --module CONFIG_USB_SERIAL_SIERRAWIRELESS
    scripts/config --module CONFIG_USB_SERIAL_IPW
    scripts/config --module CONFIG_USB_SERIAL_NAVMAN
    scripts/config --module CONFIG_USB_SERIAL_MOS7720
    scripts/config --module CONFIG_USB_SERIAL_MOS7840
    scripts/config --module CONFIG_USB_SERIAL_KOBIL_SCT
    scripts/config --module CONFIG_USB_SERIAL_CYBERJACK
    scripts/config --module CONFIG_USB_SERIAL_WHITEHEAT
    scripts/config --module CONFIG_USB_SERIAL_DIGI_ACCELEPORT
    scripts/config --module CONFIG_USB_SERIAL_KEIO
    scripts/config --module CONFIG_USB_SERIAL_OMNINET
    scripts/config --module CONFIG_USB_SERIAL_IR
    scripts/config --module CONFIG_USB_SERIAL_EDGEPORT
    scripts/config --module CONFIG_USB_SERIAL_EDGEPORT_TI
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_MPR
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_USA28
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_USA28X
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_USA28XA
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_USA28XB
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_USA19
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_USA18X
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_USA19W
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_USA19QW
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_USA19QI
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_MMA
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_USA49W
    scripts/config --module CONFIG_USB_SERIAL_KEYSPAN_USA49WLC
    # USB serial core dependencies
    scripts/config --enable CONFIG_USB_SERIAL_WRITE_FIFO_TIMEOUT
    # Enable TTY layer (required for serial)
    scripts/config --enable CONFIG_TTY
    scripts/config --enable CONFIG_UNIX98_PTYS

    # --- USB CDC (4G/5G modems, GPS, tethering) ---
    echo "Enabling USB CDC drivers..."
    scripts/config --module CONFIG_USB_ACM
    scripts/config --module CONFIG_USB_CDC_WDM
    scripts/config --enable CONFIG_USB_EHCI_HCD
    scripts/config --enable CONFIG_USB_OHCI_HCD
    scripts/config --enable CONFIG_USB_OTG

    # --- USB Networking (4G/5G, tethering, Ethernet adapters) ---
    echo "Enabling USB networking drivers..."
    scripts/config --module CONFIG_USB_USBNET
    scripts/config --module CONFIG_NETDEVICES
    scripts/config --module CONFIG_USB_NET_AX8817X
    scripts/config --module CONFIG_USB_NET_AX88179
    scripts/config --module CONFIG_USB_NET_CDCETHER
    scripts/config --module CONFIG_USB_NET_CDC_EEM
    scripts/config --module CONFIG_USB_NET_CDC_NCM
    scripts/config --module CONFIG_USB_NET_DM9601
    scripts/config --module CONFIG_USB_NET_SMSC75XX
    scripts/config --module CONFIG_USB_NET_SMSC95XX
    scripts/config --module CONFIG_USB_NET_GL620A
    scripts/config --module CONFIG_USB_NET_NET1080
    scripts/config --module CONFIG_USB_NET_PLUSB
    scripts/config --module CONFIG_USB_NET_MCS7830
    scripts/config --module CONFIG_USB_NET_RNDIS_HOST
    scripts/config --module CONFIG_USB_NET_CDC_SUBSET
    scripts/config --module CONFIG_USB_NET_ZAURUS
    scripts/config --module CONFIG_USB_NET_CX82310_ETH
    scripts/config --module CONFIG_USB_NET_KALMIA
    scripts/config --module CONFIG_USB_NET_QMI_WWAN
    scripts/config --module CONFIG_USB_HSO
    scripts/config --module CONFIG_USB_NET_INT51X1
    scripts/config --module CONFIG_USB_IPHETH
    scripts/config --module CONFIG_USB_SIERRA_NET
    scripts/config --module CONFIG_USB_VL600
    scripts/config --module CONFIG_USB_NET_MULTIPROBE
    scripts/config --module CONFIG_USB_RTL8150
    scripts/config --module CONFIG_USB_PEGASUS

    # --- USB Storage (flash drives, SD card readers, etc.) ---
    echo "Enabling USB storage drivers..."
    scripts/config --module CONFIG_USB_STORAGE
    scripts/config --module CONFIG_USB_STORAGE_DEBUG
    scripts/config --module CONFIG_USB_STORAGE_DATAFAB
    scripts/config --module CONFIG_USB_STORAGE_FREECOM
    scripts/config --module CONFIG_USB_STORAGE_ISD200
    scripts/config --module CONFIG_USB_STORAGE_USBAT
    scripts/config --module CONFIG_USB_STORAGE_SDDR09
    scripts/config --module CONFIG_USB_STORAGE_SDDR55
    scripts/config --module CONFIG_USB_STORAGE_JUMPSHOT
    scripts/config --module CONFIG_USB_STORAGE_ALAUDA
    scripts/config --module CONFIG_USB_STORAGE_ONETOUCH
    scripts/config --module CONFIG_USB_STORAGE_KARMA
    scripts/config --module CONFIG_USB_STORAGE_CYPRESS_ATACB
    scripts/config --module CONFIG_USB_STORAGE_ENE_UB6250
    # SCSI layer needed by USB storage
    scripts/config --enable CONFIG_SCSI
    scripts/config --module CONFIG_SCSI_MOD
    scripts/config --module CONFIG_SCSI_DMA
    scripts/config --enable CONFIG_BLOCK

    # --- USB HID (gamepads, joysticks for manual control) ---
    echo "Enabling USB HID drivers..."
    scripts/config --module CONFIG_USB_HID
    scripts/config --module CONFIG_HID_PID
    scripts/config --enable CONFIG_USB_HIDDEV
    scripts/config --module CONFIG_HID_GENERIC
    scripts/config --module CONFIG_HID_A4TECH
    scripts/config --module CONFIG_HID_APPLE
    scripts/config --module CONFIG_HID_BELKIN
    scripts/config --module CONFIG_HID_CHERRY
    scripts/config --module CONFIG_HID_CHICONY
    scripts/config --module CONFIG_HID_CYPRESS
    scripts/config --module CONFIG_HID_EZKEY
    scripts/config --module CONFIG_HID_GYRATION
    scripts/config --module CONFIG_HID_KYE
    scripts/config --module CONFIG_HID_LOGITECH
    scripts/config --module CONFIG_HID_MICROSOFT
    scripts/config --module CONFIG_HID_MONTEREY
    scripts/config --module CONFIG_HID_PANTHERLORD
    scripts/config --module CONFIG_HID_PETALYNX
    scripts/config --module CONFIG_HID_SAMSUNG
    scripts/config --module CONFIG_HID_SONY
    scripts/config --module CONFIG_HID_SUNPLUS
    scripts/config --module CONFIG_HID_TOPSEED
    scripts/config --module CONFIG_HID_THRUSTMASTER
    scripts/config --module CONFIG_HID_ZEROPLUS
    # USB HID game interface
    scripts/config --module CONFIG_UHID
    scripts/config --module CONFIG_USB_ACM
    # Joystick
    scripts/config --module CONFIG_INPUT_JOYSTICK
    scripts/config --module CONFIG_JOYSTICK_XPAD
    scripts/config --module CONFIG_JOYSTICK_XPAD_FF
    scripts/config --module CONFIG_JOYSTICK_XPAD_LEDS

    # --- USB Audio (speakers, microphone → audio alerts, FPV audio) ---
    echo "Enabling USB audio drivers..."
    scripts/config --module CONFIG_SND_USB_AUDIO
    scripts/config --module CONFIG_SND_USB_UA101
    scripts/config --module CONFIG_SND_USB_CAIAQ
    scripts/config --module CONFIG_SND_USB_CAIAQ_INPUT
    scripts/config --module CONFIG_SND_USB_US122L
    scripts/config --module CONFIG_SND_USB_6FIRE
    scripts/config --enable CONFIG_SND
    scripts/config --enable CONFIG_SND_PCM
    scripts/config --enable CONFIG_SND_TIMER
    scripts/config --enable CONFIG_SND_SUPPORT_OLD_API

    # --- USB Video (USB cameras via UVC) ---
    echo "Enabling USB video drivers..."
    scripts/config --module CONFIG_USB_VIDEO_CLASS
    scripts/config --module CONFIG_USB_VIDEO_CLASS_INPUT
    scripts/config --enable CONFIG_V4L_USB_DRIVERS
    scripts/config --module CONFIG_USB_GSPCA
    scripts/config --module CONFIG_USB_GSPCA_CONEX
    scripts/config --module CONFIG_USB_GSPCA_ETOMS
    scripts/config --module CONFIG_USB_GSPCA_FINEPIX
    scripts/config --module CONFIG_USB_GSPCA_JEILINJ
    scripts/config --module CONFIG_USB_GSPCA_OV534
    scripts/config --module CONFIG_USB_GSPCA_OV534_9
    scripts/config --module CONFIG_USB_GSPCA_PAC207
    scripts/config --module CONFIG_USB_GSPCA_PAC7302
    scripts/config --module CONFIG_USB_GSPCA_PAC7311
    scripts/config --module CONFIG_USB_GSPCA_SN9C2028
    scripts/config --module CONFIG_USB_GSPCA_SN9C20X
    scripts/config --module CONFIG_USB_GSPCA_SONIXB
    scripts/config --module CONFIG_USB_GSPCA_SONIXJ
    scripts/config --module CONFIG_USB_GSPCA_SPCA500
    scripts/config --module CONFIG_USB_GSPCA_SPCA501
    scripts/config --module CONFIG_USB_GSPCA_SPCA505
    scripts/config --module CONFIG_USB_GSPCA_SPCA506
    scripts/config --module CONFIG_USB_GSPCA_SPCA508
    scripts/config --module CONFIG_USB_GSPCA_SPCA561
    scripts/config --module CONFIG_USB_GSPCA_STK014
    scripts/config --module CONFIG_USB_GSPCA_STV0680
    scripts/config --module CONFIG_USB_GSPCA_SUNPLUS
    scripts/config --module CONFIG_USB_GSPCA_T613
    scripts/config --module CONFIG_USB_GSPCA_TOPRO
    scripts/config --module CONFIG_USB_GSPCA_TV8532
    scripts/config --module CONFIG_USB_GSPCA_VC032X
    scripts/config --module CONFIG_USB_GSPCA_VICAM
    scripts/config --module CONFIG_USB_GSPCA_XIRLINK_CIT
    scripts/config --module CONFIG_USB_GSPCA_ZC3XX
    scripts/config --enable CONFIG_MEDIA_SUPPORT
    scripts/config --enable CONFIG_V4L2
    scripts/config --enable CONFIG_V4L1_COMPAT

    # --- USB WiFi ---
    echo "Enabling USB WiFi drivers..."
    scripts/config --module CONFIG_WLAN
    scripts/config --module CONFIG_WIRELESS_EXT
    scripts/config --module CONFIG_NET_WIRELESS
    scripts/config --module CONFIG_MAC80211
    scripts/config --module CONFIG_CFG80211
    scripts/config --module CONFIG_ATH_COMMON
    scripts/config --module CONFIG_ATH9K_HW
    scripts/config --module CONFIG_ATH9K_COMMON
    scripts/config --module CONFIG_ATH9K_HTC
    scripts/config --module CONFIG_AR9170_USB
    scripts/config --module CONFIG_ZD1211RW
    scripts/config --module CONFIG_RTL8187
    scripts/config --module CONFIG_R8192U_PCI
    scripts/config --module CONFIG_RT73USB
    scripts/config --module CONFIG_RT2800USB
    scripts/config --module CONFIG_RT2500USB
    scripts/config --enable CONFIG_NETDEVICES
    scripts/config --enable CONFIG_NET_CORE
    scripts/config --enable CONFIG_NET_RADIO

    # --- I2C Sensors (barometer, magnetometer via external MCU) ---
    echo "Enabling I2C sensor drivers..."
    scripts/config --enable CONFIG_I2C
    scripts/config --module CONFIG_I2C_DEV
    scripts/config --module CONFIG_I2C_OMAP
    scripts/config --enable CONFIG_I2C_CHARDEV
    scripts/config --module CONFIG_SENSORS_BMP085
    scripts/config --module CONFIG_SENSORS_TSL2550
    scripts/config --module CONFIG_SENSORS_LIS3LV02D
    scripts/config --module CONFIG_SENSORS_AK8975

    # --- SPI ---
    scripts/config --module CONFIG_SPI
    scripts/config --module CONFIG_SPI_OMAP24XX
    scripts/config --module CONFIG_SPI_SPIDEV

    # --- 1-Wire (DS18B20 temp sensors, etc.) ---
    scripts/config --module CONFIG_W1
    scripts/config --module CONFIG_W1_MASTER_GPIO
    scripts/config --module CONFIG_W1_SLAVE_THERM

    # --- Industrial I/O (accelerometer, gyro, magnetometer via I2C/SPI) ---
    scripts/config --module CONFIG_IIO
    scripts/config --module CONFIG_AK8975
    scripts/config --module CONFIG_IIO_KFIFO_BUF
    scripts/config --module CONFIG_IIO_SYSFS_TRIGGER

    # --- PWM (ESC/motor control, servo via OMAP PWM) ---
    scripts/config --enable CONFIG_PWM
    scripts/config --module CONFIG_PWM_OMAP

    # --- LED drivers (status LEDs) ---
    scripts/config --module CONFIG_LEDS_GPIO
    scripts/config --module CONFIG_LEDS_TRIGGER_TIMER
    scripts/config --module CONFIG_LEDS_TRIGGER_HEARTBEAT
    scripts/config --module CONFIG_LEDS_TRIGGER_DEFAULT_ON

    # --- Misc / Parrot-specific ---
    scripts/config --enable CONFIG_PARROT_GPIO 2>/dev/null || true
    scripts/config --enable CONFIG_ARCH_OMAP_PARROT 2>/dev/null || true

    # --- File systems (for USB storage automount) ---
    echo "Enabling filesystem drivers..."
    scripts/config --module CONFIG_FAT_FS
    scripts/config --module CONFIG_VFAT_FS
    scripts/config --module CONFIG_NTFS_FS
    scripts/config --enable CONFIG_NTFS_RW
    scripts/config --module CONFIG_EXT2_FS
    scripts/config --module CONFIG_EXT3_FS
    scripts/config --module CONFIG_EXT4_FS
    scripts/config --module CONFIG_EXFAT_FS 2>/dev/null || true
    scripts/config --module CONFIG_FUSE_FS
    scripts/config --module CONFIG_AUTOFS4_FS
    scripts/config --enable CONFIG_LBDAF

    # --- Network filesystems (for NAS storage) ---
    scripts/config --module CONFIG_NFS_FS
    scripts/config --module CONFIG_NFS_V3
    scripts/config --module CONFIG_NFS_V4
    scripts/config --enable CONFIG_NFSD

    # --- CRC / crypto (needed by many modules) ---
    scripts/config --enable CONFIG_CRC16
    scripts/config --enable CONFIG_CRC32
    scripts/config --enable CONFIG_CRC_ITU_T
    scripts/config --enable CONFIG_CRC7
    scripts/config --enable CONFIG_CRC_CCITT
    scripts/config --enable CONFIG_LIBCRC32C

    # --- Network crypto (for WiFi/WPA) ---
    scripts/config --module CONFIG_LIB80211
    scripts/config --module CONFIG_LIB80211_CRYPT_WEP
    scripts/config --module CONFIG_LIB80211_CRYPT_CCMP
    scripts/config --module CONFIG_LIB80211_CRYPT_TKIP

    # --- Parrot specific audio (to make alsa work) ---
    scripts/config --module CONFIG_SND_OMAP_SOC
    scripts/config --module CONFIG_SND_OMAP_SOC_MCBSP
    scripts/config --module CONFIG_SND_OMAP_SOC_HDMI
    scripts/config --module CONFIG_SND_ARM 2>/dev/null || true

    echo "Config created with all modules enabled"
}

setup_kernel_build() {
    cd "$KERNEL_DIR"

    # Fix Makefile for cross-compilation
    sed -i 's/^ARCH\s*?=\s*$/\ARCH ?= arm/' Makefile 2>/dev/null || true
    sed -i 's|^CROSS_COMPILE\s*?=\s*$|CROSS_COMPILE ?= '"$TOOLCHAIN_PREFIX"'|' Makefile 2>/dev/null || true

    # Fix EXTRAVERSION to match drone
    sed -i 's/^EXTRAVERSION =.*$/EXTRAVERSION = .9-g980dab2/' Makefile 2>/dev/null || true

    echo "Preparing kernel (oldconfig + modules_prepare)..."
    make ARCH=arm CROSS_COMPILE="$TOOLCHAIN_PREFIX" olddefconfig 2>&1 | tail -3 || true
    make ARCH=arm CROSS_COMPILE="$TOOLCHAIN_PREFIX" modules_prepare 2>&1 | tail -5

    echo "Kernel build environment ready"
}

# ============================================================================
# Individual module builders — each builds its category
# ============================================================================
build_module() {
    local name="$1"
    local path="$2"
    local dep_paths="$3"  # space-separated paths to build first

    echo "  Building $name..."
    cd "$KERNEL_DIR"

    # Build dependencies first
    for dep in $dep_paths; do
        if [ -d "$dep" ]; then
            make ARCH=arm CROSS_COMPILE="$TOOLCHAIN_PREFIX" M="$dep" modules 2>&1 | tail -2
            # Copy any .ko from dep
            find "$dep" -name '*.ko' -exec cp {} "$OUTDIR/" \; 2>/dev/null || true
        fi
    done

    # Build the module itself
    if [ -d "$path" ]; then
        make ARCH=arm CROSS_COMPILE="$TOOLCHAIN_PREFIX" M="$path" modules 2>&1 | tail -5
        find "$path" -name '*.ko' -exec cp {} "$OUTDIR/" \; 2>/dev/null || true
        local kofile
        kofile=$(find "$path" -name "${name}.ko" 2>/dev/null | head -1)
        if [ -n "$kofile" ]; then
            echo "    ✓ ${name}.ko built"
        else
            echo "    ? ${name}.ko not found at $path (may have different name)"
        fi
    else
        echo "    ✗ Path not found: $path"
    fi
}

build_usb_serial() {
    echo "=== USB Serial Drivers ==="
    # Core first
    build_module "usbserial" "drivers/usb/serial" ""
    # Individual serial drivers
    for drv in pl2303 ftdi_sio cp210x ch341 option usb_wwan sierra ipw qualcomm navman mos7720 mos7840; do
        build_module "$drv" "drivers/usb/serial" ""
    done
    # Keyspan (special case - many sub-modules)
    build_module "keyspan" "drivers/usb/serial" ""
    build_module "keyspan_mpr" "drivers/usb/serial" ""
    build_module "keyspan_usa28" "drivers/usb/serial" ""
    build_module "keyspan_usa19" "drivers/usb/serial" ""
    build_module "keyspan_usa49w" "drivers/usb/serial" ""
    echo "Done: $(ls "$OUTDIR/"*.ko 2>/dev/null | wc -l) modules in $OUTDIR"
}

build_usb_net() {
    echo "=== USB Networking Drivers ==="
    # Core
    build_module "usbnet" "drivers/net/usb" ""

    # Individual net drivers
    for drv in asix cdc_ether cdc_eem cdc_ncm dm9601 smsc75xx smsc95xx gl620a net1080 plusb mcs7830 rndis_host cdc_subset zaurus cx82310_eth kalmia qmi_wwan hso int51x1 ipheth sierra_net vl600 rtl8150 pegasus catc kaweth; do
        build_module "$drv" "drivers/net/usb" ""
    done
    # MII support
    build_module "mii" "drivers/net" ""
    echo "Done: $(ls "$OUTDIR/"*.ko 2>/dev/null | wc -l) modules in $OUTDIR"
}

build_usb_wifi() {
    echo "=== USB WiFi Drivers ==="
    # Wireless core
    build_module "cfg80211" "net/wireless" ""
    build_module "mac80211" "net/mac80211" ""
    build_module "lib80211" "net/wireless" ""

    # Atheros
    build_module "ath" "drivers/net/wireless/ath" ""
    build_module "ath9k_hw" "drivers/net/wireless/ath/ath9k" ""
    build_module "ath9k_common" "drivers/net/wireless/ath/ath9k" ""
    build_module "ath9k_htc" "drivers/net/wireless/ath/ath9k" ""

    # Realtek
    build_module "rtl8187" "drivers/net/wireless" ""
    build_module "rtl8192cu" "drivers/net/wireless" ""
    build_module "rtlwifi" "drivers/net/wireless/rtlwifi" ""

    # ZyDAS
    build_module "zd1211rw" "drivers/net/wireless" ""

    # Ralink
    build_module "rt73usb" "drivers/net/wireless/rt2x00" ""
    build_module "rt2800usb" "drivers/net/wireless/rt2x00" ""
    build_module "rt2500usb" "drivers/net/wireless/rt2x00" ""
    build_module "rt2x00usb" "drivers/net/wireless/rt2x00" ""
    build_module "rt2x00lib" "drivers/net/wireless/rt2x00" ""

    echo "Done: $(ls "$OUTDIR/"*.ko 2>/dev/null | wc -l) modules in $OUTDIR"
}

build_usb_storage() {
    echo "=== USB Storage Drivers ==="
    # SCSI core
    for mod in scsi_mod scsi_transport_spi sd_mod; do
        build_module "$mod" "drivers/scsi" ""
    done
    # USB storage
    build_module "usb-storage" "drivers/usb/storage" ""
    # Filesystems
    for fs in vfat fat ntfs nls_cp437 nls_iso8859_1; do
        build_module "$fs" "fs" ""
    done
    echo "Done: $(ls "$OUTDIR/"*.ko 2>/dev/null | wc -l) modules in $OUTDIR"
}

build_usb_hid() {
    echo "=== USB HID Drivers ==="
    build_module "usbhid" "drivers/hid/usbhid" ""
    build_module "hid" "drivers/hid" ""
    for drv in hid_logitech hid_sony hid_microsoft hid_generic hid_apple hid_belkin hid_cherry hid_chicony hid_cypress hid_ezkey hid_gyration hid_kye hid_monterey hid_pantherlord hid_petalynx hid_samsung hid_sunplus hid_topseed hid_thrustmaster hid_zeroplus; do
        build_module "$drv" "drivers/hid" ""
    done
    # Joystick
    build_module "xpad" "drivers/input/joystick" ""
    build_module "joydev" "drivers/input" ""
    build_module "evdev" "drivers/input" ""
    echo "Done: $(ls "$OUTDIR/"*.ko 2>/dev/null | wc -l) modules in $OUTDIR"
}

build_usb_audio() {
    echo "=== USB Audio Drivers ==="
    # ALSA core
    for mod in snd snd_pcm snd_timer snd_hwdep snd_page_alloc; do
        build_module "$mod" "sound/core" ""
    done
    # USB audio
    build_module "snd-usb-audio" "sound/usb" ""
    build_module "snd-usbmidi-lib" "sound/usb" ""
    build_module "snd-usb-caiaq" "sound/usb" ""
    build_module "snd-hwdep" "sound/core" ""
    # OMAP audio
    build_module "snd-soc-omap" "sound/soc/omap" ""
    build_module "snd-soc-omap-mcbsp" "sound/soc/omap" ""
    echo "Done: $(ls "$OUTDIR/"*.ko 2>/dev/null | wc -l) modules in $OUTDIR"
}

build_usb_video() {
    echo "=== USB Video Drivers ==="
    # V4L2 core
    for mod in videodev v4l2_common v4l2_int_device; do
        build_module "$mod" "drivers/media/video" ""
    done
    # UVC
    build_module "uvcvideo" "drivers/media/video/uvc" ""
    # GSPCA (many cheap USB cameras)
    build_module "gspca_main" "drivers/media/video/gspca" ""
    for drv in gspca_ov534 gspca_zc3xx gspca_pac207 gspca_pac7302 gspca_pac7311 gspca_sn9c20x gspca_sonixb gspca_sonixj gspca_spca561 gspca_stk014 gspca_sunplus gspca_t613 gspca_topro gspca_vc032x gspca_xirlink_cit; do
        build_module "$drv" "drivers/media/video/gspca" ""
    done
    echo "Done: $(ls "$OUTDIR/"*.ko 2>/dev/null | wc -l) modules in $OUTDIR"
}

build_sensors() {
    echo "=== I2C/SPI Sensor Drivers ==="
    # I2C core
    build_module "i2c-core" "drivers/i2c" ""
    build_module "i2c-dev" "drivers/i2c" ""
    build_module "i2c-omap" "drivers/i2c/busses" ""

    # Barometer
    build_module "bmp085" "drivers/misc" ""

    # Industrial I/O
    build_module "industrialio" "drivers/iio" ""

    # Magnetometer
    build_module "ak8975" "drivers/iio/magnetometer" ""

    # 1-Wire
    build_module "wire" "drivers/w1" ""
    build_module "w1-gpio" "drivers/w1/masters" ""
    build_module "w1-therm" "drivers/w1/slaves" ""

    echo "Done: $(ls "$OUTDIR/"*.ko 2>/dev/null | wc -l) modules in $OUTDIR"
}

build_usb_gps() {
    echo "=== USB GPS Drivers ==="
    # Serial drivers for GPS
    build_usb_serial  # all serial drivers
    # CDC ACM (many GPS receivers use this)
    build_module "cdc-acm" "drivers/usb/class" ""
    echo "GPS-ready modules built"
    echo ""
    echo "Supported GPS receivers:"
    echo "  - u-blox 7/8/9 (CP210x or FTDI) → /dev/ttyUSB0"
    echo "  - u-blox NEO-6M (CP210x) → /dev/ttyUSB0"
    echo "  - GlobalSat BU-353S4 (PL2303) → /dev/ttyUSB0"
    echo "  - Parrot NMEA Flight Recorder (CP210x, ID 19CF:3000) → /dev/ttyUSB0"
    echo "  - USB GPS with CDC ACM → /dev/ttyACM0"
    echo "  - Any FTDI-based GPS → /dev/ttyUSB0"
}

build_usb_4g() {
    echo "=== 4G/5G Modem Drivers ==="
    # CDC ACM (modem AT commands)
    build_module "cdc-acm" "drivers/usb/class" ""

    # Networking
    build_usb_net  # all net modules

    # Serial (for AT command control)
    for drv in option usb_wwan; do
        build_module "$drv" "drivers/usb/serial" ""
    done

    # USB WDM (for MBIM)
    build_module "cdc-wdm" "drivers/usb/class" ""

    # QMI
    build_module "qmi_wwan" "drivers/net/usb" ""

    echo "4G/5G-ready modules built"
    echo ""
    echo "Supported modems:"
    echo "  - Huawei E3372 (cdc_ether / cdc_ncm) → ethX"
    echo "  - Huawei E3276 (cdc_ether) → ethX"
    echo "  - ZTE MF833 (cdc_ether / cdc_ncm) → ethX"
    echo "  - Quectel EC25 (QMI + serial) → wwan0 + /dev/ttyUSB*"
    echo "  - Sierra Wireless MC7455 (QMI/MBIM) → wwan0"
    echo "  - Generic RNDIS tethering → usb0"
}

# ============================================================================
# Clean
# ============================================================================
clean_build() {
    echo "Cleaning build directory..."
    rm -rf "$WORKDIR"
    echo "Cleaned $WORKDIR"
    echo "Module output directory $OUTDIR preserved"
    echo "To clean modules too: rm -rf $OUTDIR"
}

# ============================================================================
# Build all
# ============================================================================
build_all() {
    download_kernel_source
    apply_config
    setup_kernel_build
    echo ""
    echo "=============================================="
    echo "  Building ALL kernel modules"
    echo "=============================================="
    echo ""
    build_usb_serial
    build_usb_video
    build_usb_net
    build_usb_wifi
    build_usb_storage
    build_usb_hid
    build_usb_audio
    build_sensors
    echo ""
    echo "=============================================="
    echo "  Build complete!"
    echo "=============================================="
    echo ""
    echo "Modules built: $(find "$OUTDIR" -name '*.ko' | wc -l)"
    echo "Output: $OUTDIR"
    ls -la "$OUTDIR"/*.ko 2>/dev/null | head -30 || echo "(no modules found)"
    echo ""
    echo "Deploy all:"
    echo "  cd $(dirname "$0") && tar czf modules.tar.gz -C modules ."
    echo "  ftp 192.168.1.1"
    echo "    put modules.tar.gz /data/video/"
    echo "  telnet 192.168.1.1"
    echo "    cd /data/video && tar xzf modules.tar.gz"
    echo "    insmod <module>.ko"
    echo ""
    echo "Or deploy individual modules:"
    echo "  cd tools && ./deploy.sh $OUTDIR/cdc-acm.ko 192.168.1.1"
}

# ============================================================================
# Dispatch
# ============================================================================
mkdir -p "$WORKDIR" "$OUTDIR"
check_toolchain

case "${1:-all}" in
    all|full)       build_all ;;
    usb-serial)     download_kernel_source; apply_config; setup_kernel_build; build_usb_serial ;;
    usb-net)        download_kernel_source; apply_config; setup_kernel_build; build_usb_net ;;
    usb-wifi)       download_kernel_source; apply_config; setup_kernel_build; build_usb_wifi ;;
    usb-storage)    download_kernel_source; apply_config; setup_kernel_build; build_usb_storage ;;
    usb-hid)        download_kernel_source; apply_config; setup_kernel_build; build_usb_hid ;;
    usb-audio)      download_kernel_source; apply_config; setup_kernel_build; build_usb_audio ;;
    usb-video)      download_kernel_source; apply_config; setup_kernel_build; build_usb_video ;;
    sensors)        download_kernel_source; apply_config; setup_kernel_build; build_sensors ;;
    usb-gps)        download_kernel_source; apply_config; setup_kernel_build; build_usb_gps ;;
    usb-4g)         download_kernel_source; apply_config; setup_kernel_build; build_usb_4g ;;
    clean)          clean_build ;;
    *)
        echo "Usage: $0 [category]"
        echo ""
        echo "Categories:"
        echo "  all|full      Build every module category"
        echo "  usb-serial    USB serial adapters (GPS, modems, telemetry)"
        echo "  usb-net       USB networking (Ethernet, tethering)"
        echo "  usb-wifi      USB WiFi adapters (ath9k_htc, rtl, zd1211)"
        echo "  usb-storage   USB flash drives, SD card readers"
        echo "  usb-hid       Gamepads, joysticks, keyboards"
        echo "  usb-audio     USB speakers, microphones"
        echo "  usb-video     USB cameras (UVC, GSPCA)"
        echo "  sensors       I2C/SPI sensors via external MCU bridge"
        echo "  usb-gps       All GPS-relevant drivers (serial + CDC)"
        echo "  usb-4g        All 4G/5G modem drivers (net + serial)"
        echo "  clean         Remove build artifacts"
        exit 1
        ;;
esac

# ============================================================================
# Post-build summary
# ============================================================================
echo ""
echo "=== Module Build Summary ==="
echo "  Category: ${1:-all}"
echo "  Modules in $OUTDIR:"
if [ -d "$OUTDIR" ]; then
    ls -lhS "$OUTDIR"/*.ko 2>/dev/null | awk '{print "    " $5 "  " $9 " (" $NF ")"}' || echo "    (none built)"
fi
echo ""
echo "To deploy to drone:"
echo "  cd tools && ./deploy.sh $OUTDIR/<module>.ko 192.168.1.1"
