#!/bin/sh
# AR.Drone 2.0 startup script (runs on drone via telnet or /etc/init.d)
# Source: parrotFramework/tools/drone-boot.sh

REMOTE=/data/video

echo "=== parrotFramework boot ==="

# 1. Bind mount DSP libraries
if mount | grep -q '/opt/arm'; then
    echo "[OK] /opt/arm already mounted"
else
    mount --bind $REMOTE/opt/arm /opt/arm 2>/dev/null && echo "[OK] /opt/arm bind mount"
    mount --bind $REMOTE/opt/arm/lib/dsp /lib/dsp 2>/dev/null && echo "[OK] /lib/dsp bind mount"
fi

# 2. Kill conflicting program.elf
if pidof program.elf >/dev/null 2>&1; then
    killall program.elf 2>/dev/null && echo "[OK] program.elf killed"
    sleep 1
fi

# 3. Load USB camera kernel module (if available)
if [ -f /custom_modules/uvcvideo.ko ]; then
    if ! lsmod | grep -q uvcvideo; then
        insmod /custom_modules/uvcvideo.ko quirks=640 2>/dev/null && echo "[OK] uvcvideo loaded"
    else
        echo "[OK] uvcvideo already loaded"
    fi
fi

# 4. Init DSP (if binaries present)
if [ -f $REMOTE/dsp_init ]; then
    $REMOTE/dsp_init 2>/dev/null && echo "[OK] DSP init"
fi

# 5. Load USB serial for GPS
for mod in usbserial pl2303 cp210x ftdi_sio; do
    if [ -f /custom_modules/${mod}.ko ]; then
        insmod /custom_modules/${mod}.ko 2>/dev/null || true
    fi
done

# 6. Load 4G modem modules
for mod in usbnet cdc_ether cdc_ncm qmi_wwan cdc_acm; do
    if [ -f /custom_modules/${mod}.ko ]; then
        insmod /custom_modules/${mod}.ko 2>/dev/null || true
    fi
done

echo "=== parrotFramework boot complete ==="
