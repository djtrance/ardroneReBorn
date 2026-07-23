#!/bin/bash
# deploy_ath6kl.sh — Deploy ath6kl WiFi driver to AR.Drone 2.0
# Usage: ./deploy_ath6kl.sh [drone_ip]
set -euo pipefail

DRONE_IP="${1:-192.168.1.1}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEPLOY_DIR="$SCRIPT_DIR/../deploy"
FTP_USER="root"
FTP_PASS=""
REMOTE_DIR="/data/video"

echo "=== Deploying ath6kl WiFi driver to AR.Drone ==="
echo "  Target: ${DRONE_IP}"
echo ""

# Step 1: Upload kernel modules
echo "[1/4] Uploading kernel modules..."
for mod in compat.ko cfg80211.ko ath6kl_usb.ko; do
    echo "  $mod"
    curl -s --ftp-create-dirs -T "$DEPLOY_DIR/$mod" \
        "ftp://${DRONE_IP}${REMOTE_DIR}/${mod}" || {
        echo "  WARNING: curl FTP failed, trying ncftp..."
        ncftpput -R -u "$FTP_USER" "$DRONE_IP" "$REMOTE_DIR" "$DEPLOY_DIR/$mod" 2>/dev/null || {
            echo "  ERROR: Failed to upload $mod"
            echo "  Make sure drone is in FTP mode (not AP mode)"
            exit 1
        }
    }
done

# Step 2: Upload firmware
echo "[2/4] Uploading firmware..."
for fwfile in $(find "$DEPLOY_DIR/firmware" -type f); do
    relpath="${fwfile#$DEPLOY_DIR/firmware/}"
    remotefw="${REMOTE_DIR}/firmware/ath6k/${relpath}"
    echo "  $relpath"
    curl -s --ftp-create-dirs -T "$fwfile" \
        "ftp://${DRONE_IP}${remotefw}" 2>/dev/null || \
        ncftpput -R -u "$FTP_USER" "$DRONE_IP" "$(dirname "$remotefw")" "$fwfile" 2>/dev/null || true
done

# Step 3: Load modules on drone
echo "[3/4] Loading modules on drone..."
cat << 'LOADCMD' | ssh -o StrictHostKeyChecking=no root@"${DRONE_IP}" 2>/dev/null || \
    echo "SSH not available — use telnet instead"
cd /data/video
# Remove stock ar6000 if loaded
rmmod ar6000 2>/dev/null || true
# Load ath6kl driver stack (order matters!)
insmod compat.ko
insmod cfg80211.ko
insmod ath6kl_usb.ko
echo "Modules loaded:"
lsmod | grep -E "compat|cfg80211|ath6kl"
LOADCMD

echo ""
echo "[4/4] Done!"
echo ""
echo "=== Manual load commands (via telnet) ==="
echo "  telnet $DRONE_IP"
echo "  cd /data/video"
echo "  rmmod ar6000 2>/dev/null"
echo "  insmod compat.ko"
echo "  insmod cfg80211.ko"
echo "  insmod ath6kl_usb.ko"
echo ""
echo "=== To enable monitor mode ==="
echo "  iw dev wlan0 interface add mon0 type monitor"
echo "  ip link set mon0 up"
echo ""
echo "=== To unload ==="
echo "  rmmod ath6kl_usb"
echo "  rmmod cfg80211"
echo "  rmmod compat"
