#!/usr/bin/env bash
# deploy.sh — Upload binary to AR.Drone 2.0 via FTP + chmod
set -euo pipefail

usage() {
    echo "Usage: $0 <local-file> [drone-ip]"
    echo ""
    echo "Upload a binary to AR.Drone 2.0 via FTP, set permissions, and prepare for execution."
    echo ""
    echo "Arguments:"
    echo "  <local-file>   Path to binary to upload (ELF or .so)"
    echo "  [drone-ip]     Drone IP address (default: 192.168.1.1)"
    echo ""
    echo "Examples:"
    echo "  $0 ../build/bin/drone_encoder"
    echo "  $0 ../build/bin/libgstparrot_enc.so 192.168.1.1"
    exit 1
}

if [ $# -lt 1 ]; then
    usage
fi

LOCAL_FILE="$1"
DRONE_IP="${2:-192.168.1.1}"
DRONE_PATH="/data/video/"

if [ ! -f "$LOCAL_FILE" ]; then
    echo "ERROR: File not found: $LOCAL_FILE"
    exit 1
fi

FILENAME=$(basename "$LOCAL_FILE")
FILESIZE=$(stat -f%z "$LOCAL_FILE" 2>/dev/null || stat -c%s "$LOCAL_FILE" 2>/dev/null)

echo "=== Deploy to AR.Drone 2.0 ==="
echo " File:      $LOCAL_FILE ($FILESIZE bytes)"
echo " Filename:  $FILENAME"
echo " Drone IP:  $DRONE_IP"
echo " Dest:      ${DRONE_PATH}${FILENAME}"
echo ""

# Check connectivity
echo "Checking connectivity to $DRONE_IP..."
if ! ping -c 1 -W 2 "$DRONE_IP" &>/dev/null; then
    echo "ERROR: Cannot reach $DRONE_IP."
    echo "Make sure your WiFi is connected to the AR.Drone network."
    exit 1
fi
echo " OK"

# Upload via FTP
echo "Uploading via FTP..."
{
    echo "user anonymous "
    sleep 1
    echo "put $LOCAL_FILE ${DRONE_PATH}${FILENAME}"
    sleep 1
    echo "quit"
} | ftp -n "$DRONE_IP" 2>&1 || {
    echo "ERROR: FTP upload failed."
    echo "Make sure the drone is powered on and WiFi connected."
    exit 1
}
echo " Upload complete."

# Set permissions via telnet
echo "Setting permissions via telnet..."
{
    sleep 1
    echo "killall -9 $FILENAME 2>/dev/null"
    echo "chmod 777 ${DRONE_PATH}${FILENAME}"
    echo "ls -la ${DRONE_PATH}${FILENAME}"
    sleep 1
    echo "exit"
} | telnet "$DRONE_IP" 2>&1 || true

echo ""
echo "=== Deployment complete ==="
echo ""
echo "To run on the drone:"
echo "  telnet $DRONE_IP"
echo "  ${DRONE_PATH}${FILENAME}"
echo ""
echo "Or for a GStreamer plugin (.so):"
echo "  cd ../tools && python ardrone2_helper.py --host $DRONE_IP upload_gst_module ../build/bin/${FILENAME}"
