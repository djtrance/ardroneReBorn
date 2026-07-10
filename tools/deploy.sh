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

LOCAL_FILE="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
DRONE_IP="${2:-192.168.1.1}"
DRONE_PATH="/data/video/"

if [ ! -f "$LOCAL_FILE" ]; then
    echo "ERROR: File not found: $1 (resolved: $LOCAL_FILE)"
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

# Check connectivity (use timeout on telnet port 23 since drone may block ping)
echo "Checking connectivity to $DRONE_IP..."
if ! (echo "quit" | timeout 3 telnet "$DRONE_IP" 2>&1 | grep -q "Connected"); then
    echo "WARNING: Cannot reach $DRONE_IP via ping. Will try FTP anyway."
fi

# Upload via FTP
echo "Uploading via FTP..."
ftp_ok=0
{
    echo "user anonymous drone"
    sleep 1
    echo "bin"
    sleep 1
    echo  "ejecutando >  $LOCAL_FILE ${DRONE_PATH}${FILENAME}"
    echo "put $LOCAL_FILE ${DRONE_PATH}${FILENAME}"
    sleep 2
    echo "quit"
} | ftp -n -v "$DRONE_IP" 2>&1 && ftp_ok=1

if [ "$ftp_ok" != "1" ]; then
    echo "WARNING: FTP upload may have failed."
    echo "Trying alternative: netcat pipe..."
    # Alternative: pipe the file directly via a reverse shell or nc
    cat "$LOCAL_FILE" | nc -w 3 "$DRONE_IP" 5555 2>/dev/null && echo " OK" || echo " Alternative also failed."
    echo ""
    echo "You can upload manually via:"
    echo "  ftp $DRONE_IP"
    echo "  (user: anonymous, password: anything)"
    echo "  bin"
    echo "  put $(cd "$(dirname "$LOCAL_FILE")" && pwd)/$(basename "$LOCAL_FILE") ${DRONE_PATH}${FILENAME}"
fi
echo " Upload step complete."

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
echo "Or via one-liner:"
echo "  echo '${DRONE_PATH}${FILENAME}' | telnet $DRONE_IP"
echo ""
echo "For a GStreamer plugin (.so):"
echo "  cd ../tools && python ardrone2_helper.py --host $DRONE_IP upload_gst_module ../build/bin/${FILENAME}"

echo ""
echo "Test programs in build/bin/:"
echo "  test_connection   — Verify AT commands + navdata"
echo "  test_camera       — Capture frames from V4L2 camera"
echo "  test_vision       — Camera + optical flow + obstacle detection"
