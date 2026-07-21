#!/bin/bash
# Deploy everything to AR.Drone 2.0
# Usage: ./deploy-all.sh [drone-ip]

DRONE_IP=${1:-192.168.1.1}
DEPLOY=../build/deploy
REMOTE=/data/video

echo "=== Deploying parrotFramework to $DRONE_IP ==="

upload() {
  local src="$1"
  local dst="$2"
  if [ -f "$src" ]; then
    ftp -n $DRONE_IP <<EOF
user anonymous none
binary
put "$src" "$dst"
EOF
  fi
}

create_remote_dir() {
  local dirs=("$@")
  for dir in "${dirs[@]}"; do
    echo "mkdir $dir" | ftp -n $DRONE_IP >/dev/null 2>&1 || true
  done
}

# Create remote directory structure
create_remote_dir "$REMOTE/bin" "$REMOTE/libs" "$REMOTE/www" "$REMOTE/kmodules" "/custom_modules"

# Upload binaries
echo "Uploading binaries..."
for f in build/bin/*; do
  upload "$f" "$REMOTE/bin/$(basename $f)"
done

# Upload libraries
echo "Uploading libraries..."
for f in build/deploy/libs/*; do
  upload "$f" "$REMOTE/libs/$(basename $f)"
done

# Upload www
echo "Uploading www..."
for f in build/deploy/www/*; do
  upload "$f" "$REMOTE/www/$(basename $f)"
done

# Upload kernel modules
echo "Uploading kernel modules..."
if [ -f build/deploy/kmodules/uvcvideo.ko ]; then
  upload "build/deploy/kmodules/uvcvideo.ko" "/custom_modules/uvcvideo.ko"
fi

# Upload boot script
if [ -f build/deploy/drone-boot.sh ]; then
  upload "build/deploy/drone-boot.sh" "$REMOTE/drone-boot.sh"
fi

echo ""
echo "=== Deploy complete ==="
echo ""
echo "On drone (telnet 192.168.1.1):"
echo ""
echo "  # Kill drone program & check camera"
echo "  killall program.elf; sleep 2"
echo "  ls -la /dev/video*"
echo "  export LD_LIBRARY_PATH=/data/video/libs"
echo ""
echo "  # Test camera"
echo "  /data/video/bin/test_camera /dev/video3 320 240 10"
echo ""
echo "  # Start Skycontroller proxy"
echo "  /data/video/bin/skyproxy -v"
echo ""
echo "  # mjpg-streamer (USB camera)"
echo "  insmod /custom_modules/uvcvideo.ko quirks=640 2>/dev/null"
echo '  /data/video/bin/mjpg_streamer -i "/data/video/libs/input_uvc.so -f 30 -device /dev/video7 -r 320x240" -o "/data/video/libs/output_http.so -w /data/video/www"'
echo ""
