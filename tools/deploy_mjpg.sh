#!/bin/bash
# Deploy mjpg-streamer + uvcvideo.ko to AR.Drone 2.0
# Usage: ./deploy_mjpg.sh [drone-ip]

DRONE_IP=${1:-192.168.1.1}
MJDIR=../external/ardrone-camera/mjpg-streamer
REMOTE=/data/video/mjpg-streamer

echo "=== Deploying mjpg-streamer + kernel module to $DRONE_IP ==="

# Create remote dir
(
echo "mkdir $REMOTE"
echo "mkdir $REMOTE/www"
) | ftp -n $DRONE_IP 2>/dev/null

# Upload binaries
echo "Uploading binaries..."
for f in mjpg_streamer input_uvc.so output_http.so output_udp.so output_file.so output_autofocus.so libjpeg.so*; do
    if [ -f "$MJDIR/$f" ]; then
        echo "  $f"
        ftp -n $DRONE_IP <<EOF
user anonymous none
binary
put $MJDIR/$f $REMOTE/$f
EOF
    fi
done

echo "Uploading www/..."
for f in $MJDIR/www/*; do
    fn=$(basename "$f")
    ftp -n $DRONE_IP <<EOF
user anonymous none
binary
put "$f" "$REMOTE/www/$fn"
EOF
done | grep -v "^ftp>"

# Upload uvcvideo.ko
echo "Uploading uvcvideo.ko..."
ftp -n $DRONE_IP <<EOF
user anonymous none
binary
put ../external/ardrone-camera/uvcvideo.ko /custom_modules/uvcvideo.ko
EOF

echo ""
echo "=== Deploy complete ==="
echo ""
echo "On drone, run:"
echo "  insmod /custom_modules/uvcvideo.ko quirks=640"
echo "  cd $REMOTE && LD_LIBRARY_PATH=. ./mjpg_streamer \\"
echo '    -i "./input_uvc.so -f 30 -device /dev/video7 -r 320x240" \\'
echo '    -o "./output_http.so -w ./www"'
echo ""
