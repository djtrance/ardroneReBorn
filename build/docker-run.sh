#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="parrot-framework-toolchain"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "=== Starting Docker build environment ==="
echo "Workspace: ${PROJECT_DIR}"
echo ""

docker run --rm -it \
    -v "${PROJECT_DIR}:/workspace" \
    -w /workspace/build \
    --network host \
    --name parrot-build \
    "${IMAGE_NAME}" \
    /bin/bash -c "
        echo '=== parrotFramework Build Environment ==='
        echo 'ARM GCC: ' \$(arm-linux-gnueabihf-gcc --version | head -1)
        echo 'GStreamer: ' \$(pkg-config --modversion gstreamer-0.10 2>/dev/null || echo 'not found')
        echo ''
        echo 'Available commands:'
        echo '  make          # build project'
        echo '  make plugin   # build GStreamer plugin'
        echo '  make clean    # clean object files'
        echo ''
        cd /workspace/build && exec /bin/bash
    "
