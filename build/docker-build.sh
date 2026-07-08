#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="parrot-framework-toolchain"
DOCKERFILE_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"

echo "=== Building Docker image: ${IMAGE_NAME} ==="
echo "Base: Ubuntu 22.04 (amd64) + ARM toolchain + GStreamer 0.10 + V4L2"
echo "Context: ${PROJECT_DIR}"
echo ""

docker build -t "${IMAGE_NAME}" \
    -f "${DOCKERFILE_DIR}/Dockerfile" \
    "${PROJECT_DIR}"

echo ""
echo "=== Build complete ==="
echo "Run:  $(dirname "$0")/docker-run.sh"
