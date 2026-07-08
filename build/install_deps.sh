#!/usr/bin/env bash
set -euo pipefail

echo "=== Installing dependencies for parrotFramework cross-compilation ==="

# Detect platform
OS="$(uname -s)"

case "$OS" in
    Darwin)
        echo "Platform: macOS"
        echo ""
        echo "On macOS, the ARM toolchain cannot be installed directly via .deb."
        echo "Options:"
        echo ""
        echo "  Option 1: Docker (recommended)"
        echo "    docker pull ubuntu:22.04"
        echo "    docker run -it -v $(pwd):/workspace ubuntu:22.04 /bin/bash"
        echo "    # then run this script inside the container"
        echo ""
        echo "  Option 2: Manual toolchain extraction"
        echo "    brew install dpkg  # if needed"
        echo "    mkdir -p /opt/arm-2016.02-linaro"
        echo "    cd /opt/arm-2016.02-linaro"
        echo "    curl -sL https://github.com/tudelft/toolchains/raw/master/parrot-tools-linuxgnutools-2016.02-linaro_1.0.0-2_amd64.deb -o tc.deb"
        echo "    dpkg-deb -x tc.deb ."
        echo "    rm tc.deb"
        echo "    export PATH=/opt/arm-2016.02-linaro/bin:\$PATH"
        echo ""
        echo "  Option 3: Use ardrone2_gstreamer docker image"
        echo "    https://github.com/flixr/ardrone2_gstreamer"
        echo ""
        echo "After toolchain is available, install GStreamer 0.10 headers:"
        echo "  brew install gstreamer  # or compile from source"
        ;;
    Linux)
        echo "Platform: Linux"
        echo ""

        # Check if running on x86_64
        ARCH="$(uname -m)"
        if [ "$ARCH" != "x86_64" ]; then
            echo "WARNING: Only x86_64 hosts are supported for the toolchain."
            echo "Your architecture: $ARCH"
        fi

        # Install toolchain
        TC_DEB="parrot-tools-linuxgnutools-2016.02-linaro_1.0.0-2_amd64.deb"
        TC_URL="https://github.com/tudelft/toolchains/raw/master/$TC_DEB"

        if command -v arm-linux-gnueabihf-gcc &>/dev/null; then
            echo "Toolchain already installed."
            arm-linux-gnueabihf-gcc --version
        else
            echo "Downloading toolchain..."
            sudo apt-get update
            sudo apt-get install -y wget dpkg
            wget -c "$TC_URL" -O "/tmp/$TC_DEB"
            sudo dpkg -i "/tmp/$TC_DEB" || sudo apt-get install -f
            echo "Toolchain installed."
        fi

        # Install build dependencies
        echo ""
        echo "Installing build dependencies..."
        sudo apt-get install -y \
            build-essential \
            pkg-config \
            libgstreamer0.10-dev \
            libgstreamer-plugins-base0.10-dev \
            libv4l-dev \
            libglib2.0-dev

        echo ""
        echo "=== All dependencies installed ==="
        echo "Run 'make' in the build/ directory to compile."
        ;;
    *)
        echo "Unsupported platform: $OS"
        echo "This script supports macOS (partial) and Linux."
        exit 1
        ;;
esac
