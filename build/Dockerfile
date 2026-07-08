FROM --platform=linux/amd64 ubuntu:22.04

LABEL description="Cross-compilation env for Parrot AR.Drone 2.0 (TI OMAP3530)"

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Base tools + V4L2 + GStreamer 0.10 deps
RUN apt-get update && apt-get install -y \
    wget ca-certificates \
    build-essential pkg-config \
    libv4l-dev libglib2.0-dev libxml2-dev libicu-dev \
    ftp telnet file binutils \
    && rm -rf /var/lib/apt/lists/*

# ARM cross-compilation toolchain (arm-linux-gnueabihf- prefix)
# Installs to /opt/arm-2016.02-linaro/bin/
ARG TC_DEB=parrot-tools-linuxgnutools-2016.02-linaro_1.0.0-2_amd64.deb
ARG TC_URL=https://github.com/tudelft/toolchains/raw/master/${TC_DEB}
ENV PATH="/opt/arm-2016.02-linaro/bin:${PATH}"

RUN wget -q "${TC_URL}" -O "/tmp/${TC_DEB}" \
    && dpkg -i "/tmp/${TC_DEB}" 2>/dev/null; \
    apt-get install -fy; \
    rm -f "/tmp/${TC_DEB}"; \
    arm-linux-gnueabihf-gcc --version; \
    echo "*** Fix: create stubs-soft.h symlink for softfp builds ***"; \
    SYSROOT=/opt/arm-2016.02-linaro/arm-linux-gnueabihf/libc; \
    ln -sf stubs-hard.h $SYSROOT/usr/include/gnu/stubs-soft.h

# GStreamer 0.10 — download .debs from snapshot.debian.org
# (removed from modern distro repos; last available on Debian 10)
# Extract with dpkg-deb -x (not dpkg -i) to avoid Debian 10 -> Ubuntu 22.04
# dependency issues (e.g. missing multiarch-support).
#
# We install two variants:
#   amd64 — headers + .pc files for the host (pkg-config)
#   armhf — libraries for link-time in the toolchain sysroot
#
# Note: armhf debs extract with multiarch triplet (arm-linux-gnueabihf/)
# inside the sysroot. We add a symlink so libraries are findable at
# $SYSROOT/usr/lib/ without extra -L flags.
RUN set -eux; \
    SNAP="http://snapshot.debian.org/archive/debian/20200101T000000Z/pool/main"; \
    BASE="${SNAP}/g/gstreamer0.10"; \
    PLUGINS="${SNAP}/g/gst-plugins-base0.10"; \
    SYSROOT=/opt/arm-2016.02-linaro/arm-linux-gnueabihf/libc; \
    \
    fetch() { wget -q "$1" -O "/tmp/$(basename $1)"; }; \
    \
    # amd64: headers for pkg-config \
    fetch "${BASE}/libgstreamer0.10-0_0.10.36-1.5_amd64.deb"; \
    fetch "${BASE}/libgstreamer0.10-dev_0.10.36-1.5_amd64.deb"; \
    fetch "${BASE}/gstreamer0.10-tools_0.10.36-1.5_amd64.deb"; \
    fetch "${PLUGINS}/libgstreamer-plugins-base0.10-0_0.10.36-2_amd64.deb"; \
    fetch "${PLUGINS}/libgstreamer-plugins-base0.10-dev_0.10.36-2_amd64.deb"; \
    fetch "${PLUGINS}/gstreamer0.10-plugins-base_0.10.36-2_amd64.deb"; \
    for f in /tmp/*_amd64.deb; do dpkg-deb -x "$f" /; done; \
    \
    # armhf: libraries for cross-linker \
    fetch "${BASE}/libgstreamer0.10-0_0.10.36-1.5_armhf.deb"; \
    fetch "${BASE}/libgstreamer0.10-dev_0.10.36-1.5_armhf.deb"; \
    fetch "${PLUGINS}/libgstreamer-plugins-base0.10-0_0.10.36-2_armhf.deb"; \
    fetch "${PLUGINS}/libgstreamer-plugins-base0.10-dev_0.10.36-2_armhf.deb"; \
    for f in /tmp/*_armhf.deb; do dpkg-deb -x "$f" "$SYSROOT"; done; \
    \
    # Multiarch path fix: armhf debs use arm-linux-gnueabihf/ triplet \
    if [ -d "$SYSROOT/usr/lib/arm-linux-gnueabihf" ]; then \
        ln -sf arm-linux-gnueabihf/*.so* "$SYSROOT/usr/lib/" 2>/dev/null || true; \
        ln -sf arm-linux-gnueabihf/*.la "$SYSROOT/usr/lib/" 2>/dev/null || true; \
        ln -sf arm-linux-gnueabihf/*.a "$SYSROOT/usr/lib/" 2>/dev/null || true; \
        ln -sf arm-linux-gnueabihf/pkgconfig "$SYSROOT/usr/lib/pkgconfig" 2>/dev/null || true; \
    fi; \
    \
    rm -f /tmp/*.deb; \
    ldconfig

# TI Codec Engine + XDAIS headers (copied from project build/sysroot/)
ENV TI_SYSROOT=/opt/tisysroot/ti/packages
COPY build/sysroot/ti/packages ${TI_SYSROOT}

# Verify host pkg-config works for GStreamer
RUN pkg-config --exists gstreamer-0.10 && \
    echo "GStreamer $(pkg-config --modversion gstreamer-0.10) OK" || \
    echo "ERROR: GStreamer 0.10 not found"

WORKDIR /workspace
CMD ["/bin/bash"]
