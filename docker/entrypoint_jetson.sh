#!/usr/bin/env bash
# Entrypoint for the drone's image (docker/Dockerfile.jetson).
#
# Deliberately thinner than docker/entrypoint.sh: there is no BiguaSim to
# install and no simulator repo to mount. What it does do is fail loudly on the
# two runtime mistakes that otherwise look like a broken camera.
set -e

# 0) Teach the linker where the host's Tegra libraries were injected.
#    This CANNOT be done at build time: nvidia-container-runtime mounts these
#    directories when the container STARTS, so during `docker build` they do not
#    exist and ldconfig would cache nothing.
#
#    It matters for one specific failure. glvnd's EGL loader reads
#    /usr/share/glvnd/egl_vendor.d/10_nvidia.json (also injected) and dlopens
#    libEGL_nvidia.so.0 from tegra-egl. If that directory is not on the search
#    path the vendor library is not found, glvnd silently falls back to Mesa,
#    Mesa has no display in a headless container, and the ZED SDK dies at
#    IMPORT with
#        nvbuf_utils: Could not get EGL display connection
#    and exit status 255 — before any Python of ours runs. Mirrors the host's
#    own /etc/ld.so.conf.d/nvidia-tegra.conf.
if [ -d /usr/lib/aarch64-linux-gnu/tegra ]; then
    printf '%s\n' \
        /usr/lib/aarch64-linux-gnu/tegra \
        /usr/lib/aarch64-linux-gnu/tegra-egl \
        /usr/local/cuda-10.2/lib64 \
        > /etc/ld.so.conf.d/nvidia-tegra.conf 2>/dev/null || true
    ldconfig 2>/dev/null || true
fi

. /opt/ros/humble/setup.sh
[ -f /ws/install/setup.sh ] && . /ws/install/setup.sh

# 1) The nvidia runtime. Without `--runtime nvidia` the host's CUDA and
#    libcuda.so.1 are never injected, and the ZED SDK fails at dlopen with a
#    message about libcuda that reads like a driver problem rather than a
#    missing docker flag.
if [ ! -e /usr/lib/aarch64-linux-gnu/tegra/libcuda.so.1 ]; then
    echo "[entrypoint] WARNING: libcuda.so.1 is not present." >&2
    echo "[entrypoint]          Run this image with --runtime nvidia (or set" >&2
    echo "[entrypoint]          nvidia as the default runtime); the ZED SDK" >&2
    echo "[entrypoint]          cannot open the camera without it." >&2
fi

# 2) The cameras. --device or --privileged plus /dev, or there is nothing to
#    open. Checked separately because "no /dev/video*" and "the ZED is
#    unplugged" want different reactions from a person.
if ! ls /dev/video* >/dev/null 2>&1; then
    echo "[entrypoint] WARNING: no /dev/video* inside the container." >&2
    echo "[entrypoint]          Pass the cameras through: --device /dev/video0" >&2
    echo "[entrypoint]          --device /dev/video1, or -v /dev:/dev." >&2
fi

exec "$@"
