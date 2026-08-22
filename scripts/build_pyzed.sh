#!/usr/bin/env bash
# Build the ZED Python binding against the SDK and numpy INSIDE the drone image,
# producing a wheel that docker/Dockerfile.jetson then installs.
#
#   ./build_pyzed.sh          # wheel lands in this directory
#
# WHY NOT STEREOLABS' OWN WHEEL
# pyzed-4.0-cp310 is the only cp310 build they publish, and it is compiled
# against numpy 2.x. Loaded against the numpy 1.21 this image needs it dies with
#   ValueError: numpy.ndarray size changed ... Expected 96 from C header, got 88
# and numpy 1.x is not negotiable here: the image's cv2 (4.5.4, from apt) is
# built against it, and the numpy-2 mismatch is silent at import and SEGFAULTS
# at the first array conversion (docs/LANDING-SITES.md §9). Building the binding
# against the numpy and the SDK actually installed removes both skews at once.
#
# WHY THIS IS NOT A DOCKERFILE STEP
# Linking needs libsl_zed.so, whose own NEEDED list includes libcuda.so.1 — and
# libcuda is injected by nvidia-container-runtime at CONTAINER START, so it does
# not exist during `docker build`. Hence: build in a `docker run --runtime
# nvidia`, then COPY the wheel in.
#
# Four things that are not obvious, each of which failed a build first:
#   * tag v4.0, not master. master targets SDK 5.4 and refuses anything older
#     ("Required ZED SDK version: 5.0 / Aborting").
#   * /usr/local/cuda is created by hand: the runtime injects the VERSIONED
#     directory (cuda.csv says `dir, /usr/local/cuda-10.2`) and setup.py looks
#     for exactly /usr/local/cuda.
#   * setup.py's `from distutils.core import setup, Extension` is rewritten to
#     setuptools. distutils has no bdist_wheel and importing setuptools first
#     does not add one — distutils' setup() only reads distutils' own registry.
#   * libusb-1.0-0-dev. The extension links -lusb-1.0; without the -dev package
#     the 20-minute compile succeeds and only the final link fails.
#
# The checkout lives here, not in /tmp, so its object files survive the
# container: sl.cpp is one huge Cython-generated translation unit and takes
# ~20 minutes on this board.
set -euxo pipefail

IMAGE="${IMAGE:-hydrone-jetson:humble}"
HERE="$(cd "$(dirname "$0")" && pwd)"

docker run --rm --runtime nvidia \
  --entrypoint /bin/bash \
  -v "$HERE":/out \
  "$IMAGE" -lc 'set -eux
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -qq
    apt-get install -y -qq --no-install-recommends \
        git build-essential python3-dev libusb-1.0-0-dev
    python3 -m pip install -q "cython<3" wheel "setuptools<80"
    python3 -c "import numpy; print(\"building against numpy\", numpy.__version__)"

    [ -e /usr/local/cuda ] || ln -s /usr/local/cuda-10.2 /usr/local/cuda
    export CUDA_PATH=/usr/local/cuda
    export ZED_SDK_ROOT_DIR=/usr/local/zed

    cd /out
    if [ ! -d zed-python-api ]; then
      git clone --depth 1 --branch v4.0.8 https://github.com/stereolabs/zed-python-api.git
      sed -i "s/^from distutils.core import setup, Extension\$/from setuptools import setup, Extension/" \
          zed-python-api/src/setup.py
    fi
    cd zed-python-api/src
    grep -n "^from setuptools import" setup.py
    python3 setup.py bdist_wheel
    cp dist/*.whl /out/
    ls -l /out/*.whl
  '
