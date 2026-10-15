#!/usr/bin/env bash
# Builds libfreenect2-metal for ofxKinectV2 on Linux (x86_64 / aarch64), Release, with the same
# pipelines as the Windows build: CUDA (when nvcc is installed) + OpenCL + OpenGL + CPU,
# TurboJPEG. Installs headers and the shared library into libs/libfreenect2.
#
# Keep the CMake options in sync with build_libfreenect2_windows.ps1.
#
# Requirements (Debian / Ubuntu 22.04+):
#   git clone https://github.com/hbmartin/libfreenect2-metal
#   sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev \
#                    libglfw3-dev ocl-icd-opencl-dev opencl-headers
#   CUDA toolkit (optional) - nvcc on PATH or CUDA_HOME set. libfreenect2-metal compiles CUDA
#   with CMake's native CUDA support: no CUDA samples / helper_math.h needed.
#   udev rule for non-root access: sudo cp <libfreenect2-metal>/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
#
# Usage: ./build_libfreenect2_linux.sh /path/to/libfreenect2-metal [--no-cuda] [--no-opencl]

set -euo pipefail

SOURCE="$(cd "${1:?usage: $0 /path/to/libfreenect2-metal [--no-cuda] [--no-opencl]}" && pwd)"
shift
ADDON="$(cd "$(dirname "$0")/.." && pwd)"
CUDA=ON
OPENCL=ON
for arg in "$@"; do
    case "$arg" in
        --no-cuda) CUDA=OFF ;;
        --no-opencl) OPENCL=OFF ;;
    esac
done

CUDA_HOME="${CUDA_HOME:-/usr/local/cuda}"
NVCC="$(command -v nvcc || true)"
[ -z "$NVCC" ] && [ -x "$CUDA_HOME/bin/nvcc" ] && NVCC="$CUDA_HOME/bin/nvcc"
if [ "$CUDA" = ON ] && [ -z "$NVCC" ]; then
    echo "nvcc not found, building without CUDA"
    CUDA=OFF
fi

ARCH="$(uname -m)"
case "$ARCH" in
    x86_64) OF_PLATFORM=linux64 ;;
    aarch64) OF_PLATFORM=linuxaarch64 ;;
    armv7l) OF_PLATFORM=linuxarmv7l ;;
    *) OF_PLATFORM=linux ;;
esac

CMAKE_ARGS=(
    -S "$SOURCE" -B "$SOURCE/build_ofx"
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_INSTALL_PREFIX="$SOURCE/build_ofx/install"
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_EXAMPLES=ON
    -DBUILD_OPENNI2_DRIVER=OFF
    -DBUILD_TESTING=OFF
    -DBUILD_CALIBRATION_TOOLS=OFF
    -DENABLE_METAL=OFF
    -DENABLE_OPENGL=ON
    -DENABLE_OPENCL=$OPENCL
    -DENABLE_CUDA=$CUDA
    -DENABLE_VAAPI=OFF
    -DENABLE_TEGRAJPEG=OFF
)

if [ "$CUDA" = ON ]; then
    # Maxwell to Ada, plus PTX for newer GPUs - same list as Windows. CUDA 13 dropped Maxwell /
    # Pascal: remove 52 and 61 if nvcc refuses them. Jetson: use its own (53 / 62 / 72 / 87).
    CMAKE_ARGS+=(-DCMAKE_CUDA_COMPILER="$NVCC" "-DCMAKE_CUDA_ARCHITECTURES=52-real;61-real;75-real;86-real;89")
fi

rm -f "$SOURCE/build_ofx/CMakeCache.txt"
cmake "${CMAKE_ARGS[@]}"
cmake --build "$SOURCE/build_ofx" --parallel
cmake --install "$SOURCE/build_ofx"

INC_OUT="$ADDON/libs/libfreenect2/include/$OF_PLATFORM/libfreenect2"
LIB_OUT="$ADDON/libs/libfreenect2/lib/$OF_PLATFORM"
rm -rf "$INC_OUT" "$LIB_OUT"
mkdir -p "$INC_OUT" "$LIB_OUT"
cp -R "$SOURCE/build_ofx/install/include/libfreenect2/." "$INC_OUT/"
cp -P "$SOURCE/build_ofx/install/lib/"libfreenect2.so* "$LIB_OUT/"

echo
echo "libfreenect2-metal $(git -C "$SOURCE" rev-parse --short HEAD) installed into $ADDON/libs/libfreenect2 ($OF_PLATFORM, CUDA=$CUDA OpenCL=$OPENCL)"
grep "define LIBFREENECT2_WITH" "$INC_OUT/config.h"
echo "Check the app finds libfreenect2.so at run time (see HANDOFF_MAC_LINUX.md)."
