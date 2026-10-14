#!/usr/bin/env bash
# Builds libfreenect2 for ofxKinectV2 on Linux (x86_64 / aarch64), Release, with the same
# pipelines as the Windows build: CUDA (when nvcc is installed) + OpenCL + OpenGL + CPU,
# TurboJPEG. Installs headers and the shared library into libs/libfreenect2.
#
# Keep the CMake options in sync with build_libfreenect2_windows.ps1.
#
# Requirements (Debian / Ubuntu):
#   sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev \
#                    libglfw3-dev ocl-icd-opencl-dev opencl-headers
#   CUDA toolkit (optional) - nvcc on PATH or CUDA_HOME set
#   udev rule for non-root access: sudo cp <libfreenect2>/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
#
# Usage: ./build_libfreenect2_linux.sh /path/to/libfreenect2 [--no-cuda] [--no-opencl]

set -euo pipefail

SOURCE="$(cd "${1:?usage: $0 /path/to/libfreenect2 [--no-cuda] [--no-opencl]}" && pwd)"
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
if [ "$CUDA" = ON ] && ! command -v nvcc >/dev/null && [ ! -x "$CUDA_HOME/bin/nvcc" ]; then
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
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    -DBUILD_SHARED_LIBS=ON
    -DBUILD_EXAMPLES=ON
    -DBUILD_OPENNI2_DRIVER=OFF
    -DENABLE_CXX11=ON
    -DENABLE_OPENGL=ON
    -DENABLE_OPENCL=$OPENCL
    -DENABLE_CUDA=$CUDA
    -DENABLE_VAAPI=OFF
    -DENABLE_TEGRAJPEG=OFF
)

if [ "$CUDA" = ON ]; then
    # CUDA 11+ no longer installs the samples; the kernels need helper_math.h from them.
    SAMPLES="$SOURCE/depends/cuda_samples"
    if [ ! -f "$SAMPLES/common/inc/helper_math.h" ]; then
        mkdir -p "$SAMPLES/common/inc"
        curl -sSfL -o "$SAMPLES/common/inc/helper_math.h" \
            https://raw.githubusercontent.com/NVIDIA/cuda-samples/v12.1/Common/helper_math.h
    fi
    export NVCUDASAMPLES_ROOT="$SAMPLES"
    # Maxwell to Ada plus PTX, same as Windows (Jetson: add 53 / 62 / 72 / 87 as needed).
    NVCC="-Xcompiler;-fPIC"
    for a in 52 61 75 86 89; do NVCC="$NVCC;-gencode=arch=compute_$a,code=sm_$a"; done
    NVCC="$NVCC;-gencode=arch=compute_89,code=compute_89"
    CMAKE_ARGS+=(-DCUDA_TOOLKIT_ROOT_DIR="$CUDA_HOME" "-DCUDA_NVCC_FLAGS=$NVCC")
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
echo "libfreenect2 installed into $ADDON/libs/libfreenect2 ($OF_PLATFORM, CUDA=$CUDA OpenCL=$OPENCL)"
echo "Point it at the system libusb / turbojpeg at runtime; the project generator copies the .so into bin/libs."
