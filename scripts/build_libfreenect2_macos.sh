#!/usr/bin/env bash
# Builds libfreenect2-metal for ofxKinectV2 on macOS, Release, as a static library with the
# Metal (Apple GPUs), OpenCL, OpenGL and CPU depth pipelines, TurboJPEG, and VideoToolbox when
# enabled (see below). Installs headers and libs (libfreenect2.a plus static libusb /
# turbojpeg) into libs/libfreenect2. The compiled Metal shader library is embedded in
# libfreenect2.a, so nothing else needs shipping.
#
# Keep the CMake options in sync with build_libfreenect2_windows.ps1.
#
# Requirements:
#   - libfreenect2-metal source: git clone https://github.com/hbmartin/libfreenect2-metal
#   - full Xcode (the Metal compiler; on Xcode 26+ also: xcodebuild -downloadComponent MetalToolchain)
#   - Homebrew: brew install cmake pkg-config libusb jpeg-turbo glfw
#   - a NATIVE arm64 terminal on Apple Silicon (`arch` must print arm64)
# Builds for the Mac's own architecture (Homebrew libraries are single-arch). For a universal
# build, build libusb / jpeg-turbo for both architectures first and pass CMAKE_OSX_ARCHITECTURES.
#
# VideoToolbox (hardware JPEG decoding): libfreenect2-metal leaves it OFF on Apple Silicon
# because its decoder crashes on M1 and later. --videotoolbox turns it on anyway, e.g. to test
# a fixed decoder (see HANDOFF_MAC_LINUX.md: kinect4metal has one).
#
# Usage: ./build_libfreenect2_macos.sh /path/to/libfreenect2-metal [--videotoolbox]

set -euo pipefail

SOURCE="$(cd "${1:?usage: $0 /path/to/libfreenect2-metal [--videotoolbox]}" && pwd)"
shift
ADDON="$(cd "$(dirname "$0")/.." && pwd)"
BREW="$(brew --prefix)"
VT_ARGS=()
for arg in "$@"; do
    case "$arg" in
        --videotoolbox) VT_ARGS=(-DENABLE_VIDEOTOOLBOX=ON) ;;
    esac
done

rm -f "$SOURCE/build_ofx/CMakeCache.txt"
cmake -S "$SOURCE" -B "$SOURCE/build_ofx" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$SOURCE/build_ofx/install" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_OPENNI2_DRIVER=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_CALIBRATION_TOOLS=OFF \
    -DENABLE_METAL=ON \
    -DENABLE_OPENGL=ON \
    -DENABLE_OPENCL=ON \
    -DENABLE_CUDA=OFF \
    -DENABLE_VAAPI=OFF \
    -DENABLE_TEGRAJPEG=OFF \
    ${VT_ARGS[@]+"${VT_ARGS[@]}"} \
    -DCMAKE_PREFIX_PATH="$BREW"
cmake --build "$SOURCE/build_ofx" --parallel
cmake --install "$SOURCE/build_ofx"

INC_OUT="$ADDON/libs/libfreenect2/include/osx/libfreenect2"
LIB_OUT="$ADDON/libs/libfreenect2/lib/osx"
rm -rf "$INC_OUT" "$LIB_OUT"
mkdir -p "$INC_OUT" "$LIB_OUT"
cp -R "$SOURCE/build_ofx/install/include/libfreenect2/." "$INC_OUT/"
cp "$SOURCE/build_ofx/install/lib/libfreenect2.a" "$LIB_OUT/"
# A static libfreenect2 needs its dependencies linked into the app too.
cp "$BREW/lib/libusb-1.0.a" "$LIB_OUT/"
if [ -f "$BREW/opt/jpeg-turbo/lib/libturbojpeg.a" ]; then cp "$BREW/opt/jpeg-turbo/lib/libturbojpeg.a" "$LIB_OUT/"; fi

echo
echo "libfreenect2-metal $(git -C "$SOURCE" rev-parse --short HEAD) installed into $ADDON/libs/libfreenect2 (osx, $(uname -m))"
grep "define LIBFREENECT2_WITH" "$INC_OUT/config.h"
