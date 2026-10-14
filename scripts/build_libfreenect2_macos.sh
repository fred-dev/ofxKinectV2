#!/usr/bin/env bash
# Builds libfreenect2 for ofxKinectV2 on macOS, Release, as a static library with the
# OpenCL + OpenGL + CPU pipelines and VideoToolbox JPEG decoding (Apple has no CUDA).
# Installs headers and libs (libfreenect2.a plus static libusb) into libs/libfreenect2.
#
# Keep the CMake options in sync with build_libfreenect2_windows.ps1.
#
# Requirements: Xcode command line tools, Homebrew: brew install cmake libusb jpeg-turbo glfw
# Builds for the machine's own architecture (Homebrew libraries are single-arch). For a
# universal build, build libusb for both architectures first and pass CMAKE_OSX_ARCHITECTURES.
#
# Usage: ./build_libfreenect2_macos.sh /path/to/libfreenect2

set -euo pipefail

SOURCE="$(cd "${1:?usage: $0 /path/to/libfreenect2}" && pwd)"
ADDON="$(cd "$(dirname "$0")/.." && pwd)"
BREW="$(brew --prefix)"

rm -f "$SOURCE/build_ofx/CMakeCache.txt"
cmake -S "$SOURCE" -B "$SOURCE/build_ofx" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$SOURCE/build_ofx/install" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCMAKE_OSX_DEPLOYMENT_TARGET=11.0 \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_OPENNI2_DRIVER=OFF \
    -DENABLE_CXX11=ON \
    -DENABLE_OPENGL=ON \
    -DENABLE_OPENCL=ON \
    -DENABLE_CUDA=OFF \
    -DENABLE_VAAPI=OFF \
    -DENABLE_TEGRAJPEG=OFF \
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
echo "libfreenect2 installed into $ADDON/libs/libfreenect2 (osx, $(uname -m), OpenCL)"
