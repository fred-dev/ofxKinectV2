#!/usr/bin/env bash
# Builds libfreenect2-metal for ofxKinectV2 on macOS, Release, as a static library with the
# Metal (Apple GPUs), OpenCL, OpenGL and CPU depth pipelines, and VideoToolbox (hardware)
# JPEG decoding with TurboJPEG as its run-time fallback. Installs headers and static libs
# (libfreenect2.a, libusb-1.0.a, libturbojpeg.a) into libs/libfreenect2. The compiled Metal
# shader library is embedded in libfreenect2.a, so nothing else needs shipping.
#
# Keep the CMake options in sync with build_libfreenect2_windows.ps1.
#
# Requirements:
#   - libfreenect2-metal source at 600cef5 with the patches in scripts/patches/libfreenect2-metal
#     applied (VideoToolbox fix for Apple Silicon, macOS interface claiming):
#       git clone https://github.com/hbmartin/libfreenect2-metal && cd libfreenect2-metal
#       git checkout 600cef5 && git am /path/to/ofxKinectV2/scripts/patches/libfreenect2-metal/*.patch
#   - full Xcode (the Metal compiler; on Xcode 26+ also: xcodebuild -downloadComponent MetalToolchain)
#   - Homebrew: brew install cmake pkg-config glfw    (glfw only for the Protonect test tool)
#     --universal also needs: brew install nasm
#   - a NATIVE arm64 terminal on Apple Silicon (`arch` must print arm64)
#
# libusb and libjpeg-turbo are built from source (pinned, checksummed) for macOS 11.0, the
# oldest system Xcode 26+'s libc++ supports (and the first with Apple Silicon): Homebrew's
# static libraries target the macOS they were built on, so apps would link with warnings and
# could call functions older systems don't have (libusb 1.0.30 picks up pipe2(), macOS 27+).
#
# Usage: ./build_libfreenect2_macos.sh /path/to/libfreenect2-metal [--universal] [--no-videotoolbox]
#   --universal         arm64 + x86_64 (default: this Mac's architecture only)
#   --no-videotoolbox   TurboJPEG only

set -euo pipefail

SOURCE="$(cd "${1:?usage: $0 /path/to/libfreenect2-metal [--universal] [--no-videotoolbox]}" && pwd)"
shift
ADDON="$(cd "$(dirname "$0")/.." && pwd)"
BREW="$(brew --prefix)"
DEPLOYMENT_TARGET=11.0
ARCHS="$(uname -m)"
VIDEOTOOLBOX=ON
for arg in "$@"; do
    case "$arg" in
        --universal) ARCHS="arm64;x86_64" ;;
        --no-videotoolbox) VIDEOTOOLBOX=OFF ;;
        --videotoolbox) ;; # the default now, accepted for old command lines
        *) echo "unknown option: $arg" >&2; exit 1 ;;
    esac
done

LIBUSB_VERSION=1.0.30
LIBUSB_SHA256=fea36f34f9156400209595e300840767ab1a385ede1dc7ee893015aea9c6dbaf
JPEGTURBO_VERSION=3.2.0
JPEGTURBO_SHA256=6f30092cef9fb839779646608f4ee14ae3cbac989c47fa05e841b0841f09878e

# Without the patches VideoToolbox crashes on Apple Silicon and opens fail at random.
if ! grep -q kMaxConsecutiveFailures "$SOURCE/src/vt_rgb_packet_processor.cpp" ||
   ! grep -q "claim directly, retrying" "$SOURCE/src/usb_control.cpp"; then
    echo "error: $SOURCE is missing the ofxKinectV2 patches. Apply them first:" >&2
    echo "  git -C \"$SOURCE\" am \"$ADDON\"/scripts/patches/libfreenect2-metal/*.patch" >&2
    exit 1
fi

WORK="$SOURCE/build_ofx"
DEPS="$WORK/deps"
PREFIX="$DEPS/install"
mkdir -p "$DEPS"
export MACOSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET"
JOBS="$(sysctl -n hw.ncpu)"

fetch() { # url sha256 file
    if [ ! -f "$DEPS/$3" ] || ! echo "$2  $DEPS/$3" | shasum -a 256 -c - >/dev/null 2>&1; then
        curl -fL --retry 3 -o "$DEPS/$3" "$1"
    fi
    echo "$2  $DEPS/$3" | shasum -a 256 -c -
}

# --- libusb (autotools tarball, both architectures in one pass) ---
if [ ! -f "$PREFIX/.libusb-$LIBUSB_VERSION-${ARCHS//;/-}" ]; then
    fetch "https://github.com/libusb/libusb/releases/download/v$LIBUSB_VERSION/libusb-$LIBUSB_VERSION.tar.bz2" \
          "$LIBUSB_SHA256" "libusb-$LIBUSB_VERSION.tar.bz2"
    rm -rf "$DEPS/libusb-$LIBUSB_VERSION"
    tar -xjf "$DEPS/libusb-$LIBUSB_VERSION.tar.bz2" -C "$DEPS"
    ARCH_FLAGS=""
    for a in ${ARCHS//;/ }; do ARCH_FLAGS="$ARCH_FLAGS -arch $a"; done
    (cd "$DEPS/libusb-$LIBUSB_VERSION" &&
        # ac_cv_func_pipe2=no: the SDK declares pipe2 (macOS 27+), libusb would call it unguarded.
        ./configure --prefix="$PREFIX" --disable-shared --enable-static --disable-dependency-tracking \
            ac_cv_func_pipe2=no \
            CFLAGS="-O2 $ARCH_FLAGS -mmacosx-version-min=$DEPLOYMENT_TARGET" \
            LDFLAGS="$ARCH_FLAGS -mmacosx-version-min=$DEPLOYMENT_TARGET" >/dev/null &&
        make -j"$JOBS" >/dev/null && make install >/dev/null)
    touch "$PREFIX/.libusb-$LIBUSB_VERSION-${ARCHS//;/-}"
fi

# --- libjpeg-turbo (CMake, one architecture per build, then lipo) ---
if [ ! -f "$PREFIX/.jpeg-turbo-$JPEGTURBO_VERSION-${ARCHS//;/-}" ]; then
    fetch "https://github.com/libjpeg-turbo/libjpeg-turbo/releases/download/$JPEGTURBO_VERSION/libjpeg-turbo-$JPEGTURBO_VERSION.tar.gz" \
          "$JPEGTURBO_SHA256" "libjpeg-turbo-$JPEGTURBO_VERSION.tar.gz"
    rm -rf "$DEPS/libjpeg-turbo-$JPEGTURBO_VERSION"
    tar -xzf "$DEPS/libjpeg-turbo-$JPEGTURBO_VERSION.tar.gz" -C "$DEPS"
    SLICES=()
    for a in ${ARCHS//;/ }; do
        B="$DEPS/jpeg-turbo-build-$a"
        rm -rf "$B"
        cmake -S "$DEPS/libjpeg-turbo-$JPEGTURBO_VERSION" -B "$B" -DCMAKE_BUILD_TYPE=Release \
            -DCMAKE_OSX_ARCHITECTURES="$a" -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET" \
            -DCMAKE_INSTALL_PREFIX="$B/install" -DENABLE_SHARED=OFF -DENABLE_STATIC=ON \
            -DWITH_TURBOJPEG=ON -DWITH_JAVA=OFF -DWITH_TOOLS=OFF -DWITH_TESTS=OFF >/dev/null
        cmake --build "$B" --parallel >/dev/null
        cmake --install "$B" >/dev/null
        # openFrameworks.a contains FreeImage's own libjpeg (jpeg_* symbols): linked statically
        # side by side, TurboJPEG's internal jpeg_* calls would bind to it and
        # tjInitDecompress() fails. Pre-link TurboJPEG into one object that exports only its
        # tj* API, so its libjpeg stays private.
        rm -rf "$B/prelink" && mkdir -p "$B/prelink/objs"
        (cd "$B/prelink/objs" && ar -x "$B/install/lib/libturbojpeg.a")
        echo '_tj*' > "$B/prelink/exports.txt"
        ld -r -arch "$a" "$B"/prelink/objs/*.o -exported_symbols_list "$B/prelink/exports.txt" \
            -o "$B/prelink/turbojpeg.o"
        libtool -static -o "$B/prelink/libturbojpeg.a" "$B/prelink/turbojpeg.o"
        SLICES+=("$B/prelink/libturbojpeg.a")
        LAST="$B/install"
    done
    mkdir -p "$PREFIX/lib" "$PREFIX/include"
    cp "$LAST"/include/*.h "$PREFIX/include/"
    lipo -create "${SLICES[@]}" -output "$PREFIX/lib/libturbojpeg.a"
    touch "$PREFIX/.jpeg-turbo-$JPEGTURBO_VERSION-${ARCHS//;/-}"
fi

# --- libfreenect2-metal ---
# The OpenGL depth pipeline uses GLFW. Native builds take Homebrew's (and also build the
# Protonect test tool); --universal uses openFrameworks' own universal GLFW - the one the app
# links in the end - through a generated pkg-config file, without the test tools.
EXAMPLES=ON
GLFW_ARGS=()
if [ "$ARCHS" != "$(uname -m)" ]; then
    EXAMPLES=OFF
    GLFW_ARGS=(-DCMAKE_DISABLE_FIND_PACKAGE_glfw3=ON) # skip Homebrew's glfw3 CMake package
    OF_GLFW="$(cd "$ADDON/../.." && pwd)/libs/glfw"
    OF_GLFW_LIB="$(dirname "$(find "$OF_GLFW/lib/macos" -name 'libglfw3.a' | head -1)")"
    if [ ! -f "$OF_GLFW_LIB/libglfw3.a" ]; then
        echo "error: --universal needs openFrameworks' GLFW (libs/glfw/lib/macos), not found" >&2
        exit 1
    fi
    mkdir -p "$PREFIX/lib/pkgconfig"
    GLFW_ARGS+=(-DGLFW3_LIBRARY="$OF_GLFW_LIB/libglfw3.a")
    printf 'Name: glfw3\nDescription: openFrameworks GLFW\nVersion: 3.4\nLibs: -L%s -lglfw3\nCflags: -I%s\n' \
        "$OF_GLFW_LIB" "$OF_GLFW/include" > "$PREFIX/lib/pkgconfig/glfw3.pc"
else
    rm -f "$PREFIX/lib/pkgconfig/glfw3.pc"
fi

# FindLibUSB goes through pkg-config: put the static libusb built above first.
export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig${PKG_CONFIG_PATH:+:$PKG_CONFIG_PATH}"
rm -f "$WORK/CMakeCache.txt"
cmake -S "$SOURCE" -B "$WORK" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$WORK/install" \
    -DCMAKE_OSX_DEPLOYMENT_TARGET="$DEPLOYMENT_TARGET" \
    -DCMAKE_OSX_ARCHITECTURES="$ARCHS" \
    -DBUILD_SHARED_LIBS=OFF \
    -DBUILD_EXAMPLES="$EXAMPLES" \
    -DBUILD_OPENNI2_DRIVER=OFF \
    -DBUILD_TESTING=OFF \
    -DBUILD_CALIBRATION_TOOLS=OFF \
    -DENABLE_METAL=ON \
    -DENABLE_OPENGL=ON \
    -DENABLE_OPENCL=ON \
    -DENABLE_CUDA=OFF \
    -DENABLE_VAAPI=OFF \
    -DENABLE_TEGRAJPEG=OFF \
    -DENABLE_VIDEOTOOLBOX="$VIDEOTOOLBOX" \
    ${GLFW_ARGS[@]+"${GLFW_ARGS[@]}"} \
    -DTurboJPEG_INCLUDE_DIRS="$PREFIX/include" \
    -DTurboJPEG_LIBRARIES="$PREFIX/lib/libturbojpeg.a" \
    -DCMAKE_EXE_LINKER_FLAGS="-framework IOKit -framework CoreFoundation -framework Security" \
    -DCMAKE_PREFIX_PATH="$PREFIX;$BREW"
cmake --build "$WORK" --parallel
cmake --install "$WORK"

INC_OUT="$ADDON/libs/libfreenect2/include/osx/libfreenect2"
LIB_OUT="$ADDON/libs/libfreenect2/lib/osx"
rm -rf "$INC_OUT" "$LIB_OUT"
mkdir -p "$INC_OUT" "$LIB_OUT"
cp -R "$WORK/install/include/libfreenect2/." "$INC_OUT/"
# A static libfreenect2 needs its dependencies linked into the app too.
cp "$WORK/install/lib/libfreenect2.a" "$PREFIX/lib/libusb-1.0.a" "$PREFIX/lib/libturbojpeg.a" "$LIB_OUT/"

echo
echo "libfreenect2-metal $(git -C "$SOURCE" rev-parse --short HEAD) ($(git -C "$SOURCE" rev-parse --abbrev-ref HEAD))" \
     "installed into $ADDON/libs/libfreenect2 (osx, ${ARCHS}, macOS $DEPLOYMENT_TARGET+)"
for f in "$LIB_OUT"/*.a; do echo "  $(basename "$f"): $(lipo -archs "$f")"; done
grep "define LIBFREENECT2_WITH" "$INC_OUT/config.h"
