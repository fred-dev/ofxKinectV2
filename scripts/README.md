# Building libfreenect2 for ofxKinectV2

> Bringing the addon up on macOS or Linux? Read [HANDOFF_MAC_LINUX.md](HANDOFF_MAC_LINUX.md) first.

ofxKinectV2 links **[libfreenect2-metal](https://github.com/hbmartin/libfreenect2-metal)**, an
actively maintained fork of OpenKinect/libfreenect2 with a Metal depth pipeline for Apple Silicon,
the 0.4 API (pipeline health, device state, recording / replay, calibration profiles) and
self-contained CUDA. It's built from source with the scripts in this folder; every platform uses
the same CMake options, so the pipelines behave the same everywhere:

| Option | Windows | Linux | macOS |
|---|---|---|---|
| Metal pipeline | n/a | n/a | ON |
| CUDA pipeline | ON (CUDA 12.x) | ON if `nvcc` found | n/a (no CUDA on Apple) |
| OpenCL pipeline | ON (headers/lib from the CUDA toolkit) | ON (ocl-icd) | ON (system framework) |
| OpenGL + CPU pipelines | ON | ON | ON |
| JPEG decoding | TurboJPEG | TurboJPEG | VideoToolbox (hardware), TurboJPEG as run-time fallback |
| Library type | shared (`freenect2.dll`, Release + `freenect2d.dll` Debug) | shared (`libfreenect2.so`) | static, universal arm64 + x86_64, macOS 11.0+ (`libfreenect2.a` + `libusb-1.0.a` + `libturbojpeg.a`) |
| CUDA archs | 52 61 75 86 89 + PTX | same | - |
| OpenNI2 driver, VAAPI, TegraJPEG, tests, calibration tools | OFF | OFF | OFF |

Source: <https://github.com/hbmartin/libfreenect2-metal>, built and tested at `600cef5` (0.4.0).
macOS adds two fixes on top, in [`patches/libfreenect2-metal`](patches/libfreenect2-metal) (apply with
`git am`; the mac script refuses an unpatched checkout): a working VideoToolbox decoder for Apple
Silicon, and interface claiming that doesn't trip over macOS's IO registry. Both are worth sending
upstream; the Windows and Linux builds don't need them.
Clone it anywhere and pass its path to the script. The ofxKinectV2 code still compiles against
plain libfreenect2 0.2 (the 0.4-only features are left out), but the scripts target the fork.

The scripts copy their output into the addon:

```
libs/libfreenect2/include/<platform>/libfreenect2/*.h   headers (config.h differs per platform/options)
libs/libfreenect2/lib/vs/x64/Release|Debug/              freenect2.lib / freenect2d.lib
libs/libfreenect2/lib/vs/x64/                            runtime DLLs (copied next to the exe)
libs/libfreenect2/lib/osx/                               libfreenect2.a, libusb-1.0.a, libturbojpeg.a (universal)
libs/libfreenect2/lib/linux64/                           libfreenect2.so*
```

Then regenerate your project with the projectGenerator.

## Windows

Needs Visual Studio 2022 or newer (C++ workload - vcpkg, CMake and Ninja come with it), CUDA
Toolkit 12.x, and the Kinect on the libusbK or WinUSB driver (Zadig).

```powershell
.\build_libfreenect2_windows.ps1 -Source C:\path\to\libfreenect2-metal
.\build_libfreenect2_windows.ps1 -Source C:\path\to\libfreenect2-metal -NoCuda   # OpenCL / CPU only
```

What the script takes care of (libfreenect2-metal's CI doesn't build Windows):

* **Ninja**, not a Visual Studio generator: CMake's CUDA support under a VS generator needs CUDA's
  MSBuild integration for that exact VS version.
* **Compiler:** with CUDA older than 12.8 it builds with Visual Studio 2022's compiler, because nvcc
  can't parse VS 2026's standard library headers. The DLL works in VS 2026 (v145) apps: MSVC is
  binary compatible from 2015 to 2026. `-allow-unsupported-compiler` and
  `_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH` cover CUDA < 12.4 with MSVC 19.4x.
* **`NOMINMAX`**: windows.h's min / max macros break the fork's `std::max` / `numeric_limits` calls.
* **`uint` / `ushort`**: the fork's CUDA kernels use these POSIX typedefs; a generated header
  (`build_ofx/ofx_cuda_windows_compat.h`) is force-included into the CUDA compile.
* **Runtime DLLs:** libusb, turbojpeg (+ its spng / zlib), glfw, and `cudart64_12.dll` - the fork
  links the shared CUDA runtime, so without it freenect2.dll wouldn't load on machines without the
  CUDA toolkit. On a machine without an NVIDIA GPU the CUDA pipeline reports itself unusable and
  ofxKinectV2 falls back to OpenCL / CPU.
* libusb, libjpeg-turbo and GLFW come from vcpkg (`vcpkg.json`, pinned baseline).

## macOS

Built and hardware-tested on an M3 Ultra (macOS 27, Xcode 27, openFrameworks git master): all four
examples build from projectGenerator projects with no manual edits and stream on Metal +
VideoToolbox at 30 fps.

```bash
brew install cmake pkg-config glfw nasm
xcodebuild -downloadComponent MetalToolchain        # Xcode 26+
git clone https://github.com/hbmartin/libfreenect2-metal && cd libfreenect2-metal
git checkout 600cef5 && git am /path/to/ofxKinectV2/scripts/patches/libfreenect2-metal/*.patch
/path/to/ofxKinectV2/scripts/build_libfreenect2_macos.sh "$PWD" --universal   # what's in the repo
/path/to/ofxKinectV2/scripts/build_libfreenect2_macos.sh "$PWD"               # this Mac's arch only, + Protonect
```

Run it from a native arm64 terminal. What the script takes care of:

* **libusb and libjpeg-turbo from source** (pinned versions, checksummed tarballs) for macOS 11.0,
  instead of Homebrew's static libraries, which target the macOS they were built on (27 here):
  apps would link with warnings and could call functions older systems lack - libusb 1.0.30
  picks up `pipe2()` (macOS 27+), so it's configured with `ac_cv_func_pipe2=no`. 11.0 is also
  the oldest target Xcode 26+'s libc++ supports.
* **TurboJPEG is pre-linked** into one object that exports only its `tj*` API.
  `openFrameworks.a` contains FreeImage's own libjpeg; linked side by side, TurboJPEG's internal
  `jpeg_*` calls bound to it and `tjInitDecompress()` failed in every OF app.
* **Universal** (`--universal`): per-architecture libjpeg-turbo builds merged with `lipo`
  (`nasm` for the x86_64 SIMD), and openFrameworks' own universal GLFW for libfreenect2's OpenGL
  pipeline (Homebrew's is arm64 only). Without `--universal` it also builds `Protonect`.
* The Metal shader library is embedded in `libfreenect2.a`; nothing else ships.

openFrameworks side:

* **Xcode 27 refuses deployment targets below macOS 12.0** and OF's template sets 11.5: set
  `MACOSX_DEPLOYMENT_TARGET = 12.0` in the project's `Project.xcconfig` (or on the `xcodebuild`
  command line). Not specific to this addon.
* **Apps that grab every USB device** - Logitech G HUB's `lghub_agent` - hold the Kinect's
  interfaces open and libfreenect2 can't claim them ("another process has the interface open").
  Quit them while using the Kinect.

Numbers (M3 Ultra, one Kinect, Metal, `getCaptureStats().processMs`): points 4.6 ms, depth-res mesh
5.7 ms, colour-res points 8 ms, colour-res mesh (step 1) 16.5 ms per frame. Metal depth decoding
0.5 ms (0.13 ms CPU); VideoToolbox colour decoding ~0.9 ms of CPU per frame vs 3-12 ms with
TurboJPEG.

## Linux (not yet run - first build happens on the Linux machine)

```bash
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev \
                 libglfw3-dev ocl-icd-opencl-dev opencl-headers
# optional: CUDA toolkit (nvcc on PATH or CUDA_HOME)
./build_libfreenect2_linux.sh /path/to/libfreenect2-metal            # add --no-cuda / --no-opencl
sudo cp /path/to/libfreenect2-metal/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
```

## Testing a build

Each build also produces libfreenect2's `Protonect` in `<libfreenect2-metal>/build_ofx/install/bin`.
`Protonect cuda -noviewer -frames 60` (or `cl`, `metal`, `cpu`) checks a pipeline without
openFrameworks; the log reports the depth decoder's speed. On Windows put the runtime DLLs from
`lib/vs/x64` next to it first.
