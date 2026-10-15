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
| JPEG decoding | TurboJPEG | TurboJPEG | TurboJPEG; VideoToolbox with `--videotoolbox` |
| Library type | shared (`freenect2.dll`, Release + `freenect2d.dll` Debug) | shared (`libfreenect2.so`) | static (`libfreenect2.a` + `libusb-1.0.a`) |
| CUDA archs | 52 61 75 86 89 + PTX | same | - |
| OpenNI2 driver, VAAPI, TegraJPEG, tests, calibration tools | OFF | OFF | OFF |

Source: <https://github.com/hbmartin/libfreenect2-metal>, built and tested at `600cef5` (0.4.0).
Clone it anywhere and pass its path to the script. The ofxKinectV2 code still compiles against
plain libfreenect2 0.2 (the 0.4-only features are left out), but the scripts target the fork.

The scripts copy their output into the addon:

```
libs/libfreenect2/include/<platform>/libfreenect2/*.h   headers (config.h differs per platform/options)
libs/libfreenect2/lib/vs/x64/Release|Debug/              freenect2.lib / freenect2d.lib
libs/libfreenect2/lib/vs/x64/                            runtime DLLs (copied next to the exe)
libs/libfreenect2/lib/osx/                               libfreenect2.a, libusb-1.0.a, libturbojpeg.a
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

## macOS (not yet run - first build happens on the Mac)

```bash
brew install cmake pkg-config libusb jpeg-turbo glfw
./build_libfreenect2_macos.sh /path/to/libfreenect2-metal              # Metal + OpenCL + CPU
./build_libfreenect2_macos.sh /path/to/libfreenect2-metal --videotoolbox
```

Needs full Xcode (Metal compiler) and a native arm64 terminal on Apple Silicon. Builds for the Mac's
own architecture. Check the `osx:` section of `addon_config.mk` links what libfreenect2 needs
(Metal, OpenCL, VideoToolbox, CoreVideo, CoreMedia, IOKit, CoreFoundation, Security; add Metal /
Foundation if the linker asks) and that the Kinect shows up under SuperSpeed in System
Information > USB.

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
