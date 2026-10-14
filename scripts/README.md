# Building libfreenect2 for ofxKinectV2

> Bringing the addon up on macOS or Linux? Read [HANDOFF_MAC_LINUX.md](HANDOFF_MAC_LINUX.md) first.

ofxKinectV2 links a libfreenect2 built from source with the scripts in this folder. Every
platform uses the same CMake options, so the pipelines behave the same everywhere:

| Option | Windows | Linux | macOS |
|---|---|---|---|
| CUDA pipeline | ON (CUDA 12.x) | ON if `nvcc` found | n/a (no CUDA on Apple) |
| OpenCL pipeline | ON (headers/lib from CUDA toolkit) | ON (ocl-icd) | ON (system framework) |
| OpenGL + CPU pipelines | ON | ON | ON |
| JPEG decoding | TurboJPEG | TurboJPEG | VideoToolbox (+TurboJPEG) |
| Library type | shared (`freenect2.dll`) | shared (`libfreenect2.so`) | static (`libfreenect2.a` + `libusb-1.0.a`) |
| CUDA archs | sm 52 61 75 86 89 + PTX | same | - |
| OpenNI2 driver, VAAPI, TegraJPEG | OFF | OFF | OFF |

Source: <https://github.com/OpenKinect/libfreenect2> (master, `fd64c5d`, 2020 - upstream has not
changed since). Clone it next to openFrameworks or anywhere else and pass its path to the script.

The scripts copy their output into the addon:

```
libs/libfreenect2/include/<platform>/libfreenect2/*.h   headers (config.h differs per platform/options)
libs/libfreenect2/lib/vs/x64/                            freenect2.lib + runtime DLLs
libs/libfreenect2/lib/osx/                               libfreenect2.a, libusb-1.0.a
libs/libfreenect2/lib/linux64/                           libfreenect2.so*
```

Then regenerate your project with the projectGenerator.

## Windows

Needs Visual Studio 2022 or newer (C++ workload - vcpkg comes with it), CUDA Toolkit 12.x, and the
Kinect on the libusbK or WinUSB driver (Zadig).

```powershell
.\build_libfreenect2_windows.ps1 -Source C:\path\to\libfreenect2
.\build_libfreenect2_windows.ps1 -Source C:\path\to\libfreenect2 -NoCuda   # OpenCL / CPU only
```

libusb, libjpeg-turbo and GLFW come from vcpkg (`vcpkg.json`, pinned baseline). CUDA older
than 12.4 is built with `-allow-unsupported-compiler -D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH`
because it predates MSVC 19.40.

## macOS (not yet run - first build happens on the Mac)

```bash
brew install cmake libusb jpeg-turbo glfw
./build_libfreenect2_macos.sh /path/to/libfreenect2
```

Builds for the Mac's own architecture. Check the `osx:` section of `addon_config.mk` links the
frameworks libfreenect2 needs (OpenCL, VideoToolbox, IOKit, CoreFoundation, CoreVideo) and that
the Kinect shows up under SuperSpeed in System Information > USB.

## Linux (not yet run - first build happens on the Linux machine)

```bash
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev \
                 libglfw3-dev ocl-icd-opencl-dev opencl-headers
# optional: CUDA toolkit (nvcc on PATH or CUDA_HOME)
./build_libfreenect2_linux.sh /path/to/libfreenect2            # add --no-cuda / --no-opencl
sudo cp /path/to/libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/
```

## Testing a build

Each build also produces libfreenect2's `Protonect` viewer in `<libfreenect2>/build_ofx/bin`.
`Protonect cuda`, `Protonect cl`, `Protonect cpu` check each pipeline without openFrameworks.
