# Hand-off: building ofxKinectV2 on macOS and Linux

Notes for whoever (person or agent) brings this addon up on macOS and Linux. Windows is done and
tested; this file says what exists, what still has to happen per platform, and how to check it.

## Where things stand

- **The addon is built on [libfreenect2-metal](https://github.com/hbmartin/libfreenect2-metal)**
  (hbmartin's maintained fork of OpenKinect/libfreenect2: Metal pipeline for Apple Silicon, 0.4 API,
  recording / replay, self-contained CUDA). Windows is built and tested at commit `600cef5` (0.4.0).
  Use the same commit on the other platforms unless there's a reason to move, and note it.
- **Windows x64 is finished and hardware-tested** (Kinect v2 on libusbK, GTX 1080 Ti, CUDA / OpenCL /
  CPU, openFrameworks 0.12 git master, VS 2026 apps, library built with VS 2022's compiler). All four
  examples build in Release and Debug from a projectGenerator project with no manual edits and stream.
- **macOS and Linux have not been built yet.** `scripts/build_libfreenect2_macos.sh` and
  `scripts/build_libfreenect2_linux.sh` match the Windows options but have never run. Expect small fixes.
- The macOS files in the repo (`libs/libfreenect2/include/osx`, `lib/osx/libfreenect2.a`,
  `usb-1.0.0-latest-superspeed.a`) are the **old 2019 upstream 0.2 build**, almost certainly x86_64
  only. Replace them; the mac script deletes and rewrites `lib/osx` and `include/osx`.
- There is no Linux build in the repo yet (`include/linux64`, `lib/linux64` don't exist).

## Rules

- **Don't touch the Windows side:** `libs/libfreenect2/include/vs`, `libs/libfreenect2/lib/vs`,
  `scripts/build_libfreenect2_windows.ps1`, `scripts/vcpkg.json`, the `vs:` section of `addon_config.mk`.
- **Keep the libfreenect2 options identical across platforms** (table in `scripts/README.md`). If an
  option has to change on one platform, write down why in `scripts/README.md`.
- The addon code is platform-neutral and compiles against both libfreenect2-metal (0.4) and upstream
  0.2: the 0.4-only parts are behind `OFXKINECTV2_LIBFREENECT2_04` (set from
  `LIBFREENECT2_API_VERSION >= 4`). Keep it that way; fix Clang/GCC problems so MSVC still builds.
- Prefer fixing build problems in the scripts (flags, generated headers) over patching the fork's
  sources. If the fork itself is wrong, note it below so it can go upstream to hbmartin/libfreenect2-metal.
- Generated projects, `bin/` output, `obj/` and `dll/` are git-ignored; commit only sources, scripts,
  `addons.make`, `bin/data`, and the libfreenect2 headers + libraries for your platform.

## macOS (Apple Silicon first)

1. Full Xcode (for the Metal compiler; on Xcode 26+: `xcodebuild -downloadComponent MetalToolchain`),
   then `brew install cmake pkg-config libusb jpeg-turbo glfw`. Use a **native arm64** terminal.
2. `./scripts/build_libfreenect2_macos.sh /path/to/libfreenect2-metal`
   - Static `libfreenect2.a` with **Metal**, OpenCL, OpenGL and CPU pipelines; the Metal shader library
     is embedded in it. Copies it plus `libusb-1.0.a` / `libturbojpeg.a` into `lib/osx`, headers into
     `include/osx`.
3. Generate `example-streams` with the projectGenerator, build, run: the status line should say
   `metal` and libfreenect2 0.4.0. ofxKinectV2's DEFAULT pipeline order is Metal > CUDA > OpenCL > CPU.

Things likely to need attention:

- **Link errors.** A static libfreenect2 needs everything it uses linked into the app. The `osx:` section
  of `addon_config.mk` lists `Metal Foundation OpenCL VideoToolbox CoreVideo CoreMedia IOKit CoreFoundation
  Security`; add anything else the linker asks for (e.g. `QuartzCore`).
  Homebrew's `libusb-1.0.a` may also need `-lobjc`.
- **Architecture.** Homebrew libraries are single-arch. If the OF project builds universal, either set it
  to the native arch or build the dependencies for both (`CMAKE_OSX_ARCHITECTURES`) and `lipo`. Record it.
- **VideoToolbox (hardware JPEG decoding).** libfreenect2-metal turns it OFF on Apple Silicon because its
  decoder crashes on M1 and later; TurboJPEG is used instead. ofxKinectV2 already exposes the choice:
  `Settings::jpegDecoder = ofxKinectV2::JpegDecoder::VIDEOTOOLBOX` (falls back to TurboJPEG when the
  decoder fails, `PacketPipelineConfig::allow_fallback`). Building with `--videotoolbox` compiles it in.
- The Kinect must appear under **SuperSpeed** in System Information > USB.

### Then: acceleration from kinect4metal (optional, Mac only)

[kinect4metal](https://github.com/RESMP-DEV/kinect4metal) is a different, macOS-only libfreenect2 fork.
The user wants its **accelerations, not its structure** (no Swift / framework / IOKit rewrite). Candidates,
to port into a libfreenect2-metal checkout (or a branch of it) and measure against what's there:

- `src/vt_rgb_packet_processor.cpp`: a VideoToolbox JPEG decoder kinect4metal says works on Apple
  Silicon, decoding straight into CVPixelBuffers / Metal textures. The most valuable piece: the colour
  JPEG decode is a big share of each camera's CPU time (TurboJPEG at 1080p).
- `src/metal_depth_packet_processor.*` / `src/metal/`: compare with libfreenect2-metal's Metal depth
  processor (speed, unified-memory handling, avoiding copies).
- Accelerate / vImage use for colour conversion or registration, if any.
Measure with ofxKinectV2's `getCaptureStats()` (process ms per frame) and the fork's Protonect log.

## Linux

1. Packages (Ubuntu 22.04+):
   `sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev ocl-icd-opencl-dev opencl-headers`
   plus the CUDA toolkit if there's an NVIDIA GPU (`nvcc` on PATH or `CUDA_HOME`). No CUDA samples needed.
2. `./scripts/build_libfreenect2_linux.sh /path/to/libfreenect2-metal` (`--no-cuda` / `--no-opencl`).
   Output: `lib/linux64/libfreenect2.so*`, headers in `include/linux64` (`linuxaarch64` on ARM).
3. udev rule: `sudo cp /path/to/libfreenect2-metal/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/`, replug.
4. Generate and build `example-streams` (Makefile or other IDE).

Things likely to need attention:

- **Finding the .so at run time.** Check how this OF version's makefiles handle an addon `.so` (copied to
  `bin/libs` with an `$ORIGIN` rpath, or similar). If that's awkward, switch to static
  (`-DBUILD_SHARED_LIBS=OFF`, ship `libfreenect2.a`, and add `ADDON_PKG_CONFIG_LIBRARIES = libusb-1.0
  libturbojpeg OpenCL` plus CUDA's `-lcudart` / `-L$(CUDA_HOME)/lib64` to the `linux64:` section). Record it.
- **CUDA versions.** sm 52-89 + PTX. CUDA 13 drops Maxwell / Pascal (52 / 61): remove those if nvcc refuses.
- **Several Kinects:** `echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb` (permanent:
  kernel parameter `usbcore.usbfs_memory_mb=256`). One USB 3 controller per Kinect.
- VAAPI (Intel hardware JPEG decoding) is off to match the other platforms; on an Intel-graphics capture box
  it's worth `-DENABLE_VAAPI=ON` and `Settings::jpegDecoder = VAAPI`.

## Findings from Windows (apply everywhere)

- **libfreenect2-metal's `TimestampAlignedFrameListener` didn't work** at 600cef5 on Windows: zero frame
  sets delivered even with a 500 ms window (frames only dropped), and with IR subscribed deleting the device
  crashed. ofxKinectV2's `Settings::alignByTimestamp` (uses it) is therefore **off by default**; the default
  is the classic `SyncMultiFrameListener`. Retest it on macOS (the fork's tested platform); if it works
  there, it's a Windows bug worth reporting upstream.
- **Colour timestamps run ~56 ms (446 ticks) ahead of depth**, and in low light the colour camera drops to
  15 fps (then the pairs alternate 56 / 89 ms). Captures then run at 15 fps (pairs follow the colour rate):
  light the scene well for 30 fps.
- `getCaptureStats().droppedFrames` (USB sequence gaps) reads ~19 after re-opening a device in the same
  process, 0 on the first open: a counting artifact, not real loss. The CPU pipeline really drops (~25/s).
- Teardown order (handled in `ofProtonect::close()`): stop and close the device, free the frame listener,
  *then* delete the device - queued frames can belong to the pipeline's allocator (CUDA pinned memory).
- One libfreenect2 context (`ofProtonect::getContext()`) for all devices.
- libfreenect2 images are **mirrored**; `Settings::unmirror` (default on) flips images and clouds. Check
  in `example-pointcloud` that text in the scene reads correctly.
- Point cloud space must match ofxK4a: metres, +X right, +Y down, +Z forward.
- A Kinect left streaming by a killed app can refuse to start (`LIBUSB_ERROR_TIMEOUT`); another attempt
  or a power cycle clears it.
- On the Windows OF build the first `ofLog` crashes unless `ofLogToConsole()` was called; all examples call it.

## How to check a platform is done

Build all four examples from freshly generated projects, then with a Kinect connected:

1. `example-streams`: colour (1920x1080), depth, IR and colour-in-depth live; status line shows the
   pipeline (`metal` / `cuda` / `opencl` / `cpu`), libfreenect2 0.4.0, fps and per-frame ms.
2. `example-pointcloud`: textured cloud, the right way round, "Point cloud faces" gives a mesh,
   "Colour-res point cloud" switches to the 1920x1080 grid.
3. `example-shader`: lit mesh (black mesh = normals or winding wrong).
4. Quit each with the window close button: clean exit, and the Kinect reopens straight away.

Windows reference (i7-7700K, GTX 1080 Ti, one Kinect, CUDA, `getCaptureStats().processMs`): points ~15 ms,
depth-res mesh ~20 ms, colour-res mesh (step 2) ~28-30 ms per frame; CUDA depth decoding 3.5 ms (Protonect).

When it works, update the macOS / Linux sections of `scripts/README.md` and the top-level `README.md`
(remove "not yet run"), and commit the new `libs/libfreenect2/{include,lib}/<platform>` files.
