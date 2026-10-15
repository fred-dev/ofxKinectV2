# Hand-off: building ofxKinectV2 on macOS and Linux (macOS done)

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
- **macOS is finished and hardware-tested** (M3 Ultra, macOS 27, Xcode 27, Kinect v2): all four examples
  build from projectGenerator projects and stream on Metal + VideoToolbox at 30 fps. The libraries in
  `lib/osx` are universal (arm64 + x86_64, macOS 11.0+); x86_64 was run under Rosetta, not on an Intel Mac.
  macOS needs two patches on libfreenect2-metal (`scripts/patches/libfreenect2-metal`). See "macOS (done)".
- **Linux has not been built yet.** `scripts/build_libfreenect2_linux.sh` matches the Windows options but
  has never run. Expect small fixes. There is no Linux build in the repo yet (`include/linux64`,
  `lib/linux64` don't exist).

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

## macOS (done)

How it was brought up, and what to know when rebuilding (build steps: `scripts/README.md`, macOS).

**Defaults on macOS:** `PacketPipelineType::DEFAULT` opens with Metal (then OpenCL, CPU) and
`JpegDecoder::DEFAULT` decodes colour with VideoToolbox, TurboJPEG standing by as run-time fallback.
Checked in a running example: `RgbDecoderFallback` -> `VTRgbPacketProcessor` and
`MetalDepthPacketProcessor` on the stacks, TurboJPEG never called.

**Patches on libfreenect2-metal** (`scripts/patches/libfreenect2-metal`, on 600cef5; send upstream):

1. *VideoToolbox on Apple Silicon.* The fork turned VideoToolbox off on M1+ because it "crashed". Cause: a
   packet the hardware decoder rejects (truncated / corrupt JPEG) yields no pixel buffer, and the old code
   still emitted a frame with `data == NULL`. Rewritten: every status checked, bad packets dropped (as
   TurboJPEG does), session recreated if the system invalidates it, `good()` false after 30 failures in a
   row and wrapped in `RgbDecoderFallback`, so TurboJPEG takes over at run time. IOSurface-backed BGRA
   output, handed over without a copy. Unit tests added (parity with TurboJPEG, corrupt packets, fallback,
   4 decoders in parallel; clean under ASan + UBSan). `ENABLE_VIDEOTOOLBOX` now defaults ON.
2. *Claiming the interfaces.* The fork refuses to claim when `libusb_kernel_driver_active()` returns 1.
   On macOS that is true for **any** IO-registry child of the interface - another process's user client,
   or the entries macOS creates while it re-matches the interfaces after the Kinect re-enumerates (which
   it does every time it's closed). 2 of 3 opens failed. On Apple platforms it now claims directly and
   retries for up to 2 s.

**Build problems solved in the script** (not the fork): Homebrew's static libusb / libjpeg-turbo target
the build machine's macOS (27) - built from source for 11.0 instead, libusb without `pipe2()` (macOS 27+);
FreeImage's libjpeg inside `openFrameworks.a` captured TurboJPEG's `jpeg_*` calls (`tjInitDecompress()`
failed in every OF app) - TurboJPEG is pre-linked exporting only `tj*`; universal builds use OF's GLFW.

**kinect4metal** (the acceleration candidates listed here before):

- *VideoToolbox decoder:* the idea was right, the code wasn't portable - it calls `-contents` on an
  `MTLTexture` (an `MTLBuffer` method: it would throw) and "falls back" to a ProRes decoder for JPEG
  data. The decoder above was written fresh, keeping its IOSurface / Metal-compatible output.
  Measured (M3 Ultra, 1080p 4:2:2): ~0.9 ms CPU per frame vs 3-12 ms TurboJPEG; on the Kinect 3.9 ms wall.
- *Metal depth processor:* not a Kinect decoder - it reads raw depth as 16-bit pixels, while the sensor
  sends 11-bit packed phase images. Nothing to take. libfreenect2-metal's own Metal pipeline decodes a
  frame in 0.5 ms (0.13 ms CPU); OpenCL 0.68 ms; CPU 28.8 ms.
- No Accelerate / vImage use to port.

**Findings on macOS:**

- `TimestampAlignedFrameListener` (`Settings::alignByTimestamp`) **works** here (30 fps, clean close),
  so its Windows failure is a Windows bug worth reporting. Colour runs 55.75 ms ahead of depth, as on
  Windows; with a window under 56 ms it pairs with the neighbouring colour frame (22.5 ms off).
- Logitech G HUB's `lghub_agent` opens every USB device and blocks the Kinect: quit it.
- The Kinect re-enumerates on every close (libusb reports `NO_DEVICE` while releasing - harmless); a
  reopen in the same process takes ~2 s and then streams normally.
- Xcode 27 refuses OF's deployment target of 11.5: projects need `MACOSX_DEPLOYMENT_TARGET = 12.0`.
- Timing (one Kinect, Metal, `processMs`): points 4.6 ms, depth-res mesh 5.7 ms, colour-res points 8 ms,
  colour-res mesh (step 1) 16.5 ms - Windows reference 15 / 20 / - / 28-30 (step 2). x86_64 under
  Rosetta: 6.6 ms for points.
- Still to do: run on an Intel Mac (Metal on Intel / AMD GPUs, Intel USB controllers).

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
  is the classic `SyncMultiFrameListener`. It works on macOS, so it's a Windows bug worth reporting
  upstream.
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
