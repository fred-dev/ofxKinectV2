# Hand-off: building ofxKinectV2 on macOS and Linux

Notes for whoever (person or agent) brings this addon up on macOS and Linux. Windows is done and
tested; this file says what exists, what still has to happen per platform, and how to check it.

## Where things stand

- **Windows x64 is finished and hardware-tested** (Kinect v2 on libusbK, GTX 1080 Ti, CUDA + OpenCL,
  openFrameworks 0.12 git master, VS 18). All four examples build in Release and Debug from a
  projectGenerator project with no manual edits, and stream at ~30 fps.
- **macOS and Linux have not been built yet.** `scripts/build_libfreenect2_macos.sh` and
  `scripts/build_libfreenect2_linux.sh` were written to match the Windows build, but have never run.
  Expect to fix small things in them.
- The macOS files currently in the repo (`libs/libfreenect2/include/osx`, `libs/libfreenect2/lib/osx/libfreenect2.a`,
  `usb-1.0.0-latest-superspeed.a`) are the **old 2019 build** from the original fork: libfreenect2 0.2
  without the LED API, almost certainly x86_64 only. Replace them with a fresh build; the mac script
  deletes and rewrites `lib/osx` and `include/osx`.
- There is no Linux build in the repo yet (`include/linux64`, `lib/linux64` don't exist).

## Rules

- **Don't touch the Windows side:** `libs/libfreenect2/include/vs`, `libs/libfreenect2/lib/vs`,
  `scripts/build_libfreenect2_windows.ps1`, `scripts/vcpkg.json`, the `vs:` section of `addon_config.mk`.
  Those binaries were built and tested on the Windows machine.
- **Keep the libfreenect2 options identical across platforms** (table in `scripts/README.md`): CUDA where
  available, OpenCL, OpenGL, CPU; OpenNI2 / VAAPI / TegraJPEG off; C++11 on. If an option has to change on
  one platform, write down why in `scripts/README.md`.
- The addon code is meant to be platform-neutral. If it doesn't compile with Clang/GCC, fix it in a way
  that still builds with MSVC (no `#ifdef` forks of logic unless unavoidable).
- Use your own libfreenect2 checkout (upstream https://github.com/OpenKinect/libfreenect2, master
  `fd64c5d` - upstream hasn't changed since 2020). Nothing in it needs patching; the scripts pass
  everything on the command line.
- Generated projects, `bin/` output, `obj/` and `dll/` are git-ignored; commit only sources, scripts,
  `addons.make`, `bin/data`, and the libfreenect2 headers + libraries for your platform.

## macOS

1. `brew install cmake pkg-config libusb jpeg-turbo glfw`
2. `./scripts/build_libfreenect2_macos.sh /path/to/libfreenect2`
   - Builds a **static** `libfreenect2.a` (OpenCL + OpenGL + CPU, VideoToolbox JPEG) and copies it plus
     `libusb-1.0.a` (and `libturbojpeg.a` if libfreenect2 linked TurboJPEG) into `libs/libfreenect2/lib/osx`,
     headers into `include/osx`.
3. Generate `example-pointcloud` with the projectGenerator and build it in Xcode or with make.

Things likely to need attention:

- **Architecture.** Homebrew libraries are single-arch, so the build is for the Mac's own arch. If the OF
  project builds universal (arm64 + x86_64), linking fails for the other slice: either set the project to
  the native arch only, or build libusb and libfreenect2 for both architectures (`CMAKE_OSX_ARCHITECTURES`)
  and `lipo` them. A universal build is better for distribution; note what you chose in `scripts/README.md`.
- **Link errors.** A static libfreenect2 needs everything it uses linked into the app. The `osx:` section of
  `addon_config.mk` lists `OpenCL VideoToolbox CoreVideo CoreMedia IOKit CoreFoundation Security`. Add any
  framework or library the linker asks for (e.g. turbojpeg if it was found, `-lc++` is already there via OF).
  `libusb-1.0.a` from Homebrew may also need `-lobjc`.
- **OpenCL on Apple Silicon** is deprecated but usually works. If the OpenCL pipeline fails to open the
  device, the addon falls back to CPU automatically (`ofProtonect::open` tries CUDA -> OpenCL -> CPU and
  logs which one it used). CUDA isn't available on Macs; `ofProtonect::isCudaSupported()` returns false.
- **Old headers.** If you need to test before rebuilding: the addon only uses libfreenect2 API that exists in
  0.2, so it should compile against the old `include/osx`, but rebuild anyway.
- The Kinect must appear under **SuperSpeed** in System Information > USB. If it doesn't, replug the
  Kinect's power with USB still connected.

## Linux

1. Packages (Debian/Ubuntu):
   `sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev libturbojpeg0-dev libglfw3-dev ocl-icd-opencl-dev opencl-headers`
   plus the CUDA toolkit if the machine has an NVIDIA GPU (`nvcc` on PATH or `CUDA_HOME`).
2. `./scripts/build_libfreenect2_linux.sh /path/to/libfreenect2` (`--no-cuda` / `--no-opencl` to drop one).
   Output: `libs/libfreenect2/lib/linux64/libfreenect2.so*`, headers in `include/linux64`
   (`linuxaarch64` on ARM).
3. udev rule for non-root access:
   `sudo cp /path/to/libfreenect2/platform/linux/udev/90-kinect2.rules /etc/udev/rules.d/` then replug.
4. Generate and build `example-pointcloud` (Makefile or other IDE).

Things likely to need attention:

- **Finding the .so at run time.** The build is shared. Check how this OF version's makefiles handle an
  addon `.so`: it has to be linked *and* found at run time (copied to `bin/libs` with an `$ORIGIN` rpath, or
  similar). If that's awkward, switch the Linux build to static: set `-DBUILD_SHARED_LIBS=OFF` in the
  script, ship `libfreenect2.a`, and add the system libraries to the `linux64:` section of
  `addon_config.mk`: `ADDON_PKG_CONFIG_LIBRARIES = libusb-1.0 libturbojpeg OpenCL` (+ `ADDON_LDFLAGS` for
  `-lcudart_static -ldl -lrt` and `-L$(CUDA_HOME)/lib64` when CUDA is on). Static is probably simpler for
  OF; your call, but record it.
- **CUDA versions.** The script compiles for sm 52-89 plus PTX. CUDA 13 drops Maxwell/Pascal (52/61): remove
  those from the arch list if nvcc complains. Jetson boards need their own archs (53/62/72/87).
- **Several Kinects:** raise the usbfs buffer, or the second device fails to stream:
  `echo 256 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb` (make it permanent with the kernel
  parameter `usbcore.usbfs_memory_mb=256`). Each Kinect needs its own USB 3 controller.
- Under Wayland, `OPENGL` is not a good pipeline choice anyway (it needs its own GLFW window); leave the
  pipeline on `DEFAULT`.

## Things learned on Windows that apply everywhere

- **Teardown order matters** (already handled in `ofProtonect::close()`): stop and close the device, then free
  the frame listener, *then* delete the device. Freeing the listener after the device crashes with the CUDA
  pipeline, because queued frames belong to the pipeline's allocator.
- **One libfreenect2 context** (`ofProtonect::getContext()`) is shared by all devices; don't create one per
  device - a device opened by one context can't be enumerated by another.
- libfreenect2 hands out **mirror images**. `Settings::unmirror` (default on) flips images and point clouds.
  Check it on each platform: in `example-pointcloud`, text in the scene must read correctly.
- **Point cloud space** must match ofxK4a on every platform: metres, +X right, +Y down, +Z forward.
- If a test app is killed while streaming, the Kinect can refuse to start the next time
  (`LIBUSB_ERROR_TIMEOUT` in `startStreams`). Another attempt or a power cycle clears it; it isn't a code
  bug.
- On the Windows machine's openFrameworks build, the first `ofLog` crashes unless `ofLogToConsole()` was
  called first (null default channel in that OF build). All examples call it; keep that.

## How to check a platform is done

Build all four examples (`example-streams`, `example-pointcloud`, `example-pointCloudTransform`,
`example-shader`) from freshly generated projects, then with a Kinect connected:

1. `example-streams`: colour (1920x1080), depth, IR and colour-in-depth views all live; the status line shows
   the pipeline (`CUDA` / `OPENCL` / `CPU`) and ~30 fps capture.
2. `example-pointcloud`: textured cloud, the right way round (readable text), "Point cloud faces" gives a
   mesh, "Colour-res point cloud" switches to the 1920x1080 grid.
3. `example-shader`: lit mesh (lighting needs the normals; if the mesh is black, normals or winding are
   wrong).
4. Quit each with the window close button: the app exits cleanly and the Kinect reopens straight away the
   next time.

Windows reference numbers (i7-7700K, GTX 1080 Ti, one Kinect; `getCaptureStats()`): depth-res mesh ~22 ms
of capture-thread work per frame, colour-res mesh (step 2) ~32 ms, all modes ~30 fps.

When it works, update the macOS / Linux sections of `scripts/README.md` and the top-level `README.md`
(remove "not yet run"), and commit the new `libs/libfreenect2/{include,lib}/<platform>` files.
