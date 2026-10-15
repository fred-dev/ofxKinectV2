ofxKinectV2
===========

> **About this fork:** Fork of [ofTheo/ofxKinectV2](https://github.com/ofTheo/ofxKinectV2), used for *synk*. Rebuilt around
> [libfreenect2-metal](https://github.com/hbmartin/libfreenect2-metal) with Metal / CUDA / OpenCL depth decoding, several
> Kinects at once, and point clouds / meshes that use the same conventions as [ofxK4a](../ofxK4a) so Kinect v2 and Azure
> Kinect clouds can be combined in one scene.

openFrameworks addon for the Kinect for Windows v2 / Xbox One Kinect, through
[libfreenect2-metal](https://github.com/hbmartin/libfreenect2-metal) (a maintained fork of
[libfreenect2](https://github.com/OpenKinect/libfreenect2) with a Metal pipeline for Apple Silicon and the 0.4 API). The Kinect
SDK 2.0 is not needed, so any number of Kinects can be used on one machine, bandwidth permitting.

## Features

- Several Kinects at once, each on its own capture thread. One libfreenect2 context is shared by every device.
- Depth decoding on the GPU: Metal (macOS), CUDA or OpenCL (or CPU), picked automatically with fallback
  (`PacketPipelineType::DEFAULT`); pipelines that can't run on the machine are skipped before the device is opened.
- Colour JPEG decoder choice (`Settings::jpegDecoder`: TurboJPEG, VideoToolbox on macOS, VA-API on Linux), with fallback to
  TurboJPEG. On macOS the default is VideoToolbox: the colour stream is decoded in hardware (~1 ms of CPU per frame on Apple
  Silicon instead of 3-12 ms).
- Colour (1920 x 1080), depth, IR, colour registered to depth, all un-mirrored (libfreenect2 hands out mirror images).
- Point cloud or triangulated mesh with normals, built on the capture thread:
  - metres, camera space with +X right, +Y down, +Z forward - the same as ofxK4a;
  - textured from the full 1080p colour image (texture coordinates in pixels) or per-vertex colours;
  - depth resolution (512 x 424, depth camera space) or colour resolution ("Colour-res point cloud", 1920 x 1080 samples,
    colour camera space);
  - triangles across depth edges are dropped (max edge length and depth-relative jump), and points the colour camera can't
    see get no colour instead of the colour of whatever is in front of them;
  - optional edge-preserving depth denoise, step (decimation), camera -> world transformation matrix.
- Colour camera controls (exposure, gain, white balance) as `ofParameter`s, safe before and after `open()`.
- Capture statistics and device health per camera (`getCaptureStats()`: fps, ms of work per frame, ms of it spent on the
  point cloud, connected / last error, frames dropped on USB, libfreenect2's pipeline name). `getLibraryVersion()`.

## Installation

### Windows

1. Install the libusbK (or WinUSB) driver for the "Xbox NUI Sensor" with [Zadig](https://zadig.akeo.ie/).
2. Use a USB 3 port; every Kinect v2 needs most of a USB 3 controller's bandwidth, so put each one on its own controller
   (add-in USB 3 cards) when using several.
3. Clone into `openFrameworks/addons` and generate projects with the projectGenerator.
4. Make the project's platform toolset match the one `openframeworksLib` was built with. Visual Studio 2026 retargets
   `openframeworksLib.vcxproj` to `v145` while the `vs` template still generates `v143` apps. The Release OF lib uses
   whole-program optimisation, so the mismatch fails at link time with
   `C1900: Il mismatch between 'P1' version ... and 'P2' version ...`. Generate with the `vs2026` template, or set
   Project > Properties > General > Platform Toolset to the same version for both projects.

libfreenect2-metal 0.4 is prebuilt in `libs/libfreenect2` (x64, Release and Debug, CUDA + OpenCL + OpenGL + CPU pipelines;
rebuild with `scripts/build_libfreenect2_windows.ps1`). The project generator links it and copies its DLLs, including the CUDA
runtime, next to the exe. The CUDA pipeline needs an NVIDIA driver; without one the OpenCL or CPU pipeline is used.

### macOS

libfreenect2-metal 0.4 is prebuilt in `libs/libfreenect2` (static, universal arm64 + x86_64, macOS 11.0+; Metal + OpenCL + OpenGL +
CPU pipelines, VideoToolbox + TurboJPEG). Generate the project with the projectGenerator; nothing else to set up, except on
**Xcode 27**, which refuses OF's default deployment target: set `MACOSX_DEPLOYMENT_TARGET = 12.0` in `Project.xcconfig`.
Rebuild the library with `scripts/build_libfreenect2_macos.sh` (it needs two patches on libfreenect2-metal, see
[`scripts/README.md`](scripts/README.md#macos)).

Tested on Apple Silicon (M3 Ultra) with a Kinect; the x86_64 build was run under Rosetta, not yet on an Intel Mac.

### Linux

Build libfreenect2 with [`scripts/build_libfreenect2_linux.sh`](scripts/README.md) first - it uses the same options as the Windows
build and installs headers and libraries into `libs/libfreenect2/{include,lib}/linux64`.

## Usage

```cpp
// ofApp.h
std::vector<std::shared_ptr<ofxKinectV2>> kinects; // ofxKinectV2 is a thread: keep it by pointer
ofTexture texColour;

// setup()
ofEnableArbTex(); // texture coordinates are pixels of the colour image
ofxKinectV2::Settings settings;
settings.pointCloudHasFaces = true;
for (const auto& device : ofxKinectV2::listDevices()) // sorted by serial
{
    auto kinect = std::make_shared<ofxKinectV2>();
    if (kinect->open(device.serial, settings)) kinects.push_back(kinect);
}

// update()
kinects[0]->update();
if (kinects[0]->isFrameNew()) texColour.loadData(kinects[0]->getPixels());

// draw()
cam.begin();
ofScale(1000, -1000, -1000); // metres -> mm, camera space (+Y down, +Z forward) -> OF's
texColour.bind();
kinects[0]->drawPointCloud();
texColour.unbind();
cam.end();

// exit()
for (auto& k : kinects) k->close();
```

To put several cameras in one space, give each its camera -> world matrix with `setPointCloudTransformationMatrix()`: the
vertices (and normals) come back already transformed, ready to merge or export.

## Examples

Same set as ofxK4a:

* `example-streams` shows every stream of every connected Kinect: colour, depth, IR, colour registered to depth and the 3D point
  cloud, tiled to fit the window. Double-click a view (or 1-5) to maximise it; space cycles devices. Shows capture fps and
  per-frame cost.
* `example-pointcloud` draws the point cloud (points or triangulated mesh, depth or colour resolution) textured with the colour
  image.
* `example-pointCloudTransform` moves, rotates and scales the point cloud on any axis around a pivot, using the addon's own
  point cloud transform.
* `example-shader` runs 2D feedback effects over the colour image and draws the mesh with a scene shader (colour grade, crop
  box, vertex wave, lighting from the mesh normals).

## Performance

Measured on an i7-7700K + GTX 1080 Ti, one Kinect, CUDA pipeline (`getCaptureStats()`, ms of capture-thread work per frame):

| Point cloud | Vertices | Work per frame | Capture rate |
|---|---|---|---|
| depth res, points | ~145k | 15 ms | 30 fps |
| depth res, mesh | 217k, ~260k triangles | 22 ms | 30 fps |
| colour res (step 2), points | ~330k | 18 ms | 30 fps |
| colour res (step 2), mesh | 518k, ~630k triangles | 32 ms | 29 fps |

The colour camera drops to 15 fps in low light, and the synchronised colour + depth frames follow it: light the scene well
for 30 fps.

Each camera has its own capture thread and the mesh build is split over the cores shared between the open cameras
(`getBuildThreadCount()`), so cameras run in parallel as long as there are cores for them. Raise "Point cloud step" to cut the
cost (and the GPU upload) on machines with many cameras.

## Notes

- libfreenect2's images are mirrored; `Settings::unmirror` (default on) flips images and point clouds to the true view.
- Changing the bilateral / edge-aware depth filters or the depth clip range needs a reopen (`Settings`), as libfreenect2 rebuilds
  its GPU programs for them.
- `Settings::alignByTimestamp` (pair colour and depth by device timestamp, libfreenect2-metal's
  `TimestampAlignedFrameListener`) is experimental and off: with libfreenect2-metal 0.4 on Windows it delivered no frames
  (on macOS it works).
  Colour timestamps also run ~56 ms ahead of depth. See `scripts/HANDOFF_MAC_LINUX.md`.
- Debug builds link `freenect2d` (libfreenect2's C++ API passes `std::string` across the DLL boundary, so Debug and Release
  can't be mixed).
- macOS: the Kinect must show up under SuperSpeed in System Information > USB; if not, replug the power while leaving USB
  connected. Apps that open every USB device (Logitech G HUB) stop libfreenect2 claiming the Kinect: quit them. The Kinect
  re-enumerates on the bus each time it is closed, so a reopen takes a second or two.

Based on the excellent work by the [libfreenect2](https://github.com/OpenKinect/libfreenect2) team (@JoshBlake @floe and
@christiankerl plus others). Original addon by Theo Watson.
