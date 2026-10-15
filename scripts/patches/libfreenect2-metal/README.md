# libfreenect2-metal patches for macOS

Two fixes on top of [hbmartin/libfreenect2-metal](https://github.com/hbmartin/libfreenect2-metal) `600cef5`
(0.4.0) that ofxKinectV2's macOS build needs. `scripts/build_libfreenect2_macos.sh` refuses a checkout
without them.

```bash
git clone https://github.com/hbmartin/libfreenect2-metal && cd libfreenect2-metal
git checkout 600cef5
git am /path/to/ofxKinectV2/scripts/patches/libfreenect2-metal/*.patch
```

They are meant to go upstream. To send them: fork hbmartin/libfreenect2-metal, push a branch with
the two commits (they are also on the local branch `ofx/videotoolbox-apple-silicon`), and open a pull
request with the text below. Once merged, drop this folder and the check in the build script.

---

## PR: Fix VideoToolbox on Apple Silicon; claim interfaces reliably on macOS

### 1. Fix the VideoToolbox JPEG decoder on Apple Silicon and enable it by default

`ENABLE_VIDEOTOOLBOX` is off on Apple Silicon because the decoder "crashes on M1 and later". The cause:
a packet the hardware decoder rejects (truncated or corrupt JPEG after a USB hiccup) produces no pixel
buffer, and `VTRgbPacketProcessor` still emitted a frame built from `CVPixelBufferGetBaseAddress(NULL)`,
i.e. `data == NULL`, which the consumer then reads. Good packets decode fine, including four sessions in
parallel under ASan.

`VTRgbPacketProcessor` now:
- checks every CoreMedia / VideoToolbox status and drops packets that don't decode, as TurboJPEG does;
- waits for the output callback before returning (it writes to the caller's stack);
- recreates the session when the system invalidates it (sleep, media engine reset) and retries the packet;
- reports `good() == false` after 30 consecutive failures and is wrapped in `RgbDecoderFallback`
  (default and explicit selection), so TurboJPEG takes over at run time;
- decodes into IOSurface-backed, Metal-compatible BGRA buffers locked read-only, handed over without a
  copy (packed copy only if the rows are padded);
- passes the sample size (`CMSampleBufferCreateReady`).

M3 Ultra, 1080p 4:2:2: ~0.9 ms of CPU per frame vs 3-12 ms with TurboJPEG; with a Kinect, 3.9 ms wall
per frame, 30 fps, no drops over long runs. `ENABLE_VIDEOTOOLBOX` defaults to ON on all Macs; docs
updated. New `tests/test_vt_rgb_decoder.cpp`: parity with TurboJPEG, frames outliving later decodes,
corrupt / empty packets, the unhealthy threshold, fallback hand-over, four decoders in parallel. Full
suite passes (121), VT tests clean under ASan + UBSan with `-Werror`.

### 2. macOS: claim the Kinect interfaces instead of trusting `kernel_driver_active`

`UsbControl::claimInterfaces()` refuses to claim when `libusb_kernel_driver_active()` returns 1. libusb's
macOS backend returns 1 for **any** child of the interface in the IO registry: another process's user
client (Logitech G HUB's agent opens every USB device), or entries that exist only while macOS
re-matches the interfaces after the Kinect re-enumerates - which it does every time it is closed. With
nothing else using the device, 2 of 3 opens failed with "a kernel driver is active". There are no kernel
drivers for these interfaces on macOS, so on Apple platforms the interface is claimed directly, retrying
for up to 2 s on `ACCESS` / `BUSY`; a failure names the likely cause. Other platforms keep the pre-check.

Tested on an M3 Ultra (macOS 27, Xcode 27) with a Kinect v2, Metal pipeline, through Protonect and
openFrameworks (ofxKinectV2); the x86_64 build was run under Rosetta.
