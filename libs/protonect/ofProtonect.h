//  ofProtonect.h
//
//  Created by Theodore Watson on 11/16/15
//
//  Thin wrapper around one libfreenect2 device: opening it with a packet pipeline,
//  grabbing synchronised colour / IR / depth frames and registering them. Point cloud
//  building and threading live in ofxKinectV2.
//
//  Built for libfreenect2-metal (https://github.com/hbmartin/libfreenect2-metal, API 0.4):
//  Metal pipeline, decoder choice, timestamp-aligned colour / depth and device health. With
//  an older libfreenect2 (API 0.2) those parts are left out and the rest still works.

#pragma once

#include "ofPixels.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <libfreenect2/color_settings.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#if defined(LIBFREENECT2_API_VERSION) && LIBFREENECT2_API_VERSION >= 4
#define OFXKINECTV2_LIBFREENECT2_04 1
#endif

class ofProtonect
{
public:
    enum class PacketPipelineType
    {
        DEFAULT,    ///< Best available: Metal, then CUDA, then OpenCL, then CPU.
        CPU,
        OPENGL,     ///< Needs its own GL context (GLFW window) - avoid inside an OF app.
        OPENCL,
        OPENCLKDE,
        CUDA,       ///< Falls back to the others when libfreenect2 has no CUDA support.
        CUDAKDE,
        METAL       ///< Apple GPUs (libfreenect2-metal on macOS).
    };

    /// How the colour stream's JPEG frames are decoded (libfreenect2-metal only).
    enum class JpegDecoder
    {
        DEFAULT,      ///< libfreenect2's choice for the platform.
        TURBOJPEG,    ///< CPU (SIMD), every platform.
        VIDEOTOOLBOX, ///< macOS hardware decoder.
        VAAPI         ///< Linux, Intel GPUs.
    };

    struct Options
    {
        PacketPipelineType pipeline = PacketPipelineType::DEFAULT;
        int processingDevice = -1;  ///< GPU index for CUDA / OpenCL / Metal, -1 = default
        bool enableColor = true;
        bool enableDepth = true;
        libfreenect2::Freenect2Device::Config config;
        JpegDecoder jpegDecoder = JpegDecoder::DEFAULT;
        /// Experimental, see ofxKinectV2::Settings::alignByTimestamp: off by default.
        bool alignByTimestamp = false;
        float maxFrameDeltaMs = 25.f;
    };

    /// What the device and its streams are doing right now.
    struct Health
    {
        bool connected = true;
        std::string state;          ///< "streaming", "disconnected", ... ("" with libfreenect2 0.2)
        std::string lastError;
        std::string pipeline;       ///< libfreenect2's name for the pipeline in use, e.g. "cuda"
        uint64_t droppedFrames = 0; ///< frames lost on USB or dropped while pairing colour / depth
        float colorDepthOffsetMs = -1; ///< device time between the paired colour and depth frames (-1: unknown)
    };

    static bool isCudaSupported();
    static bool isOpenCLSupported();
    static bool isMetalSupported();
    static std::string toString(PacketPipelineType type);
    /// libfreenect2's version, e.g. "0.4.0".
    static std::string getLibraryVersion();

    /// The libfreenect2 context shared by every device in the app. Sharing one context
    /// matters with several Kinects: a device opened by one context can't be queried by
    /// another, so per-device contexts lose track of devices that are already open.
    static std::shared_ptr<libfreenect2::Freenect2> getContext();

    /// One synchronised set of frames. Depth / undistorted depth are millimetres, IR is raw
    /// amplitude (0 - 65535). Colour and registered colour are BGRA.
    struct Frames
    {
        ofPixels color;             ///< 1920 x 1080
        ofPixels registered;        ///< 512 x 424, colour for each undistorted depth pixel, 0 where occluded / outside
        ofFloatPixels depth;        ///< 512 x 424, raw
        ofFloatPixels undistorted;  ///< 512 x 424, lens-undistorted
        ofFloatPixels ir;           ///< 512 x 424
        ofFloatPixels bigDepth;     ///< 1920 x 1080, depth seen from the colour camera (only when asked for)
        /// For each undistorted depth pixel, the index (y * 1920 + x) of its colour pixel, or
        /// -1 when it has none.
        std::vector<int> colorMap;
        bool hasColor = false;
        bool hasDepth = false;
        bool hasRegistered = false;
        bool hasBigDepth = false;
    };

    ofProtonect();
    ~ofProtonect();

    /// Opens the device and starts the requested streams. With DEFAULT, or when the
    /// requested pipeline isn't usable on this machine or can't open the device, the other
    /// pipelines are tried in order.
    bool open(const std::string& serial, const Options& options);

    /// Waits up to timeoutMs for the next frame set. When unmirror is set every image is
    /// flipped horizontally so it reads as seen from behind the sensor (libfreenect2 hands
    /// out mirror images).
    /// bigDepth also produces Frames::bigDepth (depth registered to the 1080p colour image).
    bool grab(Frames& frames, int timeoutMs, bool keepIr, bool unmirror, bool bigDepth);

    void close();

    bool isOpen() const { return bOpened; }
    PacketPipelineType getPipelineType() const { return pipelineType; }
    /// Time the last grab() spent after the frame arrived (copies, registration, flips).
    float getLastGrabMs() const { return lastGrabMs; }
    const std::string& getSerial() const { return serial; }
    std::string getFirmwareVersion() const { return firmware; }
    /// Device state, errors and frame drops. Cheap; safe from any thread.
    Health getHealth();

    const libfreenect2::Freenect2Device::IrCameraParams& getIrCameraParams() const { return irParams; }
    const libfreenect2::Freenect2Device::ColorCameraParams& getColorCameraParams() const { return colorParams; }

    // Colour camera and depth processing controls. Safe to call from any thread, no-ops
    // while the device is closed.
    void setConfiguration(const libfreenect2::Freenect2Device::Config& config);
    void setColorAutoExposure(float exposureCompensation);
    void setColorManualExposure(float integrationTimeMs, float analogGain);
    void setColorSetting(libfreenect2::ColorSettingCommandType cmd, uint32_t value);
    void setColorSetting(libfreenect2::ColorSettingCommandType cmd, float value);

private:
    libfreenect2::PacketPipeline* createPipeline(PacketPipelineType type, const Options& options) const;
    bool waitForFrames(int timeoutMs);
    void releaseFrames();

    std::shared_ptr<libfreenect2::Freenect2> context;
    libfreenect2::Freenect2Device* dev = nullptr;
    std::unique_ptr<libfreenect2::FrameListener> listener;
    libfreenect2::SyncMultiFrameListener* syncListener = nullptr;
#ifdef OFXKINECTV2_LIBFREENECT2_04
    libfreenect2::TimestampAlignedFrameListener* alignedListener = nullptr;
#endif
    std::unique_ptr<libfreenect2::Registration> registration;
    std::unique_ptr<libfreenect2::Frame> undistorted;
    std::unique_ptr<libfreenect2::Frame> registered;
    std::unique_ptr<libfreenect2::Frame> bigdepth;
    std::vector<int> rawColorMap;
    libfreenect2::FrameMap frameMap;

    libfreenect2::Freenect2Device::IrCameraParams irParams{};
    libfreenect2::Freenect2Device::ColorCameraParams colorParams{};

    std::mutex controlMutex;
    PacketPipelineType pipelineType = PacketPipelineType::DEFAULT;
    std::string serial;
    std::string firmware;
    bool bOpened = false;
    float lastGrabMs = 0.f;
    bool bColor = false;
    bool bDepth = false;
};
