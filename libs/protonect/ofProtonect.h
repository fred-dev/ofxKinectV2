//  ofProtonect.h
//
//  Created by Theodore Watson on 11/16/15
//
//  Thin wrapper around one libfreenect2 device: opening it with a packet pipeline,
//  grabbing synchronised colour / IR / depth frames and registering them. Point cloud
//  building and threading live in ofxKinectV2.

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

class ofProtonect
{
public:
    enum class PacketPipelineType
    {
        DEFAULT,    ///< Best available: CUDA, then OpenCL, then CPU.
        CPU,
        OPENGL,     ///< Needs its own GL context (GLFW window) - avoid inside an OF app.
        OPENCL,
        OPENCLKDE,
        CUDA,       ///< Falls back to OpenCL / CPU when libfreenect2 has no CUDA support.
        CUDAKDE
    };

    static bool isCudaSupported();
    static bool isOpenCLSupported();
    static std::string toString(PacketPipelineType type);

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
    /// requested pipeline can't open the device, the other pipelines are tried in order.
    bool open(const std::string& serial,
              PacketPipelineType pipelineType,
              int processingDevice,
              bool enableColor,
              bool enableDepth,
              const libfreenect2::Freenect2Device::Config& config);

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
    libfreenect2::PacketPipeline* createPipeline(PacketPipelineType type, int processingDevice) const;

    std::shared_ptr<libfreenect2::Freenect2> context;
    libfreenect2::Freenect2Device* dev = nullptr;
    std::unique_ptr<libfreenect2::SyncMultiFrameListener> listener;
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
