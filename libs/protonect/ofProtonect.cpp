//  ofProtonect.cpp
//
//  Modified by Theodore Watson on 1165
//  from the Protonect example in https://github.com/OpenKinect/libfreenect2


/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include "ofProtonect.h"
#include "ofLog.h"
#include "ofUtils.h"

#include <algorithm>
#include <cstdint>

namespace
{
    std::mutex contextMutex;
    std::weak_ptr<libfreenect2::Freenect2> sharedContext;

    // Copies an image, flipping it horizontally on the way when asked: one pass instead of a
    // copy followed by a flip. P is one whole pixel (e.g. uint32_t for BGRX, float for depth).
    template<typename P, typename T>
    void copyImage(const unsigned char* src, size_t w, size_t h, ofPixelFormat format,
                   bool flip, ofPixels_<T>& dst)
    {
        if (dst.getWidth() != w || dst.getHeight() != h || dst.getPixelFormat() != format)
        {
            dst.allocate(w, h, format);
        }
        const P* in = reinterpret_cast<const P*>(src);
        P* out = reinterpret_cast<P*>(dst.getData());
        if (!flip)
        {
            std::copy(in, in + w * h, out);
            return;
        }
        for (size_t y = 0; y < h; ++y)
        {
            std::reverse_copy(in + y * w, in + (y + 1) * w, out + y * w);
        }
    }

}

bool ofProtonect::isCudaSupported()
{
#if defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
    return true;
#else
    return false;
#endif
}

bool ofProtonect::isOpenCLSupported()
{
#if defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
    return true;
#else
    return false;
#endif
}

bool ofProtonect::isMetalSupported()
{
#if defined(LIBFREENECT2_WITH_METAL_SUPPORT)
    return true;
#else
    return false;
#endif
}

std::string ofProtonect::toString(PacketPipelineType type)
{
    switch (type)
    {
        case PacketPipelineType::DEFAULT: return "DEFAULT";
        case PacketPipelineType::CPU: return "CPU";
        case PacketPipelineType::OPENGL: return "OPENGL";
        case PacketPipelineType::OPENCL: return "OPENCL";
        case PacketPipelineType::OPENCLKDE: return "OPENCLKDE";
        case PacketPipelineType::CUDA: return "CUDA";
        case PacketPipelineType::CUDAKDE: return "CUDAKDE";
        case PacketPipelineType::METAL: return "METAL";
    }
    return "UNKNOWN";
}

std::string ofProtonect::getLibraryVersion()
{
#ifdef OFXKINECTV2_LIBFREENECT2_04
    return libfreenect2::getVersion();
#else
    return LIBFREENECT2_VERSION;
#endif
}

std::shared_ptr<libfreenect2::Freenect2> ofProtonect::getContext()
{
    std::lock_guard<std::mutex> lock(contextMutex);
    auto context = sharedContext.lock();
    if (!context)
    {
        if (ofGetLogLevel() == OF_LOG_VERBOSE)
        {
            libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
        }
        else
        {
            libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
        }
        context = std::make_shared<libfreenect2::Freenect2>();
        sharedContext = context;
    }
    return context;
}

ofProtonect::ofProtonect()
{
}

ofProtonect::~ofProtonect()
{
    close();
}

libfreenect2::PacketPipeline* ofProtonect::createPipeline(PacketPipelineType type, const Options& options) const
{
    const int device = options.processingDevice;
#ifdef OFXKINECTV2_LIBFREENECT2_04
    libfreenect2::PacketPipelineConfig config;
    switch (options.jpegDecoder)
    {
        case JpegDecoder::TURBOJPEG: config.rgb_decoder = libfreenect2::PacketPipelineConfig::TurboJPEG; break;
        case JpegDecoder::VIDEOTOOLBOX: config.rgb_decoder = libfreenect2::PacketPipelineConfig::VideoToolbox; break;
        case JpegDecoder::VAAPI: config.rgb_decoder = libfreenect2::PacketPipelineConfig::VAAPI; break;
        default: config.rgb_decoder = libfreenect2::PacketPipelineConfig::Auto; break;
    }
    config.allow_fallback = true; // a hardware decoder that fails falls back to TurboJPEG
    switch (type)
    {
        case PacketPipelineType::CPU:
            return new libfreenect2::CpuPacketPipeline(config);
#if defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
        case PacketPipelineType::OPENGL:
            return new libfreenect2::OpenGLPacketPipeline(config);
#endif
#if defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
        case PacketPipelineType::OPENCL:
            return new libfreenect2::OpenCLPacketPipeline(config, device);
        case PacketPipelineType::OPENCLKDE:
            return new libfreenect2::OpenCLKdePacketPipeline(config, device);
#endif
#if defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
        case PacketPipelineType::CUDA:
            return new libfreenect2::CudaPacketPipeline(config, device);
        case PacketPipelineType::CUDAKDE:
            return new libfreenect2::CudaKdePacketPipeline(config, device);
#endif
#if defined(LIBFREENECT2_WITH_METAL_SUPPORT)
        case PacketPipelineType::METAL:
            return new libfreenect2::MetalPacketPipeline(config, device);
#endif
        default:
            return nullptr;
    }
#else
    switch (type)
    {
        case PacketPipelineType::CPU:
            return new libfreenect2::CpuPacketPipeline();
#if defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
        case PacketPipelineType::OPENGL:
            return new libfreenect2::OpenGLPacketPipeline();
#endif
#if defined(LIBFREENECT2_WITH_OPENCL_SUPPORT)
        case PacketPipelineType::OPENCL:
            return new libfreenect2::OpenCLPacketPipeline(device);
        case PacketPipelineType::OPENCLKDE:
            return new libfreenect2::OpenCLKdePacketPipeline(device);
#endif
#if defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
        case PacketPipelineType::CUDA:
            return new libfreenect2::CudaPacketPipeline(device);
        case PacketPipelineType::CUDAKDE:
            return new libfreenect2::CudaKdePacketPipeline(device);
#endif
        default:
            return nullptr;
    }
#endif
}

bool ofProtonect::open(const std::string& _serial, const Options& options)
{
    close();

    if (!options.enableColor && !options.enableDepth)
    {
        ofLogError("ofProtonect::open") << "nothing to stream: colour and depth are both disabled";
        return false;
    }

    context = getContext();

    // The requested pipeline first, then the others from fastest to slowest. Each attempt
    // needs a new pipeline: libfreenect2 deletes the pipeline when openDevice() fails.
    const PacketPipelineType requestedType = options.pipeline;
    std::vector<PacketPipelineType> candidates;
    if (requestedType != PacketPipelineType::DEFAULT) candidates.push_back(requestedType);
    for (auto type : { PacketPipelineType::METAL, PacketPipelineType::CUDA, PacketPipelineType::OPENCL, PacketPipelineType::CPU })
    {
        if (type != requestedType) candidates.push_back(type);
    }

    for (auto type : candidates)
    {
        libfreenect2::PacketPipeline* pipeline = createPipeline(type, options);
        if (!pipeline)
        {
            ofLogVerbose("ofProtonect::open") << toString(type) << " pipeline is not in this libfreenect2 build";
            continue;
        }
#ifdef OFXKINECTV2_LIBFREENECT2_04
        // e.g. CUDA without an NVIDIA GPU: skip it before touching the device
        if (!pipeline->good())
        {
            ofLogNotice("ofProtonect::open") << toString(type) << " pipeline can't run on this machine";
            delete pipeline;
            continue;
        }
#endif
        dev = context->openDevice(_serial, pipeline);
        if (dev)
        {
            pipelineType = type;
            break;
        }
        ofLogWarning("ofProtonect::open") << "could not open " << _serial << " with the " << toString(type) << " pipeline";
    }

    if (!dev)
    {
        ofLogError("ofProtonect::open") << "failure opening device with serial " << _serial;
        return false;
    }

    if (requestedType != PacketPipelineType::DEFAULT && pipelineType != requestedType)
    {
        ofLogWarning("ofProtonect::open") << _serial << ": " << toString(requestedType) << " failed, using " << toString(pipelineType);
    }

    int types = 0;
    if (options.enableColor) types |= libfreenect2::Frame::Color;
    if (options.enableDepth) types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

#ifdef OFXKINECTV2_LIBFREENECT2_04
    if (options.alignByTimestamp && options.enableColor && options.enableDepth)
    {
        // device timestamps tick every 0.125 ms
        const uint32_t maxTicks = static_cast<uint32_t>(std::max(1.f, options.maxFrameDeltaMs * 8.f));
        alignedListener = new libfreenect2::TimestampAlignedFrameListener(types, maxTicks);
        listener.reset(alignedListener);
    }
    else
#endif
    {
        syncListener = new libfreenect2::SyncMultiFrameListener(types);
        listener.reset(syncListener);
    }
    dev->setColorFrameListener(listener.get());
    dev->setIrAndDepthFrameListener(listener.get());
    dev->setConfiguration(options.config);

    if (!dev->startStreams(options.enableColor, options.enableDepth))
    {
        ofLogError("ofProtonect::open") << "error starting streams for: " << _serial;
        dev->close();
        listener.reset();
        syncListener = nullptr;
#ifdef OFXKINECTV2_LIBFREENECT2_04
        alignedListener = nullptr;
#endif
        delete dev;
        dev = nullptr;
        return false;
    }

    serial = dev->getSerialNumber();
    firmware = dev->getFirmwareVersion();
    irParams = dev->getIrCameraParams();
    colorParams = dev->getColorCameraParams();

    ofLogNotice("ofProtonect::open") << "opened " << serial << " (firmware " << firmware << ") with the "
                                     << toString(pipelineType) << " pipeline, libfreenect2 " << getLibraryVersion();

    registration.reset(new libfreenect2::Registration(irParams, colorParams));
    undistorted.reset(new libfreenect2::Frame(512, 424, 4));
    registered.reset(new libfreenect2::Frame(512, 424, 4));

    bColor = options.enableColor;
    bDepth = options.enableDepth;
    bOpened = true;
    return true;
}

bool ofProtonect::waitForFrames(int timeoutMs)
{
#ifdef OFXKINECTV2_LIBFREENECT2_04
    if (alignedListener) return alignedListener->waitForNewFrame(frameMap, timeoutMs);
#endif
    return syncListener && syncListener->waitForNewFrame(frameMap, timeoutMs);
}

void ofProtonect::releaseFrames()
{
#ifdef OFXKINECTV2_LIBFREENECT2_04
    if (alignedListener)
    {
        alignedListener->release(frameMap);
        return;
    }
#endif
    if (syncListener) syncListener->release(frameMap);
}

ofProtonect::Health ofProtonect::getHealth()
{
    Health h;
    std::lock_guard<std::mutex> lock(controlMutex);
    if (!dev)
    {
        h.connected = false;
        h.state = "closed";
        return h;
    }
#ifdef OFXKINECTV2_LIBFREENECT2_04
    switch (dev->getState())
    {
        case libfreenect2::DeviceCreated: h.state = "created"; break;
        case libfreenect2::DeviceOpen: h.state = "open"; break;
        case libfreenect2::DeviceStreaming: h.state = "streaming"; break;
        case libfreenect2::DeviceDisconnected: h.state = "disconnected"; h.connected = false; break;
        case libfreenect2::DeviceError: h.state = "error"; h.connected = false; break;
        case libfreenect2::DeviceClosed: h.state = "closed"; h.connected = false; break;
    }
    h.lastError = dev->getLastError();
    h.pipeline = dev->getPacketPipelineName();
    const auto stats = dev->getRuntimeStatistics();
    h.droppedFrames = stats.color.sequence_gaps + stats.depth.sequence_gaps;
    if (alignedListener)
    {
        const auto aligned = alignedListener->getStatistics();
        h.droppedFrames += aligned.dropped;
        if (aligned.delivered > 0) h.colorDepthOffsetMs = aligned.last_delta_ticks / 8.f;
    }
#else
    h.pipeline = toString(pipelineType);
#endif
    return h;
}

bool ofProtonect::grab(Frames& frames, int timeoutMs, bool keepIr, bool unmirror, bool wantBigDepth)
{
    if (!bOpened) return false;

    if (!waitForFrames(timeoutMs))
    {
        return false;
    }
    const uint64_t start = ofGetElapsedTimeMicros();

    libfreenect2::Frame* rgb = bColor ? frameMap[libfreenect2::Frame::Color] : nullptr;
    libfreenect2::Frame* ir = bDepth ? frameMap[libfreenect2::Frame::Ir] : nullptr;
    libfreenect2::Frame* depth = bDepth ? frameMap[libfreenect2::Frame::Depth] : nullptr;

    frames.hasColor = rgb != nullptr;
    frames.hasDepth = depth != nullptr;
    frames.hasRegistered = rgb && depth;
    frames.hasBigDepth = rgb && depth && wantBigDepth;

    const ofPixelFormat rgbFormat = (rgb && rgb->format == libfreenect2::Frame::RGBX) ? OF_PIXELS_RGBA : OF_PIXELS_BGRA;

    if (rgb)
    {
        copyImage<uint32_t>(rgb->data, rgb->width, rgb->height, rgbFormat, unmirror, frames.color);
    }

    if (depth)
    {
        if (frames.hasRegistered)
        {
            // bigdepth is 1920 x 1082: one padding row above and below the colour image.
            if (frames.hasBigDepth && !bigdepth) bigdepth.reset(new libfreenect2::Frame(1920, 1082, 4));
            rawColorMap.resize(static_cast<size_t>(depth->width) * depth->height);
            registration->apply(rgb, depth, undistorted.get(), registered.get(), true,
                                frames.hasBigDepth ? bigdepth.get() : nullptr, rawColorMap.data());
            copyImage<uint32_t>(registered->data, registered->width, registered->height, rgbFormat, unmirror, frames.registered);
            if (frames.hasBigDepth)
            {
                copyImage<float>(bigdepth->data + 1920 * sizeof(float), 1920, 1080, OF_PIXELS_GRAY, unmirror, frames.bigDepth);
            }
        }
        else
        {
            registration->undistortDepth(depth, undistorted.get());
        }

        copyImage<float>(depth->data, depth->width, depth->height, OF_PIXELS_GRAY, unmirror, frames.depth);
        copyImage<float>(undistorted->data, undistorted->width, undistorted->height, OF_PIXELS_GRAY, unmirror, frames.undistorted);

        if (keepIr && ir)
        {
            copyImage<float>(ir->data, ir->width, ir->height, OF_PIXELS_GRAY, unmirror, frames.ir);
        }
    }

    const int colorW = rgb ? static_cast<int>(rgb->width) : 0;
    releaseFrames();

    if (frames.hasRegistered)
    {
        // Colour map indexed like the (possibly flipped) depth images, pointing into the
        // (possibly flipped) colour image. The registration's occlusion filter zeroes
        // registered pixels the colour camera can't see; those get no colour either, so
        // foreground colour isn't smeared over the background behind it.
        const int w = static_cast<int>(frames.undistorted.getWidth());
        const int h = static_cast<int>(frames.undistorted.getHeight());
        const uint32_t* reg = reinterpret_cast<const uint32_t*>(registered->data);
        frames.colorMap.resize(rawColorMap.size());
        for (int y = 0; y < h; ++y)
        {
            for (int x = 0; x < w; ++x)
            {
                const int src = y * w + x;
                int c = rawColorMap[src];
                if (c >= 0 && reg[src] == 0) c = -1;
                if (unmirror)
                {
                    if (c >= 0) c = (c / colorW) * colorW + (colorW - 1 - c % colorW);
                    frames.colorMap[y * w + (w - 1 - x)] = c;
                }
                else
                {
                    frames.colorMap[src] = c;
                }
            }
        }
    }
    else
    {
        frames.colorMap.clear();
    }

    lastGrabMs = (ofGetElapsedTimeMicros() - start) / 1000.f;
    return true;
}

void ofProtonect::setConfiguration(const libfreenect2::Freenect2Device::Config& config)
{
    std::lock_guard<std::mutex> lock(controlMutex);
    if (dev) dev->setConfiguration(config);
}

void ofProtonect::setColorAutoExposure(float exposureCompensation)
{
    std::lock_guard<std::mutex> lock(controlMutex);
    if (dev && bColor) dev->setColorAutoExposure(exposureCompensation);
}

void ofProtonect::setColorManualExposure(float integrationTimeMs, float analogGain)
{
    std::lock_guard<std::mutex> lock(controlMutex);
    if (dev && bColor) dev->setColorManualExposure(integrationTimeMs, analogGain);
}

void ofProtonect::setColorSetting(libfreenect2::ColorSettingCommandType cmd, uint32_t value)
{
    std::lock_guard<std::mutex> lock(controlMutex);
    if (dev && bColor) dev->setColorSetting(cmd, value);
}

void ofProtonect::setColorSetting(libfreenect2::ColorSettingCommandType cmd, float value)
{
    std::lock_guard<std::mutex> lock(controlMutex);
    if (dev && bColor) dev->setColorSetting(cmd, value);
}

void ofProtonect::close()
{
    std::lock_guard<std::mutex> lock(controlMutex);
    if (dev)
    {
        // Order matters (same as libfreenect2's Protonect): stop the streams, then free the
        // listener while the pipeline still exists - frames it still holds can come from the
        // pipeline's allocators (pinned CUDA memory) - and only then delete the device, which
        // owns and deletes the pipeline.
        dev->stop();
        dev->close();
        listener.reset();
        delete dev;
        dev = nullptr;
    }
    listener.reset();
    syncListener = nullptr;
#ifdef OFXKINECTV2_LIBFREENECT2_04
    alignedListener = nullptr;
#endif
    registration.reset();
    undistorted.reset();
    registered.reset();
    bigdepth.reset();
    context.reset();
    bOpened = false;
}
