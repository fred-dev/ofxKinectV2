//
//  ofxKinectV2.cpp
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#include "ofxKinectV2.h"
#include "ofGLProgrammableRenderer.h"

#include <cmath>
#include <thread>

std::atomic<int> ofxKinectV2::openCameraCount{ 0 };

namespace
{
    void swapFrames(ofProtonect::Frames& a, ofProtonect::Frames& b)
    {
        a.color.swap(b.color);
        a.registered.swap(b.registered);
        a.depth.swap(b.depth);
        a.undistorted.swap(b.undistorted);
        a.ir.swap(b.ir);
        a.bigDepth.swap(b.bigDepth);
        a.colorMap.swap(b.colorMap);
        std::swap(a.hasColor, b.hasColor);
        std::swap(a.hasDepth, b.hasDepth);
        std::swap(a.hasRegistered, b.hasRegistered);
        std::swap(a.hasBigDepth, b.hasBigDepth);
    }

    void swapCloud(ofxKinectV2::Cloud& a, ofxKinectV2::Cloud& b)
    {
        a.positions.swap(b.positions);
        a.normals.swap(b.normals);
        a.texCoords.swap(b.texCoords);
        a.colors.swap(b.colors);
        a.indices.swap(b.indices);
    }

    // Runs f(band, begin, end) over [0, count) split into `threads` contiguous bands, the
    // last band on the calling thread.
    template<typename F>
    void parallelRows(int count, int threads, F&& f)
    {
        threads = std::max(1, std::min(threads, count / 16));
        if (threads <= 1)
        {
            f(0, 0, count);
            return;
        }
        std::vector<std::thread> workers;
        workers.reserve(threads - 1);
        for (int t = 0; t < threads - 1; ++t)
        {
            workers.emplace_back([&f, t, threads, count] { f(t, count * t / threads, count * (t + 1) / threads); });
        }
        f(threads - 1, count * (threads - 1) / threads, count);
        for (auto& worker : workers) worker.join();
    }

    // Edge-preserving 3x3 average of depth (mm) around one pixel, with neighbours `spacing`
    // pixels apart. Only neighbours on the same surface (within a depth-relative threshold)
    // are averaged, so silhouettes stay sharp, and holes are never filled. Evaluated only at
    // the samples the cloud uses, so a stepped or colour-resolution cloud doesn't pay for
    // filtering the whole image.
    inline float denoisedDepthAt(const float* depth, int w, int h, int x, int y, int spacing)
    {
        const float center = depth[y * w + x];
        if (!(center > 0.f && center < 1e6f)) return center;

        const float threshold = std::max(15.f, center * 0.015f);
        float sum = 0.f;
        int count = 0;
        for (int dy = -spacing; dy <= spacing; dy += spacing)
        {
            const int yy = y + dy;
            if (yy < 0 || yy >= h) continue;
            const float* row = depth + yy * w;
            for (int dx = -spacing; dx <= spacing; dx += spacing)
            {
                const int xx = x + dx;
                if (xx < 0 || xx >= w) continue;
                const float n = row[xx];
                if (n > 0.f && std::abs(n - center) < threshold)
                {
                    sum += n;
                    ++count;
                }
            }
        }
        return sum / count;
    }

    // Camera space ray for each column / row of an image from a camera with focal length fx, fy
    // and principal point cx, cy (libfreenect2 puts pixel centres at +0.5 for the depth camera
    // in Registration::getPointXYZ(), at 0 for the colour camera). Unmirrored, column u of the
    // flipped image is column (w - 1 - u) of libfreenect2's image and x points the other way.
    void makeRays(float fx, float fy, float cx, float cy, float pixelCentre, int w, int h, bool unmirror,
                  std::vector<float>& xRay, std::vector<float>& yRay)
    {
        xRay.resize(w);
        yRay.resize(h);
        for (int u = 0; u < w; ++u)
        {
            const int c = unmirror ? (w - 1 - u) : u;
            const float x = (c + pixelCentre - cx) / fx;
            xRay[u] = unmirror ? -x : x;
        }
        for (int v = 0; v < h; ++v)
        {
            yRay[v] = (v + pixelCentre - cy) / fy;
        }
    }
}

ofxKinectV2::ofxKinectV2()
{
    params.setName("kinectV2");

    // Metres, like ofxK4a. Points outside the range are dropped from the point cloud and
    // shown black in the depth image.
    params.add(minDistance.set("Min distance (m)", 0.5f, 0.f, 8.f));
    params.add(maxDistance.set("Max distance (m)", 4.5f, 0.f, 8.f));

    params.add(pointCloudHasFaces.set("Point cloud faces", false));
    // 1920 x 1080 samples in colour camera space instead of 512 x 424 in depth camera space.
    params.add(bigPointCloud.set("Colour-res point cloud", false));
    // Triangles are only kept when every edge is shorter than this (scaled by the step)...
    params.add(facesMaxLength.set("Max face edge (m)", 0.05f, 0.001f, 0.5f));
    // ...and the depth difference between corners stays under this fraction of the distance
    // (per step). Depth noise and pixel footprint both grow with distance, so a relative
    // limit separates foreground from background evenly at any range.
    params.add(facesMaxDepthJump.set("Max depth jump", 0.03f, 0.001f, 0.2f));
    params.add(steps.set("Point cloud step", 1, 1, 10));
    params.add(denoiseDepth.set("Denoise depth", true));
    params.add(pointCloudAlpha.set("Point cloud alpha", 255, 0, 255));

	params.add(autoExposure.set("auto exposure", true));
	autoExposure.addListener(this, &ofxKinectV2::setAutoExposureCallback);

	params.add(expIntegrationTime.set("Shutter speed", 50.0, 0.0, 66.0));
	expIntegrationTime.addListener(this, &ofxKinectV2::setIntegrationTimeCallback);

	params.add(analogueGain.set("analogueGain", 3.0, 1.0, 4.0));
	analogueGain.addListener(this, &ofxKinectV2::setAnalogueGainCallback);

	params.add(autoWhiteBalance.set("auto white balance", true));
	autoWhiteBalance.addListener(this, &ofxKinectV2::setAutoWhiteBalanceCallback);

    params.add(redGain.set("redGain", 2.0, 0.01, 4.0));
    redGain.addListener(this, &ofxKinectV2::setRedGainCallback);

    params.add(greenGain.set("greenGain", 2.0, 0.01, 4.0));
    greenGain.addListener(this, &ofxKinectV2::setGreenGainCallback);

    params.add(blueGain.set("blueGain", 2.0, 0.01, 4.0));
    blueGain.addListener(this, &ofxKinectV2::setBlueGainCallback);

    pointCloud.setMode(OF_PRIMITIVE_POINTS);
    pointCloud.setUsage(GL_STREAM_DRAW);
}

ofxKinectV2::~ofxKinectV2()
{
    close();
}

std::vector<ofxKinectV2::KinectDeviceInfo> ofxKinectV2::listDevices()
{
    std::vector<KinectDeviceInfo> devices;

    auto context = ofProtonect::getContext();
    const int num = context->enumerateDevices();

    for (int i = 0; i < num; i++)
    {
        KinectDeviceInfo kdi;
        kdi.serial = context->getDeviceSerialNumber(i);
        kdi.freenectId = i;
        kdi.deviceId = 0;
        devices.push_back(kdi);
    }

    ofSort(devices, [](const KinectDeviceInfo& A, const KinectDeviceInfo& B)
           {
               return A.serial < B.serial;
           });

    for (std::size_t i = 0; i < devices.size(); i++)
    {
        devices[i].deviceId = static_cast<int>(i);
    }

    return devices;
}

std::size_t ofxKinectV2::getDeviceCount()
{
    return listDevices().size();
}

bool ofxKinectV2::open(int deviceId, const Settings& s)
{
    std::vector<KinectDeviceInfo> devices = listDevices();

    if (devices.empty())
    {
        ofLogError("ofxKinectV2::open") << "no devices connected!";
        return false;
    }

    if (deviceId < 0 || deviceId >= static_cast<int>(devices.size()))
    {
        ofLogError("ofxKinectV2::open") << "deviceId " << deviceId << " is out of range, " << devices.size() << " device(s) connected";
        return false;
    }

    return open(devices[deviceId].serial, s);
}

bool ofxKinectV2::open(const std::string& serial)
{
    return open(serial, Settings());
}

bool ofxKinectV2::open(int deviceId)
{
    return open(deviceId, Settings());
}

bool ofxKinectV2::open(const std::string& serial, ofProtonect::PacketPipelineType packetPipelineType, int processingDevice, bool initRGB, bool initIr, bool initDepth, bool registerImages, bool usePointCloud, bool pointCloudHasFaces, bool pointCloudTexCoords)
{
    Settings s;
    s.pipeline = packetPipelineType;
    s.processingDevice = processingDevice;
    s.enableRGB = initRGB;
    s.enableIR = initIr;
    s.enableDepth = initDepth;
    s.registerImages = registerImages;
    s.usePointCloud = usePointCloud;
    s.pointCloudHasFaces = pointCloudHasFaces;
    s.pointCloudTexCoords = pointCloudTexCoords;
    return open(serial, s);
}

bool ofxKinectV2::open(int deviceId, ofProtonect::PacketPipelineType packetPipelineType, int processingDevice, bool initRGB, bool initIr, bool initDepth, bool registerImages, bool usePointCloud, bool pointCloudHasFaces, bool pointCloudTexCoords)
{
    std::vector<KinectDeviceInfo> devices = listDevices();
    if (deviceId < 0 || deviceId >= static_cast<int>(devices.size()))
    {
        ofLogError("ofxKinectV2::open") << "deviceId " << deviceId << " is out of range, " << devices.size() << " device(s) connected";
        return false;
    }
    return open(devices[deviceId].serial, packetPipelineType, processingDevice, initRGB, initIr, initDepth, registerImages, usePointCloud, pointCloudHasFaces, pointCloudTexCoords);
}

bool ofxKinectV2::open(const std::string& serial, const Settings& s)
{
    close();

    settings = s;
    // Registration needs both cameras; IR and the point cloud come from the depth stream.
    if (settings.usePointCloud || settings.enableIR) settings.enableDepth = true;
    if (settings.registerImages && !(settings.enableRGB && settings.enableDepth)) settings.registerImages = false;

    libfreenect2::Freenect2Device::Config config;
    config.MinDepth = settings.minDepth;
    config.MaxDepth = settings.maxDepth;
    config.EnableBilateralFilter = settings.bilateralFilter;
    config.EnableEdgeAwareFilter = settings.edgeAwareFilter;

    if (!protonect.open(serial, settings.pipeline, settings.processingDevice, settings.enableRGB, settings.enableDepth, config))
    {
        return false;
    }

    params.setName("kinectV2 " + serial);
    pointCloudHasFaces = settings.pointCloudHasFaces;
    bigPointCloud = settings.bigPointCloud;

    bNewFrame = false;
    bNewBuffer = false;
    lastFrameNo = std::numeric_limits<uint64_t>::max();
    bOpened = true;

    openCameraCount++;
    applyColourSettings();
    startThread();
    return true;
}

ofxKinectV2::CloudOptions ofxKinectV2::getCloudOptions() const
{
    CloudOptions o;
    o.minDistance = minDistance;
    o.maxDistance = maxDistance;
    o.faces = pointCloudHasFaces;
    o.facesMaxLength = facesMaxLength;
    o.facesMaxDepthJump = facesMaxDepthJump;
    o.steps = std::max(1, steps.get());
    o.bigPointCloud = bigPointCloud && settings.enableRGB;
    o.texCoords = settings.pointCloudTexCoords;
    o.denoise = denoiseDepth;
    o.unmirror = settings.unmirror;
    o.alpha = pointCloudAlpha / 255.f;
    o.transform = bTransformPointCloud;
    if (o.transform)
    {
        std::lock_guard<std::mutex> lock(transformMutex);
        o.transformation = pointCloudTransformationMat;
    }
    return o;
}

void ofxKinectV2::threadedFunction()
{
    ofProtonect::Frames back;
    ofPixels depthBack;
    ofPixels irBack;
    Cloud cloudBack;
    bool cloudInColourSpace = false;
    float fpsAverage = 0.f;
    float grabAverage = 0.f;
    float buildAverage = 0.f;
    uint64_t lastFrameTime = 0;

    while (isThreadRunning())
    {
        const CloudOptions options = getCloudOptions();
        // A short timeout so close() never waits long on a stalled device.
        if (!protonect.grab(back, 500, settings.enableIR, settings.unmirror, settings.usePointCloud && options.bigPointCloud))
        {
            continue;
        }
        // Work is timed from the end of the wait, so waiting for the camera isn't counted.
        const uint64_t grabbed = ofGetElapsedTimeMicros();

        if (back.hasDepth)
        {
            // Depth (mm) mapped into 0-255 over the point cloud range, near = bright; holes
            // and anything out of range stay black.
            const float minMM = minDistance * 1000.f;
            const float maxMM = maxDistance * 1000.f;
            const float scale = 254.f / std::max(1.f, maxMM - minMM);
            depthBack.allocate(back.depth.getWidth(), back.depth.getHeight(), OF_PIXELS_GRAY);
            const float* d = back.depth.getData();
            unsigned char* out = depthBack.getData();
            for (std::size_t i = 0; i < depthBack.size(); i++)
            {
                out[i] = (d[i] < minMM || d[i] > maxMM) ? 0 : static_cast<unsigned char>(255.f - (d[i] - minMM) * scale);
            }

            if (settings.enableIR)
            {
                irBack.allocate(back.ir.getWidth(), back.ir.getHeight(), OF_PIXELS_GRAY);
                const float* ir = back.ir.getData();
                unsigned char* irOut = irBack.getData();
                for (std::size_t i = 0; i < irBack.size(); i++)
                {
                    irOut[i] = static_cast<unsigned char>(std::min(ir[i] * (255.f / 4500.f), 255.f));
                }
            }

            if (settings.usePointCloud)
            {
                const uint64_t buildStart = ofGetElapsedTimeMicros();
                buildPointCloud(back, protonect.getIrCameraParams(), protonect.getColorCameraParams(), options, cloudBack);
                buildAverage = ofLerp(buildAverage, (ofGetElapsedTimeMicros() - buildStart) / 1000.f, 0.1f);
                cloudInColourSpace = back.hasBigDepth;
            }
        }

        const uint64_t now = ofGetElapsedTimeMicros();
        if (lastFrameTime > 0) fpsAverage = ofLerp(fpsAverage, 1e6f / std::max<uint64_t>(1, now - lastFrameTime), 0.1f);
        lastFrameTime = now;
        grabAverage = ofLerp(grabAverage, protonect.getLastGrabMs() + (now - grabbed) / 1000.f, 0.1f);

        lock();
        stats.fps = fpsAverage;
        stats.processMs = grabAverage;
        stats.buildMs = buildAverage;
        swapFrames(back, framesFront);
        depthBack.swap(depthPixelsFront);
        irBack.swap(irPixelsFront);
        swapCloud(cloudBack, cloudFront);
        bCloudInColourSpaceFront = cloudInColourSpace;
        bNewBuffer = true;
        unlock();
    }
}

int ofxKinectV2::getBuildThreadCount()
{
    // Share the machine between the open cameras: each camera already has its own capture
    // thread, plus libfreenect2's USB / decode threads, so one camera may use every core for
    // its point cloud but four cameras get a couple each.
    const int hw = std::max(1u, std::thread::hardware_concurrency());
    const int cameras = std::max(1, openCameraCount.load());
    return ofClamp(hw / cameras, 1, 8);
}

void ofxKinectV2::buildPointCloud(const ofProtonect::Frames& frames,
                                  const libfreenect2::Freenect2Device::IrCameraParams& ir,
                                  const libfreenect2::Freenect2Device::ColorCameraParams& colorParams,
                                  const CloudOptions& o,
                                  Cloud& cloud)
{
    cloud.positions.clear();
    cloud.normals.clear();
    cloud.texCoords.clear();
    cloud.colors.clear();
    cloud.indices.clear();

    // The grid the cloud is sampled on: the undistorted depth image (depth camera), or depth
    // registered to the colour image (colour camera).
    const bool big = o.bigPointCloud && frames.hasBigDepth;
    const ofFloatPixels& depthImage = big ? frames.bigDepth : frames.undistorted;
    const int w = static_cast<int>(depthImage.getWidth());
    const int h = static_cast<int>(depthImage.getHeight());
    if (w == 0 || h == 0) return;

    const float* depth = depthImage.getData();
    // At colour resolution one depth pixel covers about three colour pixels: filter over the
    // same physical neighbourhood.
    const int denoiseSpacing = big ? std::max(1, static_cast<int>(std::lround(colorParams.fx / ir.fx))) : 1;

    std::vector<float> xRay, yRay;
    if (big) makeRays(colorParams.fx, colorParams.fy, colorParams.cx, colorParams.cy, 0.f, w, h, o.unmirror, xRay, yRay);
    else makeRays(ir.fx, ir.fy, ir.cx, ir.cy, 0.5f, w, h, o.unmirror, xRay, yRay);

    // Colour comes from the full 1920 x 1080 image: directly at colour resolution, through the
    // registration's per-pixel map at depth resolution (-1 where the colour camera can't see
    // the point - outside its view, or hidden behind something nearer to it).
    const bool hasColor = frames.hasColor && frames.color.isAllocated()
        && (big || frames.colorMap.size() == static_cast<size_t>(w) * h);
    const int colorW = hasColor ? static_cast<int>(frames.color.getWidth()) : 0;
    const bool useTexCoords = o.texCoords && hasColor;
    const bool useColors = !o.texCoords && hasColor;
    const int channels = hasColor ? static_cast<int>(frames.color.getNumChannels()) : 0;
    const bool bgr = hasColor && frames.color.getPixelFormat() == OF_PIXELS_BGRA;
    const unsigned char* color = hasColor ? frames.color.getData() : nullptr;
    const int* colorMap = (hasColor && !big) ? frames.colorMap.data() : nullptr;

    const float minZ = o.minDistance;
    const float maxZ = o.maxDistance;
    const int step = std::max(1, o.steps);
    const int threads = o.threads > 0 ? o.threads : getBuildThreadCount();

    auto pointAt = [&](int u, int v, glm::vec3& p) {
        const float z = (o.denoise ? denoisedDepthAt(depth, w, h, u, v, denoiseSpacing) : depth[v * w + u]) * 0.001f;
        if (!(z > minZ && z < maxZ)) return false; // also rejects NaN, inf and holes (0)
        p = glm::vec3(xRay[u] * z, yRay[v] * z, z);
        return true;
    };
    // Index of the sample's colour pixel, -1 for none.
    auto colorIndexAt = [&](int u, int v) {
        if (!hasColor) return -1;
        return big ? v * colorW + u : colorMap[v * w + u];
    };
    auto colorOf = [&](int c) {
        const unsigned char* p = color + static_cast<size_t>(c) * channels;
        return bgr ? ofFloatColor(p[2] / 255.f, p[1] / 255.f, p[0] / 255.f, o.alpha)
                   : ofFloatColor(p[0] / 255.f, p[1] / 255.f, p[2] / 255.f, o.alpha);
    };

    const int gw = (w - 1) / step + 1;
    const int gh = (h - 1) / step + 1;

    if (!o.faces)
    {
        // Points: only valid samples, compacted. With colour, points the colour camera can't
        // see are dropped too - they'd only take the colour of whatever is in front of them.
        // Each band of rows fills its own buffers; they're joined in order afterwards.
        struct Band { std::vector<glm::vec3> p; std::vector<glm::vec2> t; std::vector<ofFloatColor> c; };
        std::vector<Band> bands(threads);
        parallelRows(gh, threads, [&](int band, int gy0, int gy1) {
            Band& b = bands[band];
            for (int gy = gy0; gy < gy1; ++gy)
            {
                const int v = gy * step;
                for (int gx = 0; gx < gw; ++gx)
                {
                    const int u = gx * step;
                    glm::vec3 p;
                    if (!pointAt(u, v, p)) continue;
                    const int c = colorIndexAt(u, v);
                    if (hasColor && c < 0) continue;
                    b.p.push_back(p);
                    if (useTexCoords) b.t.emplace_back(c % colorW, c / colorW);
                    if (useColors) b.c.push_back(colorOf(c));
                }
            }
        });
        size_t total = 0;
        for (auto& b : bands) total += b.p.size();
        cloud.positions.reserve(total);
        if (useTexCoords) cloud.texCoords.reserve(total);
        if (useColors) cloud.colors.reserve(total);
        for (auto& b : bands)
        {
            cloud.positions.insert(cloud.positions.end(), b.p.begin(), b.p.end());
            cloud.texCoords.insert(cloud.texCoords.end(), b.t.begin(), b.t.end());
            cloud.colors.insert(cloud.colors.end(), b.c.begin(), b.c.end());
        }
    }
    else
    {
        // Mesh: one vertex per grid sample (vertex = gy * gw + gx), so the step decimates the
        // vertex buffer too. Invalid samples stay at the origin and are never indexed.
        const size_t count = static_cast<size_t>(gw) * gh;
        cloud.positions.assign(count, glm::vec3(0.f));
        cloud.normals.assign(count, glm::vec3(0.f));
        if (useTexCoords) cloud.texCoords.assign(count, glm::vec2(0.f));
        if (useColors) cloud.colors.assign(count, ofFloatColor(0.f, 0.f));
        std::vector<uint8_t> valid(count, 0);
        auto& pos = cloud.positions;

        parallelRows(gh, threads, [&](int, int gy0, int gy1) {
            for (int gy = gy0; gy < gy1; ++gy)
            {
                const int v = gy * step;
                for (int gx = 0; gx < gw; ++gx)
                {
                    const int u = gx * step;
                    const size_t i = static_cast<size_t>(gy) * gw + gx;
                    if (!pointAt(u, v, pos[i])) continue;
                    const int c = colorIndexAt(u, v);
                    if (hasColor && c < 0) continue;
                    valid[i] = 1;
                    if (useTexCoords) cloud.texCoords[i] = glm::vec2(c % colorW, c / colorW);
                    if (useColors) cloud.colors[i] = colorOf(c);
                }
            }
        });

        // A triangle is kept when all three corners are valid, every edge is shorter than
        // facesMaxLength, and the corners' depths differ by less than facesMaxDepthJump of
        // their distance - this is what stops "skin" being stretched between a person and
        // the wall behind them. Both limits are per depth pixel and grow with the step; a
        // colour-resolution grid step is about a third of a depth pixel.
        const float gridScale = big ? step * (ir.fx / colorParams.fx) : static_cast<float>(step);
        const float maxEdge = o.facesMaxLength * std::max(1.f, gridScale);
        const float maxEdge2 = maxEdge * maxEdge;
        const float maxJump = o.facesMaxDepthJump * std::max(1.f, gridScale);

        auto edgeOk = [&](size_t a, size_t b) {
            const glm::vec3 d = pos[a] - pos[b];
            return glm::dot(d, d) < maxEdge2 && std::abs(d.z) < maxJump * std::min(pos[a].z, pos[b].z);
        };
        auto triangleOk = [&](size_t a, size_t b, size_t c) {
            return valid[a] && valid[b] && valid[c] && edgeOk(a, b) && edgeOk(b, c) && edgeOk(c, a);
        };
        auto dist2 = [&](size_t a, size_t b) { const glm::vec3 d = pos[a] - pos[b]; return glm::dot(d, d); };

        // Normals from the neighbouring samples on the grid (central differences, falling back
        // to one-sided ones at holes and depth edges), turned to face the camera. Per vertex,
        // so they parallelise without the shared writes of summing face normals.
        parallelRows(gh, threads, [&](int, int gy0, int gy1) {
            for (int gy = gy0; gy < gy1; ++gy)
            {
                for (int gx = 0; gx < gw; ++gx)
                {
                    const size_t i = static_cast<size_t>(gy) * gw + gx;
                    if (!valid[i]) continue;
                    auto neighbour = [&](int nx, int ny) -> size_t {
                        if (nx < 0 || ny < 0 || nx >= gw || ny >= gh) return i;
                        const size_t n = static_cast<size_t>(ny) * gw + nx;
                        return (valid[n] && edgeOk(i, n)) ? n : i;
                    };
                    const glm::vec3 dx = pos[neighbour(gx + 1, gy)] - pos[neighbour(gx - 1, gy)];
                    const glm::vec3 dy = pos[neighbour(gx, gy + 1)] - pos[neighbour(gx, gy - 1)];
                    glm::vec3 n = glm::cross(dx, dy);
                    const float len2 = glm::dot(n, n);
                    if (len2 < 1e-20f) continue;
                    n /= std::sqrt(len2);
                    cloud.normals[i] = glm::dot(n, pos[i]) > 0.f ? -n : n;
                }
            }
        });

        // Triangles, one index list per band of rows, joined in order. Every triangle is wound
        // to face the camera, so the mesh has consistent front faces for culling and lighting
        // whichever diagonal a quad uses.
        std::vector<std::vector<ofIndexType>> bandIndices(threads);
        parallelRows(gh - 1, threads, [&](int band, int gy0, int gy1) {
            auto& out = bandIndices[band];
            out.reserve(static_cast<size_t>(gy1 - gy0) * (gw - 1) * 6);
            auto push = [&](size_t a, size_t b, size_t c) {
                const glm::vec3 n = glm::cross(pos[b] - pos[a], pos[c] - pos[a]);
                if (glm::dot(n, pos[a]) > 0.f) std::swap(b, c);
                out.push_back(static_cast<ofIndexType>(a));
                out.push_back(static_cast<ofIndexType>(b));
                out.push_back(static_cast<ofIndexType>(c));
            };
            for (int gy = gy0; gy < gy1; ++gy)
            {
                for (int gx = 0; gx < gw - 1; ++gx)
                {
                    const size_t tl = static_cast<size_t>(gy) * gw + gx;
                    const size_t tr = tl + 1;
                    const size_t bl = tl + gw;
                    const size_t br = bl + 1;
                    const int validCount = valid[tl] + valid[tr] + valid[bl] + valid[br];
                    if (validCount < 3) continue;

                    if (validCount == 4)
                    {
                        // Split along the shorter diagonal: better shaped triangles, and less
                        // chance the diagonal runs across a depth edge.
                        if (dist2(tl, br) <= dist2(tr, bl))
                        {
                            if (triangleOk(tl, bl, br)) push(tl, bl, br);
                            if (triangleOk(tl, br, tr)) push(tl, br, tr);
                        }
                        else
                        {
                            if (triangleOk(tl, bl, tr)) push(tl, bl, tr);
                            if (triangleOk(tr, bl, br)) push(tr, bl, br);
                        }
                    }
                    else
                    {
                        // One corner missing: the remaining triangle still closes the surface
                        // up to silhouettes and holes instead of leaving a staircase.
                        if (!valid[tl]) { if (triangleOk(tr, bl, br)) push(tr, bl, br); }
                        else if (!valid[tr]) { if (triangleOk(tl, bl, br)) push(tl, bl, br); }
                        else if (!valid[bl]) { if (triangleOk(tl, br, tr)) push(tl, br, tr); }
                        else { if (triangleOk(tl, bl, tr)) push(tl, bl, tr); }
                    }
                }
            }
        });
        size_t total = 0;
        for (auto& b : bandIndices) total += b.size();
        cloud.indices.reserve(total);
        for (auto& b : bandIndices) cloud.indices.insert(cloud.indices.end(), b.begin(), b.end());
    }

    if (o.transform)
    {
        // Inverse transpose keeps normals perpendicular under non-uniform scale.
        const glm::mat4 m = o.transformation;
        const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(m)));
        const bool hasNormals = !cloud.normals.empty();
        parallelRows(static_cast<int>(cloud.positions.size()), threads, [&](int, int i0, int i1) {
            for (int i = i0; i < i1; ++i)
            {
                cloud.positions[i] = glm::vec3(m * glm::vec4(cloud.positions[i], 1.f));
                if (hasNormals)
                {
                    const glm::vec3 t = normalMatrix * cloud.normals[i];
                    const float len = glm::length(t);
                    if (len > 1e-12f) cloud.normals[i] = t / len;
                }
            }
        });
    }
}

void ofxKinectV2::update()
{
    if (ofGetFrameNum() != lastFrameNo)
    {
        bNewFrame = false;
        lastFrameNo = ofGetFrameNum();
    }

    if (!bOpened) return;

    bool got = false;
    lock();
    if (bNewBuffer)
    {
        swapFrames(framesFront, frames);
        depthPixels.swap(depthPixelsFront);
        irPixels.swap(irPixelsFront);
        swapCloud(cloudFront, cloud);
        bCloudInColourSpace = bCloudInColourSpaceFront;
        bNewBuffer = false;
        got = true;
    }
    unlock();

    if (!got) return;

    if (settings.usePointCloud)
    {
        pointCloud.getVertices().swap(cloud.positions);
        pointCloud.getIndices().swap(cloud.indices);
        pointCloud.getNormals().swap(cloud.normals);
        pointCloud.getTexCoords().swap(cloud.texCoords);
        pointCloud.getColors().swap(cloud.colors);
        pointCloud.setMode(pointCloud.hasIndices() ? OF_PRIMITIVE_TRIANGLES : OF_PRIMITIVE_POINTS);
    }

    bNewFrame = true;
}

void ofxKinectV2::drawPointCloud(ofPolyRenderMode renderMode) const
{
    auto programmable = std::dynamic_pointer_cast<ofGLProgrammableRenderer>(ofGetCurrentRenderer());
    const bool restoreLinesShaders = programmable && programmable->areLinesShadersEnabled();
    if (restoreLinesShaders) programmable->disableLinesShaders();

    pointCloud.draw(renderMode);

    if (restoreLinesShaders) programmable->enableLinesShaders();
}

bool ofxKinectV2::isOpen() const
{
    return bOpened;
}

bool ofxKinectV2::isFrameNew() const
{
    return bNewFrame;
}

std::string ofxKinectV2::getSerial() const
{
    return protonect.getSerial();
}

ofxKinectV2::CaptureStats ofxKinectV2::getCaptureStats()
{
    lock();
    const CaptureStats s = stats;
    unlock();
    return s;
}

ofxKinectV2::PacketPipelineType ofxKinectV2::getPipelineType() const
{
    return protonect.getPipelineType();
}

ofPixels ofxKinectV2::getRgbPixels()
{
    return getPixels();
}

const ofPixels& ofxKinectV2::getPixels() const
{
    return frames.color;
}

const ofPixels& ofxKinectV2::getRegisteredPixels() const
{
    return frames.registered;
}

const ofFloatPixels& ofxKinectV2::getRawDepthPixels() const
{
    return frames.depth;
}

const ofPixels& ofxKinectV2::getDepthPixels() const
{
    return depthPixels;
}

const ofFloatPixels& ofxKinectV2::getRawIRPixels() const
{
    return frames.ir;
}

const ofPixels& ofxKinectV2::getIRPixels() const
{
    return irPixels;
}

const ofFloatPixels& ofxKinectV2::getUndistortedDepthPixels() const
{
    return frames.undistorted;
}

const ofVboMesh& ofxKinectV2::getPointCloud() const
{
	return pointCloud;
}

void ofxKinectV2::setPointCloudAlpha(int alpha)
{
    pointCloudAlpha = ofClamp(alpha, 0, 255);
}

void ofxKinectV2::setPointCloudTransformationMatrix(const ofMatrix4x4& mat)
{
    // ofMatrix4x4's row-vector "v * M" is glm's column-vector "M * v" on the same memory.
    setPointCloudTransformationMatrix(glm::mat4(mat));
}

void ofxKinectV2::setPointCloudTransformationMatrix(const glm::mat4& mat)
{
    {
        std::lock_guard<std::mutex> lock(transformMutex);
        pointCloudTransformationMat = mat;
    }
    bTransformPointCloud = true;
}

glm::mat4 ofxKinectV2::getPointCloudTransformationMatrix() const
{
    std::lock_guard<std::mutex> lock(transformMutex);
    return pointCloudTransformationMat;
}

void ofxKinectV2::passTransformationMat(ofMatrix4x4 mat)
{
    setPointCloudTransformationMatrix(mat);
}

float ofxKinectV2::getDistanceAt(std::size_t x, std::size_t y) const
{
    return glm::length(getWorldCoordinateAt(x, y));
}

glm::vec3 ofxKinectV2::getWorldCoordinateAt(std::size_t x, std::size_t y) const
{
    const ofFloatPixels& depth = frames.undistorted;
    if (!depth.isAllocated())
    {
        ofLogWarning("ofxKinectV2::getWorldCoordinateAt") << "no depth frame yet, returning 0, 0, 0.";
        return glm::vec3(0.f);
    }
    if (x >= depth.getWidth() || y >= depth.getHeight())
    {
        ofLogWarning("ofxKinectV2::getWorldCoordinateAt") << "invalid x, y coordinates.";
        return glm::vec3(0.f);
    }

    const float z = depth[y * depth.getWidth() + x] * 0.001f;
    if (!(z > 0.f)) return glm::vec3(0.f);

    const auto& ir = protonect.getIrCameraParams();
    const int w = static_cast<int>(depth.getWidth());
    const int c = settings.unmirror ? (w - 1 - static_cast<int>(x)) : static_cast<int>(x);
    const float rx = (c + 0.5f - ir.cx) / ir.fx;
    const float ry = (y + 0.5f - ir.cy) / ir.fy;
    return glm::vec3((settings.unmirror ? -rx : rx) * z, ry * z, z);
}

const libfreenect2::Freenect2Device::IrCameraParams& ofxKinectV2::getIrCameraParams() const
{
    return protonect.getIrCameraParams();
}

const libfreenect2::Freenect2Device::ColorCameraParams& ofxKinectV2::getColorCameraParams() const
{
    return protonect.getColorCameraParams();
}

void ofxKinectV2::setUsePointCloud(bool _usePointCloud){
    settings.usePointCloud = _usePointCloud;
}

void ofxKinectV2::setUseRegisterImages(bool _registerImages){
    settings.registerImages = _registerImages;
}

void ofxKinectV2::setIsPointCloudFilled(bool _pointCloudFilled){
    pointCloudHasFaces = _pointCloudFilled;
}

void ofxKinectV2::setUseRgb(bool _enableRGB){
    settings.enableRGB = _enableRGB;
}

void ofxKinectV2::setUseDepth(bool _enableDepth){
    settings.enableDepth = _enableDepth;
}

void ofxKinectV2::setUseIr(bool _enableIr){
    settings.enableIR = _enableIr;
}

void ofxKinectV2::setUseTexCoords(bool _useTexCoords){
    settings.pointCloudTexCoords = _useTexCoords;
}

void ofxKinectV2::setTransformPointCloud(bool _transformPointCloud)
{
	bTransformPointCloud = _transformPointCloud;
}

bool ofxKinectV2::getUsePointCloud() const {
    return settings.usePointCloud;
}

bool ofxKinectV2::getUseRegisterImages() const {
    return settings.registerImages;
}

bool ofxKinectV2::getIsPointCloudFilled() const {
    return pointCloudHasFaces;
}

bool ofxKinectV2::getUseRgb() const {
    return settings.enableRGB;
}

bool ofxKinectV2::getUseDepth() const {
    return settings.enableDepth;
}

bool ofxKinectV2::getUseIr() const {
    return settings.enableIR;
}

bool ofxKinectV2::getUseTexCoords() const {
    return settings.pointCloudTexCoords;
}

bool ofxKinectV2::getTransformPointCloud() const
{
	return bTransformPointCloud;
}

void ofxKinectV2::applyColourSettings()
{
    bool ae = autoExposure;
    setAutoExposureCallback(ae);
    bool awb = autoWhiteBalance;
    setAutoWhiteBalanceCallback(awb);
}

void ofxKinectV2::setAutoExposureCallback(bool & auto_exposure){
    if (auto_exposure)
    {
        protonect.setColorAutoExposure(0);
    }
    else
    {
        protonect.setColorManualExposure(expIntegrationTime, analogueGain);
    }
}

void ofxKinectV2::setIntegrationTimeCallback(float & integration_time_ms){
    if (autoExposure)
    {
        autoExposure = false; // its listener applies the manual exposure
    }
    else
    {
        protonect.setColorManualExposure(integration_time_ms, analogueGain);
    }
}

void ofxKinectV2::setAnalogueGainCallback(float & analog_gain){
    if (autoExposure)
    {
        autoExposure = false;
    }
    else
    {
        protonect.setColorManualExposure(expIntegrationTime, analog_gain);
    }
}

void ofxKinectV2::setAutoWhiteBalanceCallback(bool & auto_white_balance){
    protonect.setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
    if (auto_white_balance)
    {
        protonect.setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(1));
    }
    else
    {
        protonect.setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(3));
        protonect.setColorSetting(libfreenect2::COLOR_SETTING_SET_RED_CHANNEL_GAIN, redGain.get());
        protonect.setColorSetting(libfreenect2::COLOR_SETTING_SET_GREEN_CHANNEL_GAIN, greenGain.get());
        protonect.setColorSetting(libfreenect2::COLOR_SETTING_SET_BLUE_CHANNEL_GAIN, blueGain.get());
    }
}

void ofxKinectV2::setRedGainCallback(float & red_gain){
    if (autoWhiteBalance) autoWhiteBalance = false; // its listener applies all three gains
    else protonect.setColorSetting(libfreenect2::COLOR_SETTING_SET_RED_CHANNEL_GAIN, red_gain);
}

void ofxKinectV2::setGreenGainCallback(float & green_gain){
    if (autoWhiteBalance) autoWhiteBalance = false;
    else protonect.setColorSetting(libfreenect2::COLOR_SETTING_SET_GREEN_CHANNEL_GAIN, green_gain);
}

void ofxKinectV2::setBlueGainCallback(float & blue_gain){
    if (autoWhiteBalance) autoWhiteBalance = false;
    else protonect.setColorSetting(libfreenect2::COLOR_SETTING_SET_BLUE_CHANNEL_GAIN, blue_gain);
}

void ofxKinectV2::close()
{
    if (bOpened)
    {
        waitForThread(true);
        protonect.close();
        bOpened = false;
        openCameraCount--;
    }
}
