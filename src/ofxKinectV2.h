//
//  ofxKinectV2.h
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//  One Kinect for Windows v2 / Xbox One Kinect through libfreenect2. Frames are grabbed,
//  registered and turned into a point cloud (or triangulated mesh) on a worker thread;
//  update() hands the latest frame to the app.
//
//  Point clouds use the same conventions as ofxK4a so both cameras can be mixed in one
//  scene: metres, camera space with +X right, +Y down, +Z forward (looking out of the
//  sensor), then the optional transformation matrix (camera -> world). The camera is the
//  depth camera, or the colour camera with "Colour-res point cloud" on. Texture coordinates
//  are pixels of the 1920 x 1080 colour image (getPixels()) in both cases.
//

#pragma once

#include "ofMain.h"
#include "ofProtonect.h"

#include <atomic>

class ofxKinectV2: public ofThread
{
public:
    using PacketPipelineType = ofProtonect::PacketPipelineType;

    struct KinectDeviceInfo
    {
        std::string serial;
        int deviceId;   //if you have the same devices plugged in device 0 will always be the same Kinect
        int freenectId; //don't use this one - this is the index given by freenect2 - but this can change based on order device is plugged in
    };

    struct Settings
    {
        /// DEFAULT picks CUDA, then OpenCL, then CPU. A pipeline that fails to open the
        /// device falls back to the next one.
        PacketPipelineType pipeline = PacketPipelineType::DEFAULT;
        /// GPU index for the CUDA / OpenCL pipelines, -1 for the default device.
        int processingDevice = -1;

        bool enableRGB = true;
        bool enableIR = true;
        bool enableDepth = true;
        /// Colour mapped onto the depth image (512 x 424). Needs RGB and depth.
        bool registerImages = true;

        bool usePointCloud = true;
        /// Triangulated mesh instead of points. Can be changed later with the
        /// "Point cloud faces" parameter.
        bool pointCloudHasFaces = false;
        /// Build the cloud at colour resolution (1920 x 1080 samples, colour camera space)
        /// instead of depth resolution (512 x 424, depth camera space). Same geometric detail
        /// - it comes from the depth sensor - but many more points. Can be changed later with
        /// the "Colour-res point cloud" parameter.
        bool bigPointCloud = false;
        /// true: texture coordinates into getPixels() (1920 x 1080, pixels - use ARB textures)
        /// false: per-vertex colours.
        bool pointCloudTexCoords = true;

        /// libfreenect2 hands out mirror images. true flips every image and the point cloud
        /// so they read as seen from behind the sensor (the camera space described above).
        bool unmirror = true;

        /// Depth clipping in the depth decoder, metres.
        float minDepth = 0.5f;
        float maxDepth = 4.5f;
        bool bilateralFilter = true;
        bool edgeAwareFilter = true;
    };

    ofxKinectV2();
    ~ofxKinectV2();

    /// Connected devices, sorted by serial so deviceId is stable between runs.
    static std::vector<KinectDeviceInfo> listDevices();
    static std::size_t getDeviceCount();

    std::vector<KinectDeviceInfo> getDeviceList() const { return listDevices(); }
    std::size_t getNumDevices() const { return getDeviceCount(); }

    bool open(const std::string& serial, const Settings& settings);
    bool open(int deviceId, const Settings& settings);
    bool open(const std::string& serial);
    bool open(int deviceId = 0);

    /// \brief Open the device with the given serial number (old argument list).
    bool open(const std::string& serial, ofProtonect::PacketPipelineType packetPipelineType, int processingDevice = -1, bool initRGB = true, bool initIr = true, bool initDepth = true, bool registerImages = true, bool usePointCloud = true, bool pointCloudHasFaces = true, bool pointCloudTexCoords = true);

    /// \brief Open the device with the given index in getDeviceList() (old argument list).
    bool open(int deviceId, ofProtonect::PacketPipelineType packetPipelineType, int processingDevice = -1, bool initRGB = true, bool initIr = true, bool initDepth = true, bool registerImages = true, bool usePointCloud = true, bool pointCloudHasFaces = true, bool pointCloudTexCoords = true);

    /// \brief Update the Kinect internals.
    void update();

    /// \brief Close the connection to the Kinect.
    void close();

    bool isOpen() const;

    /// \returns true if the frame has been updated.
    bool isFrameNew() const;

    std::string getSerial() const;
    const Settings& getSettings() const { return settings; }
    /// The pipeline actually in use (after any fallback).
    PacketPipelineType getPipelineType() const;

    /// Capture thread timing (running averages): frames per second delivered, time spent
    /// processing each frame after it arrives (registration, flips, depth/IR images, point
    /// cloud), and the point cloud's share of it. With several cameras each one has its own
    /// thread, so processMs only has to stay under ~33 ms per camera for full rate.
    struct CaptureStats
    {
        float fps = 0.f;
        float processMs = 0.f;
        float buildMs = 0.f;
    };
    CaptureStats getCaptureStats();

    OF_DEPRECATED_MSG("Use getPixels()", ofPixels getRgbPixels());

    /// \returns the RGB pixels (1920 x 1080, BGRA).
    const ofPixels& getPixels() const;

    /// \returns colour registered to the depth image (512 x 424, BGRA, alpha 0 where there's no colour).
    const ofPixels& getRegisteredPixels() const;

    /// \returns the raw depth pixels in millimetres.
    const ofFloatPixels& getRawDepthPixels() const;

    /// \returns the depth pixels mapped to a visible range.
    const ofPixels& getDepthPixels() const;

    /// \returns the raw IR pixels.
    const ofFloatPixels& getRawIRPixels() const;

    /// \returns the IR pixels mapped to a visible range.
    const ofPixels& getIRPixels() const;

    /// \returns the lens-undistorted depth in millimetres, aligned with getRegisteredPixels().
    const ofFloatPixels& getUndistortedDepthPixels() const;

    /// The whole current frame set: images plus the depth -> colour map (Frames::colorMap),
    /// e.g. to record frames and mesh them later with buildPointCloud().
    const ofProtonect::Frames& getFrames() const { return frames; }

    const ofVboMesh& getPointCloud() const;

    /// Draws the point cloud. With texture coordinates, bind the registered colour texture
    /// first. Prefer this to getPointCloud().draw(): openFrameworks' GL_POINTS shader
    /// samples the bound texture at gl_PointCoord and ignores per-vertex texture coordinates.
    void drawPointCloud(ofPolyRenderMode renderMode = OF_MESH_FILL) const;

    /// Camera -> world transform applied to the point cloud on the capture thread.
    void setPointCloudTransformationMatrix(const ofMatrix4x4& mat);
    void setPointCloudTransformationMatrix(const glm::mat4& mat);
    glm::mat4 getPointCloudTransformationMatrix() const;

    /// \brief Distance from the camera in metres for pixel x, y of getRegisteredPixels().
    float getDistanceAt(std::size_t x, std::size_t y) const;

    /// \brief Camera space point in metres for pixel x, y of getRegisteredPixels() (not
    /// transformed by the point cloud matrix). 0, 0, 0 where there is no depth.
    glm::vec3 getWorldCoordinateAt(std::size_t x, std::size_t y) const;

    /// Factory intrinsics read from the device.
    const libfreenect2::Freenect2Device::IrCameraParams& getIrCameraParams() const;
    const libfreenect2::Freenect2Device::ColorCameraParams& getColorCameraParams() const;

    ofParameterGroup params;
    ofParameter<float> minDistance;     ///< metres, point cloud and depth image range
    ofParameter<float> maxDistance;     ///< metres
    ofParameter<bool> pointCloudHasFaces;
    ofParameter<bool> bigPointCloud;
    ofParameter<float> facesMaxLength;  ///< metres, longest triangle edge kept
    ofParameter<float> facesMaxDepthJump; ///< fraction of the distance, see buildPointCloud()
    ofParameter<int> steps;
    ofParameter<bool> denoiseDepth;
    ofParameter<int> pointCloudAlpha;

    ofParameter<bool> autoExposure;
	ofParameter<float> expIntegrationTime;
	ofParameter<float> analogueGain;

    ofParameter<bool> autoWhiteBalance;
    ofParameter<float> redGain;
    ofParameter<float> blueGain;
    ofParameter<float> greenGain;

    void setAutoExposureCallback(bool & auto_exposure);
    void setIntegrationTimeCallback(float & integration_time_ms);
    void setAnalogueGainCallback(float & analog_gain);
    void setAutoWhiteBalanceCallback(bool & auto_white_balance);
    void setRedGainCallback(float & red_gain);
    void setGreenGainCallback(float & green_gain);
    void setBlueGainCallback(float & blue_gain);

	void setPointCloudAlpha(int alpha);

    // Stream options (take effect on the next open()), except the point cloud ones.
    void setUsePointCloud(bool _usePointCloud);
    void setUseRegisterImages(bool _registerImages);
    void setIsPointCloudFilled(bool _pointCloudFilled);
    void setUseRgb(bool _enableRGB);
    void setUseDepth(bool _enableDepth);
    void setUseIr(bool _enableIr);
    void setUseTexCoords(bool _useTexCoords);
	void setTransformPointCloud(bool _transformPointCloud);
	OF_DEPRECATED_MSG("Use setPointCloudTransformationMatrix()", void passTransformationMat(ofMatrix4x4 mat));

    bool getUsePointCloud() const;
    bool getUseRegisterImages() const;
    bool getIsPointCloudFilled() const;
    bool getUseRgb() const;
    bool getUseDepth() const;
    bool getUseIr() const;
    bool getUseTexCoords() const;
	bool getTransformPointCloud() const;
    /// True when the current point cloud was built at colour resolution (colour camera space).
    bool getPointCloudInColourSpace() const { return bCloudInColourSpace; }

    /// Point cloud buffers, built on the capture thread.
    struct Cloud
    {
        std::vector<glm::vec3> positions;
        std::vector<glm::vec3> normals;
        std::vector<glm::vec2> texCoords;
        std::vector<ofFloatColor> colors;
        std::vector<ofIndexType> indices;
    };

    struct CloudOptions
    {
        float minDistance = 0.5f;       // metres
        float maxDistance = 4.5f;       // metres
        bool faces = false;
        float facesMaxLength = 0.05f;   // metres
        float facesMaxDepthJump = 0.03f; // fraction of the distance
        int steps = 1;
        bool bigPointCloud = false;
        bool texCoords = true;
        bool denoise = false;
        bool unmirror = true;
        float alpha = 1.f;
        /// Threads used to build the cloud, 0 = getBuildThreadCount().
        int threads = 0;
        bool transform = false;
        glm::mat4 transformation = glm::mat4(1.f);
    };

    /// Builds a point cloud / mesh from one frame set: from Frames::undistorted (depth
    /// resolution) or Frames::bigDepth (options.bigPointCloud), coloured from Frames::color
    /// through Frames::colorMap. Static and thread-safe, so recorded frames can be meshed the
    /// same way as live ones.
    /// Threads each camera uses to build its point cloud: the hardware threads shared between
    /// the cameras that are open.
    static int getBuildThreadCount();

    static void buildPointCloud(const ofProtonect::Frames& frames,
                                const libfreenect2::Freenect2Device::IrCameraParams& ir,
                                const libfreenect2::Freenect2Device::ColorCameraParams& color,
                                const CloudOptions& options,
                                Cloud& cloud);

protected:
    void threadedFunction() override;
    void applyColourSettings();
    CloudOptions getCloudOptions() const;

    Settings settings;
    ofProtonect protonect;
    std::atomic<bool> bOpened{ false };
    std::atomic<bool> bTransformPointCloud{ false };
    bool bCloudInColourSpace = false;
    bool bCloudInColourSpaceFront = false;

    mutable std::mutex transformMutex;
    glm::mat4 pointCloudTransformationMat = glm::mat4(1.f);

    // Current frame, owned by the app thread.
    ofProtonect::Frames frames;
    ofPixels depthPixels;
    ofPixels irPixels;
    ofVboMesh pointCloud;
    Cloud cloud;

    // Handed over from the capture thread under lock().
    ofProtonect::Frames framesFront;
    ofPixels depthPixelsFront;
    ofPixels irPixelsFront;
    Cloud cloudFront;
    bool bNewBuffer = false;
    CaptureStats stats;

    bool bNewFrame = false;
    static std::atomic<int> openCameraCount;
    uint64_t lastFrameNo = std::numeric_limits<uint64_t>::max();
};
