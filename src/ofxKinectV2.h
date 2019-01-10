//
//  ofxKinectV2.h
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#pragma once


#include "ofProtonect.h"
#include "ofMain.h"


class ofxKinectV2: public ofThread
{
public:
    struct KinectDeviceInfo
    {
        std::string serial;
        int deviceId;   //if you have the same devices plugged in device 0 will always be the same Kinect
        int freenectId; //don't use this one - this is the index given by freenect2 - but this can change based on order device is plugged in
    };

    ofxKinectV2();
    ~ofxKinectV2();
    
    //for some reason these can't be static - so you need to make a tmp object to query them
    std::vector<KinectDeviceInfo> getDeviceList() const;
    std::size_t getNumDevices() const;

    /// \brief Open the device with the given serial number.
    /// \param serial The serial number to open.
    /// \returns true if connected successfully.
    bool open(const std::string& serial, ofProtonect::PacketPipelineType packetPipelineType = ofProtonect::PacketPipelineType::OPENCL, int processingDevice = 0, bool initRGB =true, bool initIr =true, bool initDepth = true, bool registerImages =true, bool usePointCloud = true, bool pointCloudHasFaces = true);
    
    

    /// \brief Open the device with the given serial number.
    /// \param deviceId The device id to open.
    /// \returns true if connected successfully.
    bool open(int deviceId = 0, ofProtonect::PacketPipelineType packetPipelineType = ofProtonect::PacketPipelineType::OPENCL, int processingDevice = 0, bool initRGB =true, bool initIr =true, bool initDepth = true, bool registerImages =true, bool usePointCloud = true, bool pointCloudHasFaces = true);

    /// \brief Update the Kinect internals.
    void update();
    
    /// \brief Close the connection to the Kinect.
    void close();

    /// \returns true if the frame has been updated.
    bool isFrameNew() const;

    OF_DEPRECATED_MSG("Use getPixels()", ofPixels getRgbPixels());

    /// \returns the RGB pixels.
    const ofPixels& getPixels() const;

    /// \returns pixels registred to the depth image.
    const ofPixels& getRegisteredPixels() const;

    /// \returns the raw depth pixels.
    const ofFloatPixels& getRawDepthPixels() const;

    /// \returns the depth pixels mapped to a visible range.
    const ofPixels& getDepthPixels() const;

    /// \returns the raw IR pixels.
    const ofFloatPixels& getRawIRPixels() const;

    /// \returns the IR pixels mapped to a visible range.
    const ofPixels& getIRPixels() const;

    /// \returns the distance image. Each pixels is the distance in millimeters.
    const ofFloatImage& getDistancePixels() const;

	const ofMesh& getPointCloud() const;
    
    /// \brief Get the calulated distance for point x, y in the getRegisteredPixels image.
    float getDistanceAt(std::size_t x, std::size_t y) const;
    
    /// \brief Get the world X, Y, Z coordinates in millimeters for x, y in getRegisteredPixels image.
    glm::vec3 getWorldCoordinateAt(std::size_t x, std::size_t y) const;
    
    ofParameterGroup params;
    ofParameter<float> minDistance;
    ofParameter<float> maxDistance;
	ofParameter<float> facesMaxLength;
	ofParameter<int> steps;
    
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



     void updatePointCloud();
    
    void setUsePointCloud(bool _usePointCloud);
    void setUseRegisterImages(bool _registerImages);
    void setIsPointCloudFilled(bool _pointCloudFilled);
    void setUseRgb(bool _enableRGB);
    void setUseDepth(bool _enableDepth);
    void setUseIr(bool _enableIr);
    

    
    bool getUsePointCloud();
    bool getUseRegisterImages();
    bool getIsPointCloudFilled();
    bool getUseRgb();
    bool getUseDepth();
    bool getUseIr();

protected:
    bool bUsePointCloud;
    bool bRegisterImages;
    bool bPointCloudFilled;
    bool bEnableRGB;
    bool bEnableDepth;
    bool bEnableIr;
    
    void threadedFunction();

    ofPixels pixels;
    ofPixels registeredPixels;
    ofFloatPixels rawDepthPixels;
    ofPixels depthPixels;
    ofFloatPixels distancePixels;
	ofMesh pointCloud;
	std::vector<glm::vec3> pcVerts;
	std::vector<ofDefaultColorType> pcColors;
	std::vector<ofIndexType> pcIndices;
	std::vector<glm::vec2> pcTexCoords;
    ofFloatPixels rawIRPixels;
    ofPixels irPixels;

    
    bool bNewBuffer = false;
    bool bNewFrame = false;
    bool bOpened = false;

    mutable ofProtonect protonect;
    
    ofPixels pixelsBack;
    ofPixels pixelsFront;
    ofPixels registeredPixelsBack;
    ofPixels registeredPixelsFront;
    ofFloatPixels rawDepthPixelsBack;
    ofFloatPixels rawDepthPixelsFront;
    ofFloatPixels rawIRPixelsBack;
    ofFloatPixels rawIRPixelsFront;
    ofFloatPixels distancePixelsFront;
    ofFloatPixels distancePixelsBack;

	std::vector<glm::vec3> pcVertsFront;
	std::vector<glm::vec3> pcVertsBack;

	std::vector<ofDefaultColorType> pcColorsFront;
	std::vector<ofDefaultColorType> pcColorsBack;

	std::vector<ofIndexType> pcIndicesFront;
	std::vector<ofIndexType> pcIndicesBack;

	std::vector<glm::vec2> pcTexCoordsFront;
	std::vector<glm::vec2> pcTexCoordsBack;

    int lastFrameNo = -1;
};
