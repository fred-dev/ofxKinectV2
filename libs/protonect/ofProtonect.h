//  ofProtonect.cpp
//
//  Created by Theodore Watson on 11/16/15


#include "ofMain.h"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <libfreenect2/color_settings.h>

#include "ofAppGLFWWindow.h"
#include "ofAppRunner.h"

#include <GLFW/glfw3.h>

class ofProtonect
{
public:
    enum class PacketPipelineType
    {
        DEFAULT,
        CPU,
        OPENGL,
		OPENCL,
		OPENCLKDE
#if defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
        ,CUDA,
        CUDAKDE
#endif
    };

    ofProtonect();
    
    int open(const std::string& serial,
             PacketPipelineType packetPipelineType = PacketPipelineType::OPENCL, int device = 0);
    

    void updateKinect(ofPixels& rgbPixels,
                      ofPixels& rgbRegisteredPixels,
                      ofFloatPixels& depthPixels,
                      ofFloatPixels& irPixels,
                      ofFloatPixels& distancePixels);

	void updateKinect(ofPixels& rgbPixels,
		ofPixels& rgbRegisteredPixels,
		ofFloatPixels& depthPixels,
		ofFloatPixels& irPixels,
		ofFloatPixels& distancePixels, std::vector<glm::vec3>& pcVerts, std::vector<ofDefaultColorType>& pcColors, std::vector<ofIndexType>& pcIndicies,  std::vector<glm::vec2>& pcTexCoords,int steps, float minDistance, float maxDistance, float facesMaxLength);

    int closeKinect();


    libfreenect2::Freenect2& getFreenect2Instance()
    {
        return freenect2;
    }
    void setUsePointCloud(bool _usePointCloud);
    void setRegisterImages(bool _registerImages);
    void setIsPointCloudFilled(bool _pointCloudFilled);
    void setUseRgb(bool _enableRGB);
    void setUseDepth(bool _enableDepth);
    void setUseIr(bool _enableIr);
	void setPointCloudAlpha(int alpha);
    
    bool getUsePointCloud();
    bool getRegisterImages();
    bool getIsPointCloudFilled();
    bool getUseRgb();
    bool getUseDepth();
    bool getUseIr();
	int getPointCloudAlpha();
    
    void setColorCamSettings();

protected:
    ofPixelFormat rgbFormat;
    
    bool enableRGB = true;
    bool enableIr = true;
    bool enableDepth = true;
    bool usePointCloud =true;
    bool registerImages = true;
    bool pointCloudFilled = true;
	int pointCloudAlpha = 255;

    int deviceId = -1;

    bool bOpened = false;
    
    libfreenect2::Freenect2 freenect2;

    libfreenect2::Freenect2Device* dev = nullptr;
    libfreenect2::PacketPipeline* pipeline = nullptr;

    libfreenect2::FrameMap frames;

    libfreenect2::Registration* registration = nullptr;
    libfreenect2::SyncMultiFrameListener* listener = nullptr;
    libfreenect2::Frame* undistorted = nullptr;
    libfreenect2::Frame* registered = nullptr;
    libfreenect2::Frame* bigFrame = nullptr;



    friend class ofxKinectV2;
};
