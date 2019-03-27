//
//  ofxKinectV2.cpp
//  kinectExample
//
//  Created by Theodore Watson on 6/23/14.
//
//

#include "ofxKinectV2.h"


ofxKinectV2::ofxKinectV2()
{
    //set default distance range to 50cm - 600cm
    params.add(minDistance.set("minDistance", 500, 0, 12000));
    params.add(maxDistance.set("maxDistance", 6000, 0, 12000));

	params.add(expIntegrationTime.set("Shutter speed", 50.0, 0.0, 66.0));
	expIntegrationTime.addListener(this, &ofxKinectV2::setIntegrationTimeCallback);

	params.add(analogueGain.set("analogueGain", 3.0, 1.0, 4.0));
	analogueGain.addListener(this, &ofxKinectV2::setAnalogueGainCallback);

	params.add(autoExposure.set("auto exposure", true));
	autoExposure.addListener(this, &ofxKinectV2::setAutoExposureCallback);
    
    params.add(redGain.set("redGain", 2.0, 0.01, 4.0));
    redGain.addListener(this, &ofxKinectV2::setRedGainCallback);

    params.add(blueGain.set("blueGain", 2.0, 0.01, 4.0));
    blueGain.addListener(this, &ofxKinectV2::setBlueGainCallback);

    params.add(greenGain.set("greenGain", 2.0, 0.01, 4.0));
    greenGain.addListener(this, &ofxKinectV2::setGreenGainCallback);

	params.add(autoWhiteBalance.set("auto white balance", true));
	autoWhiteBalance.addListener(this, &ofxKinectV2::setAutoWhiteBalanceCallback);
    
    params.add(facesMaxLength.set("Point cloud faces length", 0.3, 0.01, 0.5));
    params.add(steps.set("Point clooud tex steps", 1, 1, 10));
    
}


ofxKinectV2::~ofxKinectV2()
{
    close();
}


std::vector<ofxKinectV2::KinectDeviceInfo> ofxKinectV2::getDeviceList() const
{
    std::vector<KinectDeviceInfo> devices;
    
    int num = protonect.getFreenect2Instance().enumerateDevices();

    for (int i = 0; i < num; i++)
    {
        KinectDeviceInfo kdi;
        kdi.serial = protonect.getFreenect2Instance().getDeviceSerialNumber(i);
        kdi.freenectId = i; 
        devices.push_back(kdi);
    }
    
    ofSort(devices, [](const ofxKinectV2::KinectDeviceInfo& A,
                       const ofxKinectV2::KinectDeviceInfo& B)
           {
               return A.serial < B.serial;
           });

    for (std::size_t i = 0; i < num; i++)
    {
        devices[i].deviceId = i;
    }
    
    return devices;
}


std::size_t ofxKinectV2::getNumDevices() const
{
   return getDeviceList().size(); 
}


bool ofxKinectV2::open(int deviceId, ofProtonect::PacketPipelineType packetPipelineType, int processingDevice, bool initRGB, bool initIr, bool initDepth, bool registerImages, bool usePointCloud, bool pointCloudHasFaces)
{
    std::vector<KinectDeviceInfo> devices = getDeviceList();
    
    if (devices.empty())
    {
        ofLogError("ofxKinectV2::open") << "no devices connected!";
        return false;
    }
    
    if(deviceId >= devices.size())
    {
        ofLogError("ofxKinectV2::open") << " deviceId " << deviceId << " is bigger or equal to the number of connected devices " << devices.size() << endl;
        return false;
    }

    string serial = devices[deviceId].serial;
    
    return open(serial , packetPipelineType, processingDevice, initRGB, initDepth, registerImages, usePointCloud, pointCloudHasFaces);
}

bool ofxKinectV2::open(const std::string& serial, ofProtonect::PacketPipelineType packetPipelineType,  int processingDevice, bool initRGB, bool initIr, bool initDepth, bool registerImages, bool usePointCloud, bool pointCloudHasFaces)
{
    close(); 

    params.setName("kinectV2 " + serial);
    
    bNewFrame  = false;
    bNewBuffer = false;
    bOpened    = false;
    
    int retVal = protonect.open(serial,packetPipelineType,processingDevice);
    
    if (retVal != 0)
    {
        return false;
    }
    
    lastFrameNo = -1;
    
    bOpened = true;
    
    autoExposure = true;
    
    bUsePointCloud = usePointCloud;
    bRegisterImages = registerImages;
    bPointCloudFilled = pointCloudHasFaces;
    bEnableRGB = initRGB;
    bEnableDepth = initDepth;
    
    setUseRgb(initRGB);
    setUseDepth(initDepth);
    setUseRegisterImages(registerImages);
    setUsePointCloud(usePointCloud);
    setIsPointCloudFilled(pointCloudHasFaces);
    
    if(pointCloudHasFaces){
        pointCloud.setMode(OF_PRIMITIVE_TRIANGLES);
        setUseRgb(true);
        setUseDepth(true);
        setUseRegisterImages(true);
        setUsePointCloud(true);
    }
    else{
        pointCloud.setMode(OF_PRIMITIVE_POINTS);
        setUseRgb(true);
        setUseDepth(true);
        setUseRegisterImages(true);
    }
    
    if (usePointCloud) {
        setUseRgb(true);
        setUseDepth(true);
        setUseRegisterImages(true);
    }
    if (registerImages) {
        setUseRgb(true);
        setUseDepth(true);
    }
    startThread();
    return true;
}




void ofxKinectV2::threadedFunction()
{
    while (isThreadRunning())
    {
        protonect.updateKinect(pixelsBack,
                               registeredPixelsBack,
                               rawDepthPixelsBack,
                               rawIRPixelsBack,
                               distancePixelsBack, pcVertsBack, pcColorsBack,pcIndicesBack, pcTexCoordsBack,steps,minDistance,maxDistance,facesMaxLength);
        if (getUseRgb()) {
            pixelsFront.swap(pixelsBack);
        }
        
        
        if(getUseRegisterImages()){
            registeredPixelsFront.swap(registeredPixelsBack);
        }
        if(getUseDepth()){
            rawDepthPixelsFront.swap(rawDepthPixelsBack);
            distancePixelsFront.swap(distancePixelsBack);
        }
        if(getUseIr()){
            rawIRPixelsFront.swap(rawIRPixelsBack);
        }
        
        if(getUsePointCloud()){
            pcVertsFront.swap(pcVertsBack);
            pcColorsFront.swap(pcColorsBack);
            pcTexCoordsFront.swap(pcTexCoordsBack);
            pointCloud.getVertices().swap(pcVertsFront);
            pointCloud.getColors().swap(pcColorsFront);
            pointCloud.getTexCoords().swap(pcTexCoordsFront);
        }

        if (getIsPointCloudFilled()) {
            pointCloud.getIndices().swap(pcIndicesFront);
            pcIndicesFront.swap(pcIndicesBack);
        }
        
		lock();
        bNewBuffer = true;
        unlock();
    }
}


void ofxKinectV2::update()
{
    if (ofGetFrameNum() != lastFrameNo)
    {
        bNewFrame = false;
        lastFrameNo = ofGetFrameNum();
    }
    
    if (bNewBuffer)
    {
        lock();
            if(getUseRgb()){
                pixels = pixelsFront;
            }
        
            if(getUseRegisterImages()){
                registeredPixels = registeredPixelsFront;
            }
        
            if(getUseDepth()){
                rawDepthPixels = rawDepthPixelsFront;
            }
        
            if(getUseIr()){
                rawIRPixels = rawIRPixelsFront;
            }
        
            if(getUsePointCloud()){
                pcVerts = pcVertsFront;
                pcColors = pcColorsFront;
                pcTexCoords = pcTexCoordsFront;
            }
			
        
            if (getIsPointCloudFilled()) {
                pcIndices = pcIndicesFront;
            }
			
            bNewBuffer = false;
        unlock();
        if(getUseDepth()){
            // TODO: This is inefficient and we should be able to turn it off or
            // draw it directly with a shader.
            if (rawDepthPixels.size() > 0)
            {
                if (depthPixels.getWidth() != rawDepthPixels.getWidth())
                {
                    depthPixels.allocate(rawDepthPixels.getWidth(), rawDepthPixels.getHeight(), 1);
                }
            
                float* pixelsF = rawDepthPixels.getData();
                unsigned char * pixelsC = depthPixels.getData();
                    
                for (std::size_t i = 0; i < depthPixels.size(); i++)
                {
                    pixelsC[i] = ofMap(pixelsF[i], minDistance, maxDistance, 255, 0, true);
                
                    if (pixelsC[i] == 255)
                    {
                        pixelsC[i] = 0;
                    }
                }
            }
        }
        
        if (getUseIr()) {
            // TODO: This is inefficient and we should be able to turn it off or
            // draw it directly with a shader.
            if (rawIRPixels.size() > 0)
            {
                if (irPixels.getWidth() != rawIRPixels.getWidth())
                {
                    irPixels.allocate(rawIRPixels.getWidth(), rawIRPixels.getHeight(), 1);
                }
                
                float* pixelsF = rawIRPixels.getData();
                unsigned char * pixelsC = irPixels.getData();
                
                for (std::size_t i = 0; i < irPixels.size(); i++)
                {
                    pixelsC[i] = ofMap(pixelsF[i], 0, 4500, 0, 255, true);
                }
            }
        }
        
        bNewFrame = true;
    }
}

bool ofxKinectV2::isFrameNew() const
{
    return bNewFrame; 
}

ofPixels ofxKinectV2::getRgbPixels()
{
    return getPixels();
}

const ofPixels& ofxKinectV2::getPixels() const
{
    return pixels;
}

const ofPixels& ofxKinectV2::getRegisteredPixels() const
{
    return registeredPixels;
}

const ofFloatPixels& ofxKinectV2::getRawDepthPixels() const
{
    return rawDepthPixels;
}

const ofPixels& ofxKinectV2::getDepthPixels() const
{
    return depthPixels;
}

const ofFloatPixels& ofxKinectV2::getRawIRPixels() const
{
    return rawIRPixels;
}

const ofPixels& ofxKinectV2::getIRPixels() const
{
    return irPixels;
}

void ofxKinectV2::updatePointCloud() {
	
}

void ofxKinectV2::setPointCloudAlpha(int alpha)
{
	protonect.setPointCloudAlpha(alpha);
}

const ofMesh& ofxKinectV2::getPointCloud() const
{
	return pointCloud;
}

float ofxKinectV2::getDistanceAt(std::size_t x, std::size_t y) const
{
    return glm::distance(glm::vec3(0, 0, 0), getWorldCoordinateAt(x, y));
}

glm::vec3 ofxKinectV2::getWorldCoordinateAt(std::size_t x, std::size_t y) const
{
    glm::vec3 position;
    
    if (protonect.registration && protonect.undistorted)
    {
//        std::cout << x << ", " << protonect.undistorted->width << std::endl;
//        std::cout << y << ", " << protonect.undistorted->height << std::endl;
//        
        if (x < protonect.undistorted->width && y < protonect.undistorted->height)
            protonect.registration->getPointXYZ(protonect.undistorted, y, x, position.x, position.y, position.z);
        else ofLogWarning("ofxKinectV2::getWorldCoordinateAt") << "Invalid x, y coordinates.";

    }
    else ofLogWarning("ofxKinectV2::getWorldCoordinateAt") << "Kinect is not initialized, returning 0, 0, 0.";
    
    return position;
}

void ofxKinectV2::setUsePointCloud(bool _usePointCloud){
    protonect.setUsePointCloud(_usePointCloud);
}

void ofxKinectV2::setUseRegisterImages(bool _registerImages){
    protonect.setRegisterImages(_registerImages);
}

void ofxKinectV2::setIsPointCloudFilled(bool _pointCloudFilled){
    protonect.setIsPointCloudFilled(_pointCloudFilled);
}

void ofxKinectV2::setUseRgb(bool _enableRGB){
    protonect.setUseRgb(_enableRGB);
}

void ofxKinectV2::setUseDepth(bool _enableDepth){
    protonect.setUseDepth(_enableDepth);
}
void ofxKinectV2::setUseIr(bool _enableIr){
    protonect.setUseIr(_enableIr);
}
bool ofxKinectV2::getUsePointCloud(){
    return protonect.getUsePointCloud();
}

bool ofxKinectV2::getUseRegisterImages(){
    return protonect.getRegisterImages();
}

bool ofxKinectV2::getIsPointCloudFilled(){
    return protonect.getIsPointCloudFilled();
}

bool ofxKinectV2::getUseRgb(){
    return protonect.getUseRgb();
}

bool ofxKinectV2::getUseDepth(){
    return protonect.getUseDepth();
}

bool ofxKinectV2::getUseIr(){
    return protonect.getUseIr();
}

void ofxKinectV2::setAutoExposureCallback(bool & auto_exposure){
    if(auto_exposure){
        autoExposure = true;
        protonect.dev->setColorAutoExposure(0);
    }
}



void ofxKinectV2::setIntegrationTimeCallback(float & integration_time_ms){
    protonect.dev->setColorManualExposure(integration_time_ms, analogueGain);
    autoExposure = false;
}

void ofxKinectV2::setAnalogueGainCallback(float & analog_gain){

    protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
    protonect.dev->setColorManualExposure(expIntegrationTime, analog_gain);
    autoExposure = false;
}



void ofxKinectV2::setAutoWhiteBalanceCallback(bool & auto_white_balance){
    if(auto_white_balance == true){
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(1));
    }
    else{
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(3));
    }
    
}

void ofxKinectV2::setRedGainCallback(float & red_gain){
    if(autoWhiteBalance){
        autoWhiteBalance = false;
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(3));
    }
    protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_RED_CHANNEL_GAIN, red_gain);
    
}

void ofxKinectV2::setGreenGainCallback(float & green_gain){
    if(autoWhiteBalance){
        autoWhiteBalance = false;
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(3));
    }
    protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_GREEN_CHANNEL_GAIN, green_gain);
    
}

void ofxKinectV2::setBlueGainCallback(float & blue_gain){
    if(autoWhiteBalance){
        autoWhiteBalance = false;
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_ACS, uint32_t(0));
        protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_WHITE_BALANCE_MODE, uint32_t(3));
    }
    protonect.dev->setColorSetting(libfreenect2::COLOR_SETTING_SET_BLUE_CHANNEL_GAIN, blue_gain);
}

void ofxKinectV2::close()
{
    if (bOpened)
    {
        waitForThread(true);
        protonect.closeKinect();
        bOpened = false;
    }
}
