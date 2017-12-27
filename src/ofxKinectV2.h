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

class ofxKinectV2 : public ofThread{

    public:
    
        struct KinectDeviceInfo{
            string serial;
            int deviceId;   //if you have the same devices plugged in device 0 will always be the same Kinect
            int freenectId; //don't use this one - this is the index given by freenect2 - but this can change based on order device is plugged in
        };

        ofxKinectV2();
        ~ofxKinectV2(); 
        
        //for some reason these can't be static - so you need to make a tmp object to query them
        vector <KinectDeviceInfo> getDeviceList();
        unsigned int getNumDevices();
    
        bool open(string serial);
        bool open(unsigned int deviceId = 0);
        void update();
        void close();
    
        bool isFrameNew();
		ofVboMesh getMesh();
        ofPixels getDepthPixels();
        ofPixels getRgbPixels();
		ofPixels getRegisteredPixels();
        ofFloatPixels getRawDepthPixels();
    
        ofParameterGroup params;
        ofParameter <float> minDistance;
        ofParameter <float> maxDistance;

		static const int depth_w = ofProtonect::depth_w;
		static const int depth_h = ofProtonect::depth_h;
		ofProtonect &getProtonect() { return protonect; }

		float getDistanceAt(int x, int y);
		ofVec3f getWorldCoordinateAt(int x, int y);
		ofVec3f getWorldCoordinateAt(int x, int y, float z);
		void convertScreenToWorld(const vector<ofPoint> &pnt, vector<ofPoint> &pnt_world);
	






    protected:
        void threadedFunction();

        ofPixels rgbPix;
        ofPixels depthPix;
		ofPixels registeredPixels;
        ofFloatPixels rawDepthPixels;
    
        bool bNewBuffer;
        bool bNewFrame;
        bool bOpened;
    
        ofProtonect protonect; 
    
        ofPixels rgbPixelsBack;
        ofPixels rgbPixelsFront;
		ofPixels registeredPixelsFront;
		ofPixels registeredPixelsBack;
        ofFloatPixels depthPixelsBack;
        ofFloatPixels depthPixelsFront;
        int lastFrameNo; 
};