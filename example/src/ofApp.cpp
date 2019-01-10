#include "ofApp.h"

//NOTE: if you are unable to connect to your device on OS X, try unplugging and replugging in the power, while leaving the USB connected.
//ofxKinectV2 will only work if the NUI Sensor shows up in the Superspeed category of the System Profiler in the USB section.

//On OS X if you are not using the example project. Make sure to add OpenCL.framework to the Link Binary With Library Build Phase 
//and change the line in Project.xcconfig to OTHER_LDFLAGS = $(OF_CORE_LIBS) $(OF_CORE_FRAMEWORKS) -framework OpenCL


void ofApp::setup()
{
    // Uncomment for verbose info from libfreenect2
    ofSetLogLevel(OF_LOG_VERBOSE);

    ofBackground(0);
	ofEnableArbTex();
    
    //see how many devices we have.
    ofxKinectV2 tmp;
    std::vector <ofxKinectV2::KinectDeviceInfo> deviceList = tmp.getDeviceList();
    
    //allocate for this many devices
    kinects.resize(deviceList.size());
    texDepth.resize(kinects.size());
    texRGB.resize(kinects.size());
    texRGBRegistered.resize(kinects.size());
    texIR.resize(kinects.size());
	pointClouds.resize(kinects.size());

    panel.setup("", "settings.xml", 10, 100);
    
    // Note you don't have to use ofxKinectV2 as a shared pointer, but if you
    // want to have it in a vector ( ie: for multuple ) it needs to be.
    for(int d = 0; d < kinects.size(); d++)
    {
        kinects[d] = std::make_shared<ofxKinectV2>();
        kinects[d]->open(deviceList[d].serial, ofProtonect::PacketPipelineType::OPENCL, 2, false, false, true, false, false, true);
        panel.add(kinects[d]->params);
    }


    panel.loadFromFile("settings.xml");
	

}


void ofApp::update()
{
	if (kinects.size()>0)
	{
		for (int d = 0; d < kinects.size(); d++)
		{
			kinects[d]->update();

			if (kinects[d]->isFrameNew())
			{
                if (kinects[d]->getUseRgb()) {
                    texRGB[d].loadData(kinects[d]->getPixels());
                }
                if (kinects[d]->getUseRegisterImages()) {
                    texRGBRegistered[d].loadData(kinects[d]->getRegisteredPixels());
                }
                if (kinects[d]->getUseIr()) {
                    texIR[d].loadData(kinects[d]->getIRPixels());
                }
                if (kinects[d]->getUseDepth()) {
                    texDepth[d].loadData(kinects[d]->getDepthPixels());
                }

				
			}
		}
	}
    
}


void ofApp::draw()
{
	if (kinects.size()>0)
	{
		if (!showPointCloud)
		{
            if (kinects[currentKinect]->getUseRgb()) {
                drawTextureAtRowAndColumn("RGB Pixels width: " + ofToString(texRGB[currentKinect].getWidth()) + " height: " + ofToString(texRGB[currentKinect].getHeight()),
                                          texRGB[currentKinect],
                                          0, 0);
            }
			
            if (kinects[currentKinect]->getUseRegisterImages()) {
                drawTextureAtRowAndColumn("RGB Pixels, Registered width: " + ofToString(texRGBRegistered[currentKinect].getWidth()) + " height: " + ofToString(texRGBRegistered[currentKinect].getHeight()),
                                          texRGBRegistered[currentKinect],
                                          1, 0);
            }
			
            if (kinects[currentKinect]->getUseDepth()) {
                drawTextureAtRowAndColumn("Depth Pixels, Mapped width: " + ofToString(texDepth[currentKinect].getWidth()) + " height: " + ofToString(texDepth[currentKinect].getHeight()),
                                          texDepth[currentKinect],
                                          1, 1);
            }
			
            if (kinects[currentKinect]->getUseIr()) {
                drawTextureAtRowAndColumn("IR Pixels, Mapped width: " + ofToString(texIR[currentKinect].getWidth()) + " height: " + ofToString(texIR[currentKinect].getHeight()),
                                          texIR[currentKinect],
                                          0, 1);
            }
			
		}
		else
		{
		
			

			cam.begin();
			ofPushMatrix();
			ofScale(1000, -1000, -1000);
            if (kinects[currentKinect]->getUsePointCloud()) {
                kinects[currentKinect]->getPointCloud().draw();

            }
			ofPopMatrix();
			cam.end();
		}

		panel.draw();
		ofDrawBitmapString(ofToString(ofGetFrameRate()), 10, 10);
	}
   
}


void ofApp::keyPressed(int key)
{
    if (key == ' ')
    {
		if (kinects.size()>0)
		{
			currentKinect = (currentKinect + 1) % kinects.size();

		}
		
    }
	if (key == ' ')
	{
		pointClouds[currentKinect].save("test.ply");
	}
    else if (key == 'p')
    {
        showPointCloud = !showPointCloud;
    }
}


void ofApp::drawTextureAtRowAndColumn(const std::string& title,
                                      const ofTexture& tex,
                                      int row,
                                      int column)
{
    float displayWidth = ofGetWidth() / numColumns;
    float displayHeight = ofGetHeight() / numRows;
    
    ofRectangle targetRectangle(row * displayWidth,
                                column * displayHeight,
                                displayWidth,
                                displayHeight);
    
    ofNoFill();
    ofSetColor(ofColor::gray);
    ofDrawRectangle(targetRectangle);
    
    ofFill();
    ofSetColor(255);
    if (tex.isAllocated())
    {
        ofRectangle textureRectangle(0, 0, tex.getWidth(), tex.getHeight());
        
        // Scale the texture rectangle to its new location and size.
        textureRectangle.scaleTo(targetRectangle);
        tex.draw(textureRectangle);
    }
    else
    {
        ofDrawBitmapStringHighlight("...",
                                    targetRectangle.getCenter().x,
                                    targetRectangle.getCenter().y);
    }
    
    ofDrawBitmapStringHighlight(title,
                                targetRectangle.getPosition() + glm::vec3(14, 20, 0));
}
