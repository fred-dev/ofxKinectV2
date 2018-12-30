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
	//ofEnableArbTex();
	//ofEnableDepthTest();
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
        kinects[d]->open(deviceList[d].serial);
		pointClouds[d].setMode(ofPrimitiveMode::OF_PRIMITIVE_TRIANGLES);
        panel.add(kinects[d]->params);
    }
	panel.add(facesMaxLength.set("faces length", 0.3, 0.01, 0.1));
	panel.add(steps.set("Steps", 1, 1, 10));

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
				texRGB[d].loadData(kinects[d]->getPixels());
				texRGBRegistered[d].loadData(kinects[d]->getRegisteredPixels());
				texIR[d].loadData(kinects[d]->getIRPixels());
				texDepth[d].loadData(kinects[d]->getDepthPixels());

				if (showPointCloud)
				{
					
					pointClouds[d].clear();

					const int width = texRGBRegistered[d].getWidth();
					const int height = texRGBRegistered[d].getHeight();
					const auto frameSize = width * height;

					pointClouds[d].clear();
					for (std::size_t y = 0; y < texRGBRegistered[d].getHeight(); y++)

					for (std::size_t x = 0; x < texRGBRegistered[d].getWidth(); x++)
					{
						{
							pointClouds[d].addVertex(kinects[d]->getWorldCoordinateAt(x, y));
							//pointCloud.addTexCoord(ofVec2f(x, y));
							pointClouds[d].addColor(kinects[d]->getRegisteredPixels().getColor(x, y));
						}
					}
					pointClouds[d].getVertices().resize(frameSize);
					pointClouds[d].getTexCoords().resize(frameSize);

					//cout << "frame ssize " + ofToString(frameSize) << endl;
					
					auto vertices = pointClouds[d].getVerticesPointer();

				
					for (int i = 0; i < width - steps; i += steps) {
						for (int j = 0; j < height - steps; j += steps) {
							int topLeft = width * j + i;
							int topRight = topLeft + steps;
							int bottomLeft = topLeft + width * steps;
							int bottomRight = bottomLeft + steps;

							const ofVec3f & vTL = vertices[topLeft];
							const ofVec3f & vTR = vertices[topRight];
							const ofVec3f & vBL = vertices[bottomLeft];
							const ofVec3f & vBR = vertices[bottomRight];
							//upper left triangle
							if (vTL.z > float(kinects[d]->minDistance)/1000 && vTL.z < float(kinects[d]->maxDistance) / 1000 && vTR.z > float(kinects[d]->minDistance)/1000 && vTR.z < float(kinects[d]->maxDistance) / 1000 &&  vBL.z > float(kinects[d]->minDistance)/1000 && vBL.z < float(kinects[d]->maxDistance) / 1000
								&& abs(vTL.z - vTR.z) < facesMaxLength
								&& abs(vTL.z - vBL.z) < facesMaxLength) {
								const ofIndexType indices[3] = { topLeft, bottomLeft, topRight };
								pointClouds[d].addIndices(indices, 3);
							}

							//bottom right triangle
							if (vBR.z > float(kinects[d]->minDistance)/1000 && vBR.z < float(kinects[d]->maxDistance) / 1000 && vTR.z > float(kinects[d]->minDistance)/1000 && vTR.z < float(kinects[d]->maxDistance) / 1000 && vBL.z > float(kinects[d]->minDistance)/1000 && vBL.z < float(kinects[d]->maxDistance) / 1000
								&& abs(vBR.z - vTR.z) < facesMaxLength
								&& abs(vBR.z - vBL.z) < facesMaxLength) {
								const ofIndexType indices[3] = { topRight, bottomRight, bottomLeft };
								pointClouds[d].addIndices(indices, 3);

							}					
						}
					}
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
			drawTextureAtRowAndColumn("RGB Pixels width: " + ofToString(texRGB[currentKinect].getWidth()) + " height: " + ofToString(texRGB[currentKinect].getHeight()),
				texRGB[currentKinect],
				0, 0);

			drawTextureAtRowAndColumn("RGB Pixels, Registered width: " + ofToString(texRGBRegistered[currentKinect].getWidth()) + " height: " + ofToString(texRGBRegistered[currentKinect].getHeight()),
				texRGBRegistered[currentKinect],
				1, 0);

			drawTextureAtRowAndColumn("Depth Pixels, Mapped width: " + ofToString(texDepth[currentKinect].getWidth()) + " height: " + ofToString(texDepth[currentKinect].getHeight()),
				texDepth[currentKinect],
				1, 1);

			drawTextureAtRowAndColumn("IR Pixels, Mapped width: " + ofToString(texIR[currentKinect].getWidth()) + " height: " + ofToString(texIR[currentKinect].getHeight()),
				texIR[currentKinect],
				0, 1);
		}
		else
		{


			//ofDisableArbTex();
			cam.begin();
			ofPushMatrix();
			ofScale(1000, -1000, -1000);
			pointClouds[currentKinect].draw();
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
