//  ofProtonect.cpp
//
//  Modified by Theodore Watson on 11/16/15
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
//#include <iostream>
//#include <signal.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>


ofProtonect::ofProtonect()
{
    if (ofGetLogLevel() == OF_LOG_VERBOSE)
    {
        libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Debug));
    }
    else
    {
        libfreenect2::setGlobalLogger(libfreenect2::createConsoleLogger(libfreenect2::Logger::Warning));
    }
}

int ofProtonect::open(const std::string& serial, PacketPipelineType packetPipelineType)
{
    switch (packetPipelineType)
    {
        case PacketPipelineType::CPU:
            pipeline = new libfreenect2::CpuPacketPipeline();
            break;
        case PacketPipelineType::OPENGL:
            pipeline = new libfreenect2::OpenGLPacketPipeline();
            break;
		case PacketPipelineType::OPENCL:
			pipeline = new libfreenect2::OpenCLPacketPipeline();
			break;
		case PacketPipelineType::OPENCLKDE:
			pipeline = new libfreenect2::OpenCLKdePacketPipeline();
			break;
#if defined(LIBFREENECT2_WITH_CUDA_SUPPORT)
		
        case PacketPipelineType::CUDA:
			pipeline = new libfreenect2::CudaPacketPipeline(deviceId);
            break;
        case PacketPipelineType::CUDAKDE:
            pipeline = new libfreenect2::CudaKdePacketPipeline(deviceId);
            break;
#endif
        case PacketPipelineType::DEFAULT:
            break;
    }

    if (pipeline)
    {
        dev = freenect2.openDevice(serial, pipeline);
    }
    else
    {
        dev = freenect2.openDevice(serial);
    }

    if (!dev)
    {
        ofLogError("ofProtonect::openKinect")  << "failure opening device with serial " << serial;
        return -1;
    }

    int types = 0;
    
    if (enableRGB)
        types |= libfreenect2::Frame::Color;
    if (enableDepth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
    
    listener = new libfreenect2::SyncMultiFrameListener(types);
    
    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);
    
    /// [start]
    if (enableRGB && enableDepth)
    {
        if (!dev->start())
        {
            ofLogError("ofProtonect::openKinect")  << "Error starting default stream for: " << serial;
            return -1;
        }
    }
    else
    {
        if (!dev->startStreams(enableRGB, enableDepth))
        {
            ofLogError("ofProtonect::openKinect")  << "Error starting selected streams for: " << serial;
            return -1;
        }
    }

    ofLogVerbose("ofProtonect::openKinect") << "device serial: " << dev->getSerialNumber();
    ofLogVerbose("ofProtonect::openKinect") << "device firmware: " << dev->getFirmwareVersion();

    registration = new libfreenect2::Registration(dev->getIrCameraParams(),
                                                  dev->getColorCameraParams());
    undistorted = new libfreenect2::Frame(512, 424, 4);
    registered = new libfreenect2::Frame(512, 424, 4);
	//bigFrame = new libfreenect2::Frame(1920, 1082, 4);
    bOpened = true;
    
    return 0;
}

void ofProtonect::updateKinect(ofPixels& rgbPixels,
                               ofPixels& rgbRegisteredPixels,
                               ofFloatPixels& depthPixels,
                               ofFloatPixels& irPixels,
                               ofFloatPixels& distancePixels)
{
    if (bOpened)
    {
        if (!listener->waitForNewFrame(frames, 10 * 1000))
        {
            ofLogError("ofProtonect::updateKinect") << "Timeout serial: " << dev->getSerialNumber();
            return;
        }

        libfreenect2::Frame* rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame* ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];

        if (enableRGB && enableDepth)
        {
            registration->apply(rgb,
                                depth,
                                undistorted,
                                registered);
        }

        ofPixelFormat rgbFormat;
        if (rgb->format == libfreenect2::Frame::BGRX)
        {
            rgbFormat = OF_PIXELS_BGRA;
        }
        else
        {
            rgbFormat = OF_PIXELS_RGBA;
        }
        
		rgbPixels.setFromPixels(rgb->data, rgb->width, rgb->height, rgbFormat);
		rgbRegisteredPixels.setFromPixels(registered->data, registered->width, registered->height, rgbFormat);

		depthPixels.setFromPixels(reinterpret_cast<float*>(depth->data), ir->width, ir->height, 1);
		irPixels.setFromPixels(reinterpret_cast<float*>(ir->data), ir->width, ir->height, 1);;




        listener->release(frames);
    }
}
void ofProtonect::updateKinect(ofPixels& rgbPixels,
	ofPixels& rgbRegisteredPixels,
	ofFloatPixels& depthPixels,
	ofFloatPixels& irPixels,
	ofFloatPixels& distancePixels, std::vector<glm::vec3>& pcVerts, std::vector<ofDefaultColorType>& pcColors, std::vector<ofIndexType>& pcIndicies, std::vector<glm::vec2>& pcTexCoords, int steps, float minDistance, float maxDistance, float facesMaxLength)
{
	if (bOpened)
	{
		if (!listener->waitForNewFrame(frames, 10 * 1000))
		{
			ofLogError("ofProtonect::updateKinect") << "Timeout serial: " << dev->getSerialNumber();
			return;
		}

		libfreenect2::Frame* rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame* ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame* depth = frames[libfreenect2::Frame::Depth];

		if (enableRGB && enableDepth)
		{
			registration->apply(rgb,
				depth,
				undistorted,
				registered);
		}

		ofPixelFormat rgbFormat;
		if (rgb->format == libfreenect2::Frame::BGRX)
		{
			rgbFormat = OF_PIXELS_BGRA;
		}
		else
		{
			rgbFormat = OF_PIXELS_RGBA;
		}

		rgbPixels.setFromPixels(rgb->data, rgb->width, rgb->height, rgbFormat);
		rgbRegisteredPixels.setFromPixels(registered->data, registered->width, registered->height, rgbFormat);

		depthPixels.setFromPixels(reinterpret_cast<float*>(depth->data), ir->width, ir->height, 1);
		irPixels.setFromPixels(reinterpret_cast<float*>(ir->data), ir->width, ir->height, 1);

		if (registration && undistorted)
		{
			const int width = rgbRegisteredPixels.getWidth();
			const int height = rgbRegisteredPixels.getHeight();
			const auto frameSize = width * height;

			pcVerts.clear();
			pcColors.clear();
			pcIndicies.clear();
			pcTexCoords.clear();
		


			for (std::size_t y = 0; y < rgbRegisteredPixels.getHeight(); y++){
				for (std::size_t x = 0; x < rgbRegisteredPixels.getWidth(); x++)
				{
					glm::vec3 position;
					float rgb;
					registration->getPointXYZRGB(undistorted, registered, y, x, position.x, position.y, position.z,  rgb);
					const uint8_t *p = reinterpret_cast<uint8_t*>(&rgb);
					uint8_t b = p[0];
					uint8_t g = p[1];
					uint8_t r = p[2];
					
					pcVerts.push_back(position);
					pcColors.push_back(ofColor(r, g, b));
					pcTexCoords.push_back(ofVec2f(x, y));
				}
			}

			for (int i = 0; i < width - steps; i += steps) {
				for (int j = 0; j < height - steps; j += steps) {
					int topLeft = width * j + i;
					int topRight = topLeft + steps;
					int bottomLeft = topLeft + width * steps;
					int bottomRight = bottomLeft + steps;
					const ofVec3f vTL = pcVerts[topLeft];
					const ofVec3f  vTR = pcVerts[topRight];
					const ofVec3f  vBL = pcVerts[bottomLeft];
					const ofVec3f  vBR = pcVerts[bottomRight];
					//cout << ofToString(vTL) << endl;
					//upper left triangle
					if (vTL.z > float(minDistance) / 1000 && vTL.z < float(maxDistance) / 1000 && vTR.z > float(minDistance) / 1000 && vTR.z < float(maxDistance) / 1000 && vBL.z > float(minDistance) / 1000 && vBL.z < float(maxDistance) / 1000
						&& abs(vTL.z - vTR.z) < facesMaxLength
						&& abs(vTL.z - vBL.z) < facesMaxLength) {
						const ofIndexType indices[3] = { topLeft, bottomLeft, topRight };
						pcIndicies.push_back(indices[0]);
						pcIndicies.push_back(indices[1]);
						pcIndicies.push_back(indices[2]);


					}

					//bottom right triangle
					if (vBR.z > float(minDistance) / 1000 && vBR.z < float(maxDistance) / 1000 && vTR.z > float(minDistance) / 1000 && vTR.z < float(maxDistance) / 1000 && vBL.z > float(minDistance) / 1000 && vBL.z < float(maxDistance) / 1000
						&& abs(vBR.z - vTR.z) < facesMaxLength
						&& abs(vBR.z - vBL.z) < facesMaxLength) {
						const ofIndexType indices[3] = { topRight, bottomRight, bottomLeft };
						pcIndicies.push_back(indices[0]);
						pcIndicies.push_back(indices[1]);
						pcIndicies.push_back(indices[2]);

					}
				}
			}	
		}
		listener->release(frames);
	}
}

int ofProtonect::closeKinect()
{
  if (bOpened)
  {
      listener->release(frames);

      // TODO: restarting ir stream doesn't work!
      // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
      dev->stop();
      dev->close();

      delete dev;
      delete pipeline;
      delete listener;
      delete undistorted;
      delete registered;
      delete registration;
      //delete bigFrame;
      bOpened = false;
  }

  return 0;
}

