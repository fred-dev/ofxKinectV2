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
        irPixels.setFromPixels(reinterpret_cast<float*>(ir->data), ir->width, ir->height, 1);

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

