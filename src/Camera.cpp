/*
 * Camera.cpp
 *
 *  Created on: 2011-10-21
 *      Author: matlab
 */

#include "Camera.h"
#include <stdio.h>
#include <opencv2/imgproc/imgproc_c.h>

Camera::Camera(int deviceId,
		int imageWidth,
		int imageHeight,
		QObject * parent) :
	QObject(parent),
	capture_(0),
	deviceId_(deviceId),
	imageWidth_(imageWidth),
	imageHeight_(imageHeight)
{
}

Camera::~Camera()
{
	this->close();
}

bool Camera::init()
{
	if(!capture_)
	{
		capture_ = cvCaptureFromCAM(deviceId_);
		if(capture_)
		{
			cvSetCaptureProperty(capture_, CV_CAP_PROP_FRAME_WIDTH, double(imageWidth_));
			cvSetCaptureProperty(capture_, CV_CAP_PROP_FRAME_HEIGHT, double(imageHeight_));
		}
	}
	if(!capture_)
	{
		printf("Failed to create a capture object!\n");
		return false;
	}
	return true;
}

void Camera::close()
{
	if(capture_)
	{
		cvReleaseCapture(&capture_);
		capture_ = 0;
	}
}

IplImage * Camera::takeImage()
{
	IplImage * img = 0;
	if(capture_)
	{
		if(cvGrabFrame(capture_)) // capture a frame
		{
			img = cvRetrieveFrame(capture_); // retrieve the captured frame
		}
		else
		{
			printf("CameraVideo: Could not grab a frame, the end of the feed may be reached...\n");
		}
	}

	//resize
	if(img &&
		imageWidth_ &&
		imageHeight_ &&
		imageWidth_ != (unsigned int)img->width &&
		imageHeight_ != (unsigned int)img->height)
	{
		// declare a destination IplImage object with correct size, depth and channels
		IplImage * resampledImg = cvCreateImage( cvSize(imageWidth_, imageHeight_),
											   img->depth,
											   img->nChannels );

		//use cvResize to resize source to a destination image (linear interpolation)
		cvResize(img, resampledImg);
		img = resampledImg;
	}
	else if(img)
	{
		img = cvCloneImage(img);
	}

	return img;
}

