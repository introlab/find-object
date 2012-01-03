/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "Camera.h"
#include <stdio.h>
#include <opencv2/imgproc/imgproc_c.h>
#include "Settings.h"

Camera::Camera(QObject * parent) :
	QObject(parent),
	capture_(0)
{
	qRegisterMetaType<cv::Mat>("cv::Mat");
	connect(&cameraTimer_, SIGNAL(timeout()), this, SLOT(takeImage()));
}

Camera::~Camera()
{
	this->stop();
}

void Camera::stop()
{
	stopTimer();
	if(capture_)
	{
		cvReleaseCapture(&capture_);
		capture_ = 0;
	}
}

void Camera::takeImage()
{
	if(capture_)
	{
		IplImage * img = 0;
		if(cvGrabFrame(capture_)) // capture a frame
		{
			img = cvRetrieveFrame(capture_); // retrieve the captured frame
		}
		else
		{
			printf("CameraVideo: Could not grab a frame, the end of the feed may be reached...\n");
		}

		//resize
		if(img &&
			Settings::getCamera_imageWidth().toInt() &&
			Settings::getCamera_imageHeight().toInt() &&
			Settings::getCamera_imageWidth().toInt() != img->width &&
			Settings::getCamera_imageHeight().toInt() != img->height)
		{
			// declare a destination IplImage object with correct size, depth and channels
			cv::Mat headerImg = img;
			cv::Mat imgMat(Settings::getCamera_imageHeight().toInt(),
						   Settings::getCamera_imageWidth().toInt(),
						   headerImg.type());

			//use cvResize to resize source to a destination image (linear interpolation)
			IplImage resampledImg = imgMat;
			cvResize(img, &resampledImg);
			emit imageReceived(imgMat);
		}
		else
		{
			emit imageReceived(cv::Mat(img, true));
		}
	}
}

bool Camera::start()
{
	if(!capture_)
	{
		capture_ = cvCaptureFromCAM(Settings::getCamera_deviceId().toInt());
		if(capture_)
		{
			cvSetCaptureProperty(capture_, CV_CAP_PROP_FRAME_WIDTH, double(Settings::getCamera_imageWidth().toInt()));
			cvSetCaptureProperty(capture_, CV_CAP_PROP_FRAME_HEIGHT, double(Settings::getCamera_imageHeight().toInt()));
		}
	}
	if(!capture_)
	{
		printf("Failed to create a capture object!\n");
		return false;
	}

	startTimer();
	return true;
}

void Camera::startTimer()
{
	updateImageRate();
	cameraTimer_.start();
}

void Camera::stopTimer()
{
	cameraTimer_.stop();
}

void Camera::updateImageRate()
{
	if(Settings::getCamera_imageRate().toInt())
	{
		cameraTimer_.setInterval(1000/Settings::getCamera_imageRate().toInt());
	}
	else
	{
		cameraTimer_.setInterval(0);
	}
}

