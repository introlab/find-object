/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "Camera.h"
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "Settings.h"
#include <QtCore/QFile>

Camera::Camera(QObject * parent) :
	QObject(parent)
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
	capture_.release();
}

void Camera::pause()
{
	stopTimer();
}

void Camera::takeImage()
{
	if(capture_.isOpened())
	{
		cv::Mat img;
		capture_.read(img);// capture a frame
		if(img.empty())
		{
			printf("Camera: Could not grab a frame, the end of the feed may be reached...\n");
		}
		else
		{
			//resize
			if( Settings::getCamera_2imageWidth() &&
				Settings::getCamera_3imageHeight() &&
				Settings::getCamera_2imageWidth() != img.cols &&
				Settings::getCamera_3imageHeight() != img.rows)
			{
				cv::Mat resampled;
				cv::resize(img, resampled, cv::Size(Settings::getCamera_2imageWidth(), Settings::getCamera_3imageHeight()));
				emit imageReceived(resampled);
			}
			else
			{
				emit imageReceived(img.clone()); // clone required
			}
		}
	}
}

bool Camera::start()
{
	if(!capture_.isOpened())
	{
		QString videoFile = Settings::getCamera_5videoFilePath();
		if(!videoFile.isEmpty())
		{
			capture_.open(videoFile.toStdString().c_str());
			if(!capture_.isOpened())
			{
				printf("WARNING: Cannot open file \"%s\". If you want to disable loading automatically this video file, clear the Camera/videoFilePath parameter. By default, webcam will be used instead of the file.\n", videoFile.toStdString().c_str());
			}
		}
		if(!capture_.isOpened())
		{
			//set camera device
			capture_.open(Settings::getCamera_1deviceId());
			if(Settings::getCamera_2imageWidth() && Settings::getCamera_3imageHeight())
			{
				capture_.set(CV_CAP_PROP_FRAME_WIDTH, double(Settings::getCamera_2imageWidth()));
				capture_.set(CV_CAP_PROP_FRAME_HEIGHT, double(Settings::getCamera_3imageHeight()));
			}
		}
	}
	if(!capture_.isOpened())
	{
		printf("Failed to open a capture object!\n");
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
	if(Settings::getCamera_4imageRate())
	{
		cameraTimer_.setInterval(1000/Settings::getCamera_4imageRate());
	}
	else
	{
		cameraTimer_.setInterval(0);
	}
}

