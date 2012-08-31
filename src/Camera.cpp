/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "Camera.h"
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "Settings.h"
#include <QtCore/QFile>
#include "utilite/UDirectory.h"

Camera::Camera(QObject * parent) :
	QObject(parent),
	currentImageIndex_(0)
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
	images_.clear();
	currentImageIndex_ = 0;
}

void Camera::pause()
{
	stopTimer();
}

int Camera::getTotalFrames()
{
	if(images_.size())
	{
		return images_.size();
	}
	else if(capture_.isOpened())
	{
		return (int)capture_.get(CV_CAP_PROP_FRAME_COUNT);
	}
	return 0;
}

int Camera::getCurrentFrameIndex()
{
	if(images_.size())
	{
		return currentImageIndex_;
	}
	else if(capture_.isOpened())
	{
		return (int)capture_.get(CV_CAP_PROP_POS_FRAMES);
	}
	return 0;
}

void Camera::moveToFrame(int frame)
{
	printf("Moved to frame %d.\n", frame);
	if(frame < images_.size())
	{
		currentImageIndex_ = frame;
	}
	else if(capture_.isOpened() && frame < (int)capture_.get(CV_CAP_PROP_FRAME_COUNT))
	{
		capture_.set(CV_CAP_PROP_POS_FRAMES, frame);
	}
}

void Camera::takeImage()
{
	cv::Mat img;
	if(capture_.isOpened())
	{
		capture_.read(img);// capture a frame
	}
	else if(!images_.empty())
	{
		if(currentImageIndex_ < (unsigned int)images_.size())
		{
			img = cv::imread(images_[currentImageIndex_++]);
		}
	}

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
		else if(capture_.isOpened())
		{
			emit imageReceived(img.clone()); // clone required with VideoCapture::read()
		}
		else
		{
			emit imageReceived(img); // clone not required with cv::imread()
		}
	}
}

bool Camera::start()
{
	if(!capture_.isOpened() && images_.empty())
	{
		QString path = Settings::getCamera_5mediaPath();
		if(UDirectory::exists(path.toStdString()))
		{
			//Images directory
			QString ext = Settings::getGeneral_imageFormats();
			ext.remove('*');
			ext.remove('.');
			UDirectory dir(path.toStdString(), ext.toStdString()); // this will load fileNames matching the extensions (in natural order)
			const std::list<std::string> & fileNames = dir.getFileNames();
			currentImageIndex_ = 0;
			images_.clear();
			// Modify to have full path
			for(std::list<std::string>::const_iterator iter = fileNames.begin(); iter!=fileNames.end(); ++iter)
			{
				images_.append(path.toStdString() + UDirectory::separator() + *iter);
			}
			printf("Loaded %d filenames.\n", (int)images_.size());
			if(images_.isEmpty())
			{
				printf("WARNING: Directory \"%s\" is empty (no images matching the \"%s\" extensions). "
					   "If you want to disable loading automatically this directory, "
					   "clear the Camera/mediaPath parameter. By default, webcam will be used instead of the directory.\n",
					   path.toStdString().c_str(),
					   ext.toStdString().c_str());
			}
		}
		else if(!path.isEmpty())
		{
			//Video file
			capture_.open(path.toStdString().c_str());
			if(!capture_.isOpened())
			{
				printf("WARNING: Cannot open file \"%s\". If you want to disable loading automatically this video file, clear the Camera/mediaPath parameter. By default, webcam will be used instead of the file.\n", path.toStdString().c_str());
			}
		}
		if(!capture_.isOpened() && images_.empty())
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
	if(!capture_.isOpened() && images_.empty())
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
		cameraTimer_.setInterval((int)(1000.0/Settings::getCamera_4imageRate()));
	}
	else
	{
		cameraTimer_.setInterval(0);
	}
}

