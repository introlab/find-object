/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "find_object/Camera.h"
#include "find_object/Settings.h"
#include "find_object/utilite/ULogger.h"
#include "find_object/QtOpenCV.h"

#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <QtCore/QFile>
#include "utilite/UDirectory.h"
#include "CameraTcpServer.h"

namespace find_object {

Camera::Camera(QObject * parent) :
	QObject(parent),
	currentImageIndex_(0),
	cameraTcpServer_(0)
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
	if(cameraTcpServer_)
	{
		cameraTcpServer_->close();
		delete cameraTcpServer_;
		cameraTcpServer_ = 0;
	}
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
	if(frame < images_.size())
	{
		currentImageIndex_ = frame;
	}
	else if(capture_.isOpened() && frame < (int)capture_.get(CV_CAP_PROP_FRAME_COUNT))
	{
		capture_.set(CV_CAP_PROP_POS_FRAMES, frame);
	}
}

int Camera::getPort()
{
	if(cameraTcpServer_)
	{
		return cameraTcpServer_->getPort();
	}
	return 0;
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
	else if(cameraTcpServer_)
	{
		img = cameraTcpServer_->getImage();
		if(cameraTcpServer_->imagesBuffered() > 0 && Settings::getCamera_9queueSize() == 0)
		{
			UWARN("%d images buffered so far...", cameraTcpServer_->imagesBuffered());
		}
	}

	if(img.empty())
	{
		if(cameraTcpServer_)
		{
			if(!cameraTcpServer_->isConnected())
			{
				cameraTcpServer_->waitForNewConnection(100);
			}
		}
		else
		{
			// In case of a directory of images or a video
			this->stop();
			Q_EMIT finished(); // notify that there are no more images
		}
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
			Q_EMIT imageReceived(resampled);
		}
		else if(capture_.isOpened())
		{
			Q_EMIT imageReceived(img.clone()); // clone required with VideoCapture::read()
		}
		else
		{
			Q_EMIT imageReceived(img); // clone not required with cv::imread()
		}
	}
}

bool Camera::start()
{
	if(!capture_.isOpened() && images_.empty() && cameraTcpServer_ == 0)
	{
		if(Settings::getCamera_6useTcpCamera())
		{
			cameraTcpServer_ = new CameraTcpServer(Settings::getCamera_8port(), this);
			if(!cameraTcpServer_->isListening())
			{
				UWARN("CameraTCP: Cannot listen to port %d", cameraTcpServer_->getPort());
				delete cameraTcpServer_;
				cameraTcpServer_ = 0;
			}
			else
			{
				UINFO("CameraTCP: listening to port %d (IP=%s)",
						cameraTcpServer_->getPort(),
						cameraTcpServer_->getHostAddress().toString().toStdString().c_str());
			}
		}
		else
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
				UINFO("Camera: Reading %d images from directory \"%s\"...", (int)images_.size(), path.toStdString().c_str());
				if(images_.isEmpty())
				{
					UWARN("Camera: Directory \"%s\" is empty (no images matching the \"%s\" extensions). "
						   "If you want to disable loading automatically this directory, "
						   "clear the Camera/mediaPath parameter. By default, webcam will be used instead of the directory.",
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
					UWARN("Camera: Cannot open file \"%s\". If you want to disable loading "
						  "automatically this video file, clear the Camera/mediaPath parameter. "
						  "By default, webcam will be used instead of the file.", path.toStdString().c_str());
				}
				else
				{
					UINFO("Camera: Reading from video file \"%s\"...", path.toStdString().c_str());
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
				UINFO("Camera: Reading from camera device %d...", Settings::getCamera_1deviceId());
			}
		}
	}
	if(!capture_.isOpened() && images_.empty() && cameraTcpServer_ == 0)
	{
		UERROR("Camera: Failed to open a capture object!");
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

} // namespace find_object

