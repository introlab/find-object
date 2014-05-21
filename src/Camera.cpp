/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "Camera.h"
#include <stdio.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "Settings.h"
#include <QtCore/QFile>
#include "utilite/UDirectory.h"
#include "QtOpenCV.h"

CameraTcpClient::CameraTcpClient(QObject *parent) :
	QTcpSocket(parent),
    blockSize_(0)
{
	connect(this, SIGNAL(readyRead()), this, SLOT(readData()));
	connect(this, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(displayError(QAbstractSocket::SocketError)));
	connect(this, SIGNAL(disconnected()), this, SLOT(connectionLost()));
}

cv::Mat CameraTcpClient::getImage()
{
	cv::Mat img = image_;
	image_ = cv::Mat();
	return img;
}

void CameraTcpClient::readData()
{
	QDataStream in(this);
	in.setVersion(QDataStream::Qt_4_0);

	if (blockSize_ == 0)
	{
		if (this->bytesAvailable() < (int)sizeof(quint64))
		{
			return;
		}

		in >> blockSize_;
	}

	if (this->bytesAvailable() < (int)blockSize_)
	{
		return;
	}

	std::vector<unsigned char> buf(blockSize_);
	in.readRawData((char*)buf.data(), blockSize_);
	image_ = cv::imdecode(buf, cv::IMREAD_UNCHANGED);
	blockSize_ = 0;
}

void CameraTcpClient::displayError(QAbstractSocket::SocketError socketError)
{
	switch (socketError)
	{
		case QAbstractSocket::RemoteHostClosedError:
			break;
		case QAbstractSocket::HostNotFoundError:
			printf("[ERROR] CameraTcp: Tcp error: The host was not found. Please "
					"check the host name and port settings.\n");
			break;
		case QAbstractSocket::ConnectionRefusedError:
			printf("[ERROR] CameraTcp: The connection was refused by the peer. "
					"Make sure your images server is running, "
					"and check that the host name and port "
					"settings are correct.\n");
			break;
		default:
			printf("The following error occurred: %s.\n", this->errorString().toStdString().c_str());
			break;
	}
}

void CameraTcpClient::connectionLost()
{
	printf("[WARNING] CameraTcp: Connection lost!\n");
}

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
	cameraTcpClient_.close();
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
	else
	{
		img = cameraTcpClient_.getImage();
		while(img.empty() && cameraTcpClient_.waitForReadyRead())
		{
			img = cameraTcpClient_.getImage();
		}
		if(!cameraTcpClient_.waitForConnected())
		{
			printf("Connection is lost, try reconnecting to server (%s:%d)...\n",
					Settings::getCamera_7IP().toStdString().c_str(),
					Settings::getCamera_8port());
			cameraTcpClient_.connectToHost(Settings::getCamera_7IP(), Settings::getCamera_8port());
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
	if(!capture_.isOpened() && images_.empty() && !cameraTcpClient_.isOpen())
	{
		if(Settings::getCamera_6useTcpCamera())
		{
			cameraTcpClient_.connectToHost(Settings::getCamera_7IP(), Settings::getCamera_8port());
			if(!cameraTcpClient_.waitForConnected())
			{
				printf("[WARNING] Camera: Cannot connect to server \"%s:%d\"\n",
						Settings::getCamera_7IP().toStdString().c_str(),
						Settings::getCamera_8port());
				cameraTcpClient_.close();
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
				printf("Camera: Reading %d images from directory \"%s\"...\n", (int)images_.size(), path.toStdString().c_str());
				if(images_.isEmpty())
				{
					printf("[WARNING] Camera: Directory \"%s\" is empty (no images matching the \"%s\" extensions). "
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
					printf("[WARNING] Camera: Cannot open file \"%s\". If you want to disable loading automatically this video file, clear the Camera/mediaPath parameter. By default, webcam will be used instead of the file.\n", path.toStdString().c_str());
				}
				else
				{
					printf("Camera: Reading from video file \"%s\"...\n", path.toStdString().c_str());
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
				printf("Camera: Reading from camera device %d...\n", Settings::getCamera_1deviceId());
			}
		}
	}
	if(!capture_.isOpened() && images_.empty() && !cameraTcpClient_.isOpen())
	{
		printf("[ERROR] Camera: Failed to open a capture object!\n");
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

