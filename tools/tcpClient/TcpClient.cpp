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

#include "TcpClient.h"
#include "find_object/DetectionInfo.h"
#include <opencv2/opencv.hpp>
#include <QtGui/QTransform>
#include <QtCore/QPointF>
#include <QtCore/QTime>

TcpClient::TcpClient(QObject *parent) :
	QTcpSocket(parent),
    blockSize_(0)
{
	connect(this, SIGNAL(readyRead()), this, SLOT(readReceivedData()));
	connect(this, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(displayError(QAbstractSocket::SocketError)));
	connect(this, SIGNAL(disconnected()), this, SLOT(connectionLost()));
}

void TcpClient::readReceivedData()
{
	QDataStream in(this);
	in.setVersion(QDataStream::Qt_4_0);

	if (blockSize_ == 0)
	{
		if (this->bytesAvailable() < (int)sizeof(quint16))
		{
			return;
		}

		in >> blockSize_;
	}

	if (this->bytesAvailable() < blockSize_)
	{
		return;
	}

	blockSize_ = 0;

	find_object::DetectionInfo info;
	in >> info;

	printf("---\n");
	if(info.objDetected_.size() == 0)
	{
		printf("(%s) No objects detected.\n",
				QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str());
	}
	else
	{
		QMultiMap<int, QSize>::const_iterator iterSizes = info.objDetectedSizes_.constBegin();
		for(QMultiMap<int, QTransform>::const_iterator iter=info.objDetected_.constBegin();
			iter!=info.objDetected_.constEnd();
			++iter)
		{
			// get data
			int id = (int)iter.key();
			float objectWidth = iterSizes.value().width();
			float objectHeight = iterSizes.value().height();

			// Find corners Qt
			QTransform qtHomography = iter.value();

			QPointF qtTopLeft = qtHomography.map(QPointF(0,0));
			QPointF qtTopRight = qtHomography.map(QPointF(objectWidth,0));
			QPointF qtBottomLeft = qtHomography.map(QPointF(0,objectHeight));
			QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight));

			printf("(%s) Object %d detected, Qt corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
					QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
					id,
					qtTopLeft.x(), qtTopLeft.y(),
					qtTopRight.x(), qtTopRight.y(),
					qtBottomLeft.x(), qtBottomLeft.y(),
					qtBottomRight.x(), qtBottomRight.y());

			// Example with OpenCV
			if(0)
			{
				// Find corners OpenCV
				cv::Mat cvHomography(3, 3, CV_32F);
				cvHomography.at<float>(0,0) = qtHomography.m11();
				cvHomography.at<float>(1,0) = qtHomography.m12();
				cvHomography.at<float>(2,0) = qtHomography.m13();
				cvHomography.at<float>(0,1) = qtHomography.m21();
				cvHomography.at<float>(1,1) = qtHomography.m22();
				cvHomography.at<float>(2,1) = qtHomography.m23();
				cvHomography.at<float>(0,2) = qtHomography.m31();
				cvHomography.at<float>(1,2) = qtHomography.m32();
				cvHomography.at<float>(2,2) = qtHomography.m33();
				std::vector<cv::Point2f> inPts, outPts;
				inPts.push_back(cv::Point2f(0,0));
				inPts.push_back(cv::Point2f(objectWidth,0));
				inPts.push_back(cv::Point2f(0,objectHeight));
				inPts.push_back(cv::Point2f(objectWidth,objectHeight));
				cv::perspectiveTransform(inPts, outPts, cvHomography);

				printf("(%s) Object %d detected, CV corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
						QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
						id,
						outPts.at(0).x, outPts.at(0).y,
						outPts.at(1).x, outPts.at(1).y,
						outPts.at(2).x, outPts.at(2).y,
						outPts.at(3).x, outPts.at(3).y);
			}

			++iterSizes;
		}
	}
}

void TcpClient::displayError(QAbstractSocket::SocketError socketError)
{
	switch (socketError)
	{
		case QAbstractSocket::RemoteHostClosedError:
			break;
		case QAbstractSocket::HostNotFoundError:
			printf("Tcp error: The host was not found. Please "
					"check the host name and port settings.\n");
			break;
		case QAbstractSocket::ConnectionRefusedError:
			printf("The connection was refused by the peer. "
					"Make sure Find-Object is running, "
					"and check that the host name and port "
					"settings are correct.\n");
			break;
		default:
			printf("The following error occurred: %s.\n", this->errorString().toStdString().c_str());
			break;
	}
}

void TcpClient::connectionLost()
{
	printf("Connection lost!\n");
}

