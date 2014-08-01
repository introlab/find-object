/*
 * TCPClient.cpp
 *
 *  Created on: 2014-05-05
 *      Author: mathieu
 */

#include "TcpClient.h"

#include <opencv2/opencv.hpp>
#include <QtGui/QTransform>
#include <QtCore/QPointF>
#include <QtCore/QTime>

TcpClient::TcpClient(const QString & hostname, quint16 port, QObject *parent) :
	QTcpSocket(parent),
    blockSize_(0)
{
	connect(this, SIGNAL(readyRead()), this, SLOT(readReceivedData()));
	connect(this, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(displayError(QAbstractSocket::SocketError)));
	connect(this, SIGNAL(disconnected()), this, SLOT(connectionLost()));

	this->connectToHost(hostname, port);
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

	QVector<float> data;
	in >> data;

	printf("---\n");
	if(data.size() == 0)
	{
		printf("(%s) No objects detected.\n",
				QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str());
	}
	else
	{
		for(int i=0; i<data.size(); i+=12)
		{
			// get data
			int id = (int)data[i];
			float objectWidth = data[i+1];
			float objectHeight = data[i+2];

			// Find corners Qt
			QTransform qtHomography(data[i+3], data[i+4], data[i+5],
			data[i+6], data[i+7], data[i+8],
			data[i+9], data[i+10], data[i+11]);

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
				cvHomography.at<float>(0,0) = data[i+3];
				cvHomography.at<float>(1,0) = data[i+4];
				cvHomography.at<float>(2,0) = data[i+5];
				cvHomography.at<float>(0,1) = data[i+6];
				cvHomography.at<float>(1,1) = data[i+7];
				cvHomography.at<float>(2,1) = data[i+8];
				cvHomography.at<float>(0,2) = data[i+9];
				cvHomography.at<float>(1,2) = data[i+10];
				cvHomography.at<float>(2,2) = data[i+11];
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

