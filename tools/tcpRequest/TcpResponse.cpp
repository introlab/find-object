/*
 * TcpResponse.cpp
 *
 *  Created on: 2014-05-05
 *      Author: mathieu
 */

#include "TcpResponse.h"

#include <opencv2/opencv.hpp>
#include <QtGui/QTransform>
#include <QtCore/QPointF>
#include <QtCore/QTime>

TcpResponse::TcpResponse(QObject *parent) :
	QTcpSocket(parent),
    blockSize_(0),
    dataReceived_(false)
{
	connect(this, SIGNAL(readyRead()), this, SLOT(readReceivedData()));
	connect(this, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(displayError(QAbstractSocket::SocketError)));
	connect(this, SIGNAL(disconnected()), this, SLOT(connectionLost()));
}

void TcpResponse::readReceivedData()
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

	objectsDetected_.clear();
	for(int i=0; i<data.size(); i+=12)
	{
		// get data
		int id = (int)data[i];
		float width = data[i+1];
		float height = data[i+2];

		// Find corners Qt
		QTransform homography(data[i+3], data[i+4], data[i+5],
		data[i+6], data[i+7], data[i+8],
		data[i+9], data[i+10], data[i+11]);

		objectsDetected_.insert(id, QPair<QRect, QTransform>(QRect(0,0,width, height), homography));
	}

	dataReceived_ = true;
	Q_EMIT detectionReceived();
}

void TcpResponse::displayError(QAbstractSocket::SocketError socketError)
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

void TcpResponse::connectionLost()
{
	printf("Connection lost!\n");
}

