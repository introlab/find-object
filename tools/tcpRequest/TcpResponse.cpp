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

	in >> info_;

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

