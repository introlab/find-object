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
#include "find_object/Settings.h"
#include "find_object/QtOpenCV.h"

#include "ImagesTcpServer.h"

#include <QtNetwork/QNetworkInterface>
#include <QtNetwork/QTcpSocket>
#include <QtGui/QTransform>

ImagesTcpServer::ImagesTcpServer(float hz, const QString & path, QObject * parent) :
	QTcpSocket(parent)
{
	// Set camera parameters
	find_object::Settings::setCamera_4imageRate(hz);
	find_object::Settings::setCamera_5mediaPath(path);

	connect(&camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(publishImage(const cv::Mat &)));
	connect(this, SIGNAL(connected()), this, SLOT(startCamera()));
}

QHostAddress ImagesTcpServer::getHostAddress()
{
	QHostAddress hostAddress;

	QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
	// use the first non-localhost IPv4 address
	for (int i = 0; i < ipAddressesList.size(); ++i)
	{
		if (ipAddressesList.at(i) != QHostAddress::LocalHost && ipAddressesList.at(i).toIPv4Address())
		{
			hostAddress = ipAddressesList.at(i).toString();
			break;
		}
	}

	// if we did not find one, use IPv4 localhost
	if (hostAddress.isNull())
	{
		hostAddress = QHostAddress(QHostAddress::LocalHost);
	}

	return hostAddress;
}

void ImagesTcpServer::publishImage(const cv::Mat & image)
{
	if(image.empty())
	{
		printf("No more images...\n");
		camera_.pause();
		Q_EMIT connectionLost();
	}
	else
	{
		if(this->waitForConnected())
		{
			std::vector<unsigned char> buf;
			cv::imencode(".png", image, buf);

			QByteArray block;
			QDataStream out(&block, QIODevice::WriteOnly);
			out.setVersion(QDataStream::Qt_4_0);
			out << (quint64)0;
			out.writeRawData((char*)buf.data(), (int)buf.size());
			out.device()->seek(0);
			out << (quint64)(block.size() - sizeof(quint64));
			this->write(block);

		}
		else
		{
			printf("Lost connection...\n");
			camera_.pause();
			Q_EMIT connectionLost();
		}
	}
}

void ImagesTcpServer::startCamera()
{
	if(!camera_.isRunning())
	{
		printf("Start...\n");
		camera_.start();
	}
}
