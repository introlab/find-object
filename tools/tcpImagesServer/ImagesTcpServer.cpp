/*
 * ImagesTcpServer.cpp
 *
 *  Created on: 2014-05-21
 *      Author: mathieu
 */

#include "find_object/Settings.h"
#include "find_object/QtOpenCV.h"

#include "ImagesTcpServer.h"

#include <QtNetwork/QNetworkInterface>
#include <QtNetwork/QTcpSocket>
#include <QtGui/QTransform>

ImagesTcpServer::ImagesTcpServer(float hz, const QString & path, QObject * parent) :
	QTcpServer(parent)
{
	// Set camera parameters
	Settings::setCamera_4imageRate(hz);
	Settings::setCamera_5mediaPath(path);

	connect(this, SIGNAL(newConnection()), this, SLOT(addClient()));
	connect(&camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(publishImage(const cv::Mat &)));
}

QHostAddress ImagesTcpServer::getHostAddress() const
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

quint16 ImagesTcpServer::getPort() const
{
	return this->serverPort();
}

void ImagesTcpServer::publishImage(const cv::Mat & image)
{
	QList<QTcpSocket*> clients = this->findChildren<QTcpSocket*>();
	if(clients.size())
	{
		std::vector<unsigned char> buf;
		cv::imencode(".png", image, buf);

		for(QList<QTcpSocket*>::iterator iter = clients.begin(); iter!=clients.end(); ++iter)
		{
			QByteArray block;
			QDataStream out(&block, QIODevice::WriteOnly);
			out.setVersion(QDataStream::Qt_4_0);
			out << (quint64)0;
			out.writeRawData((char*)buf.data(), (int)buf.size());
			out.device()->seek(0);
			out << (quint64)(block.size() - sizeof(quint64));
			(*iter)->write(block);
		}
	}
	else
	{
		printf("Paused...\n");
		camera_.pause();
	}
}

void ImagesTcpServer::addClient()
{
	QTcpSocket * client = this->nextPendingConnection();
	connect(client, SIGNAL(disconnected()), client, SLOT(deleteLater()));
	if(!camera_.isRunning())
	{
		printf("Start...\n");
		camera_.start();
	}
}
