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
#include "find_object/TcpServer.h"
#include "find_object/utilite/ULogger.h"

#include <QtNetwork/QNetworkInterface>
#include <QtNetwork/QTcpSocket>
#include <QtGui/QTransform>
#include <opencv2/highgui/highgui.hpp>

namespace find_object {

TcpServer::TcpServer(quint16 port, QObject * parent) :
	QTcpServer(parent)
{
	if (!this->listen(QHostAddress::Any, port))
	{
		UERROR("Unable to start the TCP server: %s", this->errorString().toStdString().c_str());
		return;
	}

	connect(this, SIGNAL(newConnection()), this, SLOT(addClient()));
}

QHostAddress TcpServer::getHostAddress() const
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

quint16 TcpServer::getPort() const
{
	return this->serverPort();
}

void TcpServer::publishDetectionInfo(const DetectionInfo & info)
{
	QList<QTcpSocket*> clients = this->findChildren<QTcpSocket*>();
	if(clients.size())
	{
		QByteArray block;
		QDataStream out(&block, QIODevice::WriteOnly);
		out.setVersion(QDataStream::Qt_4_0);
		out << (quint16)0;

		out << info;

		out.device()->seek(0);
		out << (quint16)(block.size() - sizeof(quint16));

		for(QList<QTcpSocket*>::iterator iter = clients.begin(); iter!=clients.end(); ++iter)
		{
			(*iter)->write(block);
		}
	}
}

void TcpServer::addClient()
{
	while(this->hasPendingConnections())
	{
		QTcpSocket * client =  this->nextPendingConnection();
		connect(client, SIGNAL(readyRead()), this, SLOT(readReceivedData()));
		connect(client, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(displayError(QAbstractSocket::SocketError)));
		connect(client, SIGNAL(disconnected()), this, SLOT(connectionLost()));
	}
}

void TcpServer::readReceivedData()
{
	QTcpSocket * client = (QTcpSocket*)sender();
	QDataStream in(client);
	in.setVersion(QDataStream::Qt_4_0);

	if (blockSizes_.value(client->socketDescriptor(), 0) == 0)
	{
		if (client->bytesAvailable() < (int)sizeof(quint64))
		{
			return;
		}

		in >> blockSizes_[client->socketDescriptor()];
	}

	if (client->bytesAvailable() < (int)blockSizes_[client->socketDescriptor()])
	{
		return;
	}

	quint32 serviceType;
	in >> serviceType;

	bool ok = true;
	if(serviceType == kAddObject)
	{
		int id;
		in >> id;
		QString fileName;
		in >> fileName;
		quint64 imageSize;
		in >> imageSize;
		std::vector<unsigned char> buf(imageSize);
		in.readRawData((char*)buf.data(), imageSize);
		cv::Mat image = cv::imdecode(buf, cv::IMREAD_UNCHANGED);

		UINFO("TCP service: Add %d \"%s\"", id, fileName.toStdString().c_str());
		Q_EMIT addObject(image, id, fileName);
	}
	else if(serviceType == kRemoveObject)
	{
		int id;
		in >> id;

		UINFO("TCP service: Remove %d", id);
		Q_EMIT removeObject(id);
	}
	else
	{
		UERROR("Unknown service type called %d", serviceType);
		ok = false;
	}

	blockSizes_.remove(client->socketDescriptor());
	client->write(QByteArray(ok?"1":"0")); // send acknowledge
}

void TcpServer::displayError(QAbstractSocket::SocketError socketError)
{
	switch (socketError)
	{
		case QAbstractSocket::RemoteHostClosedError:
			break;
		case QAbstractSocket::HostNotFoundError:
			UWARN("CameraTcp: Tcp error: The host was not found. Please "
					"check the host name and port settings.\n");
			break;
		case QAbstractSocket::ConnectionRefusedError:
			UWARN("CameraTcp: The connection was refused by the peer. "
					"Make sure your images server is running, "
					"and check that the host name and port "
					"settings are correct.");
			break;
		default:
			//UERROR("The following error occurred: %s.", this->errorString().toStdString().c_str());
			break;
	}
}

void TcpServer::connectionLost()
{
	//printf("[WARNING] CameraTcp: Connection lost!\n");
	blockSizes_.remove(((QTcpSocket*)sender())->socketDescriptor());
	((QTcpSocket*)sender())->close();
	sender()->deleteLater();
}

} // namespace find_object
