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
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
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
	QTcpSocket * client = this->nextPendingConnection();
	connect(client, SIGNAL(disconnected()), client, SLOT(deleteLater()));
}

} // namespace find_object
