/*
 * TCPServer.cpp
 *
 *  Created on: 2014-05-05
 *      Author: mathieu
 */

#include "TcpServer.h"
#include "utilite/ULogger.h"

#include <QtNetwork/QNetworkInterface>
#include <QtNetwork/QTcpSocket>
#include <QtGui/QTransform>

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

void TcpServer::publishObjects(const QMultiMap<int, QPair<QRect, QTransform> > & objects)
{
	QList<QTcpSocket*> clients = this->findChildren<QTcpSocket*>();
	if(clients.size())
	{
		QVector<float> data(objects.size()*12);
		int i=0;
		for(QMultiMap<int, QPair<QRect, QTransform> >::const_iterator iter=objects.constBegin(); iter!=objects.constEnd(); ++iter)
		{
			data[i++] = iter.key();
			data[i++] = iter.value().first.width();
			data[i++] = iter.value().first.height();
			data[i++] = iter.value().second.m11();
			data[i++] = iter.value().second.m12();
			data[i++] = iter.value().second.m13();
			data[i++] = iter.value().second.m21();
			data[i++] = iter.value().second.m22();
			data[i++] = iter.value().second.m23();
			data[i++] = iter.value().second.m31(); // dx
			data[i++] = iter.value().second.m32(); // dy
			data[i++] = iter.value().second.m33();
		}

		for(QList<QTcpSocket*>::iterator iter = clients.begin(); iter!=clients.end(); ++iter)
		{
			QByteArray block;
			QDataStream out(&block, QIODevice::WriteOnly);
			out.setVersion(QDataStream::Qt_4_0);
			out << (quint16)0;
			out << data;
			out.device()->seek(0);
			out << (quint16)(block.size() - sizeof(quint16));
			(*iter)->write(block);
		}
	}
}

void TcpServer::addClient()
{
	QTcpSocket * client = this->nextPendingConnection();
	connect(client, SIGNAL(disconnected()), client, SLOT(deleteLater()));
}
