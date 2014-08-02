/*
 * CameraTcpClient.cpp
 *
 *  Created on: 2014-07-31
 *      Author: mathieu
 */

#include "find_object/Settings.h"
#include "find_object/utilite/ULogger.h"

#include "CameraTcpServer.h"
#include <QtNetwork/QTcpSocket>
#include <QtNetwork/QNetworkInterface>
#include <QtCore/QDataStream>

CameraTcpServer::CameraTcpServer(quint16 port, QObject *parent) :
	QTcpServer(parent),
    blockSize_(0)
{
	this->setMaxPendingConnections(1);
	if (!this->listen(QHostAddress::Any, port))
	{
		UERROR("Unable to start the Camera TCP server: %s", this->errorString().toStdString().c_str());
		return;
	}

	connect(this, SIGNAL(newConnection()), this, SLOT(addClient()));
}

cv::Mat CameraTcpServer::getImage()
{
	cv::Mat img;
	if(images_.size())
	{
		// if queue changed after tcp connection ended with images still in the buffer
		int queue = Settings::getCamera_9queueSize();
		while(queue > 0 && images_.size() > queue)
		{
			images_.pop_front();
		}

		img = images_.front();
		images_.pop_front();
	}
	if(this->findChildren<QTcpSocket*>().size() == 1)
	{
		this->findChildren<QTcpSocket*>().first()->waitForReadyRead(100);
	}
	return img;
}

bool CameraTcpServer::isConnected() const
{
	return this->findChildren<QTcpSocket*>().size() > 0;
}

QHostAddress CameraTcpServer::getHostAddress() const
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

quint16 CameraTcpServer::getPort() const
{
	return this->serverPort();
}

void CameraTcpServer::addClient()
{
	QTcpSocket * client = this->nextPendingConnection();

	QList<QTcpSocket*> clients = this->findChildren<QTcpSocket*>();
	if(clients.size() > 1)
	{
		UWARN("A client is already connected. Only one connection allowed at the same time.");
		client->close();
		client->deleteLater();
	}
	else
	{
		connect(client, SIGNAL(readyRead()), this, SLOT(readReceivedData()));
		connect(client, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(displayError(QAbstractSocket::SocketError)));
		connect(client, SIGNAL(disconnected()), this, SLOT(connectionLost()));
	}
}

void CameraTcpServer::readReceivedData()
{
	QTcpSocket * client = (QTcpSocket*)sender();
	QDataStream in(client);
	in.setVersion(QDataStream::Qt_4_0);

	if (blockSize_ == 0)
	{
		if (client->bytesAvailable() < (int)sizeof(quint64))
		{
			return;
		}

		in >> blockSize_;
	}

	if (client->bytesAvailable() < (int)blockSize_)
	{
		return;
	}

	std::vector<unsigned char> buf(blockSize_);
	in.readRawData((char*)buf.data(), blockSize_);
	images_.push_back(cv::imdecode(buf, cv::IMREAD_UNCHANGED));
	int queue = Settings::getCamera_9queueSize();
	while(queue > 0 && images_.size() > queue)
	{
		images_.pop_front();
	}
	blockSize_ = 0;
}

void CameraTcpServer::displayError(QAbstractSocket::SocketError socketError)
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

void CameraTcpServer::connectionLost()
{
	//printf("[WARNING] CameraTcp: Connection lost!\n");
	((QTcpSocket*)sender())->close();
	sender()->deleteLater();
	blockSize_ = 0; // reset
}
