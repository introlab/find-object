/*
 * CameraTcpClient.cpp
 *
 *  Created on: 2014-07-31
 *      Author: mathieu
 */

#include "CameraTcpClient.h"
#include "Settings.h"
#include "utilite/ULogger.h"

CameraTcpClient::CameraTcpClient(QObject *parent) :
	QTcpSocket(parent),
    blockSize_(0)
{
	connect(this, SIGNAL(readyRead()), this, SLOT(readReceivedData()));
	connect(this, SIGNAL(error(QAbstractSocket::SocketError)), this, SLOT(displayError(QAbstractSocket::SocketError)));
	connect(this, SIGNAL(disconnected()), this, SLOT(connectionLost()));
}

cv::Mat CameraTcpClient::getImage()
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
	return img;
}

void CameraTcpClient::readReceivedData()
{
	QDataStream in(this);
	in.setVersion(QDataStream::Qt_4_0);

	if (blockSize_ == 0)
	{
		if (this->bytesAvailable() < (int)sizeof(quint64))
		{
			return;
		}

		in >> blockSize_;
	}

	if (this->bytesAvailable() < (int)blockSize_)
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

void CameraTcpClient::displayError(QAbstractSocket::SocketError socketError)
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
			UERROR("The following error occurred: %s.", this->errorString().toStdString().c_str());
			break;
	}
}

void CameraTcpClient::connectionLost()
{
	//printf("[WARNING] CameraTcp: Connection lost!\n");
}
