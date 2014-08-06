/*
 * CameraTcpClient.h
 *
 *  Created on: 2014-07-31
 *      Author: mathieu
 */

#ifndef CAMERATCPCLIENT_H_
#define CAMERATCPCLIENT_H_

#include <QtNetwork/QTcpServer>
#include <opencv2/opencv.hpp>

class CameraTcpServer : public QTcpServer
{
	Q_OBJECT;
public:
	CameraTcpServer(quint16 port = 0, QObject * parent = 0);
	cv::Mat getImage();
	int imagesBuffered() const {return images_.size();}
	bool isConnected() const;

	QHostAddress getHostAddress() const;
	quint16 getPort() const;

protected:
	virtual void incomingConnection ( int socketDescriptor );

private Q_SLOTS:
	void readReceivedData();
	void displayError(QAbstractSocket::SocketError socketError);
	void connectionLost();

private:
	quint64 blockSize_;
	QVector<cv::Mat> images_;
};

#endif /* CAMERATCPCLIENT_H_ */
