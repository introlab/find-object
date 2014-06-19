/*
 * ImagesTcpServer.h
 *
 *  Created on: 2014-05-21
 *      Author: mathieu
 */

#ifndef TCPCLIENT_H_
#define TCPCLIENT_H_

#include "Camera.h"
#include <QtNetwork/QTcpServer>

class ImagesTcpServer : public QTcpServer
{
	Q_OBJECT

public:
	ImagesTcpServer(float hz = 10.0f, quint16 port = 0, QObject * parent = 0);

	QHostAddress getHostAddress() const;
	quint16 getPort() const;


private slots:
	void addClient();
	void publishImage(const cv::Mat & image);

private:
	Camera camera_;
};

#endif /* TCPCLIENT_H_ */
