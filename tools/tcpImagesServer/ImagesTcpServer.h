/*
 * ImagesTcpServer.h
 *
 *  Created on: 2014-05-21
 *      Author: mathieu
 */

#ifndef IMAGESTCPSERVER_H_
#define IMAGESTCPSERVER_H_

#include "find_object/Camera.h"
#include <QtNetwork/QTcpSocket>

class ImagesTcpServer : public QTcpSocket
{
	Q_OBJECT

public:
	static QHostAddress getHostAddress();

public:
	ImagesTcpServer(float hz = 10.0f, const QString & path = "", QObject * parent = 0);

private Q_SLOTS:
	void startCamera();
	void publishImage(const cv::Mat & image);

Q_SIGNALS:
	void connectionLost();

private:
	Camera camera_;
};

#endif /* TCPCLIENT_H_ */
