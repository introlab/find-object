/*
 * CameraTcpClient.h
 *
 *  Created on: 2014-07-31
 *      Author: mathieu
 */

#ifndef CAMERATCPCLIENT_H_
#define CAMERATCPCLIENT_H_

#include <QtNetwork/QTcpSocket>
#include <opencv2/opencv.hpp>

class CameraTcpClient : public QTcpSocket
{
	Q_OBJECT;
public:
	CameraTcpClient(QObject * parent = 0);
	cv::Mat getImage();
	int imagesBuffered() const {return images_.size();}

private Q_SLOTS:
	void readReceivedData();
	void displayError(QAbstractSocket::SocketError socketError);
	void connectionLost();

private:
	quint64 blockSize_;
	QVector<cv::Mat> images_;
};

#endif /* CAMERATCPCLIENT_H_ */
