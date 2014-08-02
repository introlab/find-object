/*
 * TCPResponse.h
 *
 *  Created on: 2014-05-05
 *      Author: mathieu
 */

#ifndef TCPRESPONSE_H_
#define TCPRESPONSE_H_

#include <QtNetwork/QTcpSocket>
#include <QtCore/QMultiMap>
#include <QtGui/QTransform>
#include <QtCore/QRect>

class TcpResponse : public QTcpSocket
{
	Q_OBJECT;
public:
	TcpResponse(QObject * parent = 0);
	const QMultiMap<int, QPair<QRect, QTransform> > & objectsDetected() const {return objectsDetected_;}
	bool dataReceived() const {return dataReceived_;}

private Q_SLOTS:
	void readReceivedData();
	void displayError(QAbstractSocket::SocketError socketError);
	void connectionLost();

Q_SIGNALS:
	void detectionReceived();

private:
	quint16 blockSize_;
	QMultiMap<int, QPair<QRect, QTransform> > objectsDetected_;
	bool dataReceived_;
};

#endif /* TCPCLIENT_H_ */
