/*
 * TCPClient.h
 *
 *  Created on: 2014-05-05
 *      Author: mathieu
 */

#ifndef TCPCLIENT_H_
#define TCPCLIENT_H_

#include <QtNetwork/QTcpSocket>

class TcpClient : public QTcpSocket
{
	Q_OBJECT;
public:
	TcpClient(const QString & hostname, quint16 port, QObject * parent = 0);

private slots:
	void readData();
	void displayError(QAbstractSocket::SocketError socketError);
	void connectionLost();

private:
	quint16 blockSize_;
};

#endif /* TCPCLIENT_H_ */
