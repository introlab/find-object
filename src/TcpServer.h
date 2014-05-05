/*
 * TCPServer.h
 *
 *  Created on: 2014-05-05
 *      Author: mathieu
 */

#ifndef TCPSERVER_H_
#define TCPSERVER_H_

#include <QtNetwork/QTcpServer>

class QNetworkSession;

class TcpServer : public QTcpServer
{
	Q_OBJECT

public:
	TcpServer(quint16 port = 0, QObject * parent = 0);

	QHostAddress getHostAddress() const;
	quint16 getPort() const;


private slots:
	void addClient();
	void publishObjects(const QMultiMap<int, QPair<QRect, QTransform> > & objects);
};

#endif /* TCPSERVER_H_ */
