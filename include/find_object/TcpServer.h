/*
 * TCPServer.h
 *
 *  Created on: 2014-05-05
 *      Author: mathieu
 */

#ifndef TCPSERVER_H_
#define TCPSERVER_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include <QtNetwork/QTcpServer>

class QNetworkSession;

class FINDOBJECT_EXP TcpServer : public QTcpServer
{
	Q_OBJECT

public:
	TcpServer(quint16 port = 0, QObject * parent = 0);

	QHostAddress getHostAddress() const;
	quint16 getPort() const;

public Q_SLOTS:
	void publishObjects(const QMultiMap<int, QPair<QRect, QTransform> > & objects);

private Q_SLOTS:
	void addClient();
};

#endif /* TCPSERVER_H_ */
