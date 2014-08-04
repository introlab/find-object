/*
 * main.cpp
 *
 *  Created on: 2014-05-05
 *      Author: mathieu
 */

#include <QtNetwork/QNetworkInterface>
#include <QtCore/QCoreApplication>
#include "TcpClient.h"

void showUsage()
{
	printf("\ntcpObjectsClient [hostname] port\n");
	exit(-1);
}

int main(int argc, char * argv[])
{
	if(argc < 2 || argc > 3)
	{
		showUsage();
	}

	QString ipAddress;
	quint16 port = 0;

	if(argc == 2)
	{
		port = atoi(argv[1]);
	}
	else if(argc == 3)
	{
		ipAddress = argv[1];
		port = atoi(argv[2]);
	}

	if(ipAddress.isEmpty())
	{
		 ipAddress = QHostAddress(QHostAddress::LocalHost).toString();
	}

	QCoreApplication app(argc, argv);

	printf("Connecting to \"%s:%d\"...\n", ipAddress.toStdString().c_str(), port);

	TcpClient client;

	client.connectToHost(ipAddress, port);

	if(client.waitForConnected())
	{
		printf("Connecting to \"%s:%d\"... connected!\n", ipAddress.toStdString().c_str(), port);
		app.exec();
	}
	else
	{
		printf("Connecting to \"%s:%d\"... connection failed!\n", ipAddress.toStdString().c_str(), port);
	}

	return 0;
}

