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
	printf("exampleTcpClient [hostname] port\n");
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
		port = std::atoi(argv[1]);
	}
	else if(argc == 3)
	{
		ipAddress = argv[1];
		port = std::atoi(argv[2]);
	}

	if(ipAddress.isEmpty())
	{
		// find out which IP to connect to
		 QList<QHostAddress> ipAddressesList = QNetworkInterface::allAddresses();
		 // use the first non-localhost IPv4 address
		 for (int i = 0; i < ipAddressesList.size(); ++i)
		 {
			 if (ipAddressesList.at(i) != QHostAddress::LocalHost &&
				 ipAddressesList.at(i).toIPv4Address())
			 {
				 ipAddress = ipAddressesList.at(i).toString();
				 break;
			 }
		 }
		 // if we did not find one, use IPv4 localhost
		 if (ipAddress.isEmpty())
		 {
			 ipAddress = QHostAddress(QHostAddress::LocalHost).toString();
		 }
	}

	QCoreApplication app(argc, argv);

	printf("Connecting to \"%s:%d\"...\n", ipAddress.toStdString().c_str(), port);

	TcpClient client(ipAddress, port);

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

