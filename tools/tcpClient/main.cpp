/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

