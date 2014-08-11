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
#include "ImagesTcpServer.h"

void showUsage()
{
	printf("\ntcpImagesServer [options] port\n"
			"  Options:\n"
			"    --hz #.#          Image rate (default 10 Hz).\n"
			"    --host #.#.#.#    Set host address.\n"
			"    --path \"\"       Set a path of a directory of images or a video file.\n");
	exit(-1);
}

int main(int argc, char * argv[])
{
	QString ipAddress;
	float hz = 10.0f;
	QString path;

	if(argc < 2)
	{
		showUsage();
	}

	for(int i=1; i<argc-1; ++i)
	{
		if(strcmp(argv[i], "-hz") == 0 || strcmp(argv[i], "--hz") == 0)
		{
			++i;
			if(i < argc-1)
			{
				hz = std::atof(argv[i]);
				if(hz < 0.0f)
				{
					printf("[ERROR] Image rate not valid : %s\n", argv[i]);
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-host") == 0 || strcmp(argv[i], "--host") == 0)
		{
			++i;
			if(i < argc-1)
			{
				ipAddress = argv[i];
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-path") == 0 || strcmp(argv[i], "--path") == 0)
		{
			++i;
			if(i < argc-1)
			{
				path = argv[i];
			}
			else
			{
				showUsage();
			}
			continue;
		}

		printf("Unrecognized option: %s\n", argv[i]);
		showUsage();
	}

	quint16 port = std::atoi(argv[argc-1]);

	if(!path.isEmpty())
	{
		printf("Using images from path \"%s\"\n", path.toStdString().c_str());
	}

	QCoreApplication app(argc, argv);
	ImagesTcpServer server(hz, path);

	QObject::connect(&server, SIGNAL(connectionLost()), &app, SLOT(quit()));

	if(ipAddress.isEmpty())
	{
		ipAddress = server.getHostAddress().toString();
	}
	server.connectToHost(ipAddress, port);

	if(!server.waitForReadyRead())
	{
		printf("ERROR: Unable to connect to %s:%d\n", ipAddress.toStdString().c_str(), port);
		return -1;
	}
	return app.exec();
}

