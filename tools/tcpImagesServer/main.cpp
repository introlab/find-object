/*
 * main.cpp
 *
 *  Created on: 2014-05-05
 *      Author: mathieu
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

