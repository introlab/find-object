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
	printf("imagesTcpServer [options]\n"
			"  Options:\n"
			"    -hz #.#          Image rate (default 10 Hz).\n"
			"    -p #             Set manually a port to which the clients will connect.\n"
			"    -path \"\"       Set a path of a directory of images or a video file.\n");
	exit(-1);
}

int main(int argc, char * argv[])
{
	QString ipAddress;
	float hz = 10.0f;
	quint16 port = 0;
	QString path;

	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "-hz") == 0)
		{
			++i;
			if(i < argc)
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
		if(strcmp(argv[i], "-p") == 0)
		{
			++i;
			if(i < argc)
			{
				int v = std::atoi(argv[i]);
				if(v < 0)
				{
					printf("[ERROR] Port not valid : %s\n", argv[i]);
					showUsage();
				}
				port = v;
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-path") == 0)
		{
			++i;
			if(i < argc)
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

	if(!path.isEmpty())
	{
		printf("Using images from path \"%s\"\n", path.toStdString().c_str());
	}

	QCoreApplication app(argc, argv);

	ImagesTcpServer server(hz, path);

	if (!server.listen(QHostAddress::Any, port))
	{
		printf("ERROR: Unable to start the TCP server: %s\n", server.errorString().toStdString().c_str());
		return -1;
	}

	printf("Images server waiting on \"%s:%d\"...\n",
			server.getHostAddress().toString().toStdString().c_str(), server.getPort());

	return app.exec();
}

