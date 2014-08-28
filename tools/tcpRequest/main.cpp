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
#include <QtCore/QFile>
#include <QtCore/QTime>
#include <opencv2/opencv.hpp>
#include "TcpResponse.h"
#include "find_object/JsonWriter.h"

void showUsage()
{
	printf("\ntcpRequest [options] --scene image.png --out # --in #\n"
			"  \"out\" is the port to which the image is sent.\n"
			"  \"in\" is the port from which the detection is received.\n"
			"  Options:\n"
			"    --host #.#.#.#       Set host address.\n"
			"    --json \"path\"        Path to an output JSON file.\n"
			"    --help               Show this help.\n");
	exit(-1);
}

int main(int argc, char * argv[])
{
	QString ipAddress;
	QString scenePath;
	QString jsonPath;
	quint16 portOut = 0;
	quint16 portIn = 0;

	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "--host") == 0 || strcmp(argv[i], "-host") == 0)
		{
			++i;
			if(i < argc)
			{
				ipAddress = argv[i];
			}
			else
			{
				printf("error parsing --host\n");
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "--scene") == 0 || strcmp(argv[i], "-scene") == 0)
		{
			++i;
			if(i < argc)
			{
				scenePath = argv[i];
			}
			else
			{
				printf("error parsing --scene\n");
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "--out") == 0 || strcmp(argv[i], "-out") == 0)
		{
			++i;
			if(i < argc)
			{
				portOut = std::atoi(argv[i]);
			}
			else
			{
				printf("error parsing --out\n");
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "--in") == 0 || strcmp(argv[i], "-in") == 0)
		{
			++i;
			if(i < argc)
			{
				portIn = std::atoi(argv[i]);
			}
			else
			{
				printf("error parsing --in\n");
				showUsage();
			}
			continue;
		}

		if(strcmp(argv[i], "--json") == 0 || strcmp(argv[i], "-json") == 0)
		{
			++i;
			if(i < argc)
			{
				jsonPath = argv[i];
			}
			else
			{
				printf("error parsing --json\n");
				showUsage();
			}
			continue;
		}

		if(strcmp(argv[i], "-help") == 0 ||
		   strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}

		printf("Unrecognized option: %s\n", argv[i]);
		showUsage();
	}

	if(portOut == 0)
	{
		printf("Argument --out should be set.\n");
		showUsage();
	}
	else if(portIn == 0)
	{
		printf("Argument --in should be set.\n");
	}
	else if(scenePath.isEmpty())
	{
		printf("Argument --scene should be set.\n");
	}

	if(ipAddress.isEmpty())
	{
		ipAddress = QHostAddress(QHostAddress::LocalHost).toString();
	}

	cv::Mat image = cv::imread(scenePath.toStdString());
	if(image.empty())
	{
		printf("Cannot read image from \"%s\".\n", scenePath.toStdString().c_str());
		showUsage();
	}

	QCoreApplication app(argc, argv);
	QTcpSocket request;
	TcpResponse response;

	QObject::connect(&response, SIGNAL(detectionReceived()), &app, SLOT(quit()));
	QObject::connect(&response, SIGNAL(disconnected()), &app, SLOT(quit()));
	QObject::connect(&request, SIGNAL(disconnected()), &app, SLOT(quit()));
	QObject::connect(&response, SIGNAL(error(QAbstractSocket::SocketError)), &app, SLOT(quit()));
	QObject::connect(&request, SIGNAL(error(QAbstractSocket::SocketError)), &app, SLOT(quit()));

	request.connectToHost(ipAddress, portOut);
	response.connectToHost(ipAddress, portIn);

	if(!request.waitForConnected())
	{
		printf("ERROR: Unable to connect to %s:%d\n", ipAddress.toStdString().c_str(), portOut);
		return -1;
	}

	if(!response.waitForConnected())
	{
		printf("ERROR: Unable to connect to %s:%d\n", ipAddress.toStdString().c_str(), portIn);
		return -1;
	}

	// publish image
	std::vector<unsigned char> buf;
	cv::imencode(".png", image, buf);

	QByteArray block;
	QDataStream out(&block, QIODevice::WriteOnly);
	out.setVersion(QDataStream::Qt_4_0);
	out << (quint64)0;
	out.writeRawData((char*)buf.data(), (int)buf.size());
	out.device()->seek(0);
	out << (quint64)(block.size() - sizeof(quint64));

	if(request.waitForReadyRead())
	{
		qint64 bytes = request.write(block);
		printf("Image published (%d bytes), waiting for response...\n", (int)bytes);
		QTime time;
		time.start();

		// wait for response
		app.exec();

		if(response.dataReceived())
		{
			printf("Response received! (%d ms)\n", time.elapsed());
			// print detected objects
			if(response.info().objDetected_.size())
			{
				QList<int> ids = response.info().objDetected_.uniqueKeys();
				for(int i=0; i<ids.size(); ++i)
				{
					int count = response.info().objDetected_.count(ids[i]);
					if(count == 1)
					{
						printf("Object %d detected.\n", ids[i]);
					}
					else
					{
						printf("Object %d detected %d times.\n", ids[i], count);
					}
				}
			}
			else
			{
				printf("No objects detected.\n");
			}
			// write json
			if(!jsonPath.isEmpty())
			{
				find_object::JsonWriter::write(response.info(), jsonPath);
				printf("JSON written to \"%s\"\n", jsonPath.toStdString().c_str());
			}
		}
		else
		{
			printf("Failed to receive a response...\n");
			return -1;
		}
	}
	else
	{
		printf("Server is busy...\n");
		return -1;
	}

	return 0;
}

