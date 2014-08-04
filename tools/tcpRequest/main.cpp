/*
 * main.cpp
 *
 *  Created on: 2014-05-05
 *      Author: mathieu
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
			"    --host #.#.#.#       Set host address.\n");
	if(JsonWriter::available())
	{
		printf("    --json \"path\"      Path to an output JSON file.\n");
	}
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

		if(JsonWriter::available())
		{
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
	request.write(block);
	printf("Image published, waiting for response...\n");
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
		if(!jsonPath.isEmpty() && JsonWriter::available())
		{
			JsonWriter::write(response.info(), jsonPath);
			printf("JSON written to \"%s\"\n", jsonPath.toStdString().c_str());
		}
	}
	else
	{
		printf("Failed to receive a response...\n");
	}

	return 0;
}

