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

#ifdef WITH_JSONCPP
#include <jsoncpp/json/writer.h>
#endif

void showUsage()
{
	printf("\ntcpRequest [options] --scene image.png --out # --in #\n"
			"  \"out\" is the port to which the image is sent.\n"
			"  \"in\" is the port from which the detection is received.\n"
			"  Options:\n"
			"    --host #.#.#.#       Set host address.\n"
#ifdef WITH_JSONCPP
			"    --json \"path\"      Path to an output JSON file.\n"
#endif
			);
	exit(-1);
}

void writeJSON(const QMultiMap<int, QPair<QRect, QTransform> > & objectsDetected, const QString & path)
{
#ifdef WITH_JSONCPP
	if(!path.isEmpty())
	{
		Json::Value root;
		Json::Value detections;

		if(objectsDetected.size())
		{
			for(QMultiMap<int,QPair<QRect,QTransform> >::const_iterator iter = objectsDetected.constBegin();
					iter!= objectsDetected.end();)
			{
				char index = 'a';
				QMultiMap<int,QPair<QRect,QTransform> >::const_iterator jter = iter;
				for(;jter != objectsDetected.constEnd() && jter.key() == iter.key(); ++jter)
				{
					QString name = QString("object_%1%2").arg(jter.key()).arg(objectsDetected.count(jter.key())>1?QString(index++):"");
					detections.append(name.toStdString());

					Json::Value homography;
					homography.append(jter.value().second.m11());
					homography.append(jter.value().second.m12());
					homography.append(jter.value().second.m13());
					homography.append(jter.value().second.m21());
					homography.append(jter.value().second.m22());
					homography.append(jter.value().second.m23());
					homography.append(jter.value().second.m31());  // dx
					homography.append(jter.value().second.m32());  // dy
					homography.append(jter.value().second.m33());
					root[name.toStdString()]["width"] = jter.value().first.width();
					root[name.toStdString()]["height"] = jter.value().first.height();
					root[name.toStdString()]["homography"] = homography;
				}
				iter = jter;
			}
		}

		root["objects"] = detections;

		// write in a nice readible way
		Json::StyledWriter styledWriter;
		//std::cout << styledWriter.write(root);
		QFile file(path);
		file.open(QIODevice::WriteOnly | QIODevice::Text);
		QTextStream out(&file);
		out << styledWriter.write(root).c_str();
		file.close();
		printf("JSON written to \"%s\"\n", path.toStdString().c_str());
	}
#else
	printf("Not built with JSON support!\n");
#endif
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
		if(response.objectsDetected().size())
		{
			QList<int> ids = response.objectsDetected().uniqueKeys();
			for(int i=0; i<ids.size(); ++i)
			{
				int count = response.objectsDetected().count(ids[i]);
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
			writeJSON(response.objectsDetected(), jsonPath);
		}
	}
	else
	{
		printf("Failed to receive a response...\n");
	}

	return 0;
}

