#include <QtGui/QApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include "find_object/MainWindow.h"
#include "find_object/Settings.h"
#include "find_object/FindObject.h"
#include "find_object/Camera.h"
#include "find_object/TcpServer.h"
#include "find_object/utilite/ULogger.h"

#ifdef WITH_JSONCPP
#include <jsoncpp/json/writer.h>
#endif

bool running = true;

#ifdef WIN32
#include <windows.h> 
BOOL WINAPI my_handler(DWORD signal)
{
    if (signal == CTRL_C_EVENT)
	{
        printf("\nCtrl-C caught! Quitting application...\n");
		QCoreApplication::quit();
	}
    return TRUE;
    running = false;
}
#else
#include <signal.h>
void my_handler(int s)
{
	printf("\nCtrl-C caught! Quitting application...\n");
	QCoreApplication::quit();
	running = false;
}
inline void Sleep(unsigned int ms)
{
	struct timespec req;
	struct timespec rem;
	req.tv_sec = ms / 1000;
	req.tv_nsec = (ms - req.tv_sec * 1000) * 1000 * 1000;
	nanosleep (&req, &rem);
}
#endif

void setupQuitSignal()
{
// Catch ctrl-c to close Qt
#ifdef WIN32
	if (!SetConsoleCtrlHandler(my_handler, TRUE))
	{
		UERROR("Could not set control (ctrl-c) handler");
	}
#else
	struct sigaction sigIntHandler;
	sigIntHandler.sa_handler = my_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
#endif
}

void showUsage()
{
	printf("\nUsage:\n"
#ifdef WIN32
			"  Find-Object.exe [options]\n"
#else
			"  find_object [options]\n"
#endif
			"Options:\n"
			"  --console         Don't use the GUI (by default the camera will be\n"
			"                    started automatically). Option --objects must also be\n"
			"                    used with valid objects.\n"
			"  --objects \"path\"   Directory of the objects to detect.\n"
			"  --config \"path\"    Path to configuration file (default: %s).\n"
			"  --scene \"path\"     Path to a scene image file.\n"
#ifdef WITH_JSONCPP
			"  --json \"path\"      Path to an output JSON file (only in --console mode with --scene).\n"
#endif
			"  --help   Show usage.\n", Settings::iniDefaultPath().toStdString().c_str());
	exit(-1);
}

void writeJSON(const FindObject & findObject, const QString & path)
{
#ifdef WITH_JSONCPP
	if(!path.isEmpty())
	{
		Json::Value root;
		Json::Value detections;
		Json::Value matchesValues;

		if(findObject.objectsDetected().size())
		{
			Q_ASSERT(objectsDetected.size() == findObject.inliers().size() &&
					 objectsDetected.size() == findObject.outliers().size());

			const QMultiMap<int,QPair<QRect,QTransform> > & objectsDetected = findObject.objectsDetected();
			QMultiMap<int, QMultiMap<int, int> >::const_iterator iterInliers = findObject.inliers().constBegin();
			QMultiMap<int, QMultiMap<int, int> >::const_iterator iterOutliers = findObject.outliers().constBegin();
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
					root[name.toStdString()]["inliers"] = iterInliers.value().size();
					root[name.toStdString()]["outliers"] = iterOutliers.value().size();

					++iterInliers;
					++iterOutliers;
				}
				iter = jter;
			}
		}

		const QMap<int, QMultiMap<int, int> > & matches = findObject.matches();
		for(QMap<int, QMultiMap<int, int> >::const_iterator iter = matches.constBegin();
			iter != matches.end();
			++iter)
		{
			QString name = QString("matches_%1").arg(iter.key());
			root[name.toStdString()] = iter.value().size();
			matchesValues.append(name.toStdString());
		}

		root["objects"] = detections;
		root["matches"] = matchesValues;

		// write in a nice readible way
		Json::StyledWriter styledWriter;
		//std::cout << styledWriter.write(root);
		QFile file(path);
		file.open(QIODevice::WriteOnly | QIODevice::Text);
		QTextStream out(&file);
		out << styledWriter.write(root).c_str();
		file.close();
		UINFO("JSON written to \"%s\"", path.toStdString().c_str());
	}
#else
	UERROR("Not built with JSON support!");
#endif
}

int main(int argc, char* argv[])
{
	ULogger::setType(ULogger::kTypeConsole);
	ULogger::setLevel(ULogger::kInfo);
	ULogger::setPrintWhere(false);
	ULogger::setPrintTime(false);

	//////////////////////////
	// parse options BEGIN
	//////////////////////////
	bool guiMode = true;
	QString objectsPath = "";
	QString scenePath = "";
	QString configPath = Settings::iniDefaultPath();
	QString jsonPath;

	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "-objs") == 0 ||
		   strcmp(argv[i], "--objs") == 0 ||
		   strcmp(argv[i], "-objects") == 0 ||
		   strcmp(argv[i], "--objects") == 0)
		{
			++i;
			if(i < argc)
			{
				objectsPath = argv[i];
				if(objectsPath.contains('~'))
				{
					objectsPath.replace('~', QDir::homePath());
				}
				if(!QDir(objectsPath).exists())
				{
					UERROR("Objects path not valid : %s", objectsPath.toStdString().c_str());
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-scene") == 0 ||
		   strcmp(argv[i], "--scene") == 0)
		{
			++i;
			if(i < argc)
			{
				scenePath = argv[i];
				if(scenePath.contains('~'))
				{
					scenePath.replace('~', QDir::homePath());
				}
				if(!QFile(scenePath).exists())
				{
					UERROR("Scene path not valid : %s", scenePath.toStdString().c_str());
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-config") == 0 ||
		   strcmp(argv[i], "--config") == 0)
		{
			++i;
			if(i < argc)
			{
				configPath = argv[i];
				if(configPath.contains('~'))
				{
					configPath.replace('~', QDir::homePath());
				}
				if(!QFile::exists(configPath))
				{
					UERROR("Configuration file doesn't exist : %s", configPath.toStdString().c_str());
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
#ifdef WITH_JSONCPP
		if(strcmp(argv[i], "-json") == 0 ||
		   strcmp(argv[i], "--json") == 0)
		{
			++i;
			if(i < argc)
			{
				jsonPath = argv[i];
				if(jsonPath.contains('~'))
				{
					jsonPath.replace('~', QDir::homePath());
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
#endif
		if(strcmp(argv[i], "-console") == 0 ||
		   strcmp(argv[i], "--console") == 0)
		{
			guiMode = false;
			continue;
		}
		if(strcmp(argv[i], "-help") == 0 ||
		   strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}

		UERROR("Unrecognized option : %s", argv[i]);
		showUsage();
	}

	UINFO("Options:");
	UINFO("   GUI mode = %s", guiMode?"true":"false");
	UINFO("   Objects path: \"%s\"", objectsPath.toStdString().c_str());
	UINFO("   Scene path: \"%s\"", scenePath.toStdString().c_str());
	UINFO("   Settings path: \"%s\"", configPath.toStdString().c_str());
#ifdef WITH_JSONCPP
	UINFO("   JSON path: \"%s\"", configPath.toStdString().c_str());
#endif

	//////////////////////////
	// parse options END
	//////////////////////////

	// Load settings, should be loaded before creating other objects
	Settings::init(configPath);

	// Create FindObject
	FindObject * findObject = new FindObject();

	// Load objects if path is set
	int objectsLoaded = 0;
	if(!objectsPath.isEmpty())
	{
		objectsLoaded = findObject->loadObjects(objectsPath);
		if(!objectsLoaded)
		{
			UWARN("No objects loaded from \"%s\"", objectsPath.toStdString().c_str());
		}
	}
	cv::Mat scene;
	if(!scenePath.isEmpty())
	{
		scene = cv::imread(scenePath.toStdString());
		if(scene.empty())
		{
			UERROR("Failed to load scene \"%s\"", scenePath.toStdString().c_str());
		}
	}

	if(guiMode)
	{
		QApplication app(argc, argv);
		MainWindow mainWindow(findObject, 0); // ownership transfered

		app.connect( &app, SIGNAL( lastWindowClosed() ), &app, SLOT( quit() ) );
		mainWindow.show();

		if(!scene.empty())
		{
			mainWindow.update(scene);
		}

		app.exec();

		// Save settings
		Settings::saveSettings();
	}
	else
	{
		if(objectsLoaded == 0)
		{
			UERROR("In console mode, at least one object must be loaded! See -console option.");
			delete findObject;
			showUsage();
		}

		QCoreApplication app(argc, argv);

		TcpServer tcpServer(Settings::getGeneral_port());
		printf("IP: %s\nport: %d\n", tcpServer.getHostAddress().toString().toStdString().c_str(), tcpServer.getPort());

		// connect stuff:
		// [FindObject] ---ObjectsDetected---> [TcpServer]
		QObject::connect(findObject, SIGNAL(objectsFound(QMultiMap<int,QPair<QRect,QTransform> >)), &tcpServer, SLOT(publishObjects(QMultiMap<int,QPair<QRect,QTransform> >)));

		if(!scene.empty())
		{
			// process the scene and exit
			findObject->detect(scene); // this will automatically emit objectsFound()

			if(!jsonPath.isEmpty())
			{
				writeJSON(*findObject, jsonPath);
			}
		}
		else
		{
			Camera camera;

			// [Camera] ---Image---> [FindObject]
			QObject::connect(&camera, SIGNAL(imageReceived(const cv::Mat &)), findObject, SLOT(detect(const cv::Mat &)));

			//use camera in settings
			setupQuitSignal();

			// start processing!
			while(running && !camera.start())
			{
				if(Settings::getCamera_6useTcpCamera())
				{
					UWARN("Camera initialization failed! (with server %s:%d) Trying again in 1 second...",
							Settings::getCamera_7IP().toStdString().c_str(), Settings::getCamera_8port());
					Sleep(1000);
				}
				else
				{
					UERROR("Camera initialization failed!");
					running = false;
				}
			}
			if(running)
			{
				app.exec();
			}

			// cleanup
			camera.stop();
		}

		delete findObject;
		tcpServer.close();
	}
}
