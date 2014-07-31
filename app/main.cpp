#include <QtGui/QApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include "find_object/MainWindow.h"
#include "find_object/Settings.h"
#include "find_object/FindObject.h"
#include "find_object/Camera.h"
#include "find_object/TcpServer.h"
#include "find_object/utilite/ULogger.h"

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
		printf("\nERROR: Could not set control handler");
		return 1;
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
			"  -console          Don't use the GUI (by default the camera will be\n"
			"                    started automatically). Option -objs must also be\n"
			"                    used with valid objects.\n"
			"  -objs \"path\"      Directory of the objects to detect.\n"
			"  -config \"path\"    Path to configuration file (default: %s).\n"
			"  -help or --help   Show usage.\n", Settings::iniDefaultPath().toStdString().c_str());
	exit(-1);
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
	QString configPath = Settings::iniDefaultPath();

	for(int i=1; i<argc; ++i)
	{
		if(strcmp(argv[i], "-objs") == 0)
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
					UERROR("Path not valid : %s", objectsPath.toStdString().c_str());
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-config") == 0)
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
		if(strcmp(argv[i], "-console") == 0)
		{
			guiMode = false;
			continue;
		}
		if(strcmp(argv[i], "-help") == 0 || strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}

		UERROR("Unrecognized option : %s", argv[i]);
		showUsage();
	}

	UINFO("Options:");
	UINFO("   GUI mode = %s", guiMode?"true":"false");
	UINFO("   Objects path: \"%s\"", objectsPath.toStdString().c_str());
	UINFO("   Settings path: \"%s\"", configPath.toStdString().c_str());

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

	if(guiMode)
	{
		QApplication app(argc, argv);
		MainWindow mainWindow(findObject, 0); // ownership transfered

		app.connect( &app, SIGNAL( lastWindowClosed() ), &app, SLOT( quit() ) );
		mainWindow.show();

		app.exec();
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
		Camera camera;
		TcpServer tcpServer(Settings::getGeneral_port());
		printf("IP: %s\nport: %d\n", tcpServer.getHostAddress().toString().toStdString().c_str(), tcpServer.getPort());

		// connect stuff:
		// [Camera] ---Image---> [FindObject] ---ObjectsDetected---> [TcpServer]
		QObject::connect(&camera, SIGNAL(imageReceived(const cv::Mat &)), findObject, SLOT(detect(const cv::Mat &)));
		QObject::connect(findObject, SIGNAL(objectsFound(QMultiMap<int,QPair<QRect,QTransform> >)), &tcpServer, SLOT(publishObjects(QMultiMap<int,QPair<QRect,QTransform> >)));

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
		delete findObject;
		tcpServer.close();
	}

	// Save settings
	Settings::saveSettings();
}
