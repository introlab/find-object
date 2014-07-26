#include <QtGui/QApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include "MainWindow.h"
#include "Settings.h"

#ifdef WIN32
#include <windows.h> 
BOOL WINAPI my_handler(DWORD signal)
{
    if (signal == CTRL_C_EVENT)
	{
        printf("\nCtrl-C caught! Quitting application...\n");
		QApplication::quit();
	}
    return TRUE;
}
#else
#include <signal.h>
void my_handler(int s)
{
	printf("\nCtrl-C caught! Quitting application...\n");
	QApplication::quit();
}
#endif

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
	QApplication app(argc, argv);

	// parse options
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
					printf("[ERROR] Path not valid : %s\n", objectsPath.toStdString().c_str());
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
					printf("[ERROR] Configuration file doesn't exist : %s\n", configPath.toStdString().c_str());
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

		printf("[ERROR] Unrecognized option : %s\n", argv[i]);
		showUsage();
	}

	printf("Options:\n");
	printf("   GUI mode = %s\n", guiMode?"true":"false");
	printf("   Objects path: \"%s\"\n", objectsPath.toStdString().c_str());
	printf("   Settings path: \"%s\"\n", configPath.toStdString().c_str());


	MainWindow mainWindow(0, configPath);

	int objectsLoaded = 0;
	if(!objectsPath.isEmpty())
	{
		objectsLoaded = mainWindow.loadObjects(objectsPath);
		if(!objectsLoaded)
		{
			printf("[WARNING] No objects loaded from \"%s\"\n", objectsPath.toStdString().c_str());
		}
	}

	if(objectsLoaded == 0 && !guiMode)
	{
		printf("[ERROR] In console mode, at least one object must be loaded! See -console option.\n");
		showUsage();
	}

	if(guiMode)
	{
		app.connect( &app, SIGNAL( lastWindowClosed() ), &app, SLOT( quit() ) );
		mainWindow.show();
	}
	else
	{
		mainWindow.startProcessing();
	}

	if(!guiMode)
	{
		// Catch ctrl-c to close the gui
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

    return app.exec();
}
