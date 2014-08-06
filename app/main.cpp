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
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <QtGui/QApplication>
#include <QtCore/QDir>
#include <QtCore/QFile>
#include <iostream>
#include <iomanip>
#include "find_object/MainWindow.h"
#include "find_object/Settings.h"
#include "find_object/FindObject.h"
#include "find_object/Camera.h"
#include "find_object/TcpServer.h"
#include "find_object/JsonWriter.h"
#include "find_object/utilite/ULogger.h"

using namespace find_object;

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
			"  --console              Don't use the GUI (by default the camera will be\n"
			"                           started automatically). Option --objects must also be\n"
			"                           used with valid objects.\n"
			"  --objects \"path\"       Directory of the objects to detect.\n"
			"  --config \"path\"        Path to configuration file (default: %s).\n"
			"  --scene \"path\"         Path to a scene image file.\n"
			"  --params               Show all parameters.\n"
			"  --My/Parameter \"value\" Set find-Object's parameter (look --params for parameters' name).\n"
			"                           It will override the one in --config. Example to set 4 threads:\n"
			"                           $ find_object --General/threads 4\n"
			"  --help                 Show usage.\n", Settings::iniDefaultPath().toStdString().c_str());
	if(JsonWriter::available())
	{
		printf("  --json \"path\"          Path to an output JSON file (only in --console mode with --scene).\n");
	}
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
	QString scenePath = "";
	QString configPath = Settings::iniDefaultPath();
	QString jsonPath;
	ParametersMap customParameters;

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
					UWARN("Configuration file \"%s\" doesn't exist, it will be created with default values...", configPath.toStdString().c_str());
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
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
		if(JsonWriter::available())
		{
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
		}
		if(strcmp(argv[i], "--params") == 0)
		{
			ParametersMap parameters = Settings::getDefaultParameters();
			for(ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
			{
				std::string str = "Param: " + iter.key().toStdString() + " = \"" + iter.value().toString().toStdString() + "\"";
				std::cout <<
						str <<
						std::setw(60 - str.size()) <<
						" [" <<
						Settings::getDescriptions().value(iter.key()).toStdString().c_str() <<
						"]" <<
						std::endl;
			}
			UINFO("Node will now exit after showing default Find-Object's parameters because "
					 "argument \"--params\" is detected!");
			exit(0);
		}

		// Check for custom parameters:
		ParametersMap parameters = Settings::getDefaultParameters();
		QString name = argv[i];
		if(name.size() > 2)
		{
			//strip the "--"
			name.remove(0, 2);
			if(parameters.contains(name))
			{
				++i;
				if(i < argc)
				{
					customParameters.insert(name, argv[i]);
				}
				else
				{
					showUsage();
				}
				continue;
			}
		}

		UERROR("Unrecognized option : %s", argv[i]);
		showUsage();
	}

	UINFO("Options:");
	UINFO("   GUI mode = %s", guiMode?"true":"false");
	UINFO("   Objects path: \"%s\"", objectsPath.toStdString().c_str());
	UINFO("   Scene path: \"%s\"", scenePath.toStdString().c_str());
	UINFO("   Settings path: \"%s\"", configPath.toStdString().c_str());
	if(JsonWriter::available())
	{
		UINFO("   JSON path: \"%s\"", jsonPath.toStdString().c_str());
	}
	for(ParametersMap::iterator iter= customParameters.begin(); iter!=customParameters.end(); ++iter)
	{
		UINFO("   Param \"%s\"=\"%s\"", iter.key().toStdString().c_str(), iter.value().toString().toStdString().c_str());
	}

	//////////////////////////
	// parse options END
	//////////////////////////

	// Load settings, should be loaded before creating other objects
	Settings::init(configPath);

	// Override custom parameters:
	for(ParametersMap::iterator iter= customParameters.begin(); iter!=customParameters.end(); ++iter)
	{
		Settings::setParameter(iter.key(), iter.value());
	}

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

		if(!scene.empty())
		{
			// process the scene and exit
			QTime time;
			time.start();
			DetectionInfo info;
			findObject->detect(scene, info);

			if(info.objDetected_.size() > 1)
			{
				UINFO("%d objects detected! (%d ms)", (int)info.objDetected_.size(), time.elapsed());
			}
			else if(info.objDetected_.size() == 1)
			{
				UINFO("Object %d detected! (%d ms)", (int)info.objDetected_.begin().key(), time.elapsed());
			}
			else if(Settings::getGeneral_sendNoObjDetectedEvents())
			{
				UINFO("No objects detected. (%d ms)", time.elapsed());
			}

			if(!jsonPath.isEmpty() && JsonWriter::available())
			{
				JsonWriter::write(info, jsonPath);
				UINFO("JSON written to \"%s\"", jsonPath.toStdString().c_str());
			}
		}
		else
		{
			Camera camera;
			TcpServer tcpServer(Settings::getGeneral_port());
			UINFO("Detection sent on port: %d (IP=%s)", tcpServer.getPort(), tcpServer.getHostAddress().toString().toStdString().c_str());

			// connect stuff:
			// [FindObject] ---ObjectsDetected---> [TcpServer]
			QObject::connect(findObject, SIGNAL(objectsFound(DetectionInfo)), &tcpServer, SLOT(publishDetectionInfo(DetectionInfo)));

			// [Camera] ---Image---> [FindObject]
			QObject::connect(&camera, SIGNAL(imageReceived(const cv::Mat &)), findObject, SLOT(detect(const cv::Mat &)));
			QObject::connect(&camera, SIGNAL(finished()), &app, SLOT(quit()));

			//use camera in settings
			setupQuitSignal();

			// start processing!
			while(running && !camera.start())
			{
				UERROR("Camera initialization failed!");
				running = false;
			}
			if(running)
			{
				app.exec();
			}

			// cleanup
			camera.stop();
			tcpServer.close();
		}

		delete findObject;
	}
}
