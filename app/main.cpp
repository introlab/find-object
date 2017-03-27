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

#include <QApplication>
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
#include "TcpServerPool.h"

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
			"  --session \"path\"       Path to a session to load (*.bin). Use \"--session_new\" to\n"
			"                           create a session instead (will be saved to \"path\" on exit, only\n"
			"                           on console mode).\n"
			"  --object \"path\"        Path to an object to detect.\n"
			"  --objects \"path\"       Directory of the objects to detect (--object is ignored).\n"
			"  --config \"path\"        Path to configuration file (default: %s).\n"
			"                           If set to \"\", default parameters are used\n"
			"                           without saving modified parameters on closing.\n"
			"  --scene \"path\"         Path to a scene image file.\n"
			"  --vocabulary \"path\"    Path to a vocabulary file (*.yaml *.xml). Parameters \"General/invertedSearch\"\n"
			"                           and \"General/vocabularyFixed\" will be also enabled. Ignored if \"--session\" is set.\n"
			"  --images_not_saved     Don't keep images in RAM after the features are extracted (only\n"
			"                           in console mode). Images won't be saved if an output session is set.\n"
			"  --tcp_threads #        Number of TCP threads (default 1, only in --console mode). \"--General/port\" parameter should not be 0.\n"
			"                           Port numbers start from \"General/port\" value. \"Detect\" TCP service can be\n"
			"                           executed at the same time by multiple threads. \"Add/Remove\" TCP services\n"
			"                           cannot be called by multiple threads, so calling these services on a port\n "
			"                          will block all other threads on the other ports.\n"
			"  --debug                Show debug log.\n"
			"  --log-time             Show log with time.\n"
			"  --params               Show all parameters.\n"
			"  --defaults             Use default parameters (--config is ignored).\n"
			"  --My/Parameter \"value\" Set find-Object's parameter (look --params for parameters' name).\n"
			"                           It will override the one in --config. Example to set 4 threads:\n"
			"                           $ find_object --General/threads 4\n"
			"  --json \"path\"          Path to an output JSON file (only in --console mode with --scene).\n"
			"  --help                 Show usage.\n"
			, find_object::Settings::iniDefaultPath().toStdString().c_str());
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
	QString sessionPath = "";
	bool sessionNew = false;
	QString objectsPath = "";
	QString objectPath = "";
	QString scenePath = "";
	QString configPath = find_object::Settings::iniDefaultPath();
	QString vocabularyPath = "";
	QString jsonPath;
	find_object::ParametersMap customParameters;
	bool imagesSaved = true;
	int tcpThreads = 1;

	for(int i=1; i<argc; ++i)
	{
#ifdef __APPLE__
		if(QString(argv[i]).startsWith("-psn"))
		{
			//safely ignore
			continue;
		}
#endif
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
		if(strcmp(argv[i], "-session") == 0 ||
		   strcmp(argv[i], "--session") == 0 ||
		   strcmp(argv[i], "-session_new") == 0 ||
		   strcmp(argv[i], "--session_new") == 0)
		{
			if(strcmp(argv[i], "-session_new") == 0 ||
			   strcmp(argv[i], "--session_new") == 0)
			{
				sessionNew = true;
			}
			++i;
			if(i < argc)
			{
				sessionPath = argv[i];
				if(sessionPath.contains('~'))
				{
					sessionPath.replace('~', QDir::homePath());
				}

				if(!sessionNew && !QFile(sessionPath).exists())
				{
					UERROR("Session path not valid : %s (if you want to create a new session, use \"--session_new\")", sessionPath.toStdString().c_str());
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "-object") == 0 ||
		   strcmp(argv[i], "--object") == 0)
		{
			++i;
			if(i < argc)
			{
				objectPath = argv[i];
				if(objectPath.contains('~'))
				{
					objectPath.replace('~', QDir::homePath());
				}
				if(!QFile(objectPath).exists())
				{
					UERROR("Object path not valid : %s", objectPath.toStdString().c_str());
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
		if(strcmp(argv[i], "-vocabulary") == 0 ||
		   strcmp(argv[i], "--vocabulary") == 0)
		{
			++i;
			if(i < argc)
			{
				vocabularyPath = argv[i];
				if(vocabularyPath.contains('~'))
				{
					vocabularyPath.replace('~', QDir::homePath());
				}
				if(!QFile(vocabularyPath).exists())
				{
					UERROR("Vocabulary path not valid : %s", vocabularyPath.toStdString().c_str());
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
				if(!configPath.isEmpty() && !QFile::exists(configPath))
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
		if(strcmp(argv[i], "-images_not_saved") == 0 ||
		   strcmp(argv[i], "--images_not_saved") == 0)
		{
			imagesSaved = false;
			continue;
		}
		if(strcmp(argv[i], "-debug") == 0 ||
		   strcmp(argv[i], "--debug") == 0)
		{
			customParameters.insert(find_object::Settings::kGeneral_debug(), true);
			continue;
		}
		if(strcmp(argv[i], "-log-time") == 0 ||
		   strcmp(argv[i], "--log-time") == 0)
		{
			ULogger::setPrintWhere(true);
			ULogger::setPrintTime(true);
			continue;
		}
		if(strcmp(argv[i], "-help") == 0 ||
		   strcmp(argv[i], "--help") == 0)
		{
			showUsage();
		}
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
		if(strcmp(argv[i], "-tcp_threads") == 0 ||
		   strcmp(argv[i], "--tcp_threads") == 0)
		{
			++i;
			if(i < argc)
			{
				tcpThreads = atoi(argv[i]);
				if(tcpThreads < 1)
				{
					printf("tcp_threads should be >= 1!\n");
					showUsage();
				}
			}
			else
			{
				showUsage();
			}
			continue;
		}
		if(strcmp(argv[i], "--params") == 0)
		{
			find_object::ParametersMap parameters = find_object::Settings::getDefaultParameters();
			for(find_object::ParametersMap::iterator iter=parameters.begin(); iter!=parameters.end(); ++iter)
			{
				std::string str = "Param: " + iter.key().toStdString() + " = \"" + iter.value().toString().toStdString() + "\"";
				std::cout <<
						str <<
						std::setw(60 - str.size()) <<
						" [" <<
						find_object::Settings::getDescriptions().value(iter.key()).toStdString().c_str() <<
						"]" <<
						std::endl;
			}
			UINFO("Node will now exit after showing default Find-Object's parameters because "
					 "argument \"--params\" is detected!");
			exit(0);
		}

		// Check for custom parameters:
		find_object::ParametersMap parameters = find_object::Settings::getDefaultParameters();
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
	if(!sessionPath.isEmpty())
	{
		if(sessionNew)
		{
			UINFO("   Session path: \"%s\" [NEW]", sessionPath.toStdString().c_str());
		}
		else
		{
			UINFO("   Session path: \"%s\"", sessionPath.toStdString().c_str());
		}
	}
	else if(!objectsPath.isEmpty())
	{
		UINFO("   Objects path: \"%s\"", objectsPath.toStdString().c_str());
	}
	else if(!objectPath.isEmpty())
	{
		UINFO("   Object path: \"%s\"", objectPath.toStdString().c_str());
	}
	UINFO("   Scene path: \"%s\"", scenePath.toStdString().c_str());
	if(!guiMode)
	{
		UINFO("   JSON path: \"%s\"", jsonPath.toStdString().c_str());
	}
	UINFO("   Settings path: \"%s\"", configPath.toStdString().c_str());
	UINFO("   Vocabulary path: \"%s\"", vocabularyPath.toStdString().c_str());

	if(!vocabularyPath.isEmpty())
	{
		if(customParameters.contains(find_object::Settings::kGeneral_vocabularyFixed()))
		{
			UWARN("\"General/vocabularyFixed\" custom parameter overwritten as a fixed vocabulary is used.");
		}
		if(customParameters.contains(find_object::Settings::kGeneral_invertedSearch()))
		{
			UWARN("\"General/invertedSearch\" custom parameter overwritten as a fixed vocabulary is used.");
		}
		customParameters[find_object::Settings::kGeneral_vocabularyFixed()] = true;
		customParameters[find_object::Settings::kGeneral_invertedSearch()] = true;
	}

	for(find_object::ParametersMap::iterator iter= customParameters.begin(); iter!=customParameters.end(); ++iter)
	{
		UINFO("   Param \"%s\"=\"%s\"", iter.key().toStdString().c_str(), iter.value().toString().toStdString().c_str());
	}

	//////////////////////////
	// parse options END
	//////////////////////////

	// Load settings, should be loaded before creating other objects
	find_object::Settings::init(configPath);

	// Override custom parameters:
	for(find_object::ParametersMap::iterator iter= customParameters.begin(); iter!=customParameters.end(); ++iter)
	{
		find_object::Settings::setParameter(iter.key(), iter.value());
	}

	// Create FindObject
	find_object::FindObject * findObject = new find_object::FindObject(guiMode || imagesSaved);

	// Load objects if path is set
	int objectsLoaded = 0;
	if(!sessionPath.isEmpty() && !sessionNew)
	{
		if(!vocabularyPath.isEmpty() && !findObject->loadVocabulary(vocabularyPath))
		{
			UWARN("Vocabulary \"%s\" is not loaded as a session \"%s\" is already loaded",
					vocabularyPath.toStdString().c_str(),
					sessionPath.toStdString().c_str());
		}
		if(!findObject->loadSession(sessionPath))
		{
			UERROR("Could not load session \"%s\"", sessionPath.toStdString().c_str());
		}
		else
		{
			objectsLoaded = findObject->objects().size();
		}
	}
	else if(!objectsPath.isEmpty())
	{
		if(!vocabularyPath.isEmpty() && !findObject->loadVocabulary(vocabularyPath))
		{
			UERROR("Failed to load vocabulary \"%s\"", vocabularyPath.toStdString().c_str());
		}
		objectsLoaded = findObject->loadObjects(objectsPath);
		if(!objectsLoaded)
		{
			UWARN("No objects loaded from \"%s\"", objectsPath.toStdString().c_str());
		}
	}
	else if(!objectPath.isEmpty())
	{
		if(!vocabularyPath.isEmpty() && !findObject->loadVocabulary(vocabularyPath))
		{
			UERROR("Failed to load vocabulary \"%s\"", vocabularyPath.toStdString().c_str());
		}

		const find_object::ObjSignature * obj = findObject->addObject(objectPath);
		if(obj)
		{
			++objectsLoaded;
			findObject->updateObjects();
			findObject->updateVocabulary();
		}
		else
		{
			UWARN("No object loaded from \"%s\"", objectsPath.toStdString().c_str());
		}
	}
	else if(!vocabularyPath.isEmpty() && !findObject->loadVocabulary(vocabularyPath))
	{
		UERROR("Failed to load vocabulary \"%s\"", vocabularyPath.toStdString().c_str());
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
		find_object::MainWindow mainWindow(findObject, 0); // ownership transfered

		app.connect( &app, SIGNAL( lastWindowClosed() ), &app, SLOT( quit() ) );
		mainWindow.show();

		if(!scene.empty())
		{
			mainWindow.update(scene);
		}

		app.exec();

		// Save settings
		find_object::Settings::saveSettings();
	}
	else
	{
		QCoreApplication app(argc, argv);

		if(!scene.empty())
		{
			// process the scene and exit
			QTime time;
			time.start();
			find_object::DetectionInfo info;
			findObject->detect(scene, info);

			if(info.objDetected_.size() > 1)
			{
				UINFO("%d objects detected! (%d ms)", (int)info.objDetected_.size(), time.elapsed());
			}
			else if(info.objDetected_.size() == 1)
			{
				UINFO("Object %d detected! (%d ms)", (int)info.objDetected_.begin().key(), time.elapsed());
			}
			else if(find_object::Settings::getGeneral_sendNoObjDetectedEvents())
			{
				UINFO("No objects detected. (%d ms)", time.elapsed());
			}

			if(!jsonPath.isEmpty())
			{
				find_object::JsonWriter::write(info, jsonPath);
				UINFO("JSON written to \"%s\"", jsonPath.toStdString().c_str());
			}
		}
		else
		{
			TcpServerPool tcpServerPool(findObject, tcpThreads, find_object::Settings::getGeneral_port());

			setupQuitSignal();

			//If TCP camera is used
			find_object::Camera * camera = 0;
			if(find_object::Settings::getCamera_6useTcpCamera())
			{
				camera = new find_object::Camera();

				// [Camera] ---Image---> [FindObject]
				QObject::connect(camera, SIGNAL(imageReceived(const cv::Mat &)), findObject, SLOT(detect(const cv::Mat &)));
				QObject::connect(camera, SIGNAL(finished()), &app, SLOT(quit()));

				if(!camera->start())
				{
					UERROR("Camera initialization failed!");
					running = false;
				}
			}

			// start processing!
			if(running)
			{
				app.exec();

				if(!sessionPath.isEmpty())
				{
					if(findObject->isSessionModified())
					{
						UINFO("The session has been modified, updating the session file...");
						if(findObject->saveSession(sessionPath))
						{
							UINFO("Session \"%s\" successfully saved (%d objects)!",
									sessionPath.toStdString().c_str(), findObject->objects().size());
						}
					}
					else if(sessionNew)
					{
						UINFO("The session has not been modified, session file not created...");
					}
				}
			}

			// cleanup
			if(camera)
			{
				camera->stop();
				delete camera;
			}
		}

		delete findObject;
	}
	return 0;
}
