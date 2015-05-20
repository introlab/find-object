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

#include "CameraROS.h"
#include "FindObjectROS.h"

#include <QApplication>
#include <QDir>
#include "find_object/MainWindow.h"
#include "ParametersToolBox.h"
#include "find_object/Settings.h"
#include <signal.h>

using namespace find_object;

bool gui;
std::string settingsPath;

void my_handler_gui(int s){
	QApplication::closeAllWindows();
	QApplication::quit();
}
void my_handler(int s){
	QCoreApplication::quit();
}

void setupQuitSignal(bool gui)
{
	// Catch ctrl-c to close the gui
	struct sigaction sigIntHandler;
	if(gui)
	{
		sigIntHandler.sa_handler = my_handler_gui;
	}
	else
	{
		sigIntHandler.sa_handler = my_handler;
	}
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;
	sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "find_object_2d");

	gui = true;
	std::string objectsPath;
	std::string sessionPath;
	settingsPath = QDir::homePath().append("/.ros/find_object_2d.ini").toStdString();
	bool subscribeDepth = false;
	std::string objFramePrefix = "object";

	ros::NodeHandle nh("~");

	nh.param("gui", gui, gui);
	nh.param("objects_path", objectsPath, objectsPath);
	nh.param("session_path", sessionPath, sessionPath);
	nh.param("settings_path", settingsPath, settingsPath);
	nh.param("subscribe_depth", subscribeDepth, subscribeDepth);
	nh.param("obj_frame_prefix", objFramePrefix, objFramePrefix);

	ROS_INFO("gui=%d", (int)gui);
	ROS_INFO("objects_path=%s", objectsPath.c_str());
	ROS_INFO("session_path=%s", sessionPath.c_str());
	ROS_INFO("settings_path=%s", settingsPath.c_str());
	ROS_INFO("subscribe_depth = %s", subscribeDepth?"true":"false");
	ROS_INFO("obj_frame_prefix = %s", objFramePrefix.c_str());

	if(settingsPath.empty())
	{
		settingsPath = QDir::homePath().append("/.ros/find_object_2d.ini").toStdString();
	}
	else
	{
		if(!sessionPath.empty())
		{
			ROS_WARN("\"settings_path\" parameter is ignored when \"session_path\" is set.");
		}

		QString path = settingsPath.c_str();
		if(path.contains('~'))
		{
			path.replace('~', QDir::homePath());
			settingsPath = path.toStdString();
		}
	}

	// Load settings, should be loaded before creating other objects
	Settings::init(settingsPath.c_str());

	FindObjectROS * findObjectROS = new FindObjectROS(objFramePrefix);
	if(!sessionPath.empty())
	{
		if(!objectsPath.empty())
		{
			ROS_WARN("\"objects_path\" parameter is ignored when \"session_path\" is set.");
		}
		if(!findObjectROS->loadSession(sessionPath.c_str()))
		{
			ROS_ERROR("Failed to load session \"%s\"", sessionPath.c_str());
		}
	}
	else if(!objectsPath.empty())
	{
		QString path = objectsPath.c_str();
		if(path.contains('~'))
		{
			path.replace('~', QDir::homePath());
		}
		if(!findObjectROS->loadObjects(path))
		{
			ROS_ERROR("No objects loaded from path \"%s\"", path.toStdString().c_str());
		}
	}

	CameraROS * camera = new CameraROS(subscribeDepth);

	QObject::connect(
			camera,
			SIGNAL(rosDataReceived(const std::string &, const ros::Time &, const cv::Mat &, float)),
			findObjectROS,
			SLOT(setDepthData(const std::string &, const ros::Time &, const cv::Mat &, float)));

	// Catch ctrl-c to close the gui
	setupQuitSignal(gui);

	if(gui)
	{
		QApplication app(argc, argv);
		MainWindow mainWindow(findObjectROS, camera); // take ownership

		QObject::connect(
				&mainWindow,
				SIGNAL(objectsFound(const find_object::DetectionInfo &)),
				findObjectROS,
				SLOT(publish(const find_object::DetectionInfo &)));

		QStringList topics = camera->subscribedTopics();
		if(topics.size() == 1)
		{
			mainWindow.setSourceImageText(mainWindow.tr(
					"<qt>Find-Object subscribed to <b>%1</b> topic.<br/>"
					"You can remap the topic when starting the node: <br/>\"rosrun find_object_2d find_object_2d image:=your/image/topic\".<br/>"
					"</qt>").arg(topics.first()));
		}
		else if(topics.size() == 3)
		{
			mainWindow.setSourceImageText(mainWindow.tr(
					"<qt>Find-Object subscribed to : <br/> <b>%1</b> <br/> <b>%2</b> <br/> <b>%3</b><br/>"
					"</qt>").arg(topics.at(0)).arg(topics.at(1)).arg(topics.at(2)));
		}
		mainWindow.show();
		app.connect( &app, SIGNAL( lastWindowClosed() ), &app, SLOT( quit() ) );

		// loop
		mainWindow.startProcessing();
		app.exec();
		Settings::saveSettings();
	}
	else
	{
		QCoreApplication app(argc, argv);

		// connect stuff:
		QObject::connect(camera, SIGNAL(imageReceived(const cv::Mat &)), findObjectROS, SLOT(detect(const cv::Mat &)));

		//loop
		camera->start();
		app.exec();

		delete camera;
		delete findObjectROS;
	}
	return 0;
}
