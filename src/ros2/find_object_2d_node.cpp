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

class FindObjectNode : public rclcpp::Node {

public:
	FindObjectNode() :
		rclcpp::Node("find_object_2d"),
		findObjectROS_(0),
		camera_(0),
		gui_(true)
	{
		std::string objectsPath;
		std::string sessionPath;
		std::string settingsPath = QDir::homePath().append("/.ros/find_object_2d.ini").toStdString();
		bool subscribeDepth = false;

		gui_ = this->declare_parameter("gui", gui_);
		objectsPath = this->declare_parameter("objects_path", objectsPath);
		sessionPath = this->declare_parameter("session_path", sessionPath);
		settingsPath = this->declare_parameter("settings_path", settingsPath);
		subscribeDepth = this->declare_parameter("subscribe_depth", subscribeDepth);

		RCLCPP_INFO(this->get_logger(), "gui=%d", gui_?1:0);
		RCLCPP_INFO(this->get_logger(), "objects_path=%s", objectsPath.c_str());
		RCLCPP_INFO(this->get_logger(), "session_path=%s", sessionPath.c_str());
		RCLCPP_INFO(this->get_logger(), "settings_path=%s", settingsPath.c_str());
		RCLCPP_INFO(this->get_logger(), "subscribe_depth = %s", subscribeDepth?"true":"false");

		if(settingsPath.empty())
		{
			settingsPath = QDir::homePath().append("/.ros/find_object_2d.ini").toStdString();
		}
		else
		{
			if(!sessionPath.empty())
			{
				RCLCPP_WARN(this->get_logger(), "\"settings_path\" parameter is ignored when \"session_path\" is set.");
			}

			QString path = settingsPath.c_str();
			if(path.contains('~'))
			{
				path.replace('~', QDir::homePath());
				settingsPath = path.toStdString();
			}
		}

		// Load settings, should be loaded before creating other objects
		find_object::Settings::init(settingsPath.c_str());

		findObjectROS_ = new FindObjectROS(this);
		if(!sessionPath.empty())
		{
			if(!objectsPath.empty())
			{
				RCLCPP_WARN(this->get_logger(), "\"objects_path\" parameter is ignored when \"session_path\" is set.");
			}
			if(!findObjectROS_->loadSession(sessionPath.c_str()))
			{
				RCLCPP_ERROR(this->get_logger(), "Failed to load session \"%s\"", sessionPath.c_str());
			}
		}
		else if(!objectsPath.empty())
		{
			QString path = objectsPath.c_str();
			if(path.contains('~'))
			{
				path.replace('~', QDir::homePath());
			}
			if(!findObjectROS_->loadObjects(path))
			{
				RCLCPP_ERROR(this->get_logger(), "No objects loaded from path \"%s\"", path.toStdString().c_str());
			}
		}
		camera_ = new CameraROS(subscribeDepth, this);

		// Catch ctrl-c to close the gui
		setupQuitSignal(gui_);
	}

	void exec(int argc, char ** argv, std::shared_ptr<rclcpp::Node> node)
	{
		camera_->setupExecutor(node);
		if(gui_)
		{
			QApplication app(argc, argv);
			find_object::MainWindow mainWindow(findObjectROS_, camera_); // take ownership

			QObject::connect(
					&mainWindow,
					SIGNAL(objectsFound(const find_object::DetectionInfo &, const find_object::Header &, const cv::Mat &, float)),
					findObjectROS_,
					SLOT(publish(const find_object::DetectionInfo &, const find_object::Header &, const cv::Mat &, float)));

			QStringList topics = camera_->subscribedTopics();
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

			// mainWindow has ownership
			findObjectROS_ = 0;
			camera_ = 0;

			// loop
			mainWindow.startProcessing();
			app.exec();
			find_object::Settings::saveSettings();
		}
		else
		{
			QCoreApplication app(argc, argv);

			// connect stuff:
			QObject::connect(camera_, SIGNAL(imageReceived(const cv::Mat &, const find_object::Header &, const cv::Mat &, float)), findObjectROS_, SLOT(detect(const cv::Mat &, const find_object::Header &, const cv::Mat &, float)));

			//loop
			camera_->start();
			app.exec();

			delete camera_;
			camera_=0;
			delete findObjectROS_;
			findObjectROS_=0;
		}
	}

	virtual ~FindObjectNode()
	{
		delete findObjectROS_;
		delete camera_;
	}

private:
	FindObjectROS * findObjectROS_;
	CameraROS * camera_;
	bool gui_;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<FindObjectNode>();
	node->exec(argc, argv, node);
	rclcpp::shutdown();
}
