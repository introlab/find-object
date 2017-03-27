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

#include "find_object/MainWindow.h"
#include "find_object/Camera.h"
#include "find_object/Settings.h"
#include "find_object/TcpServer.h"
#include "find_object/FindObject.h"
#include "find_object/utilite/ULogger.h"
#include "find_object/ObjWidget.h"
#include "find_object/QtOpenCV.h"

#include "AddObjectDialog.h"
#include "ui_mainWindow.h"
#include "KeypointItem.h"
#include "RectItem.h"
#include "ParametersToolBox.h"
#include "AboutDialog.h"
#include "rtabmap/PdfPlot.h"
#include "Vocabulary.h"
#include "ObjSignature.h"

#include <iostream>
#include <stdio.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv_modules.hpp>
#if CV_MAJOR_VERSION < 3
#include <opencv2/gpu/gpu.hpp>
#else
#include <opencv2/core/cuda.hpp>
#endif

#include <QtCore/QTextStream>
#include <QtCore/QFile>
#include <QtCore/QBuffer>
#include <QtCore/QThread>
#include <QtCore/QLineF>

#include <QFileDialog>
#include <QMessageBox>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QSpinBox>
#include <QStatusBar>
#include <QProgressDialog>
#include <QCloseEvent>
#include <QCheckBox>
#include <QScrollBar>
#include <QInputDialog>

#include "utilite/UDirectory.h"

namespace find_object {

// Camera ownership transferred
MainWindow::MainWindow(FindObject * findObject, Camera * camera, QWidget * parent) :
	QMainWindow(parent),
	camera_(camera),
	findObject_(findObject),
	likelihoodCurve_(0),
	inliersCurve_(0),
	lowestRefreshRate_(99),
	objectsModified_(false),
	tcpServer_(0)
{
	UASSERT(findObject_ != 0);

	ui_ = new Ui_mainWindow();
	ui_->setupUi(this);
	aboutDialog_ = new AboutDialog(this);
	this->setStatusBar(new QStatusBar());

	likelihoodCurve_ = new rtabmap::PdfPlotCurve("Likelihood", &imagesMap_, this);
	inliersCurve_ = new rtabmap::PdfPlotCurve("Inliers", &imagesMap_, this);
	likelihoodCurve_->setPen(QPen(Qt::blue));
	inliersCurve_->setPen(QPen(Qt::red));
	ui_->likelihoodPlot->addCurve(likelihoodCurve_, false);
	ui_->likelihoodPlot->addCurve(inliersCurve_, false);
	ui_->likelihoodPlot->setGraphicsView(true);

	ui_->dockWidget_statistics->setVisible(false);
	ui_->dockWidget_parameters->setVisible(false);
	ui_->dockWidget_plot->setVisible(false);
	ui_->widget_controls->setVisible(false);

	QByteArray geometry;
	QByteArray state;
	Settings::loadWindowSettings(geometry, state);
	this->restoreGeometry(geometry);
	this->restoreState(state);
	lastObjectsUpdateParameters_ = Settings::getParameters();

	ui_->toolBox->setupUi();

	if(!camera_)
	{
		camera_ = new Camera(this);
	}
	else
	{
		camera_->setParent(this);
		ui_->toolBox->getParameterWidget(Settings::kCamera_1deviceId())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kCamera_2imageWidth())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kCamera_3imageHeight())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kCamera_5mediaPath())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kCamera_6useTcpCamera())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kCamera_8port())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kCamera_9queueSize())->setEnabled(false);
		ui_->actionCamera_from_video_file->setVisible(false);
		ui_->actionCamera_from_TCP_IP->setVisible(false);
		ui_->actionCamera_from_directory_of_images->setVisible(false);
		ui_->actionLoad_scene_from_file->setVisible(false);
	}

#if CV_MAJOR_VERSION < 3
	if(cv::gpu::getCudaEnabledDeviceCount() == 0)
#else
	if(cv::cuda::getCudaEnabledDeviceCount() == 0)
#endif
	{
#if FINDOBJECT_NONFREE == 1
		ui_->toolBox->updateParameter(Settings::kFeature2D_SURF_gpu());
		ui_->toolBox->getParameterWidget(Settings::kFeature2D_SURF_gpu())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kFeature2D_SURF_keypointsRatio())->setEnabled(false);
#endif
		ui_->toolBox->updateParameter(Settings::kFeature2D_Fast_gpu());
		ui_->toolBox->updateParameter(Settings::kFeature2D_ORB_gpu());
		ui_->toolBox->updateParameter(Settings::kNearestNeighbor_BruteForce_gpu());
		ui_->toolBox->getParameterWidget(Settings::kFeature2D_Fast_gpu())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kFeature2D_Fast_keypointsRatio())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kFeature2D_ORB_gpu())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kNearestNeighbor_BruteForce_gpu())->setEnabled(false);
	}

	connect((QDoubleSpinBox*)ui_->toolBox->getParameterWidget(Settings::kCamera_4imageRate()),
			SIGNAL(editingFinished()),
			camera_,
			SLOT(updateImageRate()));
	ui_->menuView->addAction(ui_->dockWidget_statistics->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_parameters->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_objects->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_plot->toggleViewAction());
	connect(ui_->toolBox, SIGNAL(parametersChanged(const QStringList &)), this, SLOT(notifyParametersChanged(const QStringList &)));

	ui_->imageView_source->setTextLabel(tr("Press \"space\" to start the camera or drop an image here..."));
	ui_->imageView_source->setMirrorView(Settings::getGeneral_mirrorView());
	connect((QCheckBox*)ui_->toolBox->getParameterWidget(Settings::kGeneral_mirrorView()),
				SIGNAL(stateChanged(int)),
				this,
				SLOT(updateMirrorView()));

	ui_->widget_controls->setVisible(Settings::getGeneral_controlsShown());
	connect((QCheckBox*)ui_->toolBox->getParameterWidget(Settings::kGeneral_controlsShown()),
				SIGNAL(stateChanged(int)),
				this,
				SLOT(showHideControls()));

	//buttons
	connect(ui_->pushButton_restoreDefaults, SIGNAL(clicked()), ui_->toolBox, SLOT(resetCurrentPage()));
	connect(ui_->pushButton_updateObjects, SIGNAL(clicked()), this, SLOT(updateObjects()));
	connect(ui_->horizontalSlider_objectsSize, SIGNAL(valueChanged(int)), this, SLOT(updateObjectsSize()));

	ui_->actionStop_camera->setEnabled(false);
	ui_->actionPause_camera->setEnabled(false);
	ui_->actionSave_objects->setEnabled(false);
	ui_->actionSave_session->setEnabled(false);

	// Actions
	connect(ui_->actionAdd_object_from_scene, SIGNAL(triggered()), this, SLOT(addObjectFromScene()));
	connect(ui_->actionAdd_objects_from_files, SIGNAL(triggered()), this, SLOT(addObjectsFromFiles()));
	connect(ui_->actionLoad_scene_from_file, SIGNAL(triggered()), this, SLOT(loadSceneFromFile()));
	connect(ui_->actionStart_camera, SIGNAL(triggered()), this, SLOT(startProcessing()));
	connect(ui_->actionStop_camera, SIGNAL(triggered()), this, SLOT(stopProcessing()));
	connect(ui_->actionPause_camera, SIGNAL(triggered()), this, SLOT(pauseProcessing()));
	connect(ui_->actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui_->actionSave_objects, SIGNAL(triggered()), this, SLOT(saveObjects()));
	connect(ui_->actionLoad_objects, SIGNAL(triggered()), this, SLOT(loadObjects()));
	connect(ui_->actionCamera_from_video_file, SIGNAL(triggered()), this, SLOT(setupCameraFromVideoFile()));
	connect(ui_->actionCamera_from_directory_of_images, SIGNAL(triggered()), this, SLOT(setupCameraFromImagesDirectory()));
	connect(ui_->actionCamera_from_TCP_IP, SIGNAL(triggered()), this, SLOT(setupCameraFromTcpIp()));
	connect(ui_->actionAbout, SIGNAL(triggered()), aboutDialog_ , SLOT(exec()));
	connect(ui_->actionRestore_all_default_settings, SIGNAL(triggered()), ui_->toolBox, SLOT(resetAllPages()));
	connect(ui_->actionRemove_all_objects, SIGNAL(triggered()), this, SLOT(removeAllObjects()));
	connect(ui_->actionSave_settings, SIGNAL(triggered()), this, SLOT(saveSettings()));
	connect(ui_->actionLoad_settings, SIGNAL(triggered()), this, SLOT(loadSettings()));
	connect(ui_->actionSave_session, SIGNAL(triggered()), this, SLOT(saveSession()));
	connect(ui_->actionLoad_session, SIGNAL(triggered()), this, SLOT(loadSession()));
	connect(ui_->actionSave_vocabulary, SIGNAL(triggered()), this, SLOT(saveVocabulary()));
	connect(ui_->actionLoad_vocabulary, SIGNAL(triggered()), this, SLOT(loadVocabulary()));
	connect(ui_->actionShow_objects_features, SIGNAL(triggered()), this, SLOT(showObjectsFeatures()));
	connect(ui_->actionHide_objects_features, SIGNAL(triggered()), this, SLOT(hideObjectsFeatures()));

	connect(ui_->pushButton_play, SIGNAL(clicked()), this, SLOT(startProcessing()));
	connect(ui_->pushButton_stop, SIGNAL(clicked()), this, SLOT(stopProcessing()));
	connect(ui_->pushButton_pause, SIGNAL(clicked()), this, SLOT(pauseProcessing()));
	connect(ui_->horizontalSlider_frames, SIGNAL(valueChanged(int)), this, SLOT(moveCameraFrame(int)));
	connect(ui_->horizontalSlider_frames, SIGNAL(valueChanged(int)), ui_->label_frame, SLOT(setNum(int)));
	ui_->pushButton_play->setVisible(true);
	ui_->pushButton_pause->setVisible(false);
	ui_->pushButton_stop->setEnabled(false);
	ui_->horizontalSlider_frames->setEnabled(false);
	ui_->label_frame->setVisible(false);

	ui_->objects_area->addAction(ui_->actionAdd_object_from_scene);
	ui_->objects_area->addAction(ui_->actionAdd_objects_from_files);
	ui_->objects_area->setContextMenuPolicy(Qt::ActionsContextMenu);

	ui_->actionStart_camera->setShortcut(Qt::Key_Space);
	ui_->actionPause_camera->setShortcut(Qt::Key_Space);

	ui_->actionCamera_from_video_file->setChecked(!Settings::getCamera_5mediaPath().isEmpty() && !UDirectory::exists(Settings::getCamera_5mediaPath().toStdString()));
	ui_->actionCamera_from_directory_of_images->setChecked(!Settings::getCamera_5mediaPath().isEmpty() && UDirectory::exists(Settings::getCamera_5mediaPath().toStdString()));
	ui_->actionCamera_from_TCP_IP->setChecked(Settings::getCamera_6useTcpCamera());

	ui_->label_ipAddress->setTextInteractionFlags(Qt::TextSelectableByMouse);
	ui_->label_port->setTextInteractionFlags(Qt::TextSelectableByMouse);
	setupTCPServer();

	if(findObject_->objects().size())
	{
		// show objects already loaded in FindObject
		for(QMap<int, ObjSignature *>::const_iterator iter = findObject_->objects().constBegin();
			iter!=findObject_->objects().constEnd();
			++iter)
		{
			ObjWidget * obj = new ObjWidget(iter.key(), iter.value()->keypoints(), iter.value()->words(), cvtCvMat2QImage(iter.value()->image()));
			objWidgets_.insert(obj->id(), obj);
			this->showObject(obj);
		}
		ui_->actionSave_objects->setEnabled(true);
		ui_->actionSave_session->setEnabled(true);
	}
	if(findObject_->vocabulary()->size())
	{
		ui_->label_vocabularySize->setNum(findObject_->vocabulary()->size());
		ui_->actionSave_session->setEnabled(true);
	}


	if(Settings::getGeneral_autoStartCamera())
	{
		// Set 1 msec to see state on the status bar.
		QTimer::singleShot(1, this, SLOT(startProcessing()));
	}

	//Setup drag and drop images
	connect(ui_->imageDrop_objects, SIGNAL(imagesReceived(const QStringList &)), this, SLOT(addObjectsFromFiles(const QStringList &)));
	connect(ui_->imageDrop_scene, SIGNAL(imagesReceived(const QStringList &)), this, SLOT(loadSceneFromFile(const QStringList &)));
}

MainWindow::~MainWindow()
{
	disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
	disconnect(camera_, SIGNAL(finished()), this, SLOT(stopProcessing()));
	camera_->stop();
	qDeleteAll(objWidgets_);
	objWidgets_.clear();
	delete ui_;
	delete findObject_;
}

void MainWindow::closeEvent(QCloseEvent * event)
{
	bool quit = true;
	this->stopProcessing();
	if(objectsModified_ && this->isVisible() && objWidgets_.size())
	{
		int ret = QMessageBox::question(this, tr("Save new objects"), tr("Do you want to save added objects?"), QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
		switch(ret)
		{
		case QMessageBox::Yes:
			quit = this->saveObjects();
			break;
		case QMessageBox::Cancel:
			quit = false;
			break;
		case QMessageBox::No:
		default:
			break;
		}
	}
	if(quit)
	{
		Settings::saveWindowSettings(this->saveGeometry(), this->saveState());
		event->accept();
	}
	else
	{
		event->ignore();
	}
}

void MainWindow::setupTCPServer()
{
	if(tcpServer_)
	{
		tcpServer_->close();
		delete tcpServer_;
	}
	tcpServer_ = new TcpServer(Settings::getGeneral_port(), this);
	connect(this, SIGNAL(objectsFound(find_object::DetectionInfo)), tcpServer_, SLOT(publishDetectionInfo(find_object::DetectionInfo)));
	ui_->label_ipAddress->setText(tcpServer_->getHostAddress().toString());
	ui_->label_port->setNum(tcpServer_->getPort());
	UINFO("Detection sent on port: %d (IP=%s)", tcpServer_->getPort(), tcpServer_->getHostAddress().toString().toStdString().c_str());

	//connect services
	connect(tcpServer_, SIGNAL(addObject(const cv::Mat &, int, const QString &)), this, SLOT(addObjectFromTcp(const cv::Mat &, int, const QString &)));
	connect(tcpServer_, SIGNAL(removeObject(int)), this, SLOT(removeObject(int)));
}

void MainWindow::setSourceImageText(const QString & text)
{
	ui_->imageView_source->setTextLabel(text);
}

void MainWindow::loadSession()
{
	if(objWidgets_.size())
	{
		QMessageBox::StandardButton b = QMessageBox::question(this, tr("Loading session..."),
				tr("There are some objects (%1) already loaded, they will be "
				   "deleted when loading the session. Do you want to continue?").arg(objWidgets_.size()),
				QMessageBox::Yes | QMessageBox::No, QMessageBox::NoButton);
		if(b != QMessageBox::Yes)
		{
			return;
		}
	}

	QString path = QFileDialog::getOpenFileName(this, tr("Load session..."), Settings::workingDirectory(), "*.bin");
	if(!path.isEmpty())
	{
		qDeleteAll(objWidgets_);
		objWidgets_.clear();
		ui_->actionSave_objects->setEnabled(false);
		findObject_->removeAllObjects();

		if(findObject_->loadSession(path))
		{
			//update parameters tool box
			const ParametersMap & parameters = Settings::getParameters();
			for(ParametersMap::const_iterator iter = parameters.begin(); iter!= parameters.constEnd(); ++iter)
			{
				ui_->toolBox->updateParameter(iter.key());
			}

			//update object widgets
			for(QMap<int, ObjSignature *>::const_iterator iter=findObject_->objects().constBegin(); iter!=findObject_->objects().constEnd();++iter)
			{
				if(iter.value())
				{
					ObjWidget * obj = new ObjWidget(iter.key(), iter.value()->keypoints(), iter.value()->words(), cvtCvMat2QImage(iter.value()->image()));
					objWidgets_.insert(obj->id(), obj);
					ui_->actionSave_objects->setEnabled(true);
					ui_->actionSave_session->setEnabled(true);
					this->showObject(obj);

					//update object labels
					QLabel * title = this->findChild<QLabel*>(QString("%1title").arg(iter.value()->id()));
					title->setText(QString("%1 (%2)").arg(iter.value()->id()).arg(QString::number(iter.value()->keypoints().size())));
				}

			}

			QMessageBox::information(this, tr("Session loaded!"), tr("Session \"%1\" successfully loaded (%2 objects)!").arg(path).arg(objWidgets_.size()));

			if(!camera_->isRunning() && !sceneImage_.empty())
			{
				this->update(sceneImage_);
			}
		}
	}
}
void MainWindow::saveSession()
{
	if(objWidgets_.size())
	{
		QString path = QFileDialog::getSaveFileName(this, tr("Save session..."), Settings::workingDirectory(), "*.bin");
		if(!path.isEmpty())
		{
			if(QFileInfo(path).suffix().compare("bin") != 0)
			{
				path.append(".bin");
			}

			if(findObject_->saveSession(path))
			{
				QMessageBox::information(this, tr("Session saved!"), tr("Session \"%1\" successfully saved (%2 objects)!").arg(path).arg(objWidgets_.size()));
			}
		}
	}
}

void MainWindow::loadSettings()
{
	QString path = QFileDialog::getOpenFileName(this, tr("Load settings..."), Settings::workingDirectory(), "*.ini");
	if(!path.isEmpty())
	{
		if(QFileInfo(path).suffix().compare("ini") != 0)
		{
			path.append(".ini");
		}
		loadSettings(path);
	}
}
void MainWindow::saveSettings()
{
	QString path = QFileDialog::getSaveFileName(this, tr("Save settings..."), Settings::workingDirectory(), "*.ini");
	if(!path.isEmpty())
	{
		if(QFileInfo(path).suffix().compare("ini") != 0)
		{
			path.append(".ini");
		}
		saveSettings(path);
	}
}

bool MainWindow::loadSettings(const QString & path)
{
	if(!path.isEmpty() && QFileInfo(path).suffix().compare("ini") == 0)
	{
		QByteArray geometry;
		QByteArray state;
		Settings::loadSettings(path);
		Settings::loadWindowSettings(geometry, state, path);
		this->restoreGeometry(geometry);
		this->restoreState(state);

		//update parameters tool box
		const ParametersMap & parameters = Settings::getParameters();
		for(ParametersMap::const_iterator iter = parameters.begin(); iter!= parameters.constEnd(); ++iter)
		{
			ui_->toolBox->updateParameter(iter.key());
		}

		return true;
	}
	UERROR("Path \"%s\" not valid (should be *.ini)", path.toStdString().c_str());
	return false;
}

bool MainWindow::saveSettings(const QString & path) const
{
	if(!path.isEmpty() && QFileInfo(path).suffix().compare("ini") == 0)
	{
		Settings::saveSettings(path);
		Settings::saveWindowSettings(this->saveGeometry(), this->saveState(), path);
		return true;
	}
	UERROR("Path \"%s\" not valid (should be *.ini)", path.toStdString().c_str());
	return false;
}

int MainWindow::loadObjects(const QString & dirPath, bool recursive)
{
	QList<int> loadedObjects;
	QString formats = Settings::getGeneral_imageFormats().remove('*').remove('.');

	QStringList paths;
	paths.append(dirPath);

	while(paths.size())
	{
		QString currentDir = paths.front();
		UDirectory dir(currentDir.toStdString(), formats.toStdString());
		if(dir.isValid())
		{
			const std::list<std::string> & names = dir.getFileNames(); // sorted in natural order
			for(std::list<std::string>::const_iterator iter=names.begin(); iter!=names.end(); ++iter)
			{
				int id = this->addObjectFromFile((currentDir.toStdString()+dir.separator()+*iter).c_str());
				if(id >= 0)
				{
					loadedObjects.push_back(id);
				}
			}
		}

		paths.pop_front();

		if(recursive)
		{
			QDir d(currentDir);
			QStringList subDirs = d.entryList(QDir::AllDirs|QDir::NoDotAndDotDot, QDir::Name);
			for(int i=subDirs.size()-1; i>=0; --i)
			{
				paths.prepend(currentDir + QDir::separator() + subDirs[i]);
			}
		}
	}

	if(loadedObjects.size())
	{
		this->updateObjects(loadedObjects);
	}

	return loadedObjects.size();
}

int MainWindow::saveObjects(const QString & dirPath)
{
	int count = 0;
	QDir dir(dirPath);
	if(dir.exists())
	{
		for(QMap<int, ObjWidget*>::const_iterator iter=objWidgets_.constBegin(); iter!=objWidgets_.constEnd(); ++iter)
		{
			if(iter.value()->pixmap().save(QString("%1/%2.png").arg(dirPath).arg(iter.key())))
			{
				++count;
			}
			else
			{
				UERROR("Failed to save object %d", iter.key());
			}
		}
		objectsModified_ = false;
	}
	return count;
}

void MainWindow::loadObjects()
{
	QString dirPath = QFileDialog::getExistingDirectory(this, tr("Loading objects... Select a directory."), Settings::workingDirectory());
	if(!dirPath.isEmpty())
	{
		QDir d(dirPath);
		bool recursive = false;
		if(d.entryList(QDir::AllDirs | QDir::NoDotAndDotDot).size())
		{
			QMessageBox::StandardButton b = QMessageBox::question(
					this,
					tr("Loading objects..."),
					tr("The current directory contains subdirectories. Load objects recursively?"),
					QMessageBox::Yes|QMessageBox::No,
					QMessageBox::No);
			recursive = b == QMessageBox::Yes;
		}

		int count = loadObjects(dirPath, recursive);
		if(count)
		{
			QMessageBox::information(this, tr("Loading..."), tr("%1 objects loaded from \"%2\".").arg(count).arg(dirPath));
		}
		else
		{
			QMessageBox::information(this, tr("Loading..."), tr("No objects loaded from \"%1\"!").arg(dirPath));
		}
	}
}
bool MainWindow::saveObjects()
{
	QString dirPath = QFileDialog::getExistingDirectory(this, tr("Saving objects... Select a directory."), Settings::workingDirectory());
	if(!dirPath.isEmpty())
	{
		int count = saveObjects(dirPath);
		if(count)
		{
			QMessageBox::information(this, tr("Saving..."), tr("%1 objects saved to \"%2\".").arg(count).arg(dirPath));
		}
		else
		{
			QMessageBox::warning(this, tr("Saving..."), tr("No objects saved to %1!").arg(dirPath));
		}
		return count > 0;
	}
	return false;
}

void MainWindow::loadVocabulary()
{
	if(!Settings::getGeneral_vocabularyFixed() ||
	   !Settings::getGeneral_invertedSearch())
	{
		QMessageBox::StandardButton b = QMessageBox::question(this, tr("Load vocabulary..."),
				tr("Parameters \"General/vocabularyFixed\" and \"General/invertedSearch\" should be enabled to load a vocabulary. "
					"Do you want to enable them now?"),
					QMessageBox::Cancel | QMessageBox::Yes);
		if(b == QMessageBox::Yes)
		{
			Settings::setGeneral_vocabularyFixed(true);
			Settings::setGeneral_invertedSearch(true);
		}
	}
	if(Settings::getGeneral_vocabularyFixed() &&
	   Settings::getGeneral_invertedSearch())
	{
		QString path = QFileDialog::getOpenFileName(this, tr("Load vocabulary..."), Settings::workingDirectory(), "Data (*.yaml *.xml)");
		if(!path.isEmpty())
		{
			if(findObject_->loadVocabulary(path))
			{
				ui_->label_vocabularySize->setNum(findObject_->vocabulary()->size());
				ui_->actionSave_session->setEnabled(findObject_->vocabulary()->size() || findObject_->objects().size());
				QMessageBox::information(this, tr("Loading..."), tr("Vocabulary loaded from \"%1\" (%2 words).").arg(path).arg(findObject_->vocabulary()->size()));
			}
			else
			{
				QMessageBox::warning(this, tr("Loading..."), tr("Failed to load vocabulary \"%1\"!").arg(path));
			}
		}
	}
}

void MainWindow::saveVocabulary()
{
	if(findObject_->vocabulary()->size() == 0)
	{
		QMessageBox::warning(this, tr("Saving vocabulary..."), tr("Vocabulary is empty!"));
		return;
	}
	QString path = QFileDialog::getSaveFileName(this, tr("Save vocabulary..."), Settings::workingDirectory(), "Data (*.yaml *.xml)");
	if(!path.isEmpty())
	{
		if(QFileInfo(path).suffix().compare("yaml") != 0 && QFileInfo(path).suffix().compare("xml") != 0)
		{
			path.append(".yaml");
		}
		if(findObject_->saveVocabulary(path))
		{
			QMessageBox::information(this, tr("Saving..."), tr("Vocabulary saved to \"%1\" (%2 words).").arg(path).arg(findObject_->vocabulary()->size()));
		}
		else
		{
			QMessageBox::warning(this, tr("Saving..."), tr("Failed to save vocabulary \"%1\"!").arg(path));
		}
	}
}

void MainWindow::removeObject(find_object::ObjWidget * object)
{
	if(object)
	{
		objWidgets_.remove(object->id());
		if(objWidgets_.size() == 0)
		{
			ui_->actionSave_objects->setEnabled(false);
			ui_->actionSave_session->setEnabled(false);
		}
		findObject_->removeObject(object->id());
		object->deleteLater();
		if(Settings::getGeneral_autoUpdateObjects())
		{
			this->updateVocabulary();
		}
		if(!camera_->isRunning() && !sceneImage_.empty())
		{
			this->update(sceneImage_);
		}
	}
}

void MainWindow::removeObject(int id)
{
	if(objWidgets_.contains(id))
	{
		removeObject(objWidgets_[id]);
	}
	else
	{
		UERROR("Remove object: Object %d not found!", id);
	}
}

void MainWindow::removeAllObjects()
{
	qDeleteAll(objWidgets_);
	objWidgets_.clear();
	ui_->actionSave_objects->setEnabled(false);
	findObject_->removeAllObjects();
	if(!camera_->isRunning() && !sceneImage_.empty())
	{
		this->update(sceneImage_);
	}
}

void MainWindow::updateObjectsSize()
{
	for(QMap<int, ObjWidget*>::iterator iter=objWidgets_.begin(); iter!=objWidgets_.end(); ++iter)
	{
		updateObjectSize(iter.value());
	}
}

void MainWindow::updateObjectSize(ObjWidget * obj)
{
	if(obj)
	{
		int value = ui_->horizontalSlider_objectsSize->value();
		if((obj->pixmap().width()*value)/100 > 4 && (obj->pixmap().height()*value)/100 > 4)
		{
			obj->setVisible(true);
			obj->setMinimumSize((obj->pixmap().width()*value)/100, (obj->pixmap().height())*value/100);
		}
		else
		{
			obj->setVisible(false);
		}
	}
}

void MainWindow::updateMirrorView()
{
	bool mirrorView = Settings::getGeneral_mirrorView();
	ui_->imageView_source->setMirrorView(mirrorView);
	for(QMap<int, ObjWidget*>::iterator iter=objWidgets_.begin(); iter!=objWidgets_.end(); ++iter)
	{
		iter.value()->setMirrorView(mirrorView);
	}
}

void MainWindow::showHideControls()
{
	ui_->widget_controls->setVisible(Settings::getGeneral_controlsShown());
}

void MainWindow::showObjectsFeatures()
{
	for(QMap<int, ObjWidget*>::iterator iter=objWidgets_.begin(); iter!=objWidgets_.end(); ++iter)
	{
		iter.value()->setFeaturesShown(true);
	}
}

void MainWindow::hideObjectsFeatures()
{
	for(QMap<int, ObjWidget*>::iterator iter=objWidgets_.begin(); iter!=objWidgets_.end(); ++iter)
	{
		iter.value()->setFeaturesShown(false);
	}
}

void MainWindow::addObjectFromScene()
{
	disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
	disconnect(camera_, SIGNAL(finished()), this, SLOT(stopProcessing()));
	AddObjectDialog * dialog;
	bool resumeCamera = camera_->isRunning();
	if(camera_->isRunning() || sceneImage_.empty())
	{
		dialog = new AddObjectDialog(camera_, cv::Mat(), ui_->imageView_source->isMirrorView(), this);
	}
	else
	{
		dialog = new AddObjectDialog(0, sceneImage_, ui_->imageView_source->isMirrorView(), this);
	}
	if(dialog->exec() == QDialog::Accepted)
	{
		ObjWidget * obj = 0;
		ObjSignature * signature = 0;
		dialog->retrieveObject(&obj, &signature);
		UASSERT(obj!=0 && signature!=0);
		findObject_->addObject(signature);
		obj->setId(signature->id());
		objWidgets_.insert(obj->id(), obj);
		ui_->actionSave_objects->setEnabled(true);
		ui_->actionSave_session->setEnabled(true);
		showObject(obj);
		QList<int> ids;
		ids.push_back(obj->id());
		updateVocabulary(ids);
		objectsModified_ = true;
	}
	if(resumeCamera || sceneImage_.empty())
	{
		this->startProcessing();
	}
	else
	{
		connect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)), Qt::UniqueConnection);
		connect(camera_, SIGNAL(finished()), this, SLOT(stopProcessing()), Qt::UniqueConnection);
		if(!sceneImage_.empty())
		{
			this->update(sceneImage_);
		}
	}
	delete dialog;
}

void MainWindow::addObjectsFromFiles(const QStringList & fileNames)
{
	if(fileNames.size())
	{
		QList<int> ids;
		for(int i=0; i<fileNames.size(); ++i)
		{
			int id = this->addObjectFromFile(fileNames.at(i));
			if(id >= 0)
			{
				ids.push_back(id);
			}
		}
		if(ids.size())
		{
			objectsModified_ = true;
			updateObjects(ids);
		}
	}
}

void MainWindow::addObjectsFromFiles()
{
	addObjectsFromFiles(QFileDialog::getOpenFileNames(this, tr("Add objects..."), Settings::workingDirectory(), tr("Image Files (%1)").arg(Settings::getGeneral_imageFormats())));
}

int MainWindow::addObjectFromFile(const QString & filePath)
{
	const ObjSignature * s = findObject_->addObject(filePath);
	if(s)
	{
		ObjWidget * obj = new ObjWidget(s->id(), std::vector<cv::KeyPoint>(), QMultiMap<int,int>(), cvtCvMat2QImage(s->image()));
		objWidgets_.insert(obj->id(), obj);
		ui_->actionSave_objects->setEnabled(true);
		ui_->actionSave_session->setEnabled(true);
		this->showObject(obj);
		return s->id();
	}
	else
	{
		QMessageBox::critical(this, tr("Error adding object"), tr("Failed to add object from \"%1\"").arg(filePath));
		return -1;
	}
}

void MainWindow::addObjectFromTcp(const cv::Mat & image, int id, const QString & filePath)
{
	if(objWidgets_.contains(id))
	{
		UERROR("Add Object: Object %d is already added.", id);
	}
	const ObjSignature * s = findObject_->addObject(image, id, filePath);
	if(s)
	{
		ObjWidget * obj = new ObjWidget(s->id(), std::vector<cv::KeyPoint>(), QMultiMap<int,int>(), cvtCvMat2QImage(s->image()));
		objWidgets_.insert(obj->id(), obj);
		ui_->actionSave_objects->setEnabled(true);
		ui_->actionSave_session->setEnabled(true);
		this->showObject(obj);
		QList<int> ids;
		ids.push_back(obj->id());
		updateObjects(ids);
	}
	else
	{
		UERROR("Add Object: Error adding object %d.", id);
	}
}

void MainWindow::loadSceneFromFile(const QStringList & fileNames)
{
	//take the first
	if(fileNames.size())
	{
		cv::Mat img = cv::imread(fileNames.first().toStdString().c_str());
		if(!img.empty())
		{
			this->update(img);
			ui_->label_timeRefreshRate->setVisible(false);
		}
	}
}

void MainWindow::loadSceneFromFile()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Load scene..."), Settings::workingDirectory(), tr("Image Files (%1)").arg(Settings::getGeneral_imageFormats()));
	if(!fileName.isEmpty())
	{
		cv::Mat img = cv::imread(fileName.toStdString().c_str());
		if(!img.empty())
		{
			this->update(img);
			ui_->label_timeRefreshRate->setVisible(false);
		}
	}
}

void MainWindow::setupCameraFromVideoFile()
{
	if(!ui_->actionCamera_from_video_file->isChecked())
	{
		Settings::setCamera_5mediaPath("");
		ui_->toolBox->updateParameter(Settings::kCamera_5mediaPath());
	}
	else
	{
		QString fileName = QFileDialog::getOpenFileName(this, tr("Setup camera from video file..."), Settings::workingDirectory(), tr("Video Files (%1)").arg(Settings::getGeneral_videoFormats()));
		if(!fileName.isEmpty())
		{
			Settings::setCamera_6useTcpCamera(false);
			ui_->toolBox->updateParameter(Settings::kCamera_6useTcpCamera());

			Settings::setCamera_5mediaPath(fileName);
			ui_->toolBox->updateParameter(Settings::kCamera_5mediaPath());
			if(camera_->isRunning())
			{
				this->stopProcessing();
				this->startProcessing();
			}
			Settings::setGeneral_controlsShown(true);
			ui_->toolBox->updateParameter(Settings::kGeneral_controlsShown());
		}
	}
	ui_->actionCamera_from_video_file->setChecked(!Settings::getCamera_5mediaPath().isEmpty());
	ui_->actionCamera_from_directory_of_images->setChecked(false);
	ui_->actionCamera_from_TCP_IP->setChecked(false);
}

void MainWindow::setupCameraFromImagesDirectory()
{
	if(!ui_->actionCamera_from_directory_of_images->isChecked())
	{
		Settings::setCamera_5mediaPath("");
		ui_->toolBox->updateParameter(Settings::kCamera_5mediaPath());
	}
	else
	{
		QString directory = QFileDialog::getExistingDirectory(this, tr("Setup camera from directory of images..."), Settings::workingDirectory());
		if(!directory.isEmpty())
		{
			Settings::setCamera_6useTcpCamera(false);
			ui_->toolBox->updateParameter(Settings::kCamera_6useTcpCamera());

			Settings::setCamera_5mediaPath(directory);
			ui_->toolBox->updateParameter(Settings::kCamera_5mediaPath());
			if(camera_->isRunning())
			{
				this->stopProcessing();
				this->startProcessing();
			}
			Settings::setGeneral_controlsShown(true);
			ui_->toolBox->updateParameter(Settings::kGeneral_controlsShown());
		}
	}
	ui_->actionCamera_from_directory_of_images->setChecked(!Settings::getCamera_5mediaPath().isEmpty());
	ui_->actionCamera_from_video_file->setChecked(false);
	ui_->actionCamera_from_TCP_IP->setChecked(false);
}

void MainWindow::setupCameraFromTcpIp()
{
	if(!ui_->actionCamera_from_TCP_IP->isChecked())
	{
		Settings::setCamera_6useTcpCamera(false);
		ui_->toolBox->updateParameter(Settings::kCamera_6useTcpCamera());
	}
	else
	{
		bool ok;
		int port = QInputDialog::getInt(this, tr("Server port..."), "Port: ", Settings::getCamera_8port(), 1, USHRT_MAX, 1, &ok);

		if(ok)
		{
			int queue = QInputDialog::getInt(this, tr("Queue size..."), "Images buffer size (0 means infinite): ", Settings::getCamera_9queueSize(), 0, 2147483647, 1, &ok);
			if(ok)
			{
				Settings::setCamera_6useTcpCamera(true);
				ui_->toolBox->updateParameter(Settings::kCamera_6useTcpCamera());
				Settings::setCamera_8port(port);
				ui_->toolBox->updateParameter(Settings::kCamera_8port());
				Settings::setCamera_9queueSize(queue);
				ui_->toolBox->updateParameter(Settings::kCamera_9queueSize());
				if(camera_->isRunning())
				{
					this->stopProcessing();
				}
				this->startProcessing();
			}
		}
	}
	ui_->actionCamera_from_directory_of_images->setChecked(false);
	ui_->actionCamera_from_video_file->setChecked(false);
	ui_->actionCamera_from_TCP_IP->setChecked(Settings::getCamera_6useTcpCamera());
}

void MainWindow::showObject(ObjWidget * obj)
{
	if(obj)
	{
		obj->setGraphicsViewMode(false);
		obj->setMirrorView(ui_->imageView_source->isMirrorView());
		QList<ObjWidget*> objs = ui_->objects_area->findChildren<ObjWidget*>();
		QVBoxLayout * vLayout = new QVBoxLayout();
		ui_->toolBox->updateParameter(Settings::kGeneral_nextObjID());

		QLabel * title = new QLabel(QString("%1 (%2)").arg(obj->id()).arg(obj->keypoints().size()), this);
		QLabel * detectedLabel = new QLabel(this);
		title->setObjectName(QString("%1title").arg(obj->id()));
		detectedLabel->setObjectName(QString("%1detection").arg(obj->id()));
		QHBoxLayout * hLayout = new QHBoxLayout();
		hLayout->addWidget(title);
		hLayout->addStretch(1);
		hLayout->addStretch(1);
		hLayout->addWidget(detectedLabel);
		vLayout->addLayout(hLayout);
		vLayout->addWidget(obj);
		obj->setDeletable(true);
		connect(obj, SIGNAL(removalTriggered(find_object::ObjWidget*)), this, SLOT(removeObject(find_object::ObjWidget*)));
		connect(obj, SIGNAL(destroyed(QObject *)), title, SLOT(deleteLater()));
		connect(obj, SIGNAL(destroyed(QObject *)), detectedLabel, SLOT(deleteLater()));
		connect(obj, SIGNAL(destroyed(QObject *)), vLayout, SLOT(deleteLater()));
		ui_->verticalLayout_objects->insertLayout(ui_->verticalLayout_objects->count()-1, vLayout);

		QByteArray ba;
		QBuffer buffer(&ba);
		buffer.open(QIODevice::WriteOnly);
		obj->pixmap().scaledToWidth(128).save(&buffer, "JPEG"); // writes image into JPEG format
		imagesMap_.insert(obj->id(), ba);

		// update objects size slider
		int objectsPanelWidth = ui_->dockWidget_objects->width();
		if(objectsPanelWidth > 0 && (obj->pixmap().width()*ui_->horizontalSlider_objectsSize->value()) / 100 > objectsPanelWidth)
		{
			ui_->horizontalSlider_objectsSize->setValue((objectsPanelWidth * 100) / obj->pixmap().width());
		}
		else
		{
			updateObjectSize(obj);
		}
	}
}

void MainWindow::updateObjects()
{
	updateObjects(QList<int>());
}

void MainWindow::updateObjects(const QList<int> & ids)
{
	if(objWidgets_.size())
	{
		this->statusBar()->showMessage(tr("Updating %1 objects...").arg(ids.size()==0?objWidgets_.size():ids.size()));

		findObject_->updateObjects(ids);

		QList<int> idsTmp = ids;
		if(idsTmp.size() == 0)
		{
			idsTmp = objWidgets_.keys();
		}

		QList<ObjSignature*> signatures = findObject_->objects().values();
		for(int i=0; i<signatures.size(); ++i)
		{
			if(idsTmp.contains(signatures[i]->id()))
			{
				objWidgets_.value(signatures[i]->id())->updateData(signatures[i]->keypoints());

				//update object labels
				QLabel * title = this->findChild<QLabel*>(QString("%1title").arg(signatures[i]->id()));
				title->setText(QString("%1 (%2)").arg(signatures[i]->id()).arg(QString::number(signatures[i]->keypoints().size())));

				QLabel * label = this->findChild<QLabel*>(QString("%1detection").arg(signatures[i]->id()));
				label->clear();
			}
		}

		updateVocabulary(ids);

		if(!camera_->isRunning() && !sceneImage_.empty())
		{
			this->update(sceneImage_);
		}
		this->statusBar()->clearMessage();
	}
}

void MainWindow::updateVocabulary(const QList<int> & ids)
{
	this->statusBar()->showMessage(tr("Updating vocabulary..."));

	QTime time;
	time.start();
	findObject_->updateVocabulary(ids);

	QList<int> idsTmp = ids;
	if(idsTmp.size() == 0)
	{
		idsTmp = objWidgets_.keys();
	}
	QList<ObjSignature*> signatures = findObject_->objects().values();
	for(int i=0; i<signatures.size(); ++i)
	{
		if(idsTmp.contains(signatures[i]->id()))
		{
			objWidgets_.value(signatures[i]->id())->updateWords(signatures[i]->words());
		}
	}

	ui_->label_timeIndexing->setNum(time.elapsed());
	ui_->label_vocabularySize->setNum(findObject_->vocabulary()->size());
	if(ids.size() && findObject_->vocabulary()->size() == 0 && Settings::getGeneral_vocabularyFixed() && Settings::getGeneral_invertedSearch())
	{
		QMessageBox::warning(this, tr("Vocabulary update"), tr("\"General/VocabularyFixed=true\" and the "
				"vocabulary is empty. New features cannot be matched to any words in the vocabulary."));
	}
	lastObjectsUpdateParameters_ = Settings::getParameters();
	this->statusBar()->clearMessage();
	ui_->dockWidget_objects->update();
}

void MainWindow::startProcessing()
{
	UINFO("Starting camera...");
	bool updateStatusMessage = this->statusBar()->currentMessage().isEmpty();
	if(updateStatusMessage)
	{
		this->statusBar()->showMessage(tr("Starting camera..."));
	}
	if(camera_->start())
	{
		connect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)), Qt::UniqueConnection);
		connect(camera_, SIGNAL(finished()), this, SLOT(stopProcessing()), Qt::UniqueConnection);
		ui_->actionStop_camera->setEnabled(true);
		ui_->actionPause_camera->setEnabled(true);
		ui_->actionStart_camera->setEnabled(false);
		ui_->actionLoad_scene_from_file->setEnabled(false);
		ui_->actionCamera_from_directory_of_images->setEnabled(false);
		ui_->actionCamera_from_video_file->setEnabled(false);
		ui_->actionCamera_from_TCP_IP->setEnabled(false);
		ui_->label_timeRefreshRate->setVisible(true);

		//update control bar
		ui_->pushButton_play->setVisible(false);
		ui_->pushButton_pause->setVisible(true);
		ui_->pushButton_stop->setEnabled(true);
		int totalFrames = camera_->getTotalFrames();
		if(totalFrames>0)
		{
			ui_->label_frame->setVisible(true);
			ui_->horizontalSlider_frames->setEnabled(true);
			ui_->horizontalSlider_frames->setMaximum(totalFrames-1);
		}

		//update camera port if TCP is used
		ui_->label_port_image->setText("-");
		if(Settings::getCamera_6useTcpCamera() && camera_->getPort())
		{
			ui_->label_port_image->setNum(camera_->getPort());
		}

		if(updateStatusMessage)
		{
			this->statusBar()->showMessage(tr("Camera started."), 2000);
		}
	}
	else
	{
		if(updateStatusMessage)
		{
			this->statusBar()->clearMessage();
		}

		if(Settings::getCamera_6useTcpCamera())
		{
			QMessageBox::critical(this, tr("Camera error"), tr("Camera initialization failed! (with port %1)").arg(Settings::getCamera_8port()));
		}
		else
		{
			QMessageBox::critical(this, tr("Camera error"), tr("Camera initialization failed! (with device %1)").arg(Settings::getCamera_1deviceId()));
		}
	}
}

void MainWindow::stopProcessing()
{
	if(camera_)
	{
		disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
		disconnect(camera_, SIGNAL(finished()), this, SLOT(stopProcessing()));
		camera_->stop();
	}
	ui_->actionStop_camera->setEnabled(false);
	ui_->actionPause_camera->setEnabled(false);
	ui_->actionStart_camera->setEnabled(true);
	ui_->actionLoad_scene_from_file->setEnabled(true);
	ui_->actionCamera_from_directory_of_images->setEnabled(true);
	ui_->actionCamera_from_video_file->setEnabled(true);
	ui_->actionCamera_from_TCP_IP->setEnabled(true);
	ui_->pushButton_play->setVisible(true);
	ui_->pushButton_pause->setVisible(false);
	ui_->pushButton_stop->setEnabled(false);
	ui_->horizontalSlider_frames->setEnabled(false);
	ui_->horizontalSlider_frames->setValue(0);
	ui_->label_frame->setVisible(false);
	ui_->label_port_image->setText("-");
}

void MainWindow::pauseProcessing()
{
	ui_->actionStop_camera->setEnabled(true);
	ui_->actionPause_camera->setEnabled(true);
	ui_->actionStart_camera->setEnabled(false);
	if(camera_->isRunning())
	{
		ui_->pushButton_play->setVisible(true);
		ui_->pushButton_pause->setVisible(false);
		camera_->pause();
	}
	else
	{
		ui_->pushButton_play->setVisible(false);
		ui_->pushButton_pause->setVisible(true);
		camera_->start();
	}
}

void MainWindow::moveCameraFrame(int frame)
{
	if(ui_->horizontalSlider_frames->isEnabled())
	{
		camera_->moveToFrame(frame);
		if(!camera_->isRunning())
		{
			camera_->takeImage();
		}
	}
}

void MainWindow::rectHovered(int objId)
{
	if(objId>=0 && Settings::getGeneral_autoScroll())
	{
		QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1title").arg(objId));
		if(label)
		{
			ui_->objects_area->verticalScrollBar()->setValue(label->pos().y());
		}
	}
}

void MainWindow::update(const cv::Mat & image)
{
	if(image.empty())
	{
		UWARN("The image received is empty...");
		return;
	}
	sceneImage_ = image.clone();

	// reset objects color
	for(QMap<int, ObjWidget*>::iterator iter=objWidgets_.begin(); iter!=objWidgets_.end(); ++iter)
	{
		iter.value()->resetKptsColor();
		if(!Settings::getGeneral_invertedSearch())
		{
			iter.value()->resetKptsWordID();
		}
	}

	QTime guiRefreshTime;

	DetectionInfo info;
	if(findObject_->detect(sceneImage_, info))
	{
		guiRefreshTime.start();
		ui_->label_timeDetection->setNum(info.timeStamps_.value(DetectionInfo::kTimeKeypointDetection, 0));
		ui_->label_timeSkewAffine->setNum(info.timeStamps_.value(DetectionInfo::kTimeSkewAffine, 0));
		ui_->label_timeExtraction->setNum(info.timeStamps_.value(DetectionInfo::kTimeDescriptorExtraction, 0));
		ui_->label_timeSubPix->setNum(info.timeStamps_.value(DetectionInfo::kTimeSubPixelRefining, 0));
		ui_->imageView_source->updateImage(cvtCvMat2QImage(sceneImage_));
		ui_->imageView_source->updateData(info.sceneKeypoints_, info.sceneWords_);
		if(!findObject_->vocabulary()->size())
		{
			ui_->label_timeIndexing->setNum(info.timeStamps_.value(DetectionInfo::kTimeIndexing, 0));
		}
		ui_->label_timeMatching->setNum(info.timeStamps_.value(DetectionInfo::kTimeMatching, 0));
		ui_->label_timeHomographies->setNum(info.timeStamps_.value(DetectionInfo::kTimeHomography, 0));

		ui_->label_vocabularySize->setNum(findObject_->vocabulary()->size());

		// Colorize features matched
		const QMap<int, QMultiMap<int, int> > & matches = info.matches_;
		QMap<int, int> scores;
		int maxScoreId = -1;
		int maxScore = 0;
		for(QMap<int, QMultiMap<int, int> >::const_iterator jter=matches.constBegin(); jter!=matches.end();++jter)
		{
			scores.insert(jter.key(), jter.value().size());
			if(maxScoreId == -1 || maxScore < jter.value().size())
			{
				maxScoreId = jter.key();
				maxScore = jter.value().size();
			}

			int id = jter.key();
			QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1detection").arg(id));
			if(!Settings::getHomography_homographyComputed())
			{
				label->setText(QString("%1 matches").arg(jter.value().size()));

				ObjWidget * obj = objWidgets_.value(id);
				UASSERT(obj != 0);

				for(QMultiMap<int, int>::const_iterator iter = jter.value().constBegin(); iter!= jter.value().constEnd(); ++iter)
				{
					obj->setKptColor(iter.key(), obj->color());
					ui_->imageView_source->setKptColor(iter.value(), obj->color());
					if(!Settings::getGeneral_invertedSearch())
					{
						obj->setKptWordID(iter.key(), ui_->imageView_source->words().value(iter.value(), -1));
					}
				}
			}
			else if(!info.objDetected_.contains(id))
			{
				// Homography could not be computed...
				QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1detection").arg(id));
				QMultiMap<int, int> rejectedInliers = info.rejectedInliers_.value(id);
				QMultiMap<int, int> rejectedOutliers = info.rejectedOutliers_.value(id);
				int rejectedCode = info.rejectedCodes_.value(id);
				if(rejectedCode == DetectionInfo::kRejectedLowMatches)
				{
					label->setText(QString("Too low matches (%1)").arg(jter.value().size()));
				}
				else if(rejectedCode == DetectionInfo::kRejectedAllInliers)
				{
					label->setText(QString("Ignored, all inliers (%1 in %2 out)").arg(rejectedInliers.size()).arg(rejectedOutliers.size()));
				}
				else if(rejectedCode == DetectionInfo::kRejectedNotValid)
				{
					label->setText(QString("Not valid homography (%1 in %2 out)").arg(rejectedInliers.size()).arg(rejectedOutliers.size()));
				}
				else if(rejectedCode == DetectionInfo::kRejectedLowInliers)
				{
					label->setText(QString("Too low inliers (%1 in %2 out)").arg(rejectedInliers.size()).arg(rejectedOutliers.size()));
				}
				else if(rejectedCode == DetectionInfo::kRejectedCornersOutside)
				{
					label->setText(QString("Corners not visible (%1 in %2 out)").arg(rejectedInliers.size()).arg(rejectedOutliers.size()));
				}
				else if(rejectedCode == DetectionInfo::kRejectedByAngle)
				{
					label->setText(QString("Angle too small (%1 in %2 out)").arg(rejectedInliers.size()).arg(rejectedOutliers.size()));
				}
			}
		}

		if(info.objDetected_.size() && camera_->isRunning() && Settings::getGeneral_autoPauseOnDetection())
		{
			this->pauseProcessing();
		}

		// Add homography rectangles when homographies are computed
		int maxHomographyScoreId = -1;
		int maxHomographyScore = 0;
		QMultiMap<int, QMultiMap<int,int> >::const_iterator inliersIter = info.objDetectedInliers_.constBegin();
		QMultiMap<int, QMultiMap<int,int> >::const_iterator outliersIter = info.objDetectedOutliers_.constBegin();
		for(QMultiMap<int,QTransform>::iterator iter = info.objDetected_.begin();
				iter!=info.objDetected_.end();
				++iter, ++inliersIter, ++outliersIter)
		{
			int id = iter.key();

			if(maxHomographyScoreId == -1 || maxHomographyScore < inliersIter.value().size())
			{
				maxHomographyScoreId = id;
				maxHomographyScore = inliersIter.value().size();
			}

			ObjWidget * obj = objWidgets_.value(id);
			UASSERT(obj != 0);

			// COLORIZE (should be done in the GUI thread)
			QTransform hTransform = iter.value();

			QRect rect = obj->pixmap().rect();
			// add rectangle
			QPen rectPen(obj->color());
			rectPen.setWidth(Settings::getHomography_rectBorderWidth());
			RectItem * rectItemScene = new RectItem(id, rect);
			connect(rectItemScene, SIGNAL(hovered(int)), this, SLOT(rectHovered(int)));
			rectItemScene->setPen(rectPen);
			rectItemScene->setTransform(hTransform);
			ui_->imageView_source->addRect(rectItemScene);

			QGraphicsRectItem * rectItemObj = new QGraphicsRectItem(rect);
			rectItemObj->setPen(rectPen);
			obj->addRect(rectItemObj);

			for(QMultiMap<int, int>::const_iterator iter = inliersIter.value().constBegin(); iter!= inliersIter.value().constEnd(); ++iter)
			{
				obj->setKptColor(iter.key(), obj->color());
				ui_->imageView_source->setKptColor(iter.value(), obj->color());
				if(!Settings::getGeneral_invertedSearch())
				{
					obj->setKptWordID(iter.key(), ui_->imageView_source->words().value(iter.value(), -1));
				}
			}

			QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1detection").arg(id));
			if(info.objDetected_.count(id) > 1)
			{
				// if a homography is already found, set the objects count
				label->setText(QString("%1 objects found").arg(info.objDetected_.count(id)));
			}
			else
			{
				label->setText(QString("%1 in %2 out").arg(inliersIter.value().size()).arg(outliersIter.value().size()));
			}
		}

		// save screenshot of the detection
		if(info.objDetected_.size() && !Settings::getGeneral_autoScreenshotPath().isEmpty())
		{
			QDir dir(Settings::getGeneral_autoScreenshotPath());
			if(!dir.exists())
			{
				QMessageBox::warning(this, tr("Screenshot on detection"), tr("Directory \"%1\" doesn't "
						"exist, screenshot of the detection cannot be taken. Parameter \"%2\" is cleared.").arg(Settings::getGeneral_autoScreenshotPath()).arg(Settings::kGeneral_autoScreenshotPath()));
				Settings::setGeneral_autoScreenshotPath("");
				ui_->toolBox->updateParameter(Settings::kGeneral_autoScreenshotPath());
			}
			else
			{
				QString path = Settings::getGeneral_autoScreenshotPath() + QDir::separator() + (QDateTime::currentDateTime().toString("yyMMddhhmmsszzz") + ".jpg");
				if(!ui_->imageView_source->getSceneAsPixmap().save(path))
				{
					UDEBUG("Failed to save screenshot \"%s\"! (%s is set)", path.toStdString().c_str(), Settings::kGeneral_autoScreenshotPath().toStdString().c_str());
				}
				else
				{
					UINFO("Save screenshot \"%s\"! (%s is set)", path.toStdString().c_str(), Settings::kGeneral_autoScreenshotPath().toStdString().c_str());
				}
			}
		}

		//update likelihood plot
		UDEBUG("Set likelihood score curve values (%d)", scores.size());
		likelihoodCurve_->setData(scores, QMap<int, int>());
		QMap<int, int> inlierScores;
		for(QMap<int, int>::iterator iter=scores.begin(); iter!=scores.end(); ++iter)
		{
			QList<QMultiMap<int, int> > values = info.objDetectedInliers_.values(iter.key());
			int maxValue = 0;
			if(values.size())
			{
				maxValue = values[0].size();
				for(int i=1; i<values.size(); ++i)
				{
					if(maxValue < values[i].size())
					{
						maxValue = values[i].size();
					}
				}
			}
			inlierScores.insert(iter.key(), maxValue);
		}
		UDEBUG("Set inliers score curve values (%d)", inlierScores.size());
		inliersCurve_->setData(inlierScores, QMap<int, int>());
		if(ui_->likelihoodPlot->isVisible())
		{
			ui_->likelihoodPlot->update();
		}

		ui_->label_minMatchedDistance->setNum(info.minMatchedDistance_);
		ui_->label_maxMatchedDistance->setNum(info.maxMatchedDistance_);

		//Scroll objects slider to the best score
		if((maxScoreId>=0 || maxHomographyScoreId>=0) && Settings::getGeneral_autoScroll())
		{
			QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1title").arg(maxHomographyScoreId>=0?maxHomographyScoreId:maxScoreId));
			if(label)
			{
				ui_->objects_area->verticalScrollBar()->setValue(label->pos().y());
			}
		}

		// Emit homographies
		if(info.objDetected_.size() > 1)
		{
			UINFO("(%s) %d objects detected!",
					QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
					(int)info.objDetected_.size());
		}
		else if(info.objDetected_.size() == 1)
		{
			UINFO("(%s) Object %d detected!",
					QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
					(int)info.objDetected_.begin().key());
		}
		else if(Settings::getGeneral_sendNoObjDetectedEvents())
		{
			UINFO("(%s) No objects detected.",
					QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str());
		}

		if(info.objDetected_.size() > 0 || Settings::getGeneral_sendNoObjDetectedEvents())
		{
			Q_EMIT objectsFound(info);
		}
		ui_->label_objectsDetected->setNum(info.objDetected_.size());
	}
	else
	{
		guiRefreshTime.start();

		if(findObject_->vocabulary()->size())
		{
			this->statusBar()->showMessage(tr("Cannot search, objects must be updated!"));
		}
		ui_->label_timeDetection->setNum(info.timeStamps_.value(DetectionInfo::kTimeKeypointDetection, 0));
		ui_->label_timeSkewAffine->setNum(info.timeStamps_.value(DetectionInfo::kTimeSkewAffine, 0));
		ui_->label_timeExtraction->setNum(info.timeStamps_.value(DetectionInfo::kTimeDescriptorExtraction, 0));
		ui_->label_timeSubPix->setNum(info.timeStamps_.value(DetectionInfo::kTimeSubPixelRefining, 0));
		ui_->imageView_source->updateImage(cvtCvMat2QImage(sceneImage_));
		ui_->imageView_source->updateData(info.sceneKeypoints_, info.sceneWords_);
	}


	//Update object pictures
	for(QMap<int, ObjWidget*>::iterator iter=objWidgets_.begin(); iter!=objWidgets_.end(); ++iter)
	{
		iter.value()->update();
	}

	ui_->label_nfeatures->setNum((int)info.sceneKeypoints_.size());
	ui_->imageView_source->update();

	ui_->label_detectorDescriptorType->setText(QString("%1/%2").arg(Settings::currentDetectorType()).arg(Settings::currentDescriptorType()));

	//update slider
	if(ui_->horizontalSlider_frames->isEnabled())
	{
		ui_->horizontalSlider_frames->blockSignals(true);
		ui_->horizontalSlider_frames->setValue(camera_->getCurrentFrameIndex()-1);
		ui_->label_frame->setNum(camera_->getCurrentFrameIndex()-1);
		ui_->horizontalSlider_frames->blockSignals(false);
	}

	ui_->label_timeTotal->setNum(info.timeStamps_.value(DetectionInfo::kTimeTotal, 0));
	int refreshRate = qRound(1000.0f/float(updateRate_.restart()));
	if(refreshRate > 0 && refreshRate < lowestRefreshRate_)
	{
		lowestRefreshRate_ = refreshRate;
	}
	// Refresh the label only after each 1000 ms
	if(refreshStartTime_.elapsed() > 1000)
	{
		if(Settings::getCamera_4imageRate()>0.0)
		{
			ui_->label_timeRefreshRate->setText(QString("(%1 Hz - %2 Hz)").arg(QString::number(Settings::getCamera_4imageRate())).arg(QString::number(lowestRefreshRate_)));
		}
		else
		{
			ui_->label_timeRefreshRate->setText(QString("(%2 Hz)").arg(QString::number(lowestRefreshRate_)));
		}
		lowestRefreshRate_ = 99;
		refreshStartTime_.start();
	}

	ui_->label_timeRefreshGUI->setNum(guiRefreshTime.elapsed());
}

void MainWindow::notifyParametersChanged(const QStringList & paramChanged)
{
	//Selective update (to not update all objects for a simple camera's parameter modification)
	bool detectorDescriptorParamsChanged = false;
	bool nearestNeighborParamsChanged = false;
	bool parameterChanged = false;
	for(QStringList::const_iterator iter = paramChanged.begin(); iter!=paramChanged.end(); ++iter)
	{
		if(lastObjectsUpdateParameters_.value(*iter) != Settings::getParameter(*iter))
		{
			lastObjectsUpdateParameters_[*iter] = Settings::getParameter(*iter);
			parameterChanged = true;
			UINFO("Parameter changed: %s -> \"%s\"", iter->toStdString().c_str(), Settings::getParameter(*iter).toString().toStdString().c_str());

			if(iter->contains("Feature2D"))
			{
				detectorDescriptorParamsChanged = true;
			}
			else if( (iter->contains("NearestNeighbor") && Settings::getGeneral_invertedSearch()) ||
					  iter->compare(Settings::kGeneral_invertedSearch()) == 0 ||
					  (iter->compare(Settings::kGeneral_vocabularyIncremental()) == 0 && Settings::getGeneral_invertedSearch()) ||
					  (iter->compare(Settings::kGeneral_vocabularyFixed()) == 0 && Settings::getGeneral_invertedSearch()) ||
					  (iter->compare(Settings::kGeneral_threads()) == 0 && !Settings::getGeneral_invertedSearch()) )
			{
				nearestNeighborParamsChanged = true;
			}

			if(iter->compare(Settings::kGeneral_port()) == 0 &&
			   Settings::getGeneral_port() != ui_->label_port->text().toInt() &&
			   Settings::getGeneral_port() != 0)
			{
				setupTCPServer();
			}
		}
	}

	if(detectorDescriptorParamsChanged)
	{
		//Re-init detector and extractor
		findObject_->updateDetectorExtractor();
	}

	if(Settings::getGeneral_autoUpdateObjects())
	{
		if(detectorDescriptorParamsChanged)
		{
			this->updateObjects();
		}
		else if(nearestNeighborParamsChanged)
		{
			this->updateVocabulary();
		}
	}
	else if(objWidgets_.size() && (detectorDescriptorParamsChanged || nearestNeighborParamsChanged))
	{
		this->statusBar()->showMessage(tr("A parameter has changed... \"Update objects\" may be required."));
	}

	if(parameterChanged &&
		!camera_->isRunning() &&
		!sceneImage_.empty() &&
		!(Settings::getGeneral_autoUpdateObjects() && detectorDescriptorParamsChanged)) // already done in updateObjects() above
	{
		this->update(sceneImage_);
		ui_->label_timeRefreshRate->setVisible(false);
	}
	ui_->actionCamera_from_video_file->setChecked(!Settings::getCamera_5mediaPath().isEmpty() && !UDirectory::exists(Settings::getCamera_5mediaPath().toStdString()) && !Settings::getCamera_6useTcpCamera());
	ui_->actionCamera_from_directory_of_images->setChecked(!Settings::getCamera_5mediaPath().isEmpty() && UDirectory::exists(Settings::getCamera_5mediaPath().toStdString()) && !Settings::getCamera_6useTcpCamera());
	ui_->actionCamera_from_TCP_IP->setChecked(Settings::getCamera_6useTcpCamera());

	if(Settings::getGeneral_debug())
	{
		ULogger::setPrintWhere(true);
		ULogger::setLevel(ULogger::kDebug);
	}
	else
	{
		ULogger::setPrintWhere(false);
		ULogger::setLevel(ULogger::kInfo);
	}
}

} // namespace find_object
