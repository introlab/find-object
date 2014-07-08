/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "MainWindow.h"
#include "AddObjectDialog.h"
#include "ui_mainWindow.h"
#include "QtOpenCV.h"
#include "KeypointItem.h"
#include "RectItem.h"
#include "ObjWidget.h"
#include "Camera.h"
#include "Settings.h"
#include "ParametersToolBox.h"
#include "AboutDialog.h"
#include "TcpServer.h"
#include "rtabmap/PdfPlot.h"
#include "Vocabulary.h"

#include <iostream>
#include <stdio.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/gpu/gpu.hpp"

#include <QtCore/QTextStream>
#include <QtCore/QFile>
#include <QtCore/QBuffer>
#include <QtCore/QThread>

#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsRectItem>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QProgressDialog>
#include <QtGui/QCloseEvent>
#include <QtGui/QCheckBox>
#include <QtGui/QScrollBar>
#include <QtGui/QInputDialog>

#include "utilite/UDirectory.h"

// Camera ownership transferred
MainWindow::MainWindow(Camera * camera, const QString & settings, QWidget * parent) :
	QMainWindow(parent),
	camera_(camera),
	settings_(settings),
	likelihoodCurve_(0),
	inliersCurve_(0),
	vocabulary_(new Vocabulary()),
	lowestRefreshRate_(99),
	objectsModified_(false),
	tcpServer_(0),
	detector_(0),
	extractor_(0)
{
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

	if(settings_.isEmpty())
	{
		settings_ = Settings::iniDefaultPath();
	}

	QByteArray geometry;
	QByteArray state;
	Settings::loadSettings(settings_, &geometry, &state);
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
		ui_->toolBox->getParameterWidget(Settings::kCamera_4imageRate())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kCamera_5mediaPath())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kCamera_6useTcpCamera())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kCamera_7IP())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kCamera_8port())->setEnabled(false);
		ui_->actionCamera_from_video_file->setVisible(false);
		ui_->actionCamera_from_TCP_IP->setVisible(false);
		ui_->actionCamera_from_directory_of_images->setVisible(false);
		ui_->actionLoad_scene_from_file->setVisible(false);
	}

	if(cv::gpu::getCudaEnabledDeviceCount() == 0)
	{
		Settings::setFeature2D_SURF_gpu(false);
		Settings::setFeature2D_Fast_gpu(false);
		Settings::setFeature2D_ORB_gpu(false);
		ui_->toolBox->updateParameter(Settings::kFeature2D_SURF_gpu());
		ui_->toolBox->updateParameter(Settings::kFeature2D_Fast_gpu());
		ui_->toolBox->updateParameter(Settings::kFeature2D_ORB_gpu());
		ui_->toolBox->getParameterWidget(Settings::kFeature2D_SURF_gpu())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kFeature2D_SURF_keypointsRatio())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kFeature2D_Fast_gpu())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kFeature2D_Fast_keypointsRatio())->setEnabled(false);
		ui_->toolBox->getParameterWidget(Settings::kFeature2D_ORB_gpu())->setEnabled(false);
	}

	detector_ = Settings::createKeypointDetector();
	extractor_ = Settings::createDescriptorExtractor();
	Q_ASSERT(detector_ != 0 && extractor_ != 0);

	connect((QDoubleSpinBox*)ui_->toolBox->getParameterWidget(Settings::kCamera_4imageRate()),
			SIGNAL(editingFinished()),
			camera_,
			SLOT(updateImageRate()));
	ui_->menuView->addAction(ui_->dockWidget_statistics->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_parameters->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_objects->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_plot->toggleViewAction());
	connect(ui_->toolBox, SIGNAL(parametersChanged(const QStringList &)), this, SLOT(notifyParametersChanged(const QStringList &)));

	ui_->imageView_source->setGraphicsViewMode(false);
	ui_->imageView_source->setTextLabel(tr("Press \"space\" to start the camera..."));
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

	if(Settings::getGeneral_autoStartCamera())
	{
		// Set 1 msec to see state on the status bar.
		QTimer::singleShot(1, this, SLOT(startProcessing()));
	}
}

MainWindow::~MainWindow()
{
	disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
	camera_->stop();
	delete detector_;
	delete extractor_;
	delete vocabulary_;
	objectsDescriptors_.clear();
	qDeleteAll(objects_.begin(), objects_.end());
	objects_.clear();
	delete ui_;
}

void MainWindow::closeEvent(QCloseEvent * event)
{
	bool quit = true;
	this->stopProcessing();
	if(objectsModified_ && this->isVisible() && objects_.size())
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
		Settings::saveSettings(settings_, this->saveGeometry(), this->saveState());
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
	connect(this, SIGNAL(objectsFound(QMultiMap<int,QPair<QRect,QTransform> >)), tcpServer_, SLOT(publishObjects(QMultiMap<int,QPair<QRect,QTransform> >)));
	ui_->label_ipAddress->setText(tcpServer_->getHostAddress().toString());
	ui_->label_port->setNum(tcpServer_->getPort());
	printf("IP: %s\nport: %d\n",
			tcpServer_->getHostAddress().toString().toStdString().c_str(), tcpServer_->getPort());
}

void MainWindow::setSourceImageText(const QString & text)
{
	ui_->imageView_source->setTextLabel(text);
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
		Settings::loadSettings(path, &geometry, &state);
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
	printf("Path \"%s\" not valid (should be *.ini)\n", path.toStdString().c_str());
	return false;
}

bool MainWindow::saveSettings(const QString & path)
{
	if(!path.isEmpty() && QFileInfo(path).suffix().compare("ini") == 0)
	{
		Settings::saveSettings(path, this->saveGeometry(), this->saveState());
		return true;
	}
	printf("Path \"%s\" not valid (should be *.ini)\n", path.toStdString().c_str());
	return false;
}

int MainWindow::loadObjects(const QString & dirPath)
{
	int loadedObjects = 0;
	QString formats = Settings::getGeneral_imageFormats().remove('*').remove('.');
	UDirectory dir(dirPath.toStdString(), formats.toStdString());
	if(dir.isValid())
	{
		const std::list<std::string> & names = dir.getFileNames(); // sorted in natural order
		for(std::list<std::string>::const_iterator iter=names.begin(); iter!=names.end(); ++iter)
		{
			this->addObjectFromFile((dirPath.toStdString()+dir.separator()+*iter).c_str());
		}
		if(names.size())
		{
			this->updateObjects();
		}
		loadedObjects = (int)names.size();
	}
	return loadedObjects;
}

void MainWindow::saveObjects(const QString & dirPath)
{
	QDir dir(dirPath);
	if(dir.exists())
	{
		for(int i=0; i<objects_.size(); ++i)
		{
			objects_.at(i)->pixmap().save(QString("%1/%2.png").arg(dirPath).arg(objects_.at(i)->id()));
		}
		objectsModified_ = false;
	}
}

void MainWindow::loadObjects()
{
	QString dirPath = QFileDialog::getExistingDirectory(this, tr("Loading objects... Select a directory."), Settings::workingDirectory());
	if(!dirPath.isEmpty())
	{
		loadObjects(dirPath);
	}
}
bool MainWindow::saveObjects()
{
	QString dirPath = QFileDialog::getExistingDirectory(this, tr("Saving objects... Select a directory."), Settings::workingDirectory());
	if(!dirPath.isEmpty())
	{
		saveObjects(dirPath);
		return true;
	}
	return false;
}

void MainWindow::removeObject(ObjWidget * object)
{
	if(object)
	{
		objects_.removeOne(object);
		object->deleteLater();
		this->updateData();
		if(!camera_->isRunning() && !ui_->imageView_source->cvImage().empty())
		{
			this->update(ui_->imageView_source->cvImage());
		}
	}
}

void MainWindow::removeAllObjects()
{
	for(int i=0; i<objects_.size(); ++i)
	{
		delete objects_.at(i);
	}
	objects_.clear();
	this->updateData();
	if(!camera_->isRunning() && !ui_->imageView_source->cvImage().empty())
	{
		this->update(ui_->imageView_source->cvImage());
	}
}

void MainWindow::updateObjectsSize()
{
	for(int i=0; i<objects_.size(); ++i)
	{
		updateObjectSize(objects_[i]);
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
	for(int i=0; i<objects_.size(); ++i)
	{
		objects_[i]->setMirrorView(mirrorView);
	}
}

void MainWindow::showHideControls()
{
	ui_->widget_controls->setVisible(Settings::getGeneral_controlsShown());
}

void MainWindow::addObjectFromScene()
{
	disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
	AddObjectDialog * dialog;
	bool resumeCamera = camera_->isRunning();
	if(camera_->isRunning() || ui_->imageView_source->cvImage().empty())
	{
		dialog = new AddObjectDialog(camera_, cv::Mat(), ui_->imageView_source->isMirrorView(), this);
	}
	else
	{
		dialog = new AddObjectDialog(0, ui_->imageView_source->cvImage(), ui_->imageView_source->isMirrorView(), this);
	}
	if(dialog->exec() == QDialog::Accepted)
	{
		ObjWidget * obj = dialog->retrieveObject();
		if(obj)
		{
			objects_.push_back(obj);
			showObject(objects_.last());
			updateData();
			objectsModified_ = true;
		}
	}
	if(resumeCamera || ui_->imageView_source->cvImage().empty())
	{
		this->startProcessing();
	}
	else
	{
		connect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
		this->update(ui_->imageView_source->cvImage());
	}
	delete dialog;
}

void MainWindow::addObjectsFromFiles()
{
	QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Add objects..."), Settings::workingDirectory(), tr("Image Files (%1)").arg(Settings::getGeneral_imageFormats()));
	if(fileNames.size())
	{
		for(int i=0; i<fileNames.size(); ++i)
		{
			this->addObjectFromFile(fileNames.at(i));
		}
		objectsModified_ = true;
		updateObjects();
	}
}

void MainWindow::addObjectFromFile(const QString & filePath)
{
	printf("Load file %s\n", filePath.toStdString().c_str());
	if(!filePath.isNull())
	{
		cv::Mat img = cv::imread(filePath.toStdString().c_str(), cv::IMREAD_GRAYSCALE);
		if(!img.empty())
		{
			int id = 0;
			QFileInfo file(filePath);
			QStringList list = file.fileName().split('.');
			if(list.size())
			{
				bool ok = false;
				id = list.front().toInt(&ok);
				if(ok)
				{
					for(int i=0; i<objects_.size(); ++i)
					{
						if(objects_.at(i)->id() == id)
						{
							if(this->isVisible())
							{
								QMessageBox::warning(this, tr("Warning"), tr("Object %1 already added, a new ID will be generated.").arg(id));
							}
							else
							{
								printf("WARNING: Object %d already added, a new ID will be generated.", id);
							}
							id = 0;
							break;
						}
					}
				}
				else
				{
					id = 0;
				}
			}
			objects_.append(new ObjWidget(id, std::vector<cv::KeyPoint>(), cv::Mat(), img, "", ""));
			this->showObject(objects_.last());
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
		QString ip = QInputDialog::getText(this, tr("Server IP..."), "IP: ", QLineEdit::Normal, Settings::getCamera_7IP());
		if(!ip.isEmpty())
		{
			int port = QInputDialog::getInteger(this, tr("Server port..."), "Port: ", Settings::getCamera_8port());

			if(port > 0)
			{
				Settings::setCamera_6useTcpCamera(true);
				ui_->toolBox->updateParameter(Settings::kCamera_6useTcpCamera());
				Settings::setCamera_7IP(ip);
				ui_->toolBox->updateParameter(Settings::kCamera_7IP());
				Settings::setCamera_8port(port);
				ui_->toolBox->updateParameter(Settings::kCamera_8port());
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
		int id = Settings::getGeneral_nextObjID();
		if(obj->id() == 0)
		{
			obj->setId(id++);
			Settings::setGeneral_nextObjID(id);
		}
		else if(obj->id() > id)
		{
			id = obj->id()+1;
			Settings::setGeneral_nextObjID(id);
		}
		ui_->toolBox->updateParameter(Settings::kGeneral_nextObjID());

		QLabel * title = new QLabel(QString("%1 (%2)").arg(obj->id()).arg(QString::number(obj->keypoints().size())), this);
		QLabel * detectorDescriptorType = new QLabel(QString("%1/%2").arg(obj->detectorType()).arg(obj->descriptorType()), this);
		QLabel * detectedLabel = new QLabel(this);
		title->setObjectName(QString("%1title").arg(obj->id()));
		detectorDescriptorType->setObjectName(QString("%1type").arg(obj->id()));
		detectedLabel->setObjectName(QString("%1detection").arg(obj->id()));
		QHBoxLayout * hLayout = new QHBoxLayout();
		hLayout->addWidget(title);
		hLayout->addStretch(1);
		hLayout->addWidget(detectorDescriptorType);
		hLayout->addStretch(1);
		hLayout->addWidget(detectedLabel);
		vLayout->addLayout(hLayout);
		vLayout->addWidget(obj);
		objects_.last()->setDeletable(true);
		connect(obj, SIGNAL(removalTriggered(ObjWidget*)), this, SLOT(removeObject(ObjWidget*)));
		connect(obj, SIGNAL(destroyed(QObject *)), title, SLOT(deleteLater()));
		connect(obj, SIGNAL(destroyed(QObject *)), detectedLabel, SLOT(deleteLater()));
		connect(obj, SIGNAL(destroyed(QObject *)), detectorDescriptorType, SLOT(deleteLater()));
		connect(obj, SIGNAL(destroyed(QObject *)), vLayout, SLOT(deleteLater()));
		ui_->verticalLayout_objects->insertLayout(ui_->verticalLayout_objects->count()-1, vLayout);

		QByteArray ba;
		QBuffer buffer(&ba);
		buffer.open(QIODevice::WriteOnly);
		obj->pixmap().scaledToWidth(128).save(&buffer, "JPEG"); // writes image into JPEG format
		imagesMap_.insert(obj->id(), ba);

		updateObjectSize(obj);
	}
}

std::vector<cv::KeyPoint> limitKeypoints(const std::vector<cv::KeyPoint> & keypoints, int maxKeypoints)
{
	std::vector<cv::KeyPoint> kptsKept;
	if(maxKeypoints > 0 && (int)keypoints.size() > maxKeypoints)
	{
		// Sort words by response
		std::multimap<float, int> reponseMap; // <response,id>
		for(unsigned int i = 0; i <keypoints.size(); ++i)
		{
			//Keep track of the data, to be easier to manage the data in the next step
			reponseMap.insert(std::pair<float, int>(fabs(keypoints[i].response), i));
		}

		// Remove them
		std::multimap<float, int>::reverse_iterator iter = reponseMap.rbegin();
		kptsKept.resize(maxKeypoints);
		for(unsigned int k=0; k < kptsKept.size() && iter!=reponseMap.rend(); ++k, ++iter)
		{
			kptsKept[k] = keypoints[iter->second];
		}
	}
	else
	{
		kptsKept = keypoints;
	}
	return kptsKept;
}

class ExtractFeaturesThread : public QThread
{
public:
	ExtractFeaturesThread(int objectId, int objectIndex, const cv::Mat & image) :
		objectId_(objectId),
		objectIndex_(objectIndex),
		image_(image)
	{

	}
	int objectId() const {return objectId_;}
	int objectIndex() const {return objectIndex_;}
	const cv::Mat & image() const {return image_;}
	const std::vector<cv::KeyPoint> & keypoints() const {return keypoints_;}
	const cv::Mat & descriptors() const {return descriptors_;}
protected:
	virtual void run()
	{
		QTime time;
		time.start();
		printf("Extracting descriptors from object %d...\n", objectId_);
		KeypointDetector * detector = Settings::createKeypointDetector();
		keypoints_.clear();
		descriptors_ = cv::Mat();
		detector->detect(image_, keypoints_);
		delete detector;

		if(keypoints_.size())
		{
			int maxFeatures = Settings::getFeature2D_3MaxFeatures();
			if(maxFeatures > 0 && (int)keypoints_.size() > maxFeatures)
			{
				int previousCount = (int)keypoints_.size();
				keypoints_ = limitKeypoints(keypoints_, maxFeatures);
				printf("obj=%d, %d keypoints removed, (kept %d), min/max response=%f/%f\n", objectId_, previousCount-(int)keypoints_.size(), (int)keypoints_.size(), keypoints_.size()?keypoints_.back().response:0.0f, keypoints_.size()?keypoints_.front().response:0.0f);
			}

			DescriptorExtractor * extractor = Settings::createDescriptorExtractor();
			extractor->compute(image_, keypoints_, descriptors_);
			delete extractor;
			if((int)keypoints_.size() != descriptors_.rows)
			{
				printf("ERROR : obj=%d kpt=%d != descriptors=%d\n", objectId_, (int)keypoints_.size(), descriptors_.rows);
			}
		}
		else
		{
			printf("WARNING: no features detected in object %d !?!\n", objectId_);
		}
		printf("%d descriptors extracted from object %d (in %d ms)\n", descriptors_.rows, objectId_, time.elapsed());
	}
private:
	int objectId_;
	int objectIndex_;
	cv::Mat image_;
	std::vector<cv::KeyPoint> keypoints_;
	cv::Mat descriptors_;
};

void MainWindow::updateObjects()
{
	if(objects_.size())
	{
		this->statusBar()->showMessage(tr("Updating %1 objects...").arg(objects_.size()));
		QApplication::processEvents();

		int threadCounts = Settings::getGeneral_threads();
		if(threadCounts == 0)
		{
			threadCounts = objects_.size();
		}

		QTime time;
		time.start();
		printf("Features extraction from %d objects...\n", objects_.size());
		for(int i=0; i<objects_.size(); i+=threadCounts)
		{
			QVector<ExtractFeaturesThread*> threads;
			for(int k=i; k<i+threadCounts && k<objects_.size(); ++k)
			{
				threads.push_back(new ExtractFeaturesThread(objects_.at(k)->id(), k, objects_.at(k)->cvImage()));
				threads.back()->start();
			}

			for(int j=0; j<threads.size(); ++j)
			{
				threads[j]->wait();

				int index = threads[j]->objectIndex();

				objects_.at(index)->setData(threads[j]->keypoints(), threads[j]->descriptors(), threads[j]->image(), Settings::currentDetectorType(), Settings::currentDescriptorType());

				//update object labels
				QLabel * title = qFindChild<QLabel*>(this, QString("%1title").arg(objects_.at(index)->id()));
				title->setText(QString("%1 (%2)").arg(objects_.at(index)->id()).arg(QString::number(objects_.at(index)->keypoints().size())));
				QLabel * detectorDescriptorType = qFindChild<QLabel*>(this, QString("%1type").arg(objects_.at(index)->id()));
				detectorDescriptorType->setText(QString("%1/%2").arg(objects_.at(index)->detectorType()).arg(objects_.at(index)->descriptorType()));
			}
		}
		printf("Features extraction from %d objects... done! (%d ms)\n", objects_.size(), time.elapsed());

		updateData();
	}
	if(!camera_->isRunning() && !ui_->imageView_source->cvImage().empty())
	{
		this->update(ui_->imageView_source->cvImage());
	}
	this->statusBar()->clearMessage();
}

void MainWindow::updateData()
{
	if(objects_.size())
	{
		ui_->actionSave_objects->setEnabled(true);
	}
	else
	{
		ui_->actionSave_objects->setEnabled(false);
	}

	objectsDescriptors_.clear();
	dataRange_.clear();
	vocabulary_->clear();
	int count = 0;
	int dim = -1;
	int type = -1;
	// Get the total size and verify descriptors
	for(int i=0; i<objects_.size(); ++i)
	{
		if(!objects_.at(i)->descriptors().empty())
		{
			if(dim >= 0 && objects_.at(i)->descriptors().cols != dim)
			{
				if(this->isVisible())
				{
					QMessageBox::critical(this, tr("Error"), tr("Descriptors of the objects are not all the same size!\nObjects opened must have all the same size (and from the same descriptor extractor)."));
				}
				else
				{
					printf("ERROR: Descriptors of the objects are not all the same size! Objects opened must have all the same size (and from the same descriptor extractor).\n");
				}
				return;
			}
			dim = objects_.at(i)->descriptors().cols;
			if(type >= 0 && objects_.at(i)->descriptors().type() != type)
			{
				if(this->isVisible())
				{
					QMessageBox::critical(this, tr("Error"), tr("Descriptors of the objects are not all the same type!\nObjects opened must have been processed by the same descriptor extractor."));
				}
				else
				{
					printf("ERROR: Descriptors of the objects are not all the same type! Objects opened must have been processed by the same descriptor extractor.\n");
				}
				return;
			}
			type = objects_.at(i)->descriptors().type();
			count += objects_.at(i)->descriptors().rows;
		}
	}

	// Copy data
	if(count)
	{
		this->statusBar()->showMessage(tr("Updating objects data (%1 descriptors)...").arg(count));
		QApplication::processEvents();
		printf("Updating global descriptors matrix: Objects=%d, total descriptors=%d, dim=%d, type=%d\n", (int)objects_.size(), count, dim, type);
		if(Settings::getGeneral_invertedSearch() || Settings::getGeneral_threads() == 1)
		{
			// If only one thread, put all descriptors in the same cv::Mat
			objectsDescriptors_.push_back(cv::Mat(count, dim, type));
			int row = 0;
			for(int i=0; i<objects_.size(); ++i)
			{
				if(objects_.at(i)->descriptors().rows)
				{
					cv::Mat dest(objectsDescriptors_[0], cv::Range(row, row+objects_.at(i)->descriptors().rows));
					objects_.at(i)->descriptors().copyTo(dest);
					row += objects_.at(i)->descriptors().rows;
					// dataRange contains the upper_bound for each
					// object (the last descriptors position in the
					// global object descriptors matrix)
					if(objects_.at(i)->descriptors().rows)
					{
						dataRange_.insert(row-1, i);
					}
				}
			}

			if(Settings::getGeneral_invertedSearch())
			{
				QTime time;
				time.start();
				bool incremental = Settings::getGeneral_vocabularyIncremental();
				if(incremental)
				{
					printf("Creating incremental vocabulary...\n");
				}
				else
				{
					printf("Creating vocabulary...\n");
				}
				QTime localTime;
				localTime.start();
				int updateVocabularyMinWords = Settings::getGeneral_vocabularyUpdateMinWords();
				int addedWords = 0;
				for(int i=0; i<objects_.size(); ++i)
				{
					QMultiMap<int, int> words = vocabulary_->addWords(objects_[i]->descriptors(), i, incremental);
					objects_[i]->setWords(words);
					addedWords += words.uniqueKeys().size();
					bool updated = false;
					if(incremental && addedWords && addedWords >= updateVocabularyMinWords)
					{
						vocabulary_->update();
						addedWords = 0;
						updated = true;
					}
					printf("Object %d, %d words from %d descriptors (%d words, %d ms) %s\n",
							objects_[i]->id(),
							words.uniqueKeys().size(),
							objects_[i]->descriptors().rows,
							vocabulary_->size(),
							localTime.restart(),
							updated?"updated":"");
				}
				if(addedWords)
				{
					vocabulary_->update();
				}
				ui_->label_timeIndexing->setNum(time.elapsed());
				ui_->label_vocabularySize->setNum(vocabulary_->size());
				if(incremental)
				{
					printf("Creating incremental vocabulary... done! size=%d (%d ms)\n", vocabulary_->size(), time.elapsed());
				}
				else
				{
					printf("Creating vocabulary... done! size=%d (%d ms)\n", vocabulary_->size(), time.elapsed());
				}
			}
		}
		else
		{
			for(int i=0; i<objects_.size(); ++i)
			{
				objectsDescriptors_.push_back(objects_.at(i)->descriptors());
			}
		}
		this->statusBar()->clearMessage();
	}
	lastObjectsUpdateParameters_ = Settings::getParameters();
}

void MainWindow::startProcessing()
{
	printf("Starting camera...\n");
	bool updateStatusMessage = this->statusBar()->currentMessage().isEmpty();
	if(updateStatusMessage)
	{
		this->statusBar()->showMessage(tr("Starting camera..."));
	}
	if(camera_->start())
	{
		connect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
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
		if(this->isVisible())
		{
			if(Settings::getCamera_6useTcpCamera())
			{
				QMessageBox::critical(this, tr("Camera error"), tr("Camera initialization failed! (with server %1:%2)").arg(Settings::getCamera_7IP()).arg(Settings::getCamera_8port()));
			}
			else
			{
				QMessageBox::critical(this, tr("Camera error"), tr("Camera initialization failed! (with device %1)").arg(Settings::getCamera_1deviceId()));
			}
		}
		else
		{
			printf("[ERROR] Camera initialization failed! (with device %d)\n", Settings::getCamera_1deviceId());
		}
	}
}

void MainWindow::stopProcessing()
{
	if(camera_)
	{
		disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
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

class SearchThread: public QThread
{
public:
	SearchThread(Vocabulary * vocabulary, int objectIndex, const cv::Mat * descriptors, const ObjWidget * sceneObject) :
		vocabulary_(vocabulary),
		objectIndex_(objectIndex),
		descriptors_(descriptors),
		sceneObject_(sceneObject),
		minMatchedDistance_(-1.0f),
		maxMatchedDistance_(-1.0f)
	{
		Q_ASSERT(index && descriptors);
	}
	virtual ~SearchThread() {}

	int getObjectIndex() const {return objectIndex_;}
	float getMinMatchedDistance() const {return minMatchedDistance_;}
	float getMaxMatchedDistance() const {return maxMatchedDistance_;}
	const QMultiMap<int, int> & getMatches() const {return matches_;}

protected:
	virtual void run()
	{
		//QTime time;
		//time.start();

		cv::Mat results;
		cv::Mat dists;

		//match objects to scene
		int k = Settings::getNearestNeighbor_3nndrRatioUsed()?2:1;
		results = cv::Mat(descriptors_->rows, k, CV_32SC1); // results index
		dists = cv::Mat(descriptors_->rows, k, CV_32FC1); // Distance results are CV_32FC1
		vocabulary_->search(*descriptors_, results, dists, k);

		// PROCESS RESULTS
		// Get all matches for each object
		for(int i=0; i<dists.rows; ++i)
		{
			// Check if this descriptor matches with those of the objects
			bool matched = false;

			if(Settings::getNearestNeighbor_3nndrRatioUsed() &&
			   dists.at<float>(i,0) <= Settings::getNearestNeighbor_4nndrRatio() * dists.at<float>(i,1))
			{
				matched = true;
			}
			if((matched || !Settings::getNearestNeighbor_3nndrRatioUsed()) &&
			   Settings::getNearestNeighbor_5minDistanceUsed())
			{
				if(dists.at<float>(i,0) <= Settings::getNearestNeighbor_6minDistance())
				{
					matched = true;
				}
				else
				{
					matched = false;
				}
			}
			if(!matched && !Settings::getNearestNeighbor_3nndrRatioUsed() && !Settings::getNearestNeighbor_5minDistanceUsed())
			{
				matched = true; // no criterion, match to the nearest descriptor
			}
			if(minMatchedDistance_ == -1 || minMatchedDistance_ > dists.at<float>(i,0))
			{
				minMatchedDistance_ = dists.at<float>(i,0);
			}
			if(maxMatchedDistance_ == -1 || maxMatchedDistance_ < dists.at<float>(i,0))
			{
				maxMatchedDistance_ = dists.at<float>(i,0);
			}

			int wordId = results.at<int>(i,0);
			if(matched && sceneObject_->words().count(wordId) == 1)
			{
				matches_.insert(i, sceneObject_->words().value(wordId));
				matches_.insert(i, results.at<int>(i,0));
			}
		}

		//printf("Search Object %d time=%d ms\n", objectIndex_, time.elapsed());
	}
private:
	Vocabulary * vocabulary_; // would be const but flann search() method is not const!?
	int objectIndex_;
	const cv::Mat * descriptors_;
	const ObjWidget * sceneObject_;

	float minMatchedDistance_;
	float maxMatchedDistance_;
	QMultiMap<int, int> matches_;
};

class HomographyThread: public QThread
{
public:
	HomographyThread(
			const QMultiMap<int, int> * matches, // <object, scene>
			int objectIndex,
			const std::vector<cv::KeyPoint> * kptsA,
			const std::vector<cv::KeyPoint> * kptsB) :
				matches_(matches),
				objectIndex_(objectIndex),
				kptsA_(kptsA),
				kptsB_(kptsB),
				inliers_(0),
				outliers_(0)
	{
		Q_ASSERT(matches && kptsA && kptsB);
	}
	virtual ~HomographyThread() {}

	int getObjectIndex() const {return objectIndex_;}
	const std::vector<int> & getIndexesA() const {return indexesA_;}
	const std::vector<int> & getIndexesB() const {return indexesB_;}
	const std::vector<uchar> & getOutlierMask() const {return outlierMask_;}
	int getInliers() const {return inliers_;}
	int getOutliers() const {return outliers_;}
	const cv::Mat & getHomography() const {return h_;}

protected:
	virtual void run()
	{
		//QTime time;
		//time.start();

		std::vector<cv::Point2f> mpts_1(matches_->size());
		std::vector<cv::Point2f> mpts_2(matches_->size());
		indexesA_.resize(matches_->size());
		indexesB_.resize(matches_->size());

		int j=0;
		for(QMultiMap<int, int>::const_iterator iter = matches_->begin(); iter!=matches_->end(); ++iter)
		{
			mpts_1[j] = kptsA_->at(iter.key()).pt;
			indexesA_[j] = iter.key();
			mpts_2[j] = kptsB_->at(iter.value()).pt;
			indexesB_[j] = iter.value();
			++j;
		}

		if((int)mpts_1.size() >= Settings::getHomography_minimumInliers())
		{
			h_ = findHomography(mpts_1,
					mpts_2,
					Settings::getHomographyMethod(),
					Settings::getHomography_ransacReprojThr(),
					outlierMask_);

			for(unsigned int k=0; k<mpts_1.size();++k)
			{
				if(outlierMask_.at(k))
				{
					++inliers_;
				}
				else
				{
					++outliers_;
				}
			}

			// ignore homography when all features are inliers
			if(inliers_ == (int)outlierMask_.size() && !h_.empty())
			{
				if(Settings::getHomography_ignoreWhenAllInliers() || cv::countNonZero(h_) < 1)
				{
					h_ = cv::Mat();
				}
			}
		}

		//printf("Homography Object %d time=%d ms\n", objectIndex_, time.elapsed());
	}
private:
	const QMultiMap<int, int> * matches_;
	int objectIndex_;
	const std::vector<cv::KeyPoint> * kptsA_;
	const std::vector<cv::KeyPoint> * kptsB_;

	std::vector<int> indexesA_;
	std::vector<int> indexesB_;
	std::vector<uchar> outlierMask_;
	int inliers_;
	int outliers_;
	cv::Mat h_;
};

void MainWindow::update(const cv::Mat & image)
{
	//printf("====== UPDATE =======\n");

	QTime totalTime;
	totalTime.start();
	// reset objects color
	for(int i=0; i<objects_.size(); ++i)
	{
		objects_[i]->resetKptsColor();
	}

	if(!image.empty())
	{
		//Convert to grayscale
		cv::Mat grayscaleImg;
		if(image.channels() != 1 || image.depth() != CV_8U)
		{
			cv::cvtColor(image, grayscaleImg, cv::COLOR_BGR2GRAY);
		}
		else
		{
			grayscaleImg =  image;
		}

		QTime time;
		time.start();

		// EXTRACT KEYPOINTS
		std::vector<cv::KeyPoint> keypoints;
		detector_->detect(grayscaleImg, keypoints);
		ui_->label_timeDetection->setNum(time.restart());

		cv::Mat descriptors;
		if(keypoints.size())
		{
			int maxFeatures = Settings::getFeature2D_3MaxFeatures();
			if(maxFeatures > 0 && (int)keypoints.size() > maxFeatures)
			{
				//int previousCount = (int)keypoints.size();
				keypoints = limitKeypoints(keypoints, maxFeatures);
				//printf("%d keypoints removed, (kept %d), min/max response=%f/%f\n", previousCount-(int)keypoints.size(), (int)keypoints.size(), keypoints.size()?keypoints.back().response:0.0f, keypoints.size()?keypoints.front().response:0.0f);
			}

			// EXTRACT DESCRIPTORS
			extractor_->compute(grayscaleImg, keypoints, descriptors);
			if((int)keypoints.size() != descriptors.rows)
			{
				printf("ERROR : kpt=%d != descriptors=%d\n", (int)keypoints.size(), descriptors.rows);
			}
			ui_->label_timeExtraction->setNum(time.restart());
		}
		else
		{
			printf("WARNING: no features detected !?!\n");
			ui_->label_timeExtraction->setNum(0);
		}

		if(this->isVisible())
		{
			ui_->imageView_source->setData(keypoints, cv::Mat(), image, Settings::currentDetectorType(), Settings::currentDescriptorType());
		}

		bool consistentNNData = (vocabulary_->size()!=0 && vocabulary_->wordToObjects().begin().value()!=-1 && Settings::getGeneral_invertedSearch()) ||
								((vocabulary_->size()==0 || vocabulary_->wordToObjects().begin().value()==-1) && !Settings::getGeneral_invertedSearch());

		// COMPARE
		if(!objectsDescriptors_.empty() &&
		   keypoints.size() &&
		   consistentNNData &&
		   objectsDescriptors_[0].cols == descriptors.cols &&
		   objectsDescriptors_[0].type() == descriptors.type()) // binary descriptor issue, if the dataTree is not yet updated with modified settings
		{
			QVector<QMultiMap<int, int> > matches(objects_.size()); // Map< ObjectDescriptorIndex, SceneDescriptorIndex >
			QVector<int> matchesId(objects_.size(), -1);
			float minMatchedDistance = -1.0f;
			float maxMatchedDistance = -1.0f;

			if(!Settings::getGeneral_invertedSearch())
			{
				// CREATE INDEX for the scene
				//printf("Creating FLANN index (%s)\n", Settings::currentNearestNeighborType().toStdString().c_str());
				vocabulary_->clear();
				QMultiMap<int, int> words = vocabulary_->addWords(descriptors, -1, Settings::getGeneral_vocabularyIncremental());
				if(!Settings::getGeneral_vocabularyIncremental())
				{
					vocabulary_->update();
				}
				ui_->imageView_source->setWords(words);
				ui_->label_timeIndexing->setNum(time.restart());
				ui_->label_vocabularySize->setNum(vocabulary_->size());
			}

			if(Settings::getGeneral_invertedSearch() || Settings::getGeneral_threads() == 1)
			{
				cv::Mat results;
				cv::Mat dists;
				// DO NEAREST NEIGHBOR
				int k = Settings::getNearestNeighbor_3nndrRatioUsed()?2:1;
				if(!Settings::getGeneral_invertedSearch())
				{
					//match objects to scene
					results = cv::Mat(objectsDescriptors_[0].rows, k, CV_32SC1); // results index
					dists = cv::Mat(objectsDescriptors_[0].rows, k, CV_32FC1); // Distance results are CV_32FC1
					vocabulary_->search(objectsDescriptors_[0], results, dists, k);
				}
				else
				{
					//match scene to objects
					results = cv::Mat(descriptors.rows, k, CV_32SC1); // results index
					dists = cv::Mat(descriptors.rows, k, CV_32FC1); // Distance results are CV_32FC1
					vocabulary_->search(descriptors, results, dists, k);
				}

				// PROCESS RESULTS
				// Get all matches for each object
				for(int i=0; i<dists.rows; ++i)
				{
					// Check if this descriptor matches with those of the objects
					bool matched = false;

					if(Settings::getNearestNeighbor_3nndrRatioUsed() &&
					   dists.at<float>(i,0) <= Settings::getNearestNeighbor_4nndrRatio() * dists.at<float>(i,1))
					{
						matched = true;
					}
					if((matched || !Settings::getNearestNeighbor_3nndrRatioUsed()) &&
					   Settings::getNearestNeighbor_5minDistanceUsed())
					{
						if(dists.at<float>(i,0) <= Settings::getNearestNeighbor_6minDistance())
						{
							matched = true;
						}
						else
						{
							matched = false;
						}
					}
					if(!matched && !Settings::getNearestNeighbor_3nndrRatioUsed() && !Settings::getNearestNeighbor_5minDistanceUsed())
					{
						matched = true; // no criterion, match to the nearest descriptor
					}
					if(minMatchedDistance == -1 || minMatchedDistance > dists.at<float>(i,0))
					{
						minMatchedDistance = dists.at<float>(i,0);
					}
					if(maxMatchedDistance == -1 || maxMatchedDistance < dists.at<float>(i,0))
					{
						maxMatchedDistance = dists.at<float>(i,0);
					}

					if(matched)
					{
						if(Settings::getGeneral_invertedSearch())
						{
							int wordId = results.at<int>(i,0);
							QList<int> objIndexes = vocabulary_->wordToObjects().values(wordId);
							for(int j=0; j<objIndexes.size(); ++j)
							{
								// just add unique matches
								if(vocabulary_->wordToObjects().count(wordId, objIndexes[j]) == 1)
								{
									matches[objIndexes[j]].insert(objects_.at(objIndexes[j])->words().value(wordId), i);
								}
							}
						}
						else
						{
							QMap<int, int>::iterator iter = dataRange_.lowerBound(i);
							int objectIndex = iter.value();
							int fisrtObjectDescriptorIndex = (iter == dataRange_.begin())?0:(--iter).key()+1;
							int objectDescriptorIndex = i - fisrtObjectDescriptorIndex;

							int wordId = results.at<int>(i,0);
							if(ui_->imageView_source->words().count(wordId) == 1)
							{
								matches[objectIndex].insert(objectDescriptorIndex, ui_->imageView_source->words().value(wordId));
							}
						}
					}
				}
			}
			else
			{
				//multi-threaded, match objects to scene
				unsigned int threadCounts = Settings::getGeneral_threads();
				if(threadCounts == 0)
				{
					threadCounts = (int)objectsDescriptors_.size();
				}
				for(unsigned int j=0; j<objectsDescriptors_.size(); j+=threadCounts)
				{
					QVector<SearchThread*> threads;

					for(unsigned int k=j; k<j+threadCounts && k<objectsDescriptors_.size(); ++k)
					{
						threads.push_back(new SearchThread(vocabulary_, k, &objectsDescriptors_[k], ui_->imageView_source));
						threads.back()->start();
					}

					for(int k=0; k<threads.size(); ++k)
					{
						threads[k]->wait();
						matches[threads[k]->getObjectIndex()] = threads[k]->getMatches();

						if(minMatchedDistance == -1 || minMatchedDistance > threads[k]->getMinMatchedDistance())
						{
							minMatchedDistance = threads[k]->getMinMatchedDistance();
						}
						if(maxMatchedDistance == -1 || maxMatchedDistance < threads[k]->getMaxMatchedDistance())
						{
							maxMatchedDistance = threads[k]->getMaxMatchedDistance();
						}
						delete threads[k];
					}

				}
			}

			ui_->label_timeMatching->setNum(time.restart());

			// GUI: Homographies and color
			int maxScoreId = -1;
			int maxScore = 0;
			QMultiMap<int, QPair<QRect, QTransform> > objectsDetected;

			QMap<int, float> inliersScores;
			if(Settings::getHomography_homographyComputed())
			{
				// HOMOGRAHPY
				int threadCounts = Settings::getGeneral_threads();
				if(threadCounts == 0)
				{
					threadCounts = matches.size();
				}

				for(int i=0; i<matches.size(); i+=threadCounts)
				{
					QVector<HomographyThread*> threads;

					for(int k=i; k<i+threadCounts && k<matches.size(); ++k)
					{
						int objectId = matchesId[k] >=0 ? matchesId[k]:k; // the first matches ids correspond object index
						threads.push_back(new HomographyThread(&matches[k], objectId, &objects_.at(objectId)->keypoints(), &keypoints));
						threads.back()->start();
					}

					for(int j=0; j<threads.size(); ++j)
					{
						threads[j]->wait();

						int index = threads[j]->getObjectIndex();

						// COLORIZE (should be done in the GUI thread)
						int nColor = index % 10 + 7;
						QColor color((Qt::GlobalColor)(nColor==Qt::yellow?Qt::darkYellow:nColor));
						QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1detection").arg(objects_.at(index)->id()));
						if(!threads[j]->getHomography().empty())
						{
							inliersScores.insert(objects_.at(index)->id(), (float)threads[j]->getInliers());
							if(threads[j]->getInliers() >= Settings::getHomography_minimumInliers())
							{
								const cv::Mat & H = threads[j]->getHomography();
								QTransform hTransform(
									H.at<double>(0,0), H.at<double>(1,0), H.at<double>(2,0),
									H.at<double>(0,1), H.at<double>(1,1), H.at<double>(2,1),
									H.at<double>(0,2), H.at<double>(1,2), H.at<double>(2,2));

								int distance = Settings::getGeneral_multiDetectionRadius(); // in pixels
								if(Settings::getGeneral_multiDetection())
								{
									// Remove inliers and recompute homography
									QMultiMap<int, int> newMatches;
									for(unsigned int k=0; k<threads[j]->getOutlierMask().size();++k)
									{
										if(!threads[j]->getOutlierMask().at(k))
										{
											newMatches.insert(threads[j]->getIndexesA().at(k), threads[j]->getIndexesB().at(k));
										}
									}
									matches.push_back(newMatches);
									matchesId.push_back(index);

									// compute distance from previous added same objects...
									QMultiMap<int, QPair<QRect, QTransform> >::iterator objIter = objectsDetected.find(objects_.at(index)->id());
									for(;objIter!=objectsDetected.end() && objIter.key() == objects_.at(index)->id(); ++objIter)
									{
										qreal dx = objIter.value().second.m31() - hTransform.m31();
										qreal dy = objIter.value().second.m32() - hTransform.m32();
										int d = (int)sqrt(dx*dx + dy*dy);
										if(d < distance)
										{
											distance = d;
										}
									}
								}

								if(distance >= Settings::getGeneral_multiDetectionRadius())
								{
									if(this->isVisible())
									{
										for(unsigned int k=0; k<threads[j]->getOutlierMask().size();++k)
										{
											if(threads[j]->getOutlierMask().at(k))
											{
												objects_.at(index)->setKptColor(threads[j]->getIndexesA().at(k), color);
												ui_->imageView_source->setKptColor(threads[j]->getIndexesB().at(k), color);
											}
											else if(!objectsDetected.contains(objects_.at(index)->id()))
											{
												objects_.at(index)->setKptColor(threads[j]->getIndexesA().at(k), Qt::black);
											}
										}
									}

									QRect rect = objects_.at(index)->pixmap().rect();
									objectsDetected.insert(objects_.at(index)->id(), QPair<QRect, QTransform>(rect, hTransform));
									// Example getting the center of the object in the scene using the homography
									//QPoint pos(rect.width()/2, rect.height()/2);
									//hTransform.map(pos)

									// add rectangle
									if(this->isVisible())
									{
										if(objectsDetected.count(objects_.at(index)->id()) > 1)
										{
											// if a homography is already found, set the objects count
											label->setText(QString("%1 objects found").arg(objectsDetected.count(objects_.at(index)->id())));
										}
										else
										{
											label->setText(QString("%1 in %2 out").arg(threads[j]->getInliers()).arg(threads[j]->getOutliers()));
										}
										QPen rectPen(color);
										rectPen.setWidth(Settings::getHomography_rectBorderWidth());
										RectItem * rectItemScene = new RectItem(objects_.at(index)->id(), rect);
										connect(rectItemScene, SIGNAL(hovered(int)), this, SLOT(rectHovered(int)));
										rectItemScene->setPen(rectPen);
										rectItemScene->setTransform(hTransform);
										ui_->imageView_source->addRect(rectItemScene);

										QGraphicsRectItem * rectItemObj = new QGraphicsRectItem(rect);
										rectItemObj->setPen(rectPen);
										objects_.at(index)->addRect(rectItemObj);
									}
								}
							}
							else if(this->isVisible() && objectsDetected.count(objects_.at(index)->id()) == 0)
							{
								label->setText(QString("Too low inliers (%1 in %2 out)").arg(threads[j]->getInliers()).arg(threads[j]->getOutliers()));
							}
						}
						else if(this->isVisible() && objectsDetected.count(objects_.at(index)->id()) == 0)
						{
							inliersScores.insert(objects_.at(index)->id(), 0.0f);
							if(threads[j]->getInliers() >= Settings::getHomography_minimumInliers())
							{
								label->setText(QString("Ignored, all inliers (%1 in %2 out)").arg(matches[index].size()).arg(threads[j]->getOutliers()));
							}
							else
							{
								label->setText(QString("Too low matches (%1)").arg(matches[index].size()));
							}
						}
					}
				}
				ui_->label_timeHomographies->setNum(time.restart());
			}
			else
			{
				for(int i=0; i<matches.size(); ++i)
				{
					// colorize all matches if homography is not computed
					int nColor = i % 11 + 7;
					QColor color((Qt::GlobalColor)(nColor==Qt::yellow?Qt::gray:nColor));
					for(QMultiMap<int, int>::iterator iter = matches[i].begin(); iter!=matches[i].end(); ++iter)
					{
						objects_[i]->setKptColor(iter.key(), color);
						ui_->imageView_source->setKptColor(iter.value(), color);
					}
					QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1detection").arg(objects_.at(i)->id()));
					label->setText(QString("%1 matches").arg(matches[i].size()));
					inliersScores.insert(objects_.at(i)->id(), 0.0f);
				}
			}

			//scores
			QMap<int, float> scores;
			for(int i=0; i<matches.size(); ++i)
			{
				int objectIndex = matchesId.at(i) >= 0? matchesId.at(i): i;
				if(!scores.contains(objects_.at(objectIndex)->id()))
				{
					scores.insert(objects_.at(objectIndex)->id(), matches[i].size());
				}
				// If objects detected, set max score to one detected with the most
				// matches. Otherwise select any objects with the most matches.
				if(objectsDetected.empty() || objectsDetected.contains(objects_.at(objectIndex)->id()))
				{
					if(maxScoreId == -1 || maxScore < matches[i].size())
					{
						maxScoreId = objects_.at(objectIndex)->id();
						maxScore = matches[i].size();
					}
				}
			}

			//update likelihood plot
			likelihoodCurve_->setData(scores, QMap<int, int>());
			inliersCurve_->setData(inliersScores, QMap<int, int>());
			if(ui_->likelihoodPlot->isVisible())
			{
				ui_->likelihoodPlot->update();
			}

			ui_->label_minMatchedDistance->setNum(minMatchedDistance);
			ui_->label_maxMatchedDistance->setNum(maxMatchedDistance);

			//Scroll objects slider to the best score
			if(maxScoreId>=0 && Settings::getGeneral_autoScroll())
			{
				QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1title").arg(maxScoreId));
				if(label)
				{
					ui_->objects_area->verticalScrollBar()->setValue(label->pos().y());
				}
			}

			// Emit homographies
			if(objectsDetected.size() > 1)
			{
				printf("(%s) %d objects detected!\n",
						QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
						(int)objectsDetected.size());
			}
			else if(objectsDetected.size() == 1)
			{
				printf("(%s) Object %d detected!\n",
						QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str(),
						(int)objectsDetected.begin().key());
			}
			else if(Settings::getGeneral_sendNoObjDetectedEvents())
			{
				printf("(%s) No objects detected.\n",
						QTime::currentTime().toString("HH:mm:ss.zzz").toStdString().c_str());
			}

			if(objectsDetected.size() > 0 || Settings::getGeneral_sendNoObjDetectedEvents())
			{
				Q_EMIT objectsFound(objectsDetected);
			}
			ui_->label_objectsDetected->setNum(objectsDetected.size());
		}
		else
		{
			if(!objectsDescriptors_.empty() && keypoints.size())
			{
				this->statusBar()->showMessage(tr("Cannot search, objects must be updated!"));
				printf("Cannot search, objects must be updated!\n");
			}
			if(this->isVisible())
			{
				ui_->imageView_source->setData(keypoints, cv::Mat(), image, Settings::currentDetectorType(), Settings::currentDescriptorType());
			}

		}

		if(this->isVisible())
		{
			//Update object pictures
			for(int i=0; i<objects_.size(); ++i)
			{
				objects_[i]->update();
			}
		}

		ui_->label_nfeatures->setNum((int)keypoints.size());
		ui_->imageView_source->update();
	}
	ui_->label_detectorDescriptorType->setText(QString("%1/%2").arg(Settings::currentDetectorType()).arg(Settings::currentDescriptorType()));

	//update slider
	if(ui_->horizontalSlider_frames->isEnabled())
	{
		ui_->horizontalSlider_frames->blockSignals(true);
		ui_->horizontalSlider_frames->setValue(camera_->getCurrentFrameIndex()-1);
		ui_->label_frame->setNum(camera_->getCurrentFrameIndex()-1);
		ui_->horizontalSlider_frames->blockSignals(false);
	}

	ui_->label_timeTotal->setNum(totalTime.elapsed());
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
}

void MainWindow::notifyParametersChanged(const QStringList & paramChanged)
{
	//Selective update (to not update all objects for a simple camera's parameter modification)
	bool detectorDescriptorParamsChanged = false;
	bool nearestNeighborParamsChanged = false;
	for(QStringList::const_iterator iter = paramChanged.begin(); iter!=paramChanged.end(); ++iter)
	{
		printf("Parameter changed: %s -> \"%s\"\n", iter->toStdString().c_str(), Settings::getParameter(*iter).toString().toStdString().c_str());
		printf("lastObjectsUpdateParameters_.value(*iter)=%s, Settings::getParameter(*iter)=%s",
				lastObjectsUpdateParameters_.value(*iter).toString().toStdString().c_str(),
				Settings::getParameter(*iter).toString().toStdString().c_str());
		if(lastObjectsUpdateParameters_.value(*iter) != Settings::getParameter(*iter))
		{
			if(!detectorDescriptorParamsChanged && iter->contains("Feature2D"))
			{
				detectorDescriptorParamsChanged = true;
			}
			else if(!nearestNeighborParamsChanged &&
					( (iter->contains("NearestNeighbor") && Settings::getGeneral_invertedSearch()) ||
					  iter->compare(Settings::kGeneral_invertedSearch()) == 0 ||
					  (iter->compare(Settings::kGeneral_vocabularyIncremental()) == 0 && Settings::getGeneral_invertedSearch()) ||
					  (iter->compare(Settings::kGeneral_threads()) == 0 && !Settings::getGeneral_invertedSearch()) ))
			{
				nearestNeighborParamsChanged = true;
			}
			lastObjectsUpdateParameters_[*iter] = Settings::getParameter(*iter);
		}

		if(iter->compare(Settings::kGeneral_port()) == 0 &&
		   Settings::getGeneral_port() != ui_->label_port->text().toInt() &&
		   Settings::getGeneral_port() != 0)
		{
			setupTCPServer();
		}
	}

	if(detectorDescriptorParamsChanged)
	{
		//Re-init detector and extractor
		delete detector_;
		delete extractor_;
		detector_ = Settings::createKeypointDetector();
		extractor_ = Settings::createDescriptorExtractor();
		Q_ASSERT(detector_ != 0 && extractor_ != 0);
	}

	if(Settings::getGeneral_autoUpdateObjects())
	{
		if(detectorDescriptorParamsChanged)
		{
			this->updateObjects();
		}
		else if(nearestNeighborParamsChanged)
		{
			this->updateData();
		}
	}
	else if(objects_.size() && (detectorDescriptorParamsChanged || nearestNeighborParamsChanged))
	{
		this->statusBar()->showMessage(tr("A parameter has changed... \"Update objects\" may be required."));
	}
	if(!camera_->isRunning() && !ui_->imageView_source->cvImage().empty())
	{
		this->update(ui_->imageView_source->cvImage());
		ui_->label_timeRefreshRate->setVisible(false);
	}

	ui_->actionCamera_from_video_file->setChecked(!Settings::getCamera_5mediaPath().isEmpty() && !UDirectory::exists(Settings::getCamera_5mediaPath().toStdString()));
	ui_->actionCamera_from_directory_of_images->setChecked(!Settings::getCamera_5mediaPath().isEmpty() && UDirectory::exists(Settings::getCamera_5mediaPath().toStdString()));
	ui_->actionCamera_from_TCP_IP->setChecked(Settings::getCamera_6useTcpCamera());
}
