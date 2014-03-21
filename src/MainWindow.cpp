/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "MainWindow.h"
#include "AddObjectDialog.h"
#include "ui_mainWindow.h"
#include "QtOpenCV.h"
#include "KeypointItem.h"
#include "ObjWidget.h"
#include "Camera.h"
#include "Settings.h"
#include "ParametersToolBox.h"
#include "AboutDialog.h"
#include "rtabmap/PdfPlot.h"

#include <iostream>
#include <stdio.h>

#include "opencv2/calib3d/calib3d.hpp"

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

#include "utilite/UDirectory.h"

// Camera ownership transferred
MainWindow::MainWindow(Camera * camera, QWidget * parent) :
	QMainWindow(parent),
	camera_(camera),
	likelihoodCurve_(0),
	lowestRefreshRate_(99),
	objectsModified_(false)
{
	ui_ = new Ui_mainWindow();
	ui_->setupUi(this);
	aboutDialog_ = new AboutDialog(this);
	this->setStatusBar(new QStatusBar());

	likelihoodCurve_ = new rtabmap::PdfPlotCurve("Likelihood", &imagesMap_, this);
	ui_->likelihoodPlot->addCurve(likelihoodCurve_, false);
	ui_->likelihoodPlot->setGraphicsView(true);

	if(!camera_)
	{
		camera_ = new Camera(this);
	}
	else
	{
		camera_->setParent(this);
	}

	ui_->dockWidget_parameters->setVisible(false);
	ui_->dockWidget_plot->setVisible(false);
	ui_->widget_controls->setVisible(false);

	QByteArray geometry;
	QByteArray state;
	Settings::loadSettings(Settings::iniDefaultPath(), &geometry, &state);
	this->restoreGeometry(geometry);
	this->restoreState(state);

	ui_->toolBox->setupUi();
	connect((QDoubleSpinBox*)ui_->toolBox->getParameterWidget(Settings::kCamera_4imageRate()),
			SIGNAL(editingFinished()),
			camera_,
			SLOT(updateImageRate()));
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
	connect(ui_->actionAbout, SIGNAL(triggered()), aboutDialog_ , SLOT(exec()));
	connect(ui_->actionRestore_all_default_settings, SIGNAL(triggered()), ui_->toolBox, SLOT(resetAllPages()));
	connect(ui_->actionRemove_all_objects, SIGNAL(triggered()), this, SLOT(removeAllObjects()));

	connect(ui_->pushButton_play, SIGNAL(clicked()), this, SLOT(startProcessing()));
	connect(ui_->pushButton_stop, SIGNAL(clicked()), this, SLOT(stopProcessing()));
	connect(ui_->pushButton_pause, SIGNAL(clicked()), this, SLOT(pauseProcessing()));
	connect(ui_->horizontalSlider_frames, SIGNAL(valueChanged(int)), this, SLOT(moveCameraFrame(int)));
	connect(ui_->horizontalSlider_frames, SIGNAL(valueChanged(int)), ui_->label_frame, SLOT(setNum(int)));
	ui_->pushButton_play->setVisible(true);
	ui_->pushButton_pause->setVisible(false);
	ui_->pushButton_stop->setEnabled(true);
	ui_->horizontalSlider_frames->setEnabled(false);
	ui_->label_frame->setVisible(false);

	ui_->objects_area->addAction(ui_->actionAdd_object_from_scene);
	ui_->objects_area->addAction(ui_->actionAdd_objects_from_files);
	ui_->objects_area->setContextMenuPolicy(Qt::ActionsContextMenu);

	ui_->actionStart_camera->setShortcut(Qt::Key_Space);
	ui_->actionPause_camera->setShortcut(Qt::Key_Space);

	ui_->actionCamera_from_video_file->setChecked(!Settings::getCamera_5mediaPath().isEmpty() && !UDirectory::exists(Settings::getCamera_5mediaPath().toStdString()));
	ui_->actionCamera_from_directory_of_images->setChecked(!Settings::getCamera_5mediaPath().isEmpty() && UDirectory::exists(Settings::getCamera_5mediaPath().toStdString()));

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
		Settings::saveSettings(Settings::iniDefaultPath(), this->saveGeometry(), this->saveState());
		event->accept();
	}
	else
	{
		event->ignore();
	}
}

ParametersToolBox * MainWindow::parametersToolBox() const
{
	return ui_->toolBox;
}

void MainWindow::setSourceImageText(const QString & text)
{
	ui_->imageView_source->setTextLabel(text);
}

int MainWindow::loadObjects(const QString & dirPath)
{
	int loadedObjects = 0;
	QDir dir(dirPath);
	if(dir.exists())
	{
		QStringList filters = Settings::getGeneral_imageFormats().split(' ');
		QFileInfoList list = dir.entryInfoList(filters, QDir::Files, QDir::Name);
		for(int i=0; i<list.size(); ++i)
		{
			this->addObjectFromFile(list.at(i).filePath());
		}
		if(list.size())
		{
			this->updateObjects();
		}
		loadedObjects = list.size();
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
			objects_.at(i)->pixmap().save(QString("%1/%2.bmp").arg(dirPath).arg(objects_.at(i)->id()));
		}
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
		obj->setFeaturesShown(value<=50?false:true);
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
	if(!ui_->actionStart_camera->isEnabled() || ui_->imageView_source->cvImage().empty())
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

void MainWindow::updateObjects()
{
	if(objects_.size())
	{
		for(int i=0; i<objects_.size(); ++i)
		{
			QTime time;
			time.start();
			printf("Extracting descriptors from object %d...\n", objects_.at(i)->id());
			const cv::Mat & img = objects_.at(i)->cvImage();
			cv::FeatureDetector * detector = Settings::createFeaturesDetector();
			std::vector<cv::KeyPoint> keypoints;
			detector->detect(img, keypoints);
			delete detector;

			cv::Mat descriptors;
			if(keypoints.size())
			{
				cv::DescriptorExtractor * extractor = Settings::createDescriptorsExtractor();
				extractor->compute(img, keypoints, descriptors);
				delete extractor;
				if((int)keypoints.size() != descriptors.rows)
				{
					printf("ERROR : kpt=%d != descriptors=%d\n", (int)keypoints.size(), descriptors.rows);
				}
			}
			else
			{
				printf("WARNING: no features detected in object %d !?!\n", objects_.at(i)->id());
			}
			printf("%d descriptors extracted from object %d (in %d ms)\n", descriptors.rows, objects_.at(i)->id(), time.elapsed());
			objects_.at(i)->setData(keypoints, descriptors, img, Settings::currentDetectorType(), Settings::currentDescriptorType());

			//update object labels
			QLabel * title = qFindChild<QLabel*>(this, QString("%1title").arg(objects_.at(i)->id()));
			title->setText(QString("%1 (%2)").arg(objects_.at(i)->id()).arg(QString::number(objects_.at(i)->keypoints().size())));
			QLabel * detectorDescriptorType = qFindChild<QLabel*>(this, QString("%1type").arg(objects_.at(i)->id()));
			detectorDescriptorType->setText(QString("%1/%2").arg(objects_.at(i)->detectorType()).arg(objects_.at(i)->descriptorType()));
		}
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
		printf("Updating global descriptors matrix: Objects=%d, total descriptors=%d, dim=%d, type=%d\n", (int)objects_.size(), count, dim, type);
		if(Settings::getGeneral_invertedSearch() || Settings::getGeneral_threads() == 1)
		{
			// If inverted index or only one thread, put all descriptors in the same cv::Mat
			objectsDescriptors_.push_back(cv::Mat(count, dim, type));
			int row = 0;
			for(int i=0; i<objects_.size(); ++i)
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
			if(Settings::getGeneral_invertedSearch())
			{
				printf("Creating FLANN index (%s) with objects' descriptors...\n", Settings::currentNearestNeighborType().toStdString().c_str());
				// CREATE INDEX
				QTime time;
				time.start();
				cv::flann::IndexParams * params = Settings::createFlannIndexParams();
				flannIndex_.build(objectsDescriptors_[0], *params, Settings::getFlannDistanceType());
				delete params;
				ui_->label_timeIndexing->setNum(time.elapsed());
				ui_->label_vocabularySize->setNum(objectsDescriptors_[0].rows);
				printf("Creating FLANN index (%s) with objects' descriptors... done! (%d ms)\n", Settings::currentNearestNeighborType().toStdString().c_str(), time.elapsed());
			}
		}
		else
		{
			for(int i=0; i<objects_.size(); ++i)
			{
				objectsDescriptors_.push_back(objects_.at(i)->descriptors());
			}
		}
	}
}

void MainWindow::startProcessing()
{
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
			QMessageBox::critical(this, tr("Camera error"), tr("Camera initialization failed! (with device %1)").arg(Settings::getCamera_1deviceId()));
		}
		else
		{
			printf("ERROR: Camera initialization failed! (with device %d)", Settings::getCamera_1deviceId());
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

class SearchThread: public QThread
{
public:
	SearchThread(cv::flann::Index * index, int objectIndex, const cv::Mat * descriptors) :
		index_(index),
		objectIndex_(objectIndex),
		descriptors_(descriptors),
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
		index_->knnSearch(*descriptors_, results, dists, k, Settings::getFlannSearchParams() ); // maximum number of leafs checked

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
			if(minMatchedDistance_ == -1 || minMatchedDistance_ > dists.at<float>(i,0))
			{
				minMatchedDistance_ = dists.at<float>(i,0);
			}
			if(maxMatchedDistance_ == -1 || maxMatchedDistance_ < dists.at<float>(i,0))
			{
				maxMatchedDistance_ = dists.at<float>(i,0);
			}

			if(matched)
			{
				matches_.insert(i, results.at<int>(i,0));
			}
		}

		//printf("Search Object %d time=%d ms\n", objectIndex_, time.elapsed());
	}
private:
	cv::flann::Index * index_; // would be const but flann search() method is not const!?
	int objectIndex_;
	const cv::Mat * descriptors_;

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
					cv::RANSAC,
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
		IplImage iplImage = image;

		QTime time;
		time.start();

		//Convert to grayscale
		IplImage * imageGrayScale = 0;
		if(iplImage.nChannels != 1 || iplImage.depth != IPL_DEPTH_8U)
		{
			imageGrayScale = cvCreateImage(cvSize(iplImage.width,iplImage.height), IPL_DEPTH_8U, 1);
			cvCvtColor(&iplImage, imageGrayScale, CV_BGR2GRAY);
		}
		cv::Mat img;
		if(imageGrayScale)
		{
			img = cv::Mat(imageGrayScale);
		}
		else
		{
			img =  cv::Mat(&iplImage);
		}

		// EXTRACT KEYPOINTS
		cv::FeatureDetector * detector = Settings::createFeaturesDetector();
		std::vector<cv::KeyPoint> keypoints;
		detector->detect(img, keypoints);
		delete detector;
		ui_->label_timeDetection->setNum(time.restart());

		cv::Mat descriptors;
		if(keypoints.size())
		{
			// EXTRACT DESCRIPTORS
			cv::DescriptorExtractor * extractor = Settings::createDescriptorsExtractor();
			extractor->compute(img, keypoints, descriptors);
			delete extractor;
			if((int)keypoints.size() != descriptors.rows)
			{
				printf("ERROR : kpt=%d != descriptors=%d\n", (int)keypoints.size(), descriptors.rows);
			}
			if(imageGrayScale)
			{
				cvReleaseImage(&imageGrayScale);
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
			ui_->imageView_source->setData(keypoints, cv::Mat(), &iplImage, Settings::currentDetectorType(), Settings::currentDescriptorType());
		}

		// COMPARE
		if(!objectsDescriptors_.empty() &&
		   keypoints.size() &&
		   (Settings::getNearestNeighbor_3nndrRatioUsed() || Settings::getNearestNeighbor_5minDistanceUsed()) &&
		   objectsDescriptors_[0].type() == descriptors.type()) // binary descriptor issue, if the dataTree is not yet updated with modified settings
		{
			QVector<QMultiMap<int, int> > matches(objects_.size()); // Map< ObjectDescriptorIndex, SceneDescriptorIndex >
			float minMatchedDistance = -1.0f;
			float maxMatchedDistance = -1.0f;

			if(!Settings::getGeneral_invertedSearch())
			{
				// CREATE INDEX for the scene
				//printf("Creating FLANN index (%s)\n", Settings::currentNearestNeighborType().toStdString().c_str());
				cv::flann::IndexParams * params = Settings::createFlannIndexParams();
				flannIndex_.build(descriptors, *params, Settings::getFlannDistanceType());
				delete params;
				ui_->label_timeIndexing->setNum(time.restart());
				ui_->label_vocabularySize->setNum(descriptors.rows);
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
					flannIndex_.knnSearch(objectsDescriptors_[0], results, dists, k, Settings::getFlannSearchParams() ); // maximum number of leafs checked
				}
				else
				{
					//match scene to objects
					results = cv::Mat(descriptors.rows, k, CV_32SC1); // results index
					dists = cv::Mat(descriptors.rows, k, CV_32FC1); // Distance results are CV_32FC1
					flannIndex_.knnSearch(descriptors, results, dists, k, Settings::getFlannSearchParams() ); // maximum number of leafs checked
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
							QMap<int, int>::iterator iter = dataRange_.lowerBound(results.at<int>(i,0));
							int objectIndex = iter.value();
							int previousDescriptorIndex = (iter == dataRange_.begin())?0:(--iter).key()+1;
							int objectDescriptorIndex = results.at<int>(i,0) - previousDescriptorIndex;
							matches[objectIndex].insert(objectDescriptorIndex, i);
						}
						else
						{
							QMap<int, int>::iterator iter = dataRange_.lowerBound(i);
							int objectIndex = iter.value();
							int fisrtObjectDescriptorIndex = (iter == dataRange_.begin())?0:(--iter).key()+1;
							int objectDescriptorIndex = i - fisrtObjectDescriptorIndex;
							matches[objectIndex].insert(objectDescriptorIndex, results.at<int>(i,0));
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
					threadCounts = objectsDescriptors_.size();
				}
				for(unsigned int j=0; j<objectsDescriptors_.size(); j+=threadCounts)
				{
					QVector<SearchThread*> threads;

					for(unsigned int k=j; k<j+threadCounts && k<objectsDescriptors_.size(); ++k)
					{
						threads.push_back(new SearchThread(&flannIndex_, k, &objectsDescriptors_[k]));
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
			QMap<int, float> scores;
			int maxScoreId = -1;
			int maxScore = 0;
			QMap<int, QPair<QRect, QTransform> > objectsDetected;

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
						threads.push_back(new HomographyThread(&matches[k], k, &objects_.at(k)->keypoints(), &keypoints));
						threads.back()->start();
					}

					for(int j=0; j<threads.size(); ++j)
					{
						threads[j]->wait();

						int index = threads[j]->getObjectIndex();

						// COLORIZE (should be done in the GUI thread)
						int nColor = index % 11 + 7;
						QColor color((Qt::GlobalColor)(nColor==Qt::yellow?Qt::gray:nColor));
						QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1detection").arg(objects_.at(index)->id()));
						if(!threads[j]->getHomography().empty())
						{
							if(threads[j]->getInliers() >= Settings::getHomography_minimumInliers())
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
										else
										{
											objects_.at(index)->setKptColor(threads[j]->getIndexesA().at(k), QColor(0,0,0));
										}
									}
								}

								const cv::Mat & H = threads[j]->getHomography();
								QTransform hTransform(
									H.at<double>(0,0), H.at<double>(1,0), H.at<double>(2,0),
									H.at<double>(0,1), H.at<double>(1,1), H.at<double>(2,1),
									H.at<double>(0,2), H.at<double>(1,2), H.at<double>(2,2));

								// find center of object
								QRect rect = objects_.at(index)->pixmap().rect();
								objectsDetected.insert(objects_.at(index)->id(), QPair<QRect, QTransform>(rect, hTransform));
								// Example getting the center of the object in the scene using the homography
								//QPoint pos(rect.width()/2, rect.height()/2);
								//hTransform.map(pos)

								// add rectangle
								if(this->isVisible())
								{
									label->setText(QString("%1 in %2 out").arg(threads[j]->getInliers()).arg(threads[j]->getOutliers()));
									QPen rectPen(color);
									rectPen.setWidth(4);
									QGraphicsRectItem * rectItem = new QGraphicsRectItem(rect);
									rectItem->setPen(rectPen);
									rectItem->setTransform(hTransform);
									ui_->imageView_source->addRect(rectItem);
								}
							}
							else
							{
								label->setText(QString("Too low inliers (%1 in %2 out)").arg(threads[j]->getInliers()).arg(threads[j]->getOutliers()));
							}
						}
						else
						{
							label->setText(QString("Too low matches (%1)").arg(matches[index].size()));
						}
					}
				}
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
				}
			}

			//scores
			for(int i=0; i<matches.size(); ++i)
			{
				scores.insert(objects_.at(i)->id(), matches[i].size());
				if(maxScoreId == -1 || maxScore < matches[i].size())
				{
					maxScoreId = objects_.at(i)->id();
					maxScore = matches[i].size();
				}
			}

			//update likelihood plot
			likelihoodCurve_->setData(scores, QMap<int, int>());
			if(ui_->likelihoodPlot->isVisible())
			{
				ui_->likelihoodPlot->update();
			}

			ui_->label_minMatchedDistance->setNum(minMatchedDistance);
			ui_->label_maxMatchedDistance->setNum(maxMatchedDistance);

			//Scroll objects slider to the best score
			if(maxScoreId>=0)
			{
				QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1title").arg(maxScoreId));
				if(label)
				{
					ui_->objects_area->verticalScrollBar()->setValue(label->pos().y());
				}
			}

			// Emit homographies
			if(objectsDetected.size())
			{
				emit objectsFound(objectsDetected);
			}
		}
		else if(this->isVisible())
		{
			ui_->imageView_source->setData(keypoints, cv::Mat(), &iplImage, Settings::currentDetectorType(), Settings::currentDescriptorType());
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
		ui_->label_timeGui->setNum(time.restart());
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
	QString currentDetectorType = Settings::currentDetectorType();
	QString currentDescriptorType = Settings::currentDescriptorType();
	QString currentNNType = Settings::currentNearestNeighborType();
	//printf("currentDescriptorType: %s\n", currentDescriptorType.toStdString().c_str());
	//printf("currentNNType: %s\n", currentNNType.toStdString().c_str());
	for(QStringList::const_iterator iter = paramChanged.begin(); iter!=paramChanged.end(); ++iter)
	{
		printf("Parameter changed: %s\n", iter->toStdString().c_str());
		if(!detectorDescriptorParamsChanged &&
		   ( iter->contains(currentDetectorType) ||
		     iter->contains(currentDescriptorType) ||
		     iter->compare(Settings::kFeature2D_1Detector()) == 0 ||
		     iter->compare(Settings::kFeature2D_2Descriptor()) == 0 ))
		{
			detectorDescriptorParamsChanged = true;
		}
		else if(!nearestNeighborParamsChanged &&
			    ( iter->contains(currentNNType) ||
			      iter->compare(Settings::kGeneral_invertedSearch()) == 0 ||
			      iter->compare(Settings::kGeneral_threads()) == 0 ||
			      iter->compare(Settings::kNearestNeighbor_1Strategy()) == 0 ||
			      iter->compare(Settings::kNearestNeighbor_2Distance_type()) == 0))
		{
			nearestNeighborParamsChanged = true;
		}
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
	else if(objects_.size())
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

}
