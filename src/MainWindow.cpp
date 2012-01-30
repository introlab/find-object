/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "MainWindow.h"
#include "AddObjectDialog.h"
#include "ui_mainWindow.h"
#include "qtipl.h"
#include "KeypointItem.h"
#include "ObjWidget.h"
#include "Camera.h"
#include "Settings.h"
#include "ParametersToolBox.h"
#include "AboutDialog.h"

#include <iostream>
#include <stdio.h>

#include "opencv2/calib3d/calib3d.hpp"

#include <QtCore/QTextStream>
#include <QtCore/QFile>

#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsRectItem>
#include <QtGui/QSpinBox>
#include <QtGui/QStatusBar>
#include <QtGui/QProgressDialog>

// Camera ownership transferred
MainWindow::MainWindow(Camera * camera, QWidget * parent) :
	QMainWindow(parent),
	camera_(camera),
	lowestRefreshRate_(99)
{
	ui_ = new Ui_mainWindow();
	ui_->setupUi(this);
	aboutDialog_ = new AboutDialog(this);
	this->setStatusBar(new QStatusBar());

	if(!camera_)
	{
		camera_ = new Camera(this);
	}
	else
	{
		camera_->setParent(this);
	}

	QByteArray geometry;
	Settings::loadSettings(Settings::iniDefaultPath(), &geometry);
	this->restoreGeometry(geometry);

	ui_->toolBox->setupUi();
	connect((QSpinBox*)ui_->toolBox->getParameterWidget(Settings::kCamera_imageRate()),
			SIGNAL(editingFinished()),
			camera_,
			SLOT(updateImageRate()));
	ui_->dockWidget_parameters->hide();
	ui_->menuView->addAction(ui_->dockWidget_parameters->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_objects->toggleViewAction());
	connect(ui_->toolBox, SIGNAL(parametersChanged()), this, SLOT(notifyParametersChanged()));

	ui_->imageView_source->setGraphicsViewMode(false);

	//reset button
	connect(ui_->pushButton_restoreDefaults, SIGNAL(clicked()), ui_->toolBox, SLOT(resetCurrentPage()));
	// udpate objects button
	connect(ui_->pushButton_updateObjects, SIGNAL(clicked()), this, SLOT(updateObjects()));

	ui_->actionStop_camera->setEnabled(false);
	ui_->actionPause_camera->setEnabled(false);
	ui_->actionSave_objects->setEnabled(false);

	// Actions
	connect(ui_->actionAdd_object, SIGNAL(triggered()), this, SLOT(addObject()));
	connect(ui_->actionAdd_objects_from_files, SIGNAL(triggered()), this, SLOT(addObjectsFromFiles()));
	connect(ui_->actionLoad_scene_from_file, SIGNAL(triggered()), this, SLOT(loadSceneFromFile()));
	connect(ui_->actionStart_camera, SIGNAL(triggered()), this, SLOT(startProcessing()));
	connect(ui_->actionStop_camera, SIGNAL(triggered()), this, SLOT(stopProcessing()));
	connect(ui_->actionPause_camera, SIGNAL(triggered()), this, SLOT(pauseProcessing()));
	connect(ui_->actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui_->actionSave_objects, SIGNAL(triggered()), this, SLOT(saveObjects()));
	connect(ui_->actionLoad_objects, SIGNAL(triggered()), this, SLOT(loadObjects()));
	connect(ui_->actionSetup_camera_from_video_file, SIGNAL(triggered()), this, SLOT(setupCameraFromVideoFile()));
	connect(ui_->actionSetup_camera_from_video_file_2, SIGNAL(triggered()), this, SLOT(setupCameraFromVideoFile()));
	connect(ui_->actionAbout, SIGNAL(triggered()), aboutDialog_ , SLOT(exec()));
	connect(ui_->actionRestore_all_default_settings, SIGNAL(triggered()), ui_->toolBox, SLOT(resetAllPages()));

	ui_->actionSetup_camera_from_video_file->setCheckable(true);
	ui_->actionSetup_camera_from_video_file_2->setCheckable(true);
	ui_->actionSetup_camera_from_video_file->setChecked(!Settings::getCamera_videoFilePath().isEmpty());
	ui_->actionSetup_camera_from_video_file_2->setChecked(!Settings::getCamera_videoFilePath().isEmpty());

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
	dataTree_ = cv::Mat();
	qDeleteAll(objects_.begin(), objects_.end());
	objects_.clear();
	delete ui_;
}

void MainWindow::closeEvent(QCloseEvent * event)
{
	Settings::saveSettings(Settings::iniDefaultPath(), this->saveGeometry());
	QMainWindow::closeEvent(event);
}

ParametersToolBox * MainWindow::parametersToolBox() const
{
	return ui_->toolBox;
}

bool MainWindow::loadObjects(const QString & fileName)
{
	QFile file(fileName);
	if(file.open(QIODevice::ReadOnly))
	{
		QDataStream in(&file);
		while(!in.atEnd())
		{
			ObjWidget * obj = new ObjWidget();
			obj->load(in);
			bool alreadyLoaded = false;
			for(int i=0; i<objects_.size(); ++i)
			{
				if(objects_.at(i)->id() == obj->id())
				{
					alreadyLoaded = true;
					break;
				}
			}
			if(!alreadyLoaded)
			{
				objects_.append(obj);
				showObject(obj);
			}
			else
			{
				delete obj;
			}
		}
		file.close();
		return true;
	}
	return false;
}

void MainWindow::saveObjects(const QString & fileName)
{
	QFile file(fileName);
	file.open(QIODevice::WriteOnly);
	QDataStream out(&file);
	for(int i=0; i<objects_.size(); ++i)
	{
		objects_.at(i)->save(out);
	}
	file.close();
}

void MainWindow::loadObjects()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Load objects..."), Settings::workingDirectory(), "*.obj");
	if(!fileName.isEmpty())
	{
		loadObjects(fileName);
	}
}
void MainWindow::saveObjects()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save objects..."), (Settings::workingDirectory() + "/") +Settings::currentDetectorType()+Settings::currentDescriptorType()+QString("%1.obj").arg(objects_.size()), "*.obj");
	if(!fileName.isEmpty())
	{
		if(!fileName.endsWith(".obj"))
		{
			fileName.append(".obj");//default
		}
		saveObjects(fileName);
	}
}

void MainWindow::removeObject(ObjWidget * object)
{
	if(object)
	{
		objects_.removeOne(object);
		object->deleteLater();
		this->updateData();
	}
}

void MainWindow::addObject()
{
	disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
	AddObjectDialog dialog(camera_, &objects_, ui_->imageView_source->isMirrorView(), this);
	if(dialog.exec() == QDialog::Accepted)
	{
		showObject(objects_.last());
	}
	this->startProcessing();
}

void MainWindow::addObjectsFromFiles()
{
	QStringList fileNames = QFileDialog::getOpenFileNames(this, tr("Add objects..."), Settings::workingDirectory(), tr("Image Files (*.png *.jpg *.bmp *.tiff)"));
	if(fileNames.size())
	{
		for(int i=0; i<fileNames.size(); ++i)
		{
			IplImage * img = cvLoadImage(fileNames[i].toStdString().c_str(), CV_LOAD_IMAGE_GRAYSCALE);
			if(img)
			{
				objects_.append(new ObjWidget(0, std::vector<cv::KeyPoint>(), cv::Mat(), img, "", ""));
				this->showObject(objects_.last());
				cvReleaseImage(&img);
			}
		}
		updateObjects();
	}
}

void MainWindow::loadSceneFromFile()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Load scene..."), Settings::workingDirectory(), tr("Image Files (*.png *.jpg *.bmp *.tiff)"));
	if(!fileName.isEmpty())
	{
		IplImage * img = cvLoadImage(fileName.toStdString().c_str());
		if(img)
		{
			cv::Mat imageMat(img);
			this->update(imageMat);
			cvReleaseImage(&img);
			ui_->label_timeRefreshRate->setVisible(false);
		}
	}
}

void MainWindow::setupCameraFromVideoFile()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Setup camera from video file..."), Settings::workingDirectory(), tr("Video Files (*.avi *.m4v)"));
	if(!fileName.isEmpty())
	{
		Settings::setCamera_videoFilePath(fileName);
		ui_->toolBox->updateParameter(Settings::kCamera_videoFilePath());
		if(camera_->isRunning())
		{
			this->stopProcessing();
			this->startProcessing();
		}
	}
}

void MainWindow::showObject(ObjWidget * obj)
{
	if(obj)
	{
		obj->setGraphicsViewMode(false);
		obj->setMirrorView(ui_->imageView_source->isMirrorView());
		QList<ObjWidget*> objs = ui_->objects_area->findChildren<ObjWidget*>();
		QVBoxLayout * vLayout = new QVBoxLayout();
		obj->setMinimumSize(obj->image().width(), obj->image().height());
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
		hLayout->addWidget(detectorDescriptorType);
		hLayout->addWidget(detectedLabel);
		hLayout->addStretch(1);
		vLayout->addLayout(hLayout);
		vLayout->addWidget(obj);
		objects_.last()->setDeletable(true);
		connect(obj, SIGNAL(removalTriggered(ObjWidget*)), this, SLOT(removeObject(ObjWidget*)));
		connect(obj, SIGNAL(destroyed(QObject *)), title, SLOT(deleteLater()));
		connect(obj, SIGNAL(destroyed(QObject *)), detectedLabel, SLOT(deleteLater()));
		connect(obj, SIGNAL(destroyed(QObject *)), detectorDescriptorType, SLOT(deleteLater()));
		connect(obj, SIGNAL(destroyed(QObject *)), vLayout, SLOT(deleteLater()));
		ui_->verticalLayout_objects->insertLayout(ui_->verticalLayout_objects->count()-1, vLayout);

		this->updateData();
	}
}

void MainWindow::updateObjects()
{
	if(objects_.size())
	{
		for(int i=0; i<objects_.size(); ++i)
		{
			IplImage * img = cvCloneImage(objects_.at(i)->iplImage());
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
			objects_.at(i)->setData(keypoints, descriptors, img, Settings::currentDetectorType(), Settings::currentDescriptorType());

			//update object labels
			QLabel * title = qFindChild<QLabel*>(this, QString("%1title").arg(objects_.at(i)->id()));
			title->setText(QString("%1 (%2)").arg(objects_.at(i)->id()).arg(QString::number(objects_.at(i)->keypoints().size())));
			QLabel * detectorDescriptorType = qFindChild<QLabel*>(this, QString("%1type").arg(objects_.at(i)->id()));
			detectorDescriptorType->setText(QString("%1/%2").arg(objects_.at(i)->detectorType()).arg(objects_.at(i)->descriptorType()));
		}
		updateData();
		notifyParametersChanged(); // this will update the scene if camera is stopped

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

	dataTree_ = cv::Mat();
	dataRange_.clear();
	int count = 0;
	int dim = -1;
	int type = -1;
	// Get the total size and verify descriptors
	for(int i=0; i<objects_.size(); ++i)
	{
		if(dim >= 0 && objects_.at(i)->descriptors().cols != dim)
		{
			if(this->isVisible())
			{
				QMessageBox::critical(this, tr("Error"), tr("Descriptors of the objects are not all the same size!\nObjects opened must have all the same size (and from the same descriptor extractor)."));
			}
			else
			{
				printf("ERROR: Descriptors of the objects are not all the same size! Objects opened must have all the same size (and from the same descriptor extractor).");
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
				printf("ERROR: Descriptors of the objects are not all the same type! Objects opened must have been processed by the same descriptor extractor.");
			}
			return;
		}
		type = objects_.at(i)->descriptors().type();
		count += objects_.at(i)->descriptors().rows;
	}

	// Copy data
	if(count)
	{
		cv::Mat data(count, dim, type);
		printf("Total descriptors=%d, dim=%d, type=%d\n",count, dim, type);
		int row = 0;
		for(int i=0; i<objects_.size(); ++i)
		{
			cv::Mat dest(data, cv::Range(row, row+objects_.at(i)->descriptors().rows));
			objects_.at(i)->descriptors().copyTo(dest);
			row += objects_.at(i)->descriptors().rows;
			dataRange_.append(row);
		}
		data.convertTo(dataTree_, CV_32F);
	}
}

void MainWindow::startProcessing()
{
	this->statusBar()->showMessage(tr("Starting camera..."));
	if(camera_->start())
	{
		connect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
		ui_->actionStop_camera->setEnabled(true);
		ui_->actionPause_camera->setEnabled(true);
		ui_->actionStart_camera->setEnabled(false);
		ui_->actionLoad_scene_from_file->setEnabled(false);
		ui_->label_timeRefreshRate->setVisible(true);
		this->statusBar()->showMessage(tr("Camera started."), 2000);
	}
	else
	{
		this->statusBar()->clearMessage();
		if(this->isVisible())
		{
			QMessageBox::critical(this, tr("Camera error"), tr("Camera initialization failed! (with device %1)").arg(Settings::getCamera_deviceId()));
		}
		else
		{
			printf("ERROR: Camera initialization failed! (with device %d)", Settings::getCamera_deviceId());
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
}

void MainWindow::pauseProcessing()
{
	ui_->actionStop_camera->setEnabled(true);
	ui_->actionPause_camera->setEnabled(true);
	ui_->actionStart_camera->setEnabled(false);
	if(camera_->isRunning())
	{
		camera_->pause();
	}
	else
	{
		camera_->start();
	}
}

void MainWindow::update(const cv::Mat & image)
{
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
		ui_->label_timeDetection->setText(QString::number(time.restart()));

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
			ui_->label_timeExtraction->setText(QString::number(time.restart()));
		}
		else
		{
			printf("WARNING: no features detected !?!\n");
			ui_->label_timeExtraction->setText(QString::number(0));
		}

		// COMPARE
		if(!dataTree_.empty() && keypoints.size() && (Settings::getNearestNeighbor_nndrRatioUsed() || Settings::getNearestNeighbor_minDistanceUsed()))
		{
			// CREATE INDEX
			cv::Mat environment(descriptors.rows, descriptors.cols, CV_32F);
			descriptors.convertTo(environment, CV_32F);
			cv::flann::Index treeFlannIndex(environment, cv::flann::KDTreeIndexParams());
			ui_->label_timeIndexing->setText(QString::number(time.restart()));
			
			// DO NEAREST NEIGHBOR
			int k = 1;
			if(Settings::getNearestNeighbor_nndrRatioUsed())
			{
				k = 2;
			}
			int emax = 64;
			cv::Mat results(dataTree_.rows, k, CV_32SC1); // results index
			cv::Mat dists(dataTree_.rows, k, CV_32FC1); // Distance results are CV_32FC1
			treeFlannIndex.knnSearch(dataTree_, results, dists, k, cv::flann::SearchParams(emax) ); // maximum number of leafs checked
			ui_->label_timeMatching->setText(QString::number(time.restart()));

			// PROCESS RESULTS
			if(this->isVisible())
			{
				ui_->imageView_source->setData(keypoints, cv::Mat(), &iplImage, Settings::currentDetectorType(), Settings::currentDescriptorType());
			}
			int j=0;
			std::vector<cv::Point2f> mpts_1, mpts_2;
			std::vector<int> indexes_1, indexes_2;
			std::vector<uchar> outlier_mask;
			QMap<int, QPair<QRect, QTransform> > objectsDetected;
			float minMatchedDistance = -1.0f;
			float maxMatchedDistance = -1.0f;
			for(int i=0; i<dataTree_.rows; ++i)
			{
				QColor color((Qt::GlobalColor)(j % 12 + 7 ));
				bool matched = false;
				// Check if this descriptor matches with those of the objects
				if(Settings::getNearestNeighbor_nndrRatioUsed() &&
				   dists.at<float>(i,0) <= Settings::getNearestNeighbor_nndrRatio() * dists.at<float>(i,1))
				{
					matched = true;
				}
				if((matched || !Settings::getNearestNeighbor_nndrRatioUsed()) &&
				   Settings::getNearestNeighbor_minDistanceUsed())
				{
					if(dists.at<float>(i,0) <= Settings::getNearestNeighbor_minDistance())
					{
						matched = true;
					}
					else
					{
						matched = false;
					}
				}
				if(minMatchedDistance == -1 || minMatchedDistance>dists.at<float>(i,0))
				{
					minMatchedDistance = dists.at<float>(i,0);
				}
				if(maxMatchedDistance == -1 || maxMatchedDistance<dists.at<float>(i,0))
				{
					maxMatchedDistance = dists.at<float>(i,0);
				}

				if(matched)
				{
					if(j>0)
					{
						mpts_1.push_back(objects_.at(j)->keypoints().at(i-dataRange_.at(j-1)).pt);
						indexes_1.push_back(i-dataRange_.at(j-1));
					}
					else
					{
						mpts_1.push_back(objects_.at(j)->keypoints().at(i).pt);
						indexes_1.push_back(i);
					}
					mpts_2.push_back(keypoints.at(results.at<int>(i,0)).pt);
					indexes_2.push_back(results.at<int>(i,0));

					// colorize all matched if homography is not computed
					if(!Settings::getHomography_homographyComputed())
					{
						objects_.at(j)->setKptColor(indexes_1.back(), color);
						ui_->imageView_source->setKptColor(results.at<int>(i,0), color);
					}
				}

				if(i+1 >= dataRange_.at(j))
				{
					QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1detection").arg(objects_.at(j)->id()));
					if(Settings::getHomography_homographyComputed())
					{
						if(mpts_1.size() >= Settings::getHomography_minimumInliers())
						{
							cv::Mat H = findHomography(mpts_1,
									mpts_2,
									cv::RANSAC,
									Settings::getHomography_ransacReprojThr(),
									outlier_mask);
							uint inliers=0, outliers=0;
							for(unsigned int k=0; k<mpts_1.size();++k)
							{
								if(outlier_mask.at(k))
								{
									++inliers;
								}
								else
								{
									++outliers;
								}
							}

							// COLORIZE
							if(inliers >= Settings::getHomography_minimumInliers())
							{
								if(this->isVisible())
								{
									for(unsigned int k=0; k<mpts_1.size();++k)
									{
										if(outlier_mask.at(k))
										{
											objects_.at(j)->setKptColor(indexes_1.at(k), color);
											ui_->imageView_source->setKptColor(indexes_2.at(k), color);
										}
										else
										{
											objects_.at(j)->setKptColor(indexes_1.at(k), QColor(0,0,0));
										}
									}
								}

								QTransform hTransform(
									H.at<double>(0,0), H.at<double>(1,0), H.at<double>(2,0),
									H.at<double>(0,1), H.at<double>(1,1), H.at<double>(2,1),
									H.at<double>(0,2), H.at<double>(1,2), H.at<double>(2,2));

								// find center of object
								QRect rect = objects_.at(j)->image().rect();
								objectsDetected.insert(objects_.at(j)->id(), QPair<QRect, QTransform>(rect, hTransform));
								// Example getting the center of the object in the scene using the homography
								//QPoint pos(rect.width()/2, rect.height()/2);
								//hTransform.map(pos)

								// add rectangle
								if(this->isVisible())
								{
									label->setText(QString("%1 in %2 out").arg(inliers).arg(outliers));
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
								label->setText(QString("Too low inliers (%1)").arg(inliers));
							}
						}
						else
						{
							label->setText(QString("Too low matches (%1)").arg(mpts_1.size()));
						}
					}
					else
					{
						label->setText(QString("%1 matches").arg(mpts_1.size()));
					}
					mpts_1.clear();
					mpts_2.clear();
					indexes_1.clear();
					indexes_2.clear();
					outlier_mask.clear();
					++j;
				}
			}
			ui_->label_minMatchedDistance->setNum(minMatchedDistance);
			ui_->label_maxMatchedDistance->setNum(maxMatchedDistance);

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

		ui_->label_nfeatures->setText(QString::number(keypoints.size()));
		ui_->imageView_source->update();
		ui_->label_timeGui->setText(QString::number(time.restart()));
	}
	ui_->label_detectorDescriptorType->setText(QString("%1/%2").arg(Settings::currentDetectorType()).arg(Settings::currentDescriptorType()));

	int refreshRate = qRound(1000.0f/float(updateRate_.restart()));
	if(refreshRate > 0 && refreshRate < lowestRefreshRate_)
	{
		lowestRefreshRate_ = refreshRate;
	}
	// Refresh the label only after each 1000 ms
	if(refreshStartTime_.elapsed() > 1000)
	{
		if(Settings::getCamera_imageRate()>0)
		{
			ui_->label_timeRefreshRate->setText(QString("(%1 Hz - %2 Hz)").arg(QString::number(Settings::getCamera_imageRate())).arg(QString::number(lowestRefreshRate_)));
		}
		else
		{
			ui_->label_timeRefreshRate->setText(QString("(%2 Hz)").arg(QString::number(lowestRefreshRate_)));
		}
		lowestRefreshRate_ = 99;
		refreshStartTime_.start();
	}
}

void MainWindow::notifyParametersChanged()
{
	if(objects_.size())
	{
		this->statusBar()->showMessage(tr("A parameter has changed... \"Update objects\" may be required."));
	}
	if(!camera_->isRunning() && ui_->imageView_source->iplImage())
	{
		cv::Mat image(ui_->imageView_source->iplImage(), true);
		this->update(image);
		ui_->label_timeRefreshRate->setVisible(false);
	}

	ui_->actionSetup_camera_from_video_file->setChecked(!Settings::getCamera_videoFilePath().isEmpty());
	ui_->actionSetup_camera_from_video_file_2->setChecked(!Settings::getCamera_videoFilePath().isEmpty());
}
