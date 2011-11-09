
#include "MainWindow.h"
#include "AddObjectDialog.h"
#include "ui_mainWindow.h"
#include "qtipl.h"
#include "KeypointItem.h"
#include "Object.h"
#include "Camera.h"
#include "Settings.h"
#include "ParametersToolBox.h"

#include <iostream>
#include <stdio.h>

#include "opencv2/calib3d/calib3d.hpp"

#include <QtCore/QTextStream>
#include <QtCore/QFile>

#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsRectItem>

MainWindow::MainWindow(QWidget * parent) :
	QMainWindow(parent),
	camera_(0)
{
	ui_ = new Ui_mainWindow();
	ui_->setupUi(this);

	connect(&cameraTimer_, SIGNAL(timeout()), this, SLOT(update()));

	QByteArray geometry;
	Settings::loadSettings(Settings::iniDefaultFileName, &geometry);
	this->restoreGeometry(geometry);

	ui_->toolBox->setupUi();
	ui_->dockWidget_parameters->hide();
	ui_->menuView->addAction(ui_->dockWidget_parameters->toggleViewAction());
	ui_->menuView->addAction(ui_->dockWidget_objects->toggleViewAction());

	ui_->imageView_source->setGraphicsViewMode(false);

	//reset button
	connect(ui_->pushButton_restoreDefaults, SIGNAL(clicked()), ui_->toolBox, SLOT(resetCurrentPage()));

	ui_->actionStop_camera->setEnabled(false);
	ui_->actionSave_objects->setEnabled(false);

	// Actions
	connect(ui_->actionAdd_object, SIGNAL(triggered()), this, SLOT(addObject()));
	connect(ui_->actionStart_camera, SIGNAL(triggered()), this, SLOT(startCamera()));
	connect(ui_->actionStop_camera, SIGNAL(triggered()), this, SLOT(stopCamera()));
	connect(ui_->actionExit, SIGNAL(triggered()), this, SLOT(close()));
	connect(ui_->actionSave_objects, SIGNAL(triggered()), this, SLOT(saveObjects()));
	connect(ui_->actionLoad_objects, SIGNAL(triggered()), this, SLOT(loadObjects()));
}

MainWindow::~MainWindow()
{
	this->stopCamera();
	if(camera_)
	{
		delete camera_;
		camera_ = 0;
	}
	dataTree_ = cv::Mat();
	qDeleteAll(objects_.begin(), objects_.end());
	objects_.clear();
	delete ui_;
}

void MainWindow::closeEvent(QCloseEvent * event)
{
	Settings::saveSettings(Settings::iniDefaultFileName, this->saveGeometry());
	QMainWindow::closeEvent(event);
}

void MainWindow::loadObjects()
{
	QString fileName = QFileDialog::getOpenFileName(this, tr("Load objects..."), "", "*.obj");
	if(!fileName.isEmpty())
	{
		QFile file(fileName);
		file.open(QIODevice::ReadOnly);
		QDataStream in(&file);
		while(!in.atEnd())
		{
			Object * obj = new Object();
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
	}
}
void MainWindow::saveObjects()
{
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save objects..."), Settings::currentDetectorType()+Settings::currentDescriptorType()+QString("%1.obj").arg(objects_.size()), "*.obj");
	if(!fileName.isEmpty())
	{
		if(!fileName.endsWith(".obj"))
		{
			fileName.append(".obj");//default
		}

		QFile file(fileName);
		file.open(QIODevice::WriteOnly);
		QDataStream out(&file);
		for(int i=0; i<objects_.size(); ++i)
		{
			objects_.at(i)->save(out);
		}
		file.close();
	}
}

void MainWindow::removeObject(Object * object)
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
	this->stopCamera();
	AddObjectDialog dialog(&objects_, this);
	if(dialog.exec() == QDialog::Accepted)
	{
		showObject(objects_.last());
	}
}

void MainWindow::showObject(Object * obj)
{
	if(obj)
	{
		obj->setGraphicsViewMode(false);
		QList<Object*> objs = ui_->objects_area->findChildren<Object*>();
		QVBoxLayout * vLayout = new QVBoxLayout();
		int id = Settings::getGeneral_nextObjID().toInt();
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

		QLabel * title = new QLabel(QString("%1 (%2)").arg(obj->id()).arg(QString::number(obj->keypoints().size())), this);
		QLabel * detectedLabel = new QLabel(this);
		QLabel * detectorDescriptorType = new QLabel(QString("%1/%2").arg(obj->detectorType()).arg(obj->descriptorType()), this);
		detectedLabel->setObjectName(QString("%1detection").arg(obj->id()));
		QHBoxLayout * hLayout = new QHBoxLayout();
		hLayout->addWidget(title);
		hLayout->addWidget(detectorDescriptorType);
		hLayout->addWidget(detectedLabel);
		hLayout->addStretch(1);
		vLayout->addLayout(hLayout);
		vLayout->addWidget(obj);
		objects_.last()->setDeletable(true);
		connect(obj, SIGNAL(removalTriggered(Object*)), this, SLOT(removeObject(Object*)));
		connect(obj, SIGNAL(destroyed(QObject *)), title, SLOT(deleteLater()));
		connect(obj, SIGNAL(destroyed(QObject *)), detectedLabel, SLOT(deleteLater()));
		connect(obj, SIGNAL(destroyed(QObject *)), detectorDescriptorType, SLOT(deleteLater()));
		connect(obj, SIGNAL(destroyed(QObject *)), vLayout, SLOT(deleteLater()));
		ui_->verticalLayout_objects->insertLayout(ui_->verticalLayout_objects->count()-1, vLayout);

		this->updateData();
	}
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
			QMessageBox::critical(this, tr("Error"), tr("Descriptors of the objects are not all the same size!\nObjects opened must have all the same size (and from the same descriptor extractor)."));
			return;
		}
		dim = objects_.at(i)->descriptors().cols;
		if(type >= 0 && objects_.at(i)->descriptors().type() != type)
		{
			QMessageBox::critical(this, tr("Error"), tr("Descriptors of the objects are not all the same type!\nObjects opened must have been processed by the same descriptor extractor."));
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

void MainWindow::startCamera()
{
	if(camera_)
	{
		camera_->close();
		delete camera_;
	}
	camera_ = Settings::createCamera();
	connect(camera_, SIGNAL(ready()), this, SLOT(update()));
	if(camera_->init())
	{
		cameraTimer_.start(1000/Settings::getCamera_imageRate().toInt());
		ui_->actionStop_camera->setEnabled(true);
		ui_->actionStart_camera->setEnabled(false);
	}
	else
	{
		QMessageBox::critical(this, tr("Camera error"), tr("Camera initialization failed! (with device %1)").arg(Settings::getCamera_deviceId().toInt()));
	}
}

void MainWindow::stopCamera()
{
	if(camera_)
	{
		cameraTimer_.stop();
		camera_->close();
		delete camera_;
		camera_ = 0;
	}
	ui_->actionStop_camera->setEnabled(false);
	ui_->actionStart_camera->setEnabled(true);
}

void MainWindow::update()
{
	// reset objects color
	for(int i=0; i<objects_.size(); ++i)
	{
		objects_[i]->resetKptsColor();
	}

	if(camera_)
	{
		IplImage * cvImage = camera_->takeImage();
		if(cvImage)
		{
			QTime time;

			//Convert to grayscale
			IplImage * imageGrayScale = 0;
			if(cvImage->nChannels != 1 || cvImage->depth != IPL_DEPTH_8U)
			{
				imageGrayScale = cvCreateImage(cvSize(cvImage->width,cvImage->height), IPL_DEPTH_8U, 1);
				cvCvtColor(cvImage, imageGrayScale, CV_BGR2GRAY);
			}
			cv::Mat img;
			if(imageGrayScale)
			{
				img = cv::Mat(imageGrayScale);
			}
			else
			{
				img =  cv::Mat(cvImage);
			}

			// EXTRACT KEYPOINTS
			time.start();
			cv::FeatureDetector * detector = Settings::createFeaturesDetector();
			std::vector<cv::KeyPoint> keypoints;
			detector->detect(img, keypoints);
			delete detector;
			ui_->label_timeDetection->setText(QString::number(time.elapsed()));

			// EXTRACT DESCRIPTORS
			time.start();
			cv::Mat descriptors;
			cv::DescriptorExtractor * extractor = Settings::createDescriptorsExtractor();
			extractor->compute(img, keypoints, descriptors);
			delete extractor;
			if(keypoints.size() != descriptors.rows)
			{
				printf("ERROR : kpt=%lu != descriptors=%d\n", keypoints.size(), descriptors.rows);
			}
			if(imageGrayScale)
			{
				cvReleaseImage(&imageGrayScale);
			}
			ui_->label_timeExtraction->setText(QString::number(time.elapsed()));

			// COMPARE
			int alpha = 20*255/100;
			if(!dataTree_.empty())
			{
				// CREATE INDEX
				time.start();
				cv::Mat environment(descriptors.rows, descriptors.cols, CV_32F);
				descriptors.convertTo(environment, CV_32F);
				cv::flann::Index treeFlannIndex(environment, cv::flann::KDTreeIndexParams());
				ui_->label_timeIndexing->setText(QString::number(time.elapsed()));

				// DO NEAREST NEIGHBOR
				time.start();
				int k = 2;
				int emax = 64;
				cv::Mat results(dataTree_.rows, k, CV_32SC1); // results index
				cv::Mat dists(dataTree_.rows, k, CV_32FC1); // Distance results are CV_32FC1
				treeFlannIndex.knnSearch(dataTree_, results, dists, k, cv::flann::SearchParams(emax) ); // maximum number of leafs checked
				ui_->label_timeMatching->setText(QString::number(time.elapsed()));


				// PROCESS RESULTS
				time.start();
				ui_->imageView_source->setData(keypoints, cv::Mat(), cvImage);
				int j=0;
				std::vector<cv::Point2f> mpts_1, mpts_2;
				std::vector<int> indexes_1, indexes_2;
				std::vector<uchar> outlier_mask;
				for(unsigned int i=0; i<dataTree_.rows; ++i)
				{
					// Check if this descriptor matches with those of the objects
					// Apply NNDR
					if(dists.at<float>(i,0) <= Settings::getNN_nndrRatio().toFloat() * dists.at<float>(i,1))
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
						mpts_2.push_back(ui_->imageView_source->keypoints().at(results.at<int>(i,0)).pt);
						indexes_2.push_back(results.at<int>(i,0));
					}

					if(i+1 >= dataRange_.at(j))
					{
						QLabel * label = ui_->dockWidget_objects->findChild<QLabel*>(QString("%1detection").arg(objects_.at(j)->id()));
						if(mpts_1.size() >= Settings::getHomography_minimumInliers().toInt())
						{
							cv::Mat H = findHomography(mpts_1,
									mpts_2,
									cv::RANSAC,
									Settings::getHomography_ransacReprojThr().toDouble(),
									outlier_mask);
							int inliers=0, outliers=0;
							QColor color((Qt::GlobalColor)(j % 12 + 7 ));
							color.setAlpha(alpha);
							for(int k=0; k<mpts_1.size();++k)
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
							if(inliers >= Settings::getHomography_minimumInliers().toInt())
							{
								for(int k=0; k<mpts_1.size();++k)
								{
									if(outlier_mask.at(k))
									{
										objects_.at(j)->setKptColor(indexes_1.at(k), color);
										ui_->imageView_source->setKptColor(indexes_2.at(k), color);
									}
									else
									{
										objects_.at(j)->setKptColor(indexes_1.at(k), QColor(0,0,0,alpha));
									}
								}

								label->setText(QString("%1 in %2 out").arg(inliers).arg(outliers));
								QTransform hTransform(
										H.at<double>(0,0), H.at<double>(1,0), H.at<double>(2,0),
										H.at<double>(0,1), H.at<double>(1,1), H.at<double>(2,1),
										H.at<double>(0,2), H.at<double>(1,2), H.at<double>(2,2));
								QPen rectPen(color);
								rectPen.setWidth(4);
								QGraphicsRectItem * rectItem = new QGraphicsRectItem(objects_.at(j)->image().rect());
								rectItem->setPen(rectPen);
								rectItem->setTransform(hTransform);
								ui_->imageView_source->addRect(rectItem);

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
						mpts_1.clear();
						mpts_2.clear();
						indexes_1.clear();
						indexes_2.clear();
						outlier_mask.clear();
						++j;
					}
				}
			}
			else
			{
				ui_->imageView_source->setData(keypoints, cv::Mat(), cvImage);
			}

			//Update object pictures
			for(int i=0; i<objects_.size(); ++i)
			{
				objects_[i]->update();
			}

			ui_->label_nfeatures->setText(QString::number(keypoints.size()));
			ui_->imageView_source->update();
			ui_->label_timeGui->setText(QString::number(time.elapsed()));

			cvReleaseImage(&cvImage);
		}
	}
	ui_->label_detectorDescriptorType->setText(QString("%1/%2").arg(Settings::currentDetectorType()).arg(Settings::currentDescriptorType()));
	ui_->label_timeRefreshRate->setText(QString("(%1 Hz - %2 Hz)").arg(QString::number(1000/cameraTimer_.interval())).arg(QString::number(int(1000.0f/(float)(updateRate_.elapsed())))));
	//printf("GUI refresh rate %f Hz\n", 1000.0f/(float)(updateRate_.elapsed()));
	updateRate_.start();
}
