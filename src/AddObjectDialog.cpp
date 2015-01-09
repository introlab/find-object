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

#include "find_object/Camera.h"
#include "find_object/Settings.h"
#include "find_object/utilite/ULogger.h"
#include "find_object/ObjWidget.h"
#include "find_object/QtOpenCV.h"

#include "AddObjectDialog.h"
#include "ui_addObjectDialog.h"
#include "KeypointItem.h"
#include "ObjSignature.h"

#include <stdio.h>

#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsPixmapItem>
#include <QtGui/QMessageBox>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace find_object {

AddObjectDialog::AddObjectDialog(Camera * camera, const cv::Mat & image, bool mirrorView, QWidget * parent, Qt::WindowFlags f) :
		QDialog(parent, f),
		camera_(camera),
		objWidget_(0),
		objSignature_(0)
{
	ui_ = new Ui_addObjectDialog();
	ui_->setupUi(this);

	detector_ = Settings::createKeypointDetector();
	extractor_ = Settings::createDescriptorExtractor();
	UASSERT(detector_ != 0 && extractor_ != 0);

	connect(ui_->pushButton_cancel, SIGNAL(clicked()), this, SLOT(cancel()));
	connect(ui_->pushButton_back, SIGNAL(clicked()), this, SLOT(back()));
	connect(ui_->pushButton_next, SIGNAL(clicked()), this, SLOT(next()));
	connect(ui_->pushButton_takePicture, SIGNAL(clicked()), this, SLOT(takePicture()));
	connect(ui_->comboBox_selection, SIGNAL(currentIndexChanged(int)), this, SLOT(changeSelectionMode()));

	connect(ui_->cameraView, SIGNAL(selectionChanged()), this, SLOT(updateNextButton()));
	connect(ui_->cameraView, SIGNAL(roiChanged(const cv::Rect &)), this, SLOT(updateNextButton(const cv::Rect &)));
	ui_->cameraView->setMirrorView(mirrorView);

	if((camera_ && camera_->isRunning()) || image.empty())
	{
		this->setState(kTakePicture);
	}
	else if(!image.empty())
	{
		update(image);
		this->setState(kSelectFeatures);
	}
}

AddObjectDialog::~AddObjectDialog()
{
	delete detector_;
	delete extractor_;
	if(objWidget_)
	{
		delete objWidget_;
		objWidget_ = 0;
	}
	if(objSignature_)
	{
		delete objSignature_;
		objSignature_ = 0;
	}
	delete ui_;
}

void AddObjectDialog::retrieveObject(ObjWidget ** widget, ObjSignature ** signature)
{
	*widget = objWidget_;
	objWidget_= 0;
	*signature = objSignature_;
	objSignature_ = 0;
}

void AddObjectDialog::closeEvent(QCloseEvent* event)
{
	if(camera_)
	{
		disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
	}
	QDialog::closeEvent(event);
}

void AddObjectDialog::next()
{
	setState(state_+1);
}
void AddObjectDialog::back()
{
	setState(state_-1);
}
void AddObjectDialog::cancel()
{
	this->reject();
}
void AddObjectDialog::takePicture()
{
	next();
}

void AddObjectDialog::updateNextButton()
{
	updateNextButton(cv::Rect());
}

void AddObjectDialog::updateNextButton(const cv::Rect & rect)
{
	roi_ = rect;
	if(roi_.height && roi_.width && cameraImage_.cols)
	{
		//clip roi
		if( roi_.x >= cameraImage_.cols ||
			roi_.x+roi_.width <= 0 ||
			roi_.y >= cameraImage_.rows ||
			roi_.y+roi_.height <= 0)
		{
			//Not valid...
			roi_ = cv::Rect();
		}
		else
		{
			if(roi_.x < 0)
			{
				roi_.x = 0;
			}
			if(roi_.x + roi_.width > cameraImage_.cols)
			{
				roi_.width = cameraImage_.cols - roi_.x;
			}
			if(roi_.y < 0)
			{
				roi_.y = 0;
			}
			if(roi_.y + roi_.height > cameraImage_.rows)
			{
				roi_.height = cameraImage_.rows - roi_.y;
			}
		}
	}
	if(state_ == kSelectFeatures)
	{
		if(ui_->comboBox_selection->currentIndex() == 1)
		{
			if(ui_->cameraView->selectedItems().size() > 0)
			{
				ui_->pushButton_next->setEnabled(true);
			}
			else
			{
				ui_->pushButton_next->setEnabled(false);
			}
		}
		else
		{
			if(roi_.width == 0 || roi_.height == 0)
			{
				ui_->pushButton_next->setEnabled(false);
			}
			else
			{
				ui_->pushButton_next->setEnabled(true);
			}
		}
	}
}

void AddObjectDialog::changeSelectionMode()
{
	this->setState(kSelectFeatures);
}

void AddObjectDialog::setState(int state)
{
	state_ = state;
	if(state == kTakePicture)
	{
		ui_->pushButton_cancel->setEnabled(true);
		ui_->pushButton_back->setEnabled(false);
		ui_->pushButton_next->setEnabled(false);
		ui_->pushButton_takePicture->setEnabled(true);
		ui_->label_instruction->setText(tr("Place the object in front of the camera and click \"Take picture\"."));
		ui_->pushButton_next->setText(tr("Next"));
		ui_->cameraView->setVisible(true);
		ui_->cameraView->clearRoiSelection();
		ui_->objectView->setVisible(false);
		ui_->cameraView->setGraphicsViewMode(false);
		ui_->comboBox_selection->setVisible(false);
		if(!camera_ || !camera_->start())
		{
			QMessageBox::critical(this, tr("Camera error"), tr("Camera is not started!"));
			ui_->pushButton_takePicture->setEnabled(false);
		}
		else
		{
			connect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
		}
	}
	else if(state == kSelectFeatures)
	{
		if(camera_)
		{
			disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
			camera_->pause();
		}

		ui_->pushButton_cancel->setEnabled(true);
		ui_->pushButton_back->setEnabled(camera_);
		ui_->pushButton_next->setEnabled(false);
		ui_->pushButton_takePicture->setEnabled(false);
		ui_->pushButton_next->setText(tr("Next"));
		ui_->cameraView->setVisible(true);
		ui_->cameraView->clearRoiSelection();
		ui_->objectView->setVisible(false);
		ui_->comboBox_selection->setVisible(true);

		if(ui_->comboBox_selection->currentIndex() == 1)
		{
			ui_->label_instruction->setText(tr("Select features representing the object."));
			ui_->cameraView->setGraphicsViewMode(true);
		}
		else
		{
			ui_->label_instruction->setText(tr("Select region representing the object."));
			ui_->cameraView->setGraphicsViewMode(false);
		}
		updateNextButton(cv::Rect());
	}
	else if(state == kVerifySelection)
	{
		if(camera_)
		{
			disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
			camera_->pause();
		}

		ui_->pushButton_cancel->setEnabled(true);
		ui_->pushButton_back->setEnabled(true);
		ui_->pushButton_takePicture->setEnabled(false);
		ui_->pushButton_next->setText(tr("End"));
		ui_->cameraView->setVisible(true);
		ui_->objectView->setVisible(true);
		ui_->objectView->setMirrorView(ui_->cameraView->isMirrorView());
		ui_->objectView->setSizedFeatures(ui_->cameraView->isSizedFeatures());
		ui_->comboBox_selection->setVisible(false);
		if(ui_->comboBox_selection->currentIndex() == 1)
		{
			ui_->cameraView->setGraphicsViewMode(true);
		}
		else
		{
			ui_->cameraView->setGraphicsViewMode(false);
		}

		std::vector<cv::KeyPoint> selectedKeypoints = ui_->cameraView->selectedKeypoints();

		// Select keypoints
		if(!cameraImage_.empty() &&
				((ui_->comboBox_selection->currentIndex() == 1 && selectedKeypoints.size()) ||
				 (ui_->comboBox_selection->currentIndex() == 0 && roi_.width && roi_.height)))
		{
			if(ui_->comboBox_selection->currentIndex() == 1)
			{
				roi_ = computeROI(selectedKeypoints);
			}

			cv::Mat imgRoi(cameraImage_, roi_);

			if(ui_->comboBox_selection->currentIndex() == 1)
			{
				if(roi_.x != 0 || roi_.y != 0)
				{
					for(unsigned int i=0; i<selectedKeypoints.size(); ++i)
					{
						selectedKeypoints.at(i).pt.x -= roi_.x;
						selectedKeypoints.at(i).pt.y -= roi_.y;
					}
				}
			}
			else
			{
				// Extract keypoints
				selectedKeypoints.clear();
				detector_->detect(imgRoi, selectedKeypoints);
			}
			ui_->objectView->setData(selectedKeypoints, cvtCvMat2QImage(imgRoi.clone()));
			ui_->objectView->setMinimumSize(roi_.width, roi_.height);
			ui_->objectView->update();
			ui_->pushButton_next->setEnabled(true);
		}
		else
		{
			UINFO("Please select items");
			ui_->pushButton_next->setEnabled(false);
		}
		ui_->label_instruction->setText(tr("Selection : %1 features").arg(selectedKeypoints.size()));
	}
	else if(state == kClosing)
	{
		std::vector<cv::KeyPoint> keypoints = ui_->objectView->keypoints();
		if((ui_->comboBox_selection->currentIndex() == 1 && keypoints.size()) ||
		   (ui_->comboBox_selection->currentIndex() == 0 && roi_.width && roi_.height))
		{
			cv::Mat descriptors;
			cv::Mat imgRoi(cameraImage_, roi_);
			if(keypoints.size())
			{
				// Extract descriptors
				extractor_->compute(imgRoi, keypoints, descriptors);

				if(keypoints.size() != (unsigned int)descriptors.rows)
				{
					UERROR("keypoints=%d != descriptors=%d", (int)keypoints.size(), descriptors.rows);
				}
			}

			if(objWidget_)
			{
				delete objWidget_;
				objWidget_ = 0;
			}
			if(objSignature_)
			{
				delete objSignature_;
				objSignature_ = 0;
			}
			objSignature_ = new ObjSignature(0, imgRoi.clone(), "");
			objSignature_->setData(keypoints, descriptors);
			objWidget_ = new ObjWidget(0, keypoints, cvtCvMat2QImage(imgRoi.clone()));

			this->accept();
		}
	}
}

void AddObjectDialog::update(const cv::Mat & image)
{
	cameraImage_ = cv::Mat();
	if(!image.empty())
	{
		// convert to grayscale
		if(image.channels() != 1 || image.depth() != CV_8U)
		{
			cv::cvtColor(image, cameraImage_, CV_BGR2GRAY);
		}
		else
		{
			cameraImage_ = image.clone();
		}

		// Extract keypoints
		cv::vector<cv::KeyPoint> keypoints;
		detector_->detect(cameraImage_, keypoints);

		ui_->cameraView->setData(keypoints, cvtCvMat2QImage(cameraImage_));
		ui_->cameraView->update();
	}
	else
	{
		UWARN("Camera cannot get more images (maybe the end of stream is reached)...");
		camera_->stop();
	}
}

cv::Rect AddObjectDialog::computeROI(const std::vector<cv::KeyPoint> & kpts)
{
	cv::Rect roi(0,0,0,0);
	int x1=0,x2=0,h1=0,h2=0;
	for(unsigned int i=0; i<kpts.size(); ++i)
	{
		float radius = kpts.at(i).size / 2;
		if(i==0)
		{
			x1 = int(kpts.at(i).pt.x - radius);
			x2 = int(kpts.at(i).pt.x + radius);
			h1 = int(kpts.at(i).pt.y - radius);
			h2 = int(kpts.at(i).pt.y + radius);
		}
		else
		{
			if(x1 > int(kpts.at(i).pt.x - radius))
			{
				x1 = int(kpts.at(i).pt.x - radius);
			}
			else if(x2 < int(kpts.at(i).pt.x + radius))
			{
				x2 = int(kpts.at(i).pt.x + radius);
			}
			if(h1 > int(kpts.at(i).pt.y - radius))
			{
				h1 = int(kpts.at(i).pt.y - radius);
			}
			else if(h2 < int(kpts.at(i).pt.y + radius))
			{
				h2 = int(kpts.at(i).pt.y + radius);
			}
		}
		roi.x = x1;
		roi.y = h1;
		roi.width = x2-x1;
		roi.height = h2-h1;
		//UINFO("ptx=%d, pty=%d", (int)kpts.at(i).pt.x, (int)kpts.at(i).pt.y);
		//UINFO("x=%d, y=%d, w=%d, h=%d", roi.x, roi.y, roi.width, roi.height);
	}

	return roi;
}

} // namespace find_object
