
#include "AddObjectDialog.h"
#include "ui_addObjectDialog.h"
#include "ObjWidget.h"
#include "KeypointItem.h"
#include "Camera.h"
#include "qtipl.h"
#include "Settings.h"

#include <stdio.h>

#include <QtGui/QGraphicsScene>
#include <QtGui/QGraphicsPixmapItem>
#include <QtGui/QMessageBox>

#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui_c.h>

AddObjectDialog::AddObjectDialog(Camera * camera, QList<ObjWidget*> * objects, QWidget * parent, Qt::WindowFlags f) :
		QDialog(parent, f),
		camera_(camera),
		objects_(objects),
		cvImage_(0)
{
	ui_ = new Ui_addObjectDialog();
	ui_->setupUi(this);

	connect(ui_->pushButton_cancel, SIGNAL(clicked()), this, SLOT(cancel()));
	connect(ui_->pushButton_back, SIGNAL(clicked()), this, SLOT(back()));
	connect(ui_->pushButton_next, SIGNAL(clicked()), this, SLOT(next()));
	connect(ui_->pushButton_takePicture, SIGNAL(clicked()), this, SLOT(takePicture()));

	connect(ui_->cameraView, SIGNAL(selectionChanged()), this, SLOT(updateNextButton()));

	this->setState(kTakePicture);
}

AddObjectDialog::~AddObjectDialog()
{
	if(cvImage_)
	{
		cvReleaseImage(&cvImage_);
	}
	delete ui_;
}

void AddObjectDialog::closeEvent(QCloseEvent* event)
{
	disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
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
	if(state_ == kSelectFeatures)
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
		ui_->objectView->setVisible(false);
		ui_->cameraView->setGraphicsViewMode(false);
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
		disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
		camera_->stop();

		ui_->pushButton_cancel->setEnabled(true);
		ui_->pushButton_back->setEnabled(true);
		ui_->pushButton_next->setEnabled(false);
		ui_->pushButton_takePicture->setEnabled(false);
		ui_->label_instruction->setText(tr("Select features representing the object."));
		ui_->pushButton_next->setText(tr("Next"));
		ui_->cameraView->setVisible(true);
		ui_->objectView->setVisible(false);
		ui_->cameraView->setGraphicsViewMode(true);
		updateNextButton();
	}
	else if(state == kVerifySelection)
	{
		disconnect(camera_, SIGNAL(imageReceived(const cv::Mat &)), this, SLOT(update(const cv::Mat &)));
		camera_->stop();

		ui_->pushButton_cancel->setEnabled(true);
		ui_->pushButton_back->setEnabled(true);
		ui_->pushButton_takePicture->setEnabled(false);
		ui_->pushButton_next->setText(tr("End"));
		ui_->cameraView->setVisible(true);
		ui_->objectView->setVisible(true);
		ui_->cameraView->setGraphicsViewMode(true);

		std::vector<cv::KeyPoint> selectedKeypoints = ui_->cameraView->selectedKeypoints();

		// Select keypoints
		if(selectedKeypoints.size() && cvImage_)
		{
			CvRect roi = computeROI(selectedKeypoints);
			cvSetImageROI(cvImage_, roi);
			if(roi.x != 0 || roi.y != 0)
			{
				for(unsigned int i=0; i<selectedKeypoints.size(); ++i)
				{
					selectedKeypoints.at(i).pt.x -= roi.x;
					selectedKeypoints.at(i).pt.y -= roi.y;
				}
			}
			ui_->objectView->setData(selectedKeypoints, cv::Mat(), cvImage_);
			ui_->objectView->setMinimumSize(roi.width, roi.height);
			ui_->objectView->update();
			cvResetImageROI(cvImage_);
			ui_->pushButton_next->setEnabled(true);
		}
		else
		{
			printf("Please select items\n");
			ui_->pushButton_next->setEnabled(false);
		}
		ui_->label_instruction->setText(tr("Selection : %1 features").arg(selectedKeypoints.size()));
	}
	else if(state == kClosing)
	{
		std::vector<cv::KeyPoint> selectedKeypoints = ui_->cameraView->selectedKeypoints();
		if(selectedKeypoints.size())
		{
			// Extract descriptors
			cv::Mat descriptors;
			cv::DescriptorExtractor * extractor = Settings::createDescriptorsExtractor();
			extractor->compute(cvImage_, selectedKeypoints, descriptors);
			delete extractor;
			if(selectedKeypoints.size() != (unsigned int)descriptors.rows)
			{
				printf("ERROR : keypoints=%lu != descriptors=%d\n", selectedKeypoints.size(), descriptors.rows);
			}

			CvRect roi = computeROI(selectedKeypoints);
			cvSetImageROI(cvImage_, roi);
			if(roi.x != 0 || roi.y != 0)
			{
				for(unsigned int i=0; i<selectedKeypoints.size(); ++i)
				{
					selectedKeypoints.at(i).pt.x -= roi.x;
					selectedKeypoints.at(i).pt.y -= roi.y;
				}
			}
			objects_->append(new ObjWidget(0, selectedKeypoints, descriptors, cvImage_, Settings::currentDetectorType(), Settings::currentDescriptorType()));
			cvResetImageROI(cvImage_);



			this->accept();
		}
	}
}

void AddObjectDialog::update(const cv::Mat & image)
{
	if(cvImage_)
	{
		cvReleaseImage(&cvImage_);
		cvImage_ = 0;
	}
	IplImage iplImage = image;
	cvImage_ = cvCloneImage(& iplImage);
	if(cvImage_)
	{
		// convert to grayscale
		if(cvImage_->nChannels != 1 || cvImage_->depth != IPL_DEPTH_8U)
		{
			IplImage * imageGrayScale = cvCreateImage(cvSize(cvImage_->width, cvImage_->height), IPL_DEPTH_8U, 1);
			cvCvtColor(cvImage_, imageGrayScale, CV_BGR2GRAY);
			cvReleaseImage(&cvImage_);
			cvImage_ = imageGrayScale;
		}

		// Extract keypoints
		cv::FeatureDetector * detector = Settings::createFeaturesDetector();
		cv::vector<cv::KeyPoint> keypoints;
		detector->detect(cvImage_, keypoints);
		delete detector;

		ui_->cameraView->setData(keypoints, cv::Mat(), cvImage_);
		ui_->cameraView->update();
	}
}

CvRect AddObjectDialog::computeROI(const std::vector<cv::KeyPoint> & kpts)
{
	CvRect roi = cvRect(0,0,0,0);
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
		//printf("ptx=%d, pty=%d\n", (int)kpts.at(i).pt.x, (int)kpts.at(i).pt.y);
		//printf("x=%d, y=%d, w=%d, h=%d\n", roi.x, roi.y, roi.width, roi.height);
	}

	return roi;
}
