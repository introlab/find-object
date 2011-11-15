/*
 * Object.cpp
 *
 *  Created on: 2011-10-23
 *      Author: matlab
 */


/*
 * VisualObject.h
 *
 *  Created on: 2011-10-21
 *      Author: matlab
 */

#include "ObjWidget.h"
#include "KeypointItem.h"
#include "qtipl.h"
#include "Settings.h"

#include <opencv2/highgui/highgui.hpp>

#include <QtGui/QWidget>
#include <QtGui/QContextMenuEvent>
#include <QtGui/QMenu>
#include <QtGui/QMenu>
#include <QtGui/QFileDialog>
#include <QtGui/QAction>
#include <QtGui/QGraphicsView>
#include <QtGui/QGraphicsScene>
#include <QtGui/QVBoxLayout>
#include <QtGui/QGraphicsRectItem>

#include <QtCore/QDir>

#include <stdio.h>

ObjWidget::ObjWidget(QWidget * parent) :
	QWidget(parent),
	iplImage_(0),
	graphicsView_(0),
	id_(0),
	graphicsViewMode_(true),
	detectorType_("NA"),
	descriptorType_("NA")
{
	setupUi();
}
ObjWidget::ObjWidget(int id,
		const std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & descriptors,
		const IplImage * iplImage,
		const QString & detectorType,
		const QString & descriptorType,
		QWidget * parent) :
	QWidget(parent),
	iplImage_(0),
	graphicsView_(0),
	id_(id),
	graphicsViewMode_(true),
	detectorType_(detectorType),
	descriptorType_(descriptorType)
{
	setupUi();
	this->setData(keypoints, descriptors, iplImage);
}
ObjWidget::~ObjWidget()
{
	if(iplImage_)
	{
		cvReleaseImage(&iplImage_);
	}
}

void ObjWidget::setupUi()
{
	graphicsView_ = new QGraphicsView(this);
	graphicsView_->setVisible(true);
	graphicsView_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	graphicsView_->setScene(new QGraphicsScene(graphicsView_));

	this->setLayout(new QVBoxLayout(this));
	this->layout()->addWidget(graphicsView_);
	this->layout()->setContentsMargins(0,0,0,0);

	_menu = new QMenu(tr(""), this);
	_showImage = _menu->addAction(tr("Show image"));
	_showImage->setCheckable(true);
	_showImage->setChecked(true);
	_showFeatures = _menu->addAction(tr("Show features"));
	_showFeatures->setCheckable(true);
	_showFeatures->setChecked(true);
	_mirrorView = _menu->addAction(tr("Mirror view"));
	_mirrorView->setCheckable(true);
	_mirrorView->setChecked(false);
	_plainView = _menu->addAction(tr("Plain view"));
	_plainView->setCheckable(true);
	_plainView->setChecked(!graphicsViewMode_);
	_menu->addSeparator();
	_saveImage = _menu->addAction(tr("Save picture..."));
	_menu->addSeparator();
	_delete = _menu->addAction(tr("Delete"));
	_delete->setEnabled(false);

	this->setId(id_);

	graphicsView_->setRubberBandSelectionMode(Qt::ContainsItemShape);
	graphicsView_->setDragMode(QGraphicsView::RubberBandDrag);

	connect(graphicsView_->scene(), SIGNAL(selectionChanged()), this, SIGNAL(selectionChanged()));
}

void ObjWidget::setId(int id)
{
	id_=id;
	if(id_)
	{
		_savedFileName = QString("object_%1.png").arg(id_);
	}
}

void ObjWidget::setGraphicsViewMode(bool on)
{
	graphicsViewMode_ = on;
	graphicsView_->setVisible(on);
	//update items' color
	if(on)
	{
		if(keypointItems_.size() == 0)
		{
			this->setupGraphicsView();
		}
		else
		{
			for(int i=0; i<keypointItems_.size(); ++i)
			{
				keypointItems_[i]->setColor(kptColors_.at(i));
			}
		}
	}
	graphicsView_->fitInView(graphicsView_->sceneRect(), Qt::KeepAspectRatio);
	_plainView->setChecked(!on);
	this->update();
}

void ObjWidget::setData(const std::vector<cv::KeyPoint> & keypoints, const cv::Mat & descriptors, const IplImage * image)
{
	keypoints_ = keypoints;
	descriptors_ = descriptors;
	kptColors_ = QVector<QColor>(keypoints.size(), defaultColor());
	keypointItems_.clear();
	rectItems_.clear();
	if(iplImage_)
	{
		cvReleaseImage(&iplImage_);
		iplImage_ = 0;
	}
	if(image)
	{
		/* create destination image
		   Note that cvGetSize will return the width and the height of ROI */
		iplImage_ = cvCreateImage(cvGetSize(image),
				image->depth,
				image->nChannels);

		/* copy subimage */
		cvCopy(image, iplImage_, NULL);

		image_ = QPixmap::fromImage(Ipl2QImage(iplImage_));
		this->setMinimumSize(image_.size());
	}
	if(graphicsViewMode_)
	{
		this->setupGraphicsView();
	}
}

void ObjWidget::resetKptsColor()
{
	for(int i=0; i<kptColors_.size(); ++i)
	{
		kptColors_[i] = defaultColor();
		if(graphicsViewMode_)
		{
			keypointItems_[i]->setColor(this->defaultColor());
		}
	}
	rectItems_.clear();
}

void ObjWidget::setKptColor(unsigned int index, const QColor & color)
{
	if(index < kptColors_.size())
	{
		kptColors_[index] = color;
	}

	if(graphicsViewMode_)
	{
		if(index < keypointItems_.size())
		{
			keypointItems_.at(index)->setColor(color);
		}
	}
}

void ObjWidget::addRect(QGraphicsRectItem * rect)
{
	graphicsView_->scene()->addItem(rect);
	rectItems_.append(rect);
}

QList<QGraphicsItem*> ObjWidget::selectedItems() const
{
	return graphicsView_->scene()->selectedItems();
}

bool ObjWidget::isImageShown() const
{
	return _showImage->isChecked();
}

bool ObjWidget::isFeaturesShown() const
{
	return _showFeatures->isChecked();
}

bool ObjWidget::isMirrorView() const
{
	return _mirrorView->isChecked();
}

void ObjWidget::setDeletable(bool deletable)
{
	_delete->setEnabled(deletable);
}

void ObjWidget::save(QDataStream & streamPtr) const
{
	streamPtr << id_ << detectorType_ << descriptorType_;
	streamPtr << (int)keypoints_.size();
	for(int j=0; j<keypoints_.size(); ++j)
	{
		streamPtr << keypoints_.at(j).angle <<
				keypoints_.at(j).class_id <<
				keypoints_.at(j).octave <<
				keypoints_.at(j).pt.x <<
				keypoints_.at(j).pt.y <<
				keypoints_.at(j).response <<
				keypoints_.at(j).size;
	}

	qint64 dataSize = descriptors_.elemSize()*descriptors_.cols*descriptors_.rows;
	streamPtr << descriptors_.rows <<
			descriptors_.cols <<
			descriptors_.type() <<
			dataSize;
	streamPtr << QByteArray((char*)descriptors_.data, dataSize);
	streamPtr << image_;
}

void ObjWidget::load(QDataStream & streamPtr)
{
	std::vector<cv::KeyPoint> kpts;
	cv::Mat descriptors;

	int nKpts;
	streamPtr >> id_ >> detectorType_ >> descriptorType_ >> nKpts;
	for(int i=0;i<nKpts;++i)
	{
		cv::KeyPoint kpt;
		streamPtr >>
		kpt.angle >>
		kpt.class_id >>
		kpt.octave >>
		kpt.pt.x >>
		kpt.pt.y >>
		kpt.response >>
		kpt.size;
		kpts.push_back(kpt);
	}

	int rows,cols,type;
	qint64 dataSize;
	streamPtr >> rows >> cols >> type >> dataSize;
	QByteArray data;
	streamPtr >> data;
	descriptors = cv::Mat(rows, cols, type, data.data()).clone();
	streamPtr >> image_;
	this->setData(kpts, descriptors, 0);
	this->setMinimumSize(image_.size());
}

void ObjWidget::paintEvent(QPaintEvent *event)
{
	if(graphicsViewMode_)
	{
		QWidget::paintEvent(event);
	}
	else
	{
		if(!image_.isNull())
		{
			//Scale
			float w = image_.width();
			float h = image_.height();
			float widthRatio = this->rect().width() / w;
			float heightRatio = this->rect().height() / h;
			float ratio = 1.0f;
			//printf("w=%f, h=%f, wR=%f, hR=%f, sW=%f, sH=%f\n", w, h, widthRatio, heightRatio, sceneRect.width(), sceneRect.height());
			if(widthRatio < heightRatio)
			{
				ratio = widthRatio;
			}
			else
			{
				ratio = heightRatio;
			}

			//printf("ratio=%f\n",ratio);

			w *= ratio;
			h *= ratio;

			float offsetX = 0.0f;
			float offsetY = 0.0f;
			if(w < this->rect().width())
			{
				offsetX = (this->rect().width() - w)/2.0f;
			}
			if(h < this->rect().height())
			{
				offsetY = (this->rect().height() - h)/2.0f;
			}
			QPainter painter(this);


			if(_mirrorView->isChecked())
			{
				painter.translate(offsetX+w, offsetY);
				painter.scale(-ratio, ratio);
			}
			else
			{
				painter.translate(offsetX, offsetY);
				painter.scale(ratio, ratio);
			}

			if(_showImage->isChecked())
			{
				painter.drawPixmap(QPoint(0,0), image_);
			}
			if(_showFeatures->isChecked())
			{
				drawKeypoints(&painter);
			}

			for(int i=0; i<rectItems_.size(); ++i)
			{
				painter.save();
				painter.setTransform(rectItems_.at(i)->transform(), true);
				painter.setPen(rectItems_.at(i)->pen());
				painter.drawRect(rectItems_.at(i)->rect());
				painter.restore();
			}
		}
	}
}

void ObjWidget::resizeEvent(QResizeEvent* event)
{
	QWidget::resizeEvent(event);
	if(graphicsViewMode_)
	{
		graphicsView_->fitInView(graphicsView_->sceneRect(), Qt::KeepAspectRatio);
	}
}

void ObjWidget::contextMenuEvent(QContextMenuEvent * event)
{
	QAction * action = _menu->exec(event->globalPos());
	if(action == _saveImage)
	{
		QString text;
		if(_savedFileName.isEmpty())
		{
			_savedFileName=Settings::workingDirectory()+"/figure.png";
		}
		text = QFileDialog::getSaveFileName(this, tr("Save figure to ..."), _savedFileName, "*.png *.xpm *.jpg *.pdf");
		if(!text.isEmpty())
		{
			if(!text.endsWith(".png") && !text.endsWith(".xpm") && !text.endsWith(".jpg") && !text.endsWith(".pdf"))
			{
				text.append(".png");//default
			}
			_savedFileName = text;
			getSceneAsPixmap().save(text);
		}
	}
	else if(action == _showFeatures || action == _showImage)
	{
		if(graphicsViewMode_)
		{
			this->updateItemsShown();
		}
		else
		{
			this->update();
		}
	}
	else if(action == _mirrorView)
	{
		graphicsView_->setTransform(QTransform().scale(this->isMirrorView()?-1.0:1.0, 1.0));
		if(graphicsViewMode_)
		{
			graphicsView_->fitInView(graphicsView_->sceneRect(), Qt::KeepAspectRatio);
		}
		else
		{
			this->update();
		}
	}
	else if(action == _delete)
	{
		emit removalTriggered(this);
	}
	else if(action == _plainView)
	{
		this->setGraphicsViewMode(!_plainView->isChecked());
	}
	QWidget::contextMenuEvent(event);
}

QPixmap ObjWidget::getSceneAsPixmap()
{
	if(graphicsViewMode_)
	{
		QPixmap img(graphicsView_->sceneRect().width(), graphicsView_->sceneRect().height());
		QPainter p(&img);
		graphicsView_->scene()->render(&p, graphicsView_->sceneRect(), graphicsView_->sceneRect());
		return img;
	}
	else
	{
		return QPixmap::grabWidget(this);
	}
}

void ObjWidget::updateItemsShown()
{
	QList<QGraphicsItem*> items = graphicsView_->scene()->items();
	for(int i=0; i<items.size(); ++i)
	{
		if(qgraphicsitem_cast<KeypointItem*>(items.at(i)))
		{
			items.at(i)->setVisible(_showFeatures->isChecked());
		}
		else if(qgraphicsitem_cast<QGraphicsPixmapItem*>(items.at(i)))
		{
			items.at(i)->setVisible(_showImage->isChecked());
		}
	}
}

void ObjWidget::drawKeypoints(QPainter * painter)
{
	QList<KeypointItem *> items;
	KeypointItem * item = 0;

	int i = 0;
	for(std::vector<cv::KeyPoint>::const_iterator iter = keypoints_.begin(); iter != keypoints_.end(); ++iter, ++i )
	{
		const cv::KeyPoint & r = *iter;
		float radius = 14*1.2/9.*2;//r.size*1.2/9.*2;
		if(graphicsViewMode_)
		{
			QString info = QString( "ID = %1\n"
									"Response = %2\n"
									"Angle = %3\n"
									"X = %4\n"
									"Y = %5\n"
									"Size = %6").arg(i+1).arg(r.response).arg(r.angle).arg(r.pt.x).arg(r.pt.y).arg(r.size);
			// YELLOW = NEW and multiple times
			item = new KeypointItem(i+1, r.pt.x-radius, r.pt.y-radius, radius*2, info, kptColors_.at(i));
			item->setVisible(this->isFeaturesShown());
			item->setZValue(1);
			graphicsView_->scene()->addItem(item);
			keypointItems_.append(item);
		}

		if(painter)
		{
			painter->save();
			painter->setPen(kptColors_.at(i));
			painter->setBrush(kptColors_.at(i));
			painter->drawEllipse(r.pt.x-radius, r.pt.y-radius, radius*2, radius*2);
			painter->restore();
		}
	}
}

QColor ObjWidget::defaultColor() const
{
	int alpha = 20*255/100;
	return QColor(255, 255, 0, alpha);
}

std::vector<cv::KeyPoint> ObjWidget::selectedKeypoints() const
{
	std::vector<cv::KeyPoint> selected;
	if(graphicsViewMode_)
	{
		QList<QGraphicsItem*> items = graphicsView_->scene()->selectedItems();
		for(int i=0; i<items.size(); ++i)
		{
			if(qgraphicsitem_cast<KeypointItem*>(items.at(i)))
			{
				selected.push_back(keypoints_.at(((KeypointItem*)items.at(i))->id()-1)); // ids start at 1
			}
		}
	}
	return selected;
}

void ObjWidget::setupGraphicsView()
{
	graphicsView_->scene()->clear();
	if(!image_.isNull())
	{
		graphicsView_->scene()->setSceneRect(image_.rect());
		QList<KeypointItem*> items;
		if(image_.width() > 0 && image_.height() > 0)
		{
			QRectF sceneRect = graphicsView_->sceneRect();

			QGraphicsPixmapItem * pixmapItem = graphicsView_->scene()->addPixmap(image_);
			pixmapItem->setVisible(this->isImageShown());
			this->drawKeypoints();

			graphicsView_->fitInView(sceneRect, Qt::KeepAspectRatio);
		}
	}
}

