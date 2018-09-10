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

#include "find_object/Settings.h"
#include "find_object/utilite/ULogger.h"
#include "find_object/ObjWidget.h"
#include "find_object/QtOpenCV.h"

#include "KeypointItem.h"

#include <opencv2/highgui/highgui.hpp>

#include <QWidget>
#include <QContextMenuEvent>
#include <QMenu>
#include <QFileDialog>
#include <QAction>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QVBoxLayout>
#include <QGraphicsRectItem>
#include <QInputDialog>
#include <QPen>
#include <QLabel>
#include <QColorDialog>

#include <QtCore/QDir>

#include <stdio.h>

namespace find_object {

ObjWidget::ObjWidget(QWidget * parent) :
	QWidget(parent),
	id_(0),
	graphicsView_(0),
	graphicsViewInitialized_(false),
	alpha_(100),
	color_(Qt::red)
{
	setupUi();
}
ObjWidget::ObjWidget(int id, const std::vector<cv::KeyPoint> & keypoints, const QMultiMap<int,int> & words, const QImage & image, QWidget * parent) :
	QWidget(parent),
	id_(id),
	graphicsView_(0),
	graphicsViewInitialized_(false),
	alpha_(100),
	color_(QColor((Qt::GlobalColor)((id % 10 + 7)==Qt::yellow?Qt::darkYellow:(id % 10 + 7))))
{
	setupUi();
	this->updateImage(image);
	this->updateData(keypoints, words);
}
ObjWidget::~ObjWidget()
{
}

void ObjWidget::setupUi()
{
	graphicsView_ = new QGraphicsView(this);
	graphicsView_->setVisible(false);
	graphicsView_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	graphicsView_->setScene(new QGraphicsScene(graphicsView_));

	label_ = new QLabel();
	label_->setAlignment(Qt::AlignCenter);
	label_->setTextInteractionFlags(Qt::TextSelectableByMouse);

	this->setLayout(new QVBoxLayout(this));
	this->layout()->addWidget(graphicsView_);
	this->layout()->addWidget(label_);
	this->layout()->setContentsMargins(0,0,0,0);

	menu_ = new QMenu(tr(""), this);
	showImage_ = menu_->addAction(tr("Show image"));
	showImage_->setCheckable(true);
	showImage_->setChecked(true);
	showFeatures_ = menu_->addAction(tr("Show features"));
	showFeatures_->setCheckable(true);
	showFeatures_->setChecked(true);
	mirrorView_ = menu_->addAction(tr("Mirror view"));
	mirrorView_->setCheckable(true);
	mirrorView_->setChecked(false);
	graphicsViewMode_ = menu_->addAction(tr("Graphics view"));
	graphicsViewMode_->setCheckable(true);
	graphicsViewMode_->setChecked(false);
	autoScale_ = menu_->addAction(tr("Scale view"));
	autoScale_->setCheckable(true);
	autoScale_->setChecked(true);
	autoScale_->setEnabled(false);
	sizedFeatures_ = menu_->addAction(tr("Sized features"));
	sizedFeatures_->setCheckable(true);
	sizedFeatures_->setChecked(false);
	menu_->addSeparator();
	setColor_ = menu_->addAction(tr("Set color..."));
	setAlpha_ = menu_->addAction(tr("Set alpha..."));
	menu_->addSeparator();
	saveImage_ = menu_->addAction(tr("Save picture..."));
	menu_->addSeparator();
	delete_ = menu_->addAction(tr("Delete"));
	delete_->setEnabled(false);

	this->setId(id_);

	graphicsView_->setRubberBandSelectionMode(Qt::ContainsItemShape);
	graphicsView_->setDragMode(QGraphicsView::RubberBandDrag);

	connect(graphicsView_->scene(), SIGNAL(selectionChanged()), this, SIGNAL(selectionChanged()));
}

void ObjWidget::setId(int id)
{
	color_ = QColor((Qt::GlobalColor)((id % 10 + 7)==Qt::yellow?Qt::darkYellow:(id % 10 + 7)));
	id_=id;
	if(id_)
	{
		savedFileName_ = QString("object_%1.png").arg(id_);
	}
}

void ObjWidget::setGraphicsViewMode(bool on)
{
	graphicsViewMode_->setChecked(on);
	graphicsView_->setVisible(on && graphicsView_->scene()->items().size());
	autoScale_->setEnabled(on);
	//update items' color
	if(on)
	{
		if(!graphicsViewInitialized_)
		{
			this->setupGraphicsView();
		}
		else
		{
			for(int i=0; i<keypointItems_.size(); ++i)
			{
				QColor color = kptColors_.at(i);
				color.setAlpha(alpha_);
				keypointItems_[i]->setColor(color);
			}
		}
	}
	if(autoScale_->isChecked())
	{
		graphicsView_->fitInView(graphicsView_->sceneRect(), Qt::KeepAspectRatio);
	}
	else
	{
		graphicsView_->resetTransform();
		graphicsView_->setTransform(QTransform().scale(this->isMirrorView()?-1.0:1.0, 1.0));
	}
}

void ObjWidget::setAutoScale(bool autoScale)
{
	autoScale_->setChecked(autoScale);
	if(graphicsViewMode_)
	{
		if(autoScale)
		{
			graphicsView_->fitInView(graphicsView_->sceneRect(), Qt::KeepAspectRatio);
		}
		else
		{
			graphicsView_->resetTransform();
			graphicsView_->setTransform(QTransform().scale(this->isMirrorView()?-1.0:1.0, 1.0));
		}
	}
}

void ObjWidget::setSizedFeatures(bool on)
{
	sizedFeatures_->setChecked(on);
	if(graphicsViewInitialized_)
	{
		for(unsigned int i=0; i<(unsigned int)keypointItems_.size() && i<keypoints_.size(); ++i)
		{
			float size = 14;
			if(on && keypoints_[i].size>14.0f)
			{
				size = keypoints_[i].size;
			}
			float radius = size*1.2f/9.0f*2.0f;
			keypointItems_.at(i)->setRect(keypoints_[i].pt.x-radius, keypoints_[i].pt.y-radius, radius*2, radius*2);
		}
	}
	if(!graphicsViewMode_->isChecked())
	{
		this->update();
	}
}

void ObjWidget::setMirrorView(bool on)
{
	mirrorView_->setChecked(on);
	graphicsView_->setTransform(QTransform().scale(this->isMirrorView()?-1.0:1.0, 1.0));
	if(graphicsViewMode_->isChecked() && autoScale_->isChecked())
	{
		graphicsView_->fitInView(graphicsView_->sceneRect(), Qt::KeepAspectRatio);
	}
	else if(!graphicsViewMode_->isChecked())
	{
		this->update();
	}
}

void ObjWidget::setAlpha(int alpha)
{
	if(alpha>=0 && alpha<=255)
	{
		alpha_ = alpha;
		if(graphicsViewInitialized_)
		{
			for(int i=0; i<keypointItems_.size() && i<kptColors_.size(); ++i)
			{
				QColor color = kptColors_.at(i);
				color.setAlpha(alpha_);
				keypointItems_.at(i)->setColor(color);
			}
		}
		for(int i=0; i<rectItems_.size(); ++i)
		{
			QPen pen = rectItems_.at(i)->pen();
			QColor color = pen.color();
			color.setAlpha(alpha_);
			pen.setColor(color);
			rectItems_.at(i)->setPen(pen);
		}

		if(!graphicsViewMode_->isChecked())
		{
			this->update();
		}
	}
}

void ObjWidget::setTextLabel(const QString & text)
{
	label_->setText(text);
}

void ObjWidget::updateImage(const QImage & image)
{
	pixmap_ = QPixmap();
	rect_ = QRect();
	if(!image.isNull())
	{
		pixmap_ = QPixmap::fromImage(image);
		rect_ = pixmap_.rect();
	}
	label_->setVisible(image.isNull());
}
void ObjWidget::updateData(const std::vector<cv::KeyPoint> & keypoints, const QMultiMap<int, int> & words)
{
	keypoints_ = keypoints;
	kptColors_ = QVector<QColor>((int)keypoints.size(), defaultColor(0));
	keypointItems_.clear();
	rectItems_.clear();
	this->updateWords(words);
	graphicsView_->scene()->clear();
	graphicsViewInitialized_ = false;
	mouseCurrentPos_ = mousePressedPos_; // this will reset roi selection

	//this->setMinimumSize(image_.size());

	if(graphicsViewMode_->isChecked())
	{
		this->setupGraphicsView();
	}
	else
	{
		this->update();
	}
}

void ObjWidget::updateWords(const QMultiMap<int,int> & words)
{
	words_.clear();
	for(QMultiMap<int,int>::const_iterator iter=words.begin(); iter!=words.end(); ++iter)
	{
		words_.insert(iter.value(), iter.key());
	}
	for(int i=0; i<kptColors_.size(); ++i)
	{
		kptColors_[i] = defaultColor(words_.size()?words_.value(i,-1):0);
		if(keypointItems_.size() == kptColors_.size())
		{
			keypointItems_[i]->setWordID(words_.value(i,-1));
			keypointItems_[i]->setColor(defaultColor(words_.size()?keypointItems_[i]->wordID():0));
		}
	}
}

void ObjWidget::resetKptsColor()
{
	for(int i=0; i<kptColors_.size(); ++i)
	{
		if(keypointItems_.size() == kptColors_.size())
		{
			kptColors_[i] = defaultColor(keypointItems_[i]->wordID());
			keypointItems_[i]->setColor(this->defaultColor(keypointItems_[i]->wordID()));
		}
		else
		{
			kptColors_[i] = defaultColor(words_.value(i,-1));
		}
	}
	qDeleteAll(rectItems_.begin(), rectItems_.end());
	rectItems_.clear();
}

void ObjWidget::resetKptsWordID()
{
	words_.clear();
	for(int i=0; i<keypointItems_.size(); ++i)
	{
		keypointItems_[i]->setWordID(-1);
	}
}

void ObjWidget::setKptColor(int index, const QColor & color)
{
	if(index < kptColors_.size())
	{
		kptColors_[index] = color;
	}
	else
	{
		UWARN("PROBLEM index =%d > size=%d\n", index, kptColors_.size());
	}

	if(graphicsViewMode_->isChecked())
	{
		if(index < keypointItems_.size())
		{
			QColor c = color;
			c.setAlpha(alpha_);
			keypointItems_.at(index)->setColor(c);
		}
	}
}

void ObjWidget::setKptWordID(int index, int wordID)
{
	words_.insert(index, wordID);
	if(index < keypointItems_.size())
	{
		keypointItems_.at(index)->setWordID(wordID);
	}
}

void ObjWidget::addRect(QGraphicsRectItem * rect)
{
	if(graphicsViewInitialized_)
	{
		graphicsView_->scene()->addItem(rect);
	}
	rect->setZValue(1);
	QPen pen = rect->pen();
	QColor color = pen.color();
	color.setAlpha(alpha_);
	pen.setColor(color);
	rect->setPen(pen);
	rectItems_.append(rect);
}

QList<QGraphicsItem*> ObjWidget::selectedItems() const
{
	return graphicsView_->scene()->selectedItems();
}

bool ObjWidget::isImageShown() const
{
	return showImage_->isChecked();
}

bool ObjWidget::isFeaturesShown() const
{
	return showFeatures_->isChecked();
}

bool ObjWidget::isSizedFeatures() const
{
	return sizedFeatures_->isChecked();
}

bool ObjWidget::isMirrorView() const
{
	return mirrorView_->isChecked();
}

void ObjWidget::setDeletable(bool deletable)
{
	delete_->setEnabled(deletable);
}

void ObjWidget::setImageShown(bool shown)
{
	showImage_->setChecked(shown);
	if(graphicsViewMode_->isChecked())
	{
		this->updateItemsShown();
	}
	else
	{
		this->update();
	}
}

void ObjWidget::setFeaturesShown(bool shown)
{
	showFeatures_->setChecked(shown);
	if(graphicsViewMode_->isChecked())
	{
		this->updateItemsShown();
	}
	else
	{
		this->update();
	}
}

void ObjWidget::computeScaleOffsets(float & scale, float & offsetX, float & offsetY)
{
	scale = 1.0f;
	offsetX = 0.0f;
	offsetY = 0.0f;

	if(!rect_.isNull())
	{
		float w = rect_.width();
		float h = rect_.height();
		float widthRatio = float(this->rect().width()) / w;
		float heightRatio = float(this->rect().height()) / h;

		//printf("w=%f, h=%f, wR=%f, hR=%f, sW=%d, sH=%d\n", w, h, widthRatio, heightRatio, this->rect().width(), this->rect().height());
		if(widthRatio < heightRatio)
		{
			scale = widthRatio;
		}
		else
		{
			scale = heightRatio;
		}

		//printf("ratio=%f\n",ratio);

		w *= scale;
		h *= scale;

		if(w < this->rect().width())
		{
			offsetX = (this->rect().width() - w)/2.0f;
		}
		if(h < this->rect().height())
		{
			offsetY = (this->rect().height() - h)/2.0f;
		}
		//printf("offsetX=%f, offsetY=%f\n",offsetX, offsetY);
	}
}

void ObjWidget::paintEvent(QPaintEvent *event)
{
	if(graphicsViewMode_->isChecked())
	{
		QWidget::paintEvent(event);
	}
	else
	{
		if(!rect_.isNull())
		{
			//Scale
			float ratio, offsetX, offsetY;
			this->computeScaleOffsets(ratio, offsetX, offsetY);
			QPainter painter(this);

			if(mirrorView_->isChecked())
			{
				painter.translate(offsetX+rect_.width()*ratio, offsetY);
				painter.scale(-ratio, ratio);
			}
			else
			{
				painter.translate(offsetX, offsetY);
				painter.scale(ratio, ratio);
			}

			if(!pixmap_.isNull() && showImage_->isChecked())
			{
				painter.drawPixmap(QPoint(0,0), pixmap_);
			}

			if(showFeatures_->isChecked())
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

			if(mouseCurrentPos_ != mousePressedPos_)
			{
				painter.save();
				int left, top, right, bottom;
				left = mousePressedPos_.x() < mouseCurrentPos_.x() ? mousePressedPos_.x():mouseCurrentPos_.x();
				top = mousePressedPos_.y() < mouseCurrentPos_.y() ? mousePressedPos_.y():mouseCurrentPos_.y();
				right = mousePressedPos_.x() > mouseCurrentPos_.x() ? mousePressedPos_.x():mouseCurrentPos_.x();
				bottom = mousePressedPos_.y() > mouseCurrentPos_.y() ? mousePressedPos_.y():mouseCurrentPos_.y();
				if(mirrorView_->isChecked())
				{
					int l = left;
					left = qAbs(right - rect_.width());
					right = qAbs(l - rect_.width());
				}
				painter.setPen(Qt::NoPen);
				painter.setBrush(QBrush(QColor(0,0,0,100)));
				painter.drawRect(0, 0, rect_.width(), top);
				painter.drawRect(0, top, left, bottom-top);
				painter.drawRect(right, top, rect_.width()-right, bottom-top);
				painter.drawRect(0, bottom, rect_.width(), rect_.height()-bottom);
				painter.restore();
			}
		}
	}
}

void ObjWidget::resizeEvent(QResizeEvent* event)
{
	QWidget::resizeEvent(event);
	if(graphicsViewMode_->isChecked() && autoScale_->isChecked())
	{
		graphicsView_->fitInView(graphicsView_->sceneRect(), Qt::KeepAspectRatio);
	}
}

void ObjWidget::mousePressEvent(QMouseEvent * event)
{
	float scale, offsetX, offsetY;
	this->computeScaleOffsets(scale, offsetX, offsetY);
	mousePressedPos_.setX((event->pos().x()-offsetX)/scale);
	mousePressedPos_.setY((event->pos().y()-offsetY)/scale);
	mouseCurrentPos_ = mousePressedPos_;
	this->update();
	QWidget::mousePressEvent(event);
}

void ObjWidget::mouseMoveEvent(QMouseEvent * event)
{
	float scale, offsetX, offsetY;
	this->computeScaleOffsets(scale, offsetX, offsetY);
	mouseCurrentPos_.setX((event->pos().x()-offsetX)/scale);
	mouseCurrentPos_.setY((event->pos().y()-offsetY)/scale);
	this->update();
	QWidget::mouseMoveEvent(event);
}

void ObjWidget::mouseReleaseEvent(QMouseEvent * event)
{
	if(!rect_.isNull())
	{
		int left,top,bottom,right;

		left = mousePressedPos_.x() < mouseCurrentPos_.x() ? mousePressedPos_.x():mouseCurrentPos_.x();
		top = mousePressedPos_.y() < mouseCurrentPos_.y() ? mousePressedPos_.y():mouseCurrentPos_.y();
		right = mousePressedPos_.x() > mouseCurrentPos_.x() ? mousePressedPos_.x():mouseCurrentPos_.x();
		bottom = mousePressedPos_.y() > mouseCurrentPos_.y() ? mousePressedPos_.y():mouseCurrentPos_.y();

		if(mirrorView_->isChecked())
		{
			int l = left;
			left = qAbs(right - rect_.width());
			right = qAbs(l - rect_.width());
		}

		Q_EMIT roiChanged(cv::Rect(left, top, right-left, bottom-top));
	}
	QWidget::mouseReleaseEvent(event);
}

void ObjWidget::contextMenuEvent(QContextMenuEvent * event)
{
	QAction * action = menu_->exec(event->globalPos());
	if(action == saveImage_)
	{
		QString text;
		if(savedFileName_.isEmpty())
		{
			savedFileName_=Settings::workingDirectory()+"/figure.png";
		}
		text = QFileDialog::getSaveFileName(this, tr("Save figure to ..."), savedFileName_, "*.png *.xpm *.jpg *.pdf");
		if(!text.isEmpty())
		{
			if(!text.endsWith(".png") && !text.endsWith(".xpm") && !text.endsWith(".jpg") && !text.endsWith(".pdf"))
			{
				text.append(".png");//default
			}
			savedFileName_ = text;
			getSceneAsPixmap().save(text);
		}
	}
	else if(action == showFeatures_ || action == showImage_)
	{
		if(graphicsViewMode_->isChecked())
		{
			this->updateItemsShown();
		}
		else
		{
			this->update();
		}
	}
	else if(action == mirrorView_)
	{
		this->setMirrorView(mirrorView_->isChecked());
	}
	else if(action == delete_)
	{
		Q_EMIT removalTriggered(this);
	}
	else if(action == graphicsViewMode_)
	{
		this->setGraphicsViewMode(graphicsViewMode_->isChecked());
	}
	else if(action == autoScale_)
	{
		this->setAutoScale(autoScale_->isChecked());
	}
	else if(action == sizedFeatures_)
	{
		this->setSizedFeatures(sizedFeatures_->isChecked());
	}
	else if(action == setColor_)
	{
		QColor color = QColorDialog::getColor(color_, this);
		if(color.isValid())
		{
			for(int i=0; i<kptColors_.size(); ++i)
			{
				if(kptColors_[i] == color_)
				{
					kptColors_[i] = color;
					if(graphicsViewMode_->isChecked())
					{
						keypointItems_[i]->setColor(color);
					}
				}
			}
			for(int i=0; i<rectItems_.size(); ++i)
			{
				if(rectItems_[i]->pen().color() == color_)
				{
					QPen p = rectItems_[i]->pen();
					p.setColor(color);
					rectItems_[i]->setPen(p);
				}
			}
			color_ = color;

		}
	}
	else if(action == setAlpha_)
	{
		bool ok;
		int newAlpha = QInputDialog::getInt(this, tr("Set alpha"), tr("Alpha:"), alpha_, 0, 255, 5, &ok);
		if(ok)
		{
			this->setAlpha(newAlpha);
		}
	}
}

QPixmap ObjWidget::getSceneAsPixmap()
{
	if(graphicsViewMode_->isChecked())
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
			items.at(i)->setVisible(showFeatures_->isChecked());
		}
		else if(qgraphicsitem_cast<QGraphicsPixmapItem*>(items.at(i)))
		{
			items.at(i)->setVisible(showImage_->isChecked());
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
		float size = 14;
		if(r.size>14.0f && sizedFeatures_->isChecked())
		{
			size = r.size;
		}
		float radius = size*1.2f/9.0f*2.0f;
		QColor color(kptColors_.at(i).red(), kptColors_.at(i).green(), kptColors_.at(i).blue(), alpha_);
		if(graphicsViewMode_->isChecked())
		{
			// YELLOW = NEW and multiple times
			item = new KeypointItem(i, r.pt.x-radius, r.pt.y-radius, radius*2, r, words_.value(i, -1), color);
			item->setVisible(this->isFeaturesShown());
			item->setZValue(2);
			graphicsView_->scene()->addItem(item);
			item->setColor(defaultColor(item->wordID()));
			kptColors_[i] = defaultColor(item->wordID());
			keypointItems_.append(item);
		}

		if(painter)
		{
			painter->save();
			painter->setPen(color);
			painter->setBrush(color);
			painter->drawEllipse(r.pt.x-radius, r.pt.y-radius, radius*2, radius*2);
			painter->restore();
		}
	}
}

QColor ObjWidget::defaultColor(int id) const
{
	QColor color(id >= 0 ? Qt::yellow : Qt::white);
	color.setAlpha(alpha_);
	return color;
}

std::vector<cv::KeyPoint> ObjWidget::selectedKeypoints() const
{
	std::vector<cv::KeyPoint> selected;
	if(graphicsViewMode_->isChecked())
	{
		QList<QGraphicsItem*> items = graphicsView_->scene()->selectedItems();
		for(int i=0; i<items.size(); ++i)
		{
			if(qgraphicsitem_cast<KeypointItem*>(items.at(i)))
			{
				selected.push_back(keypoints_.at(((KeypointItem*)items.at(i))->id()));
			}
		}
	}
	return selected;
}

void ObjWidget::setupGraphicsView()
{
	if(!rect_.isNull())
	{
		graphicsView_->setVisible(true);
		graphicsView_->scene()->setSceneRect(rect_);
		QList<KeypointItem*> items;

		QRectF sceneRect = graphicsView_->sceneRect();

		QGraphicsPixmapItem * pixmapItem = graphicsView_->scene()->addPixmap(pixmap_);
		pixmapItem->setVisible(this->isImageShown());
		this->drawKeypoints();

		for(int i=0; i<rectItems_.size(); ++i)
		{
			graphicsView_->scene()->addItem(rectItems_.at(i));
		}

		if(autoScale_->isChecked())
		{
			graphicsView_->fitInView(sceneRect, Qt::KeepAspectRatio);
		}
		graphicsViewInitialized_ = true;
	}
	else
	{
		graphicsView_->setVisible(false);
	}
}

} // namespace find_object

