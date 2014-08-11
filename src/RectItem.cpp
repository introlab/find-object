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

#include "RectItem.h"

#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QGraphicsScene>

namespace find_object {

RectItem::RectItem(int id, const QRectF &rect, QGraphicsItem * parent) :
	QGraphicsRectItem(rect, parent),
	placeHolder_(0),
	id_(id)
{
	this->setAcceptsHoverEvents(true);
	this->setFlag(QGraphicsItem::ItemIsFocusable, true);
	this->setFlag(QGraphicsItem::ItemIsSelectable, true);
}

RectItem::~RectItem()
{
}

void RectItem::setColor(const QColor & color)
{
	this->setPen(QPen(color));
	this->setBrush(QBrush(color));
	if(placeHolder_)
	{
		QList<QGraphicsItem *> items = placeHolder_->children();
		if(items.size())
		{
			((QGraphicsTextItem *)items.front())->setDefaultTextColor(this->pen().color().rgb());
		}
	}
}

void RectItem::showDescription()
{
	if(!placeHolder_ || !placeHolder_->isVisible())
	{
		if(!placeHolder_)
		{
			placeHolder_ = new QGraphicsRectItem();
			placeHolder_->setVisible(false);
			this->scene()->addItem(placeHolder_);
			placeHolder_->setBrush(QBrush(QColor ( 0, 0, 0, 170 ))); // Black transparent background
			QGraphicsTextItem * text = new QGraphicsTextItem(placeHolder_);
			text->setDefaultTextColor(this->pen().color().rgb());
			QTransform t = this->transform();
			QPolygonF rectH = this->mapToScene(this->rect());
			float angle = 90.0f;
			for(int a=0; a<rectH.size(); ++a)
			{
				//  Find the smaller angle
				QLineF ab(rectH.at(a).x(), rectH.at(a).y(), rectH.at((a+1)%4).x(), rectH.at((a+1)%4).y());
				QLineF cb(rectH.at((a+1)%4).x(), rectH.at((a+1)%4).y(), rectH.at((a+2)%4).x(), rectH.at((a+2)%4).y());
				float angleTmp =  ab.angle(cb);
				if(angleTmp > 90.0f)
				{
					angleTmp  = 180.0f - angleTmp;
				}
				if(angleTmp < angle)
				{
					angle = angleTmp;
				}
			}
			text->setPlainText(tr(
					"Object=%1\n"
					"Homography= [\n"
					"            %2 %3 %4\n"
					"            %5 %6 %7\n"
					"            %8 %9 %10]\n"
					"Angle=%11").arg(id_)
					.arg(t.m11()).arg(t.m12()).arg(t.m13())
					.arg(t.m21()).arg(t.m22()).arg(t.m23())
					.arg(t.m31()).arg(t.m32()).arg(t.m33())
					.arg(angle));
			placeHolder_->setRect(text->boundingRect());
		}


		QPen pen = this->pen();
		this->setPen(QPen(pen.color(), pen.width()*2));
		placeHolder_->setZValue(this->zValue()+1);
		placeHolder_->setPos(0,0);
		placeHolder_->setVisible(true);

		Q_EMIT hovered(id_);
	}
}

void RectItem::hideDescription()
{
	if(placeHolder_ && placeHolder_->isVisible())
	{
		placeHolder_->setVisible(false);
		this->setPen(QPen(pen().color(), pen().width()/2));
	}
}

void RectItem::hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
{
	this->showDescription();
	QGraphicsRectItem::hoverEnterEvent(event);
}

void RectItem::hoverLeaveEvent ( QGraphicsSceneHoverEvent * event )
{
	if(!this->hasFocus())
	{
		this->hideDescription();
	}
	QGraphicsRectItem::hoverEnterEvent(event);
}

void RectItem::focusInEvent ( QFocusEvent * event )
{
	this->showDescription();
	QGraphicsRectItem::focusInEvent(event);
}

void RectItem::focusOutEvent ( QFocusEvent * event )
{
	this->hideDescription();
	QGraphicsRectItem::focusOutEvent(event);
}

} // namespace find_object
