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

#include "KeypointItem.h"

#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QGraphicsScene>

namespace find_object {

KeypointItem::KeypointItem(int id, qreal x, qreal y, int r, const QString & info, const QColor & color, QGraphicsItem * parent) :
	QGraphicsEllipseItem(x, y, r, r, parent),
	info_(info),
	placeHolder_(0),
	id_(id)
{
	this->setPen(QPen(color));
	this->setBrush(QBrush(color));
	this->setAcceptsHoverEvents(true);
	this->setFlag(QGraphicsItem::ItemIsFocusable, true);
	this->setFlag(QGraphicsItem::ItemIsSelectable, true);
}

KeypointItem::~KeypointItem()
{
}

void KeypointItem::setColor(const QColor & color)
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

void KeypointItem::showDescription()
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
			text->setPlainText(info_);
			placeHolder_->setRect(text->boundingRect());
		}


		QPen pen = this->pen();
		this->setPen(QPen(pen.color(), pen.width()+2));
		placeHolder_->setZValue(this->zValue()+1);
		placeHolder_->setPos(this->mapToScene(0,0));
		placeHolder_->setVisible(true);
	}
}

void KeypointItem::hideDescription()
{
	if(placeHolder_ && placeHolder_->isVisible())
	{
		placeHolder_->setVisible(false);
		this->setPen(QPen(pen().color(), pen().width()-2));
	}
}

void KeypointItem::hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
{
	this->showDescription();
	QGraphicsEllipseItem::hoverEnterEvent(event);
}

void KeypointItem::hoverLeaveEvent ( QGraphicsSceneHoverEvent * event )
{
	if(!this->hasFocus())
	{
		this->hideDescription();
	}
	QGraphicsEllipseItem::hoverEnterEvent(event);
}

void KeypointItem::focusInEvent ( QFocusEvent * event )
{
	this->showDescription();
	QGraphicsEllipseItem::focusInEvent(event);
}

void KeypointItem::focusOutEvent ( QFocusEvent * event )
{
	this->hideDescription();
	QGraphicsEllipseItem::focusOutEvent(event);
}

} // namespace find_object
