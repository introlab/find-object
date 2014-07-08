/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "RectItem.h"

#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QGraphicsScene>

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
			text->setPlainText(tr("Object=%1").arg(id_));
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
