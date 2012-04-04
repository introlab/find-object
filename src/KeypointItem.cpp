/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "KeypointItem.h"

#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QGraphicsScene>

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
	width_ = pen().width();
}

KeypointItem::~KeypointItem()
{
	/*if(placeHolder_)
	{
		delete placeHolder_;
	}*/
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
	this->setPen(QPen(pen.color(), width_+2));
	placeHolder_->setZValue(this->zValue()+1);
	placeHolder_->setPos(this->mapToScene(0,0));
	placeHolder_->setVisible(true);
}

void KeypointItem::hideDescription()
{
	if(placeHolder_)
	{
		placeHolder_->setVisible(false);
	}
	this->setPen(QPen(pen().color(), width_));
}

void KeypointItem::hoverEnterEvent ( QGraphicsSceneHoverEvent * event )
{
	QGraphicsScene * scene = this->scene();
	if(scene && scene->focusItem() == 0)
	{
		this->showDescription();
	}
	else
	{
		this->setPen(QPen(pen().color(), width_+2));
	}
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
