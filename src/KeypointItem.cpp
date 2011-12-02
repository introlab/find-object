/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "KeypointItem.h"

#include <QtGui/QPen>
#include <QtGui/QBrush>
#include <QtGui/QGraphicsScene>

KeypointItem::KeypointItem(int id, qreal x, qreal y, int r, const QString & info, const QColor & color, QGraphicsItem * parent) :
	QGraphicsEllipseItem(x, y, r, r, parent),
	_info(info),
	_placeHolder(0),
	_id(id)
{
	this->setPen(QPen(color));
	this->setBrush(QBrush(color));
	this->setAcceptsHoverEvents(true);
	this->setFlag(QGraphicsItem::ItemIsFocusable, true);
	this->setFlag(QGraphicsItem::ItemIsSelectable, true);
	_width = pen().width();
}

KeypointItem::~KeypointItem()
{
	/*if(_placeHolder)
	{
		delete _placeHolder;
	}*/
}

void KeypointItem::setColor(const QColor & color)
{
	this->setPen(QPen(color));
	this->setBrush(QBrush(color));
	if(_placeHolder)
	{
		QList<QGraphicsItem *> items = _placeHolder->children();
		if(items.size())
		{
			((QGraphicsTextItem *)items.front())->setDefaultTextColor(this->pen().color().rgb());
		}
	}
}

void KeypointItem::showDescription()
{
	if(!_placeHolder)
	{
		_placeHolder = new QGraphicsRectItem();
		_placeHolder->setVisible(false);
		this->scene()->addItem(_placeHolder);
		_placeHolder->setBrush(QBrush(QColor ( 0, 0, 0, 170 ))); // Black transparent background
		QGraphicsTextItem * text = new QGraphicsTextItem(_placeHolder);
		text->setDefaultTextColor(this->pen().color().rgb());
		text->setPlainText(_info);
		_placeHolder->setRect(text->boundingRect());
	}


	QPen pen = this->pen();
	this->setPen(QPen(pen.color(), _width+2));
	_placeHolder->setZValue(this->zValue()+1);
	_placeHolder->setPos(this->mapToScene(0,0));
	_placeHolder->setVisible(true);
}

void KeypointItem::hideDescription()
{
	if(_placeHolder)
	{
		_placeHolder->setVisible(false);
	}
	this->setPen(QPen(pen().color(), _width));
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
		this->setPen(QPen(pen().color(), _width+2));
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
