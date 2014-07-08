/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef RECTITEM_H_
#define RECTITEM_H_

#include <QtGui/QGraphicsRectItem>
#include <QtGui/QGraphicsTextItem>
#include <QtGui/QPen>
#include <QtGui/QBrush>

class RectItem : public QObject, public QGraphicsRectItem
{
	Q_OBJECT;

public:
	RectItem(int id, const QRectF &rect, QGraphicsItem * parent = 0);
	virtual ~RectItem();

	void setColor(const QColor & color);
	int id() const {return id_;}

Q_SIGNALS:
	void hovered(int);

protected:
	virtual void hoverEnterEvent ( QGraphicsSceneHoverEvent * event );
	virtual void hoverLeaveEvent ( QGraphicsSceneHoverEvent * event );
	virtual void focusInEvent ( QFocusEvent * event );
	virtual void focusOutEvent ( QFocusEvent * event );

private:
	void showDescription();
	void hideDescription();

private:
	QGraphicsRectItem * placeHolder_;
	int id_;
};


#endif /* RECTITEM_H_ */
