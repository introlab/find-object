/*
 * ImageDropWidget.h
 *
 *  Created on: Dec 22, 2014
 *      Author: mathieu
 */

#ifndef IMAGEDROPWIDGET_H_
#define IMAGEDROPWIDGET_H_

#include <QtGui/QWidget>

namespace find_object {

class ImageDropWidget : public QWidget {

	Q_OBJECT;

public:
	ImageDropWidget(QWidget *parent = 0, Qt::WindowFlags flags = 0);
	virtual ~ImageDropWidget();

Q_SIGNALS:
	void imagesReceived(const QStringList &);


protected:
	virtual void dragEnterEvent(QDragEnterEvent *event);
	virtual void dropEvent(QDropEvent *event);
};

}

#endif /* IMAGEDROPWIDGET_H_ */
