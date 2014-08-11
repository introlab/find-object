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

#ifndef OBJWIDGET_H_
#define OBJWIDGET_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include <opencv2/features2d/features2d.hpp>
#include <QtGui/QWidget>
#include <QtCore/QMultiMap>

class QAction;
class QMenu;
class QGraphicsView;
class QGraphicsScene;
class QGraphicsRectItem;
class QGraphicsItem;
class QLabel;

namespace find_object {

class KeypointItem;
class ImageKptsView;

class FINDOBJECT_EXP ObjWidget : public QWidget
{
	Q_OBJECT

public:
	ObjWidget(QWidget * parent = 0);
	ObjWidget(int id, const std::vector<cv::KeyPoint> & keypoints, const QImage & image, QWidget * parent = 0);
	virtual ~ObjWidget();

	void setId(int id);
	void setData(const std::vector<cv::KeyPoint> & keypoints, const QImage & image);
	void setTextLabel(const QString & text);
	void resetKptsColor();
	void setKptColor(int index, const QColor & color);
	void setGraphicsViewMode(bool on);
	void setAutoScale(bool autoScale);
	void setSizedFeatures(bool on);
	void setMirrorView(bool on);
	void setAlpha(int alpha);
	void setDeletable(bool deletable);
	void setImageShown(bool shown);
	void setFeaturesShown(bool shown);
	void addRect(QGraphicsRectItem * rect);
	void clearRoiSelection() {mousePressedPos_ = mouseCurrentPos_ = QPoint();update();}

	int id() const {return id_;}
	const QColor & color() const {return color_;}
	const std::vector<cv::KeyPoint> keypoints() const {return keypoints_;}
	const QPixmap & pixmap() const {return pixmap_;}
	QColor defaultColor() const;
	bool isImageShown() const;
	bool isFeaturesShown() const;
	bool isSizedFeatures() const;
	bool isMirrorView() const;
	//QGraphicsScene * scene() const;
	std::vector<cv::KeyPoint> selectedKeypoints() const;
	QList<QGraphicsItem*> selectedItems() const;

	QPixmap getSceneAsPixmap();

protected:
	virtual void paintEvent(QPaintEvent *event);
	virtual void contextMenuEvent(QContextMenuEvent * event);
	virtual void resizeEvent(QResizeEvent* event);
	virtual void mousePressEvent(QMouseEvent * event);
	virtual void mouseMoveEvent(QMouseEvent * event);
	virtual void mouseReleaseEvent(QMouseEvent * event);

Q_SIGNALS:
	void removalTriggered(find_object::ObjWidget *);
	void selectionChanged();
	void roiChanged(const cv::Rect &);

private:
	void setupGraphicsView();
	void drawKeypoints(QPainter * painter = 0);
	void setupUi();
	void updateItemsShown();
	void computeScaleOffsets(float & scale, float & offsetX, float & offsetY);

private:
	int id_;
	std::vector<cv::KeyPoint> keypoints_;
	QPixmap pixmap_;
	QList<KeypointItem*> keypointItems_;
	QGraphicsView * graphicsView_;
	QVector<QColor> kptColors_;
	QList<QGraphicsRectItem*> rectItems_;
	bool graphicsViewInitialized_;
	int alpha_;
	QLabel * label_;
	QColor color_;

	// menu stuff
	QString savedFileName_;
	QMenu * menu_;
	QAction * showImage_;
	QAction * showFeatures_;
	QAction * saveImage_;
	QAction * mirrorView_;
	QAction * delete_;
	QAction * graphicsViewMode_;
	QAction * autoScale_;
	QAction * sizedFeatures_;
	QAction * setAlpha_;
	QAction * setColor_;

	// selection stuff
	QPoint mousePressedPos_;
	QPoint mouseCurrentPos_;
};

} // namespace find_object

#endif /* OBJWIDGET_H_ */
