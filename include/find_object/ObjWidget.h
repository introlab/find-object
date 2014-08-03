/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef OBJWIDGET_H_
#define OBJWIDGET_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include <opencv2/features2d/features2d.hpp>
#include <QtGui/QWidget>
#include <QtCore/QMultiMap>

class KeypointItem;
class ImageKptsView;
class QAction;
class QMenu;
class QGraphicsView;
class QGraphicsScene;
class QGraphicsRectItem;
class QGraphicsItem;
class QLabel;

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
	void removalTriggered(ObjWidget *);
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


#endif /* OBJWIDGET_H_ */
