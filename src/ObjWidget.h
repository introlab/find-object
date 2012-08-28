/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef OBJWIDGET_H_
#define OBJWIDGET_H_

#include <opencv2/features2d/features2d.hpp>
#include <QtGui/QWidget>

class KeypointItem;
class ImageKptsView;
class QAction;
class QMenu;
class QGraphicsView;
class QGraphicsScene;
class QGraphicsRectItem;
class QGraphicsItem;
class QLabel;

class ObjWidget : public QWidget
{
	Q_OBJECT

public:
	ObjWidget(QWidget * parent = 0);
	ObjWidget(int id,
			const std::vector<cv::KeyPoint> & keypoints,
			const cv::Mat & descriptors,
			const cv::Mat & image,
			const QString & detectorType = "NA",
			const QString & descriptorType = "NA",
			QWidget * parent = 0);
	virtual ~ObjWidget();

	void setId(int id);
	void setData(const std::vector<cv::KeyPoint> & keypoints,
			const cv::Mat & descriptors,
			const cv::Mat & image,
			const QString & detectorType,
			const QString & descriptorType);
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

	const std::vector<cv::KeyPoint> & keypoints() const {return keypoints_;}
	const cv::Mat & descriptors() const {return descriptors_;}
	const QPixmap & pixmap() const {return pixmap_;}
	const cv::Mat & cvImage() const {return cvImage_;}
	int id() const {return id_;}
	QColor defaultColor() const;
	bool isImageShown() const;
	bool isFeaturesShown() const;
	bool isSizedFeatures() const;
	bool isMirrorView() const;
	//QGraphicsScene * scene() const;
	std::vector<cv::KeyPoint> selectedKeypoints() const;
	const QString & detectorType() const {return detectorType_;}
	const QString & descriptorType() const {return descriptorType_;}
	QList<QGraphicsItem*> selectedItems() const;

	QPixmap getSceneAsPixmap();

	void save(QDataStream & streamPtr) const;
	void load(QDataStream & streamPtr);

protected:
	virtual void paintEvent(QPaintEvent *event);
	virtual void contextMenuEvent(QContextMenuEvent * event);
	virtual void resizeEvent(QResizeEvent* event);
	virtual void mousePressEvent(QMouseEvent * event);
	virtual void mouseMoveEvent(QMouseEvent * event);
	virtual void mouseReleaseEvent(QMouseEvent * event);

signals:
	void removalTriggered(ObjWidget *);
	void selectionChanged();
	void roiChanged(const QRect &);

private:
	void setupGraphicsView();
	void drawKeypoints(QPainter * painter = 0);
	void setupUi();
	void updateItemsShown();
	void computeScaleOffsets(float & scale, float & offsetX, float & offsetY);

private:
	std::vector<cv::KeyPoint> keypoints_;
	cv::Mat descriptors_;
	QPixmap pixmap_;
	cv::Mat cvImage_;
	QList<KeypointItem*> keypointItems_;
	QGraphicsView * graphicsView_;
	int id_;
	QVector<QColor> kptColors_;
	QString detectorType_;
	QString descriptorType_;
	QList<QGraphicsRectItem*> rectItems_;
	bool graphicsViewInitialized_;
	int alpha_;
	QLabel * label_;

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

	// selection stuff
	QPoint mousePressedPos_;
	QPoint mouseCurrentPos_;
};


#endif /* OBJWIDGET_H_ */
