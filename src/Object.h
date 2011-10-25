/*
 * VisualObject.h
 *
 *  Created on: 2011-10-21
 *      Author: matlab
 */

#ifndef OBJECT_H_
#define OBJECT_H_

#include <opencv2/features2d/features2d.hpp>
#include <QtGui/QWidget>

class KeypointItem;
class ImageKptsView;
class QAction;
class QMenu;
class QGraphicsView;
class QGraphicsScene;

class Object : public QWidget
{
	Q_OBJECT

public:
	Object(QWidget * parent = 0);
	Object(int id,
			const std::vector<cv::KeyPoint> & keypoints,
			const cv::Mat & descriptors,
			const IplImage * image,
			const QString & detectorType = "NA",
			const QString & descriptorType = "NA",
			QWidget * parent = 0);
	virtual ~Object();

	void setId(int id);
	void setData(const std::vector<cv::KeyPoint> & keypoints,
			const cv::Mat & descriptors,
			const IplImage * image);
	void resetKptsColor();
	void setKptColor(unsigned int index, const QColor & color);
	void setGraphicsViewMode(bool on);
	void setDeletable(bool deletable);

	const std::vector<cv::KeyPoint> & keypoints() const {return keypoints_;}
	const cv::Mat & descriptors() const {return descriptors_;}
	const QPixmap & image() const {return image_;}
	const IplImage * iplImage() const {return iplImage_;}
	int id() const {return id_;}
	QColor defaultColor() const;
	bool isImageShown() const;
	bool isFeaturesShown() const;
	bool isMirrorView() const;
	QGraphicsScene * scene() const;
	std::vector<cv::KeyPoint> selectedKeypoints() const;
	const QString & detectorType() const {return detectorType_;}
	const QString & descriptorType() const {return descriptorType_;}

	QPixmap getSceneAsPixmap();

	void save(QDataStream & streamPtr) const;
	void load(QDataStream & streamPtr);

protected:
	virtual void paintEvent(QPaintEvent *event);
	virtual void contextMenuEvent(QContextMenuEvent * event);
	virtual void resizeEvent(QResizeEvent* event);

signals:
	void removalTriggered(Object *);

private:
	void setupGraphicsView();
	void drawKeypoints(QPainter * painter = 0);
	void setupUi();
	void updateItemsShown();

private:
	std::vector<cv::KeyPoint> keypoints_;
	cv::Mat descriptors_;
	QPixmap image_;
	IplImage * iplImage_;
	QList<KeypointItem*> keypointItems_;
	QGraphicsView * graphicsView_;
	int id_;
	bool graphicsViewMode_;
	QVector<QColor> kptColors_;
	QString detectorType_;
	QString descriptorType_;

	// menu stuff
	QString _savedFileName;
	QMenu * _menu;
	QAction * _showImage;
	QAction * _showFeatures;
	QAction * _saveImage;
	QAction * _mirrorView;
	QAction * _delete;
	QAction * _plainView;
};


#endif /* OBJECT_H_ */
