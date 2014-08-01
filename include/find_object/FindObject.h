/*
 * FindObject.h
 *
 *  Created on: 2014-07-30
 *      Author: mathieu
 */

#ifndef FINDOBJECT_H_
#define FINDOBJECT_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include <QtCore/QObject>
#include <QtCore/QString>
#include <QtCore/QMap>
#include <QtCore/QMultiMap>
#include <QtCore/QPair>
#include <QtCore/QVector>
#include <QtGui/QTransform>
#include <QtCore/QRect>
#include <opencv2/opencv.hpp>
#include <vector>

class ObjSignature;
class Vocabulary;
class KeypointDetector;
class DescriptorExtractor;

class FINDOBJECT_EXP FindObject : public QObject
{
	Q_OBJECT;

public:
	enum TimeStamp{kTimeKeypointDetection, kTimeDescriptorExtraction, kTimeIndexing, kTimeMatching, kTimeHomography, kTimeTotal};

public:
	FindObject(QObject * parent = 0);
	virtual ~FindObject();

	int loadObjects(const QString & dirPath); // call updateObjects()
	const ObjSignature * addObject(const QString & filePath);
	const ObjSignature * addObject(const cv::Mat & image, int id=0);
	bool addObject(ObjSignature * obj); // take ownership when true is returned
	void removeObject(int id);
	void removeAllObjects();

	bool detect(const cv::Mat & image, QMultiMap<int,QPair<QRect,QTransform> > & objectsDetected);

	void updateDetectorExtractor();
	void updateObjects();
	void updateVocabulary();

	const QMap<int, ObjSignature*> & objects() const {return objects_;}
	const Vocabulary * vocabulary() const {return vocabulary_;}

	const QMultiMap<int,QPair<QRect,QTransform> > & objectsDetected() const {return objectsDetected_;}
	const QMap<TimeStamp, float> & timeStamps() const {return timeStamps_;}
	const std::vector<cv::KeyPoint> & sceneKeypoints() const {return sceneKeypoints_;}
	const cv::Mat & sceneDescriptors() const {return sceneDescriptors_;}
	const QMultiMap<int, int> & sceneWords() const {return sceneWords_;}
	const QMap<int, QMultiMap<int, int> > & matches() const {return matches_;}
	const QMultiMap<int, QMultiMap<int, int> > & inliers() const {return inliers_;}
	const QMultiMap<int, QMultiMap<int, int> > & outliers() const {return outliers_;}
	const QMultiMap<int, QMultiMap<int, int> > & rejectedInliers() const {return rejectedInliers_;}
	const QMultiMap<int, QMultiMap<int, int> > & rejectedOutliers() const {return rejectedOutliers_;}
	float minMatchedDistance() const {return minMatchedDistance_;}
	float maxMatchedDistance() const {return maxMatchedDistance_;}

public Q_SLOTS:
	void detect(const cv::Mat & image); // emit objectsfound()

Q_SIGNALS:
	void objectsFound(const QMultiMap<int, QPair<QRect, QTransform> > &);

private:
	void clearVocabulary();

private:
	QMap<int, ObjSignature*> objects_;
	Vocabulary * vocabulary_;
	QMap<int, cv::Mat> objectsDescriptors_;
	QMap<int, int> dataRange_; // <last id of object's descriptor, id>
	KeypointDetector * detector_;
	DescriptorExtractor * extractor_;

	QMultiMap<int,QPair<QRect,QTransform> > objectsDetected_;
	QMap<TimeStamp, float> timeStamps_;
	std::vector<cv::KeyPoint> sceneKeypoints_;
	cv::Mat sceneDescriptors_;
	QMultiMap<int, int> sceneWords_;
	QMap<int, QMultiMap<int, int> > matches_; // ObjectID Map< ObjectDescriptorIndex, SceneDescriptorIndex >, match the number of objects
	QMultiMap<int, QMultiMap<int, int> > inliers_; // ObjectID Map< ObjectDescriptorIndex, SceneDescriptorIndex >, match the number of detected objects
	QMultiMap<int, QMultiMap<int, int> > outliers_; // ObjectID Map< ObjectDescriptorIndex, SceneDescriptorIndex >, match the number of detected objects
	QMultiMap<int, QMultiMap<int, int> > rejectedInliers_; // ObjectID Map< ObjectDescriptorIndex, SceneDescriptorIndex >
	QMultiMap<int, QMultiMap<int, int> > rejectedOutliers_; // ObjectID Map< ObjectDescriptorIndex, SceneDescriptorIndex >
	float minMatchedDistance_;
	float maxMatchedDistance_;
};

#endif /* FINDOBJECT_H_ */
