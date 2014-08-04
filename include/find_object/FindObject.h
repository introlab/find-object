/*
 * FindObject.h
 *
 *  Created on: 2014-07-30
 *      Author: mathieu
 */

#ifndef FINDOBJECT_H_
#define FINDOBJECT_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include "find_object/DetectionInfo.h"

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
	FindObject(QObject * parent = 0);
	virtual ~FindObject();

	int loadObjects(const QString & dirPath); // call updateObjects()
	const ObjSignature * addObject(const QString & filePath);
	const ObjSignature * addObject(const cv::Mat & image, int id=0, const QString & filename = QString());
	bool addObject(ObjSignature * obj); // take ownership when true is returned
	void removeObject(int id);
	void removeAllObjects();

	bool detect(const cv::Mat & image, DetectionInfo & info);

	void updateDetectorExtractor();
	void updateObjects();
	void updateVocabulary();

	const QMap<int, ObjSignature*> & objects() const {return objects_;}
	const Vocabulary * vocabulary() const {return vocabulary_;}

public Q_SLOTS:
	void detect(const cv::Mat & image); // emit objectsFound()

Q_SIGNALS:
	void objectsFound(const DetectionInfo &);

private:
	void clearVocabulary();

private:
	QMap<int, ObjSignature*> objects_;
	Vocabulary * vocabulary_;
	QMap<int, cv::Mat> objectsDescriptors_;
	QMap<int, int> dataRange_; // <last id of object's descriptor, id>
	KeypointDetector * detector_;
	DescriptorExtractor * extractor_;
};

#endif /* FINDOBJECT_H_ */
