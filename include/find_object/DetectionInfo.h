/*
 * DetectionInfo.h
 *
 *  Created on: 2014-08-03
 *      Author: mathieu
 */

#ifndef DETECTIONINFO_H_
#define DETECTIONINFO_H_

#include <QtCore/QMultiMap>
#include <QtGui/QTransform>
#include <QtCore/QSize>
#include <QtCore/QString>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

class DetectionInfo
{
public:
	enum TimeStamp{
		kTimeKeypointDetection,
		kTimeDescriptorExtraction,
		kTimeIndexing,
		kTimeMatching,
		kTimeHomography,
		kTimeTotal
	};
	enum RejectedCode{
		kRejectedUndef,
		kRejectedLowMatches,
		kRejectedLowInliers,
		kRejectedSuperposed,
		kRejectedAllInliers,
		kRejectedNotValid,
		kRejectedCornersOutside,
		kRejectedByAngle
	};

public:
	DetectionInfo() :
		minMatchedDistance_(-1),
		maxMatchedDistance_(-1)
	{}

public:
	// Those maps have the same size
	QMultiMap<int, QTransform> objDetected_;
	QMultiMap<int, QSize> objDetectedSizes_; // Object ID <width, height> match the number of detected objects
	QMultiMap<int, QString > objDetectedFilenames_; // Object ID <filename> match the number of detected objects
	QMultiMap<int, int> objDetectedInliersCount_; // ObjectID <count> match the number of detected objects
	QMultiMap<int, int> objDetectedOutliersCount_; // ObjectID <count> match the number of detected objects
	QMultiMap<int, QMultiMap<int, int> > objDetectedInliers_; // ObjectID Map< ObjectDescriptorIndex, SceneDescriptorIndex >, match the number of detected objects
	QMultiMap<int, QMultiMap<int, int> > objDetectedOutliers_; // ObjectID Map< ObjectDescriptorIndex, SceneDescriptorIndex >, match the number of detected objects

	QMap<TimeStamp, float> timeStamps_;
	std::vector<cv::KeyPoint> sceneKeypoints_;
	cv::Mat sceneDescriptors_;
	QMultiMap<int, int> sceneWords_;
	QMap<int, QMultiMap<int, int> > matches_; // ObjectID Map< ObjectDescriptorIndex, SceneDescriptorIndex >, match the number of objects

	// Those maps have the same size
	QMultiMap<int, QMultiMap<int, int> > rejectedInliers_; // ObjectID Map< ObjectDescriptorIndex, SceneDescriptorIndex >
	QMultiMap<int, QMultiMap<int, int> > rejectedOutliers_; // ObjectID Map< ObjectDescriptorIndex, SceneDescriptorIndex >
	QMultiMap<int, RejectedCode> rejectedCodes_; // ObjectID rejected code

	float minMatchedDistance_;
	float maxMatchedDistance_;
};

inline QDataStream & operator<<(QDataStream &out, const DetectionInfo & info)
{
	out << quint32(info.objDetected_.size());

	QMultiMap<int, int>::const_iterator iterInliers = info.objDetectedInliersCount_.constBegin();
	QMultiMap<int, int>::const_iterator iterOutliers = info.objDetectedOutliersCount_.constBegin();
	QMultiMap<int, QSize>::const_iterator iterSizes = info.objDetectedSizes_.constBegin();
	QMultiMap<int, QString>::const_iterator iterFilenames = info.objDetectedFilenames_.constBegin();
	for(QMultiMap<int, QTransform>::const_iterator iter=info.objDetected_.constBegin();
		iter!=info.objDetected_.constEnd();
		++iter)
	{
		// ID
		out << iter.key();

		// Size
		out << iterSizes.value();

		// Transform
		out << iter.value();

		// Filename
		out << iterFilenames.value();

		// inliers and outliers count
		out << iterInliers.value();
		out << iterOutliers.value();

		++iterInliers;
		++iterOutliers;
		++iterSizes;
		++iterFilenames;
	}
	return out;
}

inline QDataStream & operator>>(QDataStream &in, DetectionInfo & info)
{
	QDataStream::Status oldStatus = in.status();
	in.resetStatus();
	info = DetectionInfo();

	quint32 n;
	in >> n;

	for (quint32 i = 0; i < n; ++i) {
		if (in.status() != QDataStream::Ok)
			break;

		int id;
		QSize size;
		QTransform homography;
		QString filename;
		int inliers, outliers;
		in >> id >> size >> homography >> filename >> inliers >> outliers;
		info.objDetected_.insert(id, homography);
		info.objDetectedSizes_.insert(id, size);
		info.objDetectedFilenames_.insert(id, filename);
		info.objDetectedInliersCount_.insert(id, inliers);
		info.objDetectedOutliersCount_.insert(id, outliers);
	}
	if (in.status() != QDataStream::Ok)
		info = DetectionInfo();
	if (oldStatus != QDataStream::Ok)
		in.setStatus(oldStatus);
	return in;
}


#endif /* DETECTIONINFO_H_ */
