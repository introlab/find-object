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
#ifndef DETECTIONINFO_H_
#define DETECTIONINFO_H_

#include <QtCore/QMultiMap>
#include <QtGui/QTransform>
#include <QtCore/QSize>
#include <QtCore/QString>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

namespace find_object {

class DetectionInfo
{
public:
	enum TimeStamp{
		kTimeKeypointDetection,
		kTimeDescriptorExtraction,
		kTimeSkewAffine,
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

} // namespace find_object

#endif /* DETECTIONINFO_H_ */
