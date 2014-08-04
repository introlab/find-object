/*
 * ObjSignature.h
 *
 *  Created on: 2014-07-30
 *      Author: mathieu
 */

#ifndef OBJSIGNATURE_H_
#define OBJSIGNATURE_H_

#include <opencv2/opencv.hpp>
#include <QtCore/QString>
#include <QtCore/QMultiMap>
#include <QtCore/QRect>

class ObjSignature {
public:
	ObjSignature(int id, const cv::Mat & image, const QString & filename) :
		id_(id),
		image_(image),
		filename_(filename)
	{}
	virtual ~ObjSignature() {}

	void setData(const std::vector<cv::KeyPoint> & keypoints,
				const cv::Mat & descriptors,
				const QString & detectorType,
				const QString & descriptorType)
	{
		keypoints_ = keypoints;
		descriptors_ = descriptors;
		detectorType_ = detectorType;
		descriptorType_ = descriptorType;
	}
	void setWords(const QMultiMap<int, int> & words) {words_ = words;}
	void setId(int id) {id_ = id;}

	QRect rect() const {return QRect(0,0,image_.cols, image_.rows);}

	int id() const {return id_;}
	const QString & filename() const {return filename_;}
	const cv::Mat & image() const {return image_;}
	const std::vector<cv::KeyPoint> & keypoints() const {return keypoints_;}
	const cv::Mat & descriptors() const {return descriptors_;}
	const QMultiMap<int, int> & words() const {return words_;}
	const QString & detectorType() const {return detectorType_;}
	const QString & descriptorType() const {return descriptorType_;}

private:
	int id_;
	cv::Mat image_;
	QString filename_;
	std::vector<cv::KeyPoint> keypoints_;
	cv::Mat descriptors_;
	QMultiMap<int, int> words_; // <word id, keypoint indexes>
	QString detectorType_;
	QString descriptorType_;
};

#endif /* OBJSIGNATURE_H_ */
