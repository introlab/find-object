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
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef OBJSIGNATURE_H_
#define OBJSIGNATURE_H_

#include <opencv2/opencv.hpp>
#include <QtCore/QString>
#include <QtCore/QMultiMap>
#include <QtCore/QRect>

namespace find_object {

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

} // namespace find_object

#endif /* OBJSIGNATURE_H_ */
