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

#ifndef OBJSIGNATURE_H_
#define OBJSIGNATURE_H_

#include <opencv2/opencv.hpp>
#include <QtCore/QString>
#include <QtCore/QMultiMap>
#include <QtCore/QRect>
#include <QtCore/QDataStream>
#include <QtCore/QByteArray>

namespace find_object {

class ObjSignature {
public:
	ObjSignature() :
		id_(-1)
	{}
	ObjSignature(int id, const cv::Mat & image, const QString & filename) :
		id_(id),
		image_(image),
		filename_(filename)
	{}
	virtual ~ObjSignature() {}

	void setData(const std::vector<cv::KeyPoint> & keypoints, const cv::Mat & descriptors)
	{
		keypoints_ = keypoints;
		descriptors_ = descriptors;
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

	void save(QDataStream & streamPtr) const
	{
		streamPtr << id_;
		streamPtr << filename_;
		streamPtr << (int)keypoints_.size();
		for(unsigned int j=0; j<keypoints_.size(); ++j)
		{
				streamPtr << keypoints_.at(j).angle <<
								keypoints_.at(j).class_id <<
								keypoints_.at(j).octave <<
								keypoints_.at(j).pt.x <<
								keypoints_.at(j).pt.y <<
								keypoints_.at(j).response <<
								keypoints_.at(j).size;
		}

		qint64 dataSize = descriptors_.elemSize()*descriptors_.cols*descriptors_.rows;
		streamPtr << descriptors_.rows <<
						descriptors_.cols <<
						descriptors_.type() <<
						dataSize;
		streamPtr << QByteArray((char*)descriptors_.data, dataSize);

		streamPtr << words_;

		std::vector<unsigned char> bytes;
		cv::imencode(".png", image_, bytes);
		streamPtr << QByteArray((char*)bytes.data(), bytes.size());
	}

	void load(QDataStream & streamPtr)
	{
		int nKpts;
		streamPtr >> id_ >> filename_ >> nKpts;
		keypoints_.resize(nKpts);
		for(int i=0;i<nKpts;++i)
		{
				streamPtr >>
				keypoints_[i].angle >>
				keypoints_[i].class_id >>
				keypoints_[i].octave >>
				keypoints_[i].pt.x >>
				keypoints_[i].pt.y >>
				keypoints_[i].response >>
				keypoints_[i].size;
		}

		int rows,cols,type;
		qint64 dataSize;
		streamPtr >> rows >> cols >> type >> dataSize;
		QByteArray data;
		streamPtr >> data;
		descriptors_ = cv::Mat(rows, cols, type, data.data()).clone();

		streamPtr >> words_;

		QByteArray image;
		streamPtr >> image;
		std::vector<unsigned char> bytes(image.size());
		memcpy(bytes.data(), image.data(), image.size());
		image_ = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
	}

private:
	int id_;
	cv::Mat image_;
	QString filename_;
	std::vector<cv::KeyPoint> keypoints_;
	cv::Mat descriptors_;
	QMultiMap<int, int> words_; // <word id, keypoint indexes>
};

} // namespace find_object

#endif /* OBJSIGNATURE_H_ */
