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
#include <QtCore/QFileInfo>
#include <Compression.h>

namespace find_object {

class ObjSignature {
public:
	ObjSignature() :
		id_(-1)
	{}
	ObjSignature(int id, const cv::Mat & image, const QString & filePath) :
		id_(id),
		image_(image),
		rect_(0,0,image.cols, image.rows),
		filePath_(filePath)
	{}
	virtual ~ObjSignature() {}

	void setData(const std::vector<cv::KeyPoint> & keypoints, const cv::Mat & descriptors)
	{
		keypoints_ = keypoints;
		descriptors_ = descriptors;
	}
	void setWords(const QMultiMap<int, int> & words) {words_ = words;}
	void setId(int id) {id_ = id;}
	void removeImage() {image_ = cv::Mat();}

	const QRect & rect() const {return rect_;}

	int id() const {return id_;}
	const QString & filePath() const {return filePath_;}
	const cv::Mat & image() const {return image_;}
	const std::vector<cv::KeyPoint> & keypoints() const {return keypoints_;}
	const cv::Mat & descriptors() const {return descriptors_;}
	const QMultiMap<int, int> & words() const {return words_;}

	void save(QDataStream & streamPtr) const
	{
		streamPtr << id_;
		streamPtr << filePath_;
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

		std::vector<unsigned char> bytes  = compressData(descriptors_);

		qint64 dataSize = bytes.size();
		int old = 0;
		if(dataSize <= std::numeric_limits<int>::max())
		{
			// old: rows, cols, type
			streamPtr << old << old << old << dataSize;
			streamPtr << QByteArray::fromRawData((const char*)bytes.data(), dataSize);
		}
		else
		{
			UERROR("Descriptors (compressed) are too large (%d MB) to be saved! Limit is 2 GB (based on max QByteArray size).",
					dataSize/(1024*1024));
			// old: rows, cols, type, dataSize
			streamPtr << old << old << old << old;
			streamPtr << QByteArray(); // empty
		}

		streamPtr << words_;

		if(!image_.empty())
		{
			std::vector<unsigned char> bytes;
			QString ext = QFileInfo(filePath_).suffix();
			if(ext.isEmpty())
			{
				// default png
				cv::imencode(".png", image_, bytes);
			}
			else
			{
				cv::imencode(std::string(".")+ext.toStdString(), image_, bytes);
			}
			streamPtr << QByteArray::fromRawData((const char*)bytes.data(), (int)bytes.size());
		}
		else
		{
			streamPtr << QByteArray();
		}

		streamPtr << rect_;
	}

	void load(QDataStream & streamPtr, bool ignoreImage)
	{
		int nKpts;
		streamPtr >> id_ >> filePath_ >> nKpts;
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
		if(rows == 0 && cols == 0 && type == 0)
		{
			// compressed descriptors
			UASSERT(dataSize <= std::numeric_limits<int>::max());
			QByteArray data;
			streamPtr >> data;
			descriptors_ = uncompressData((unsigned const char*)data.data(), dataSize);
		}
		else
		{
			// old raw format
			QByteArray data;
			streamPtr >> data;
			if(data.size())
			{
				descriptors_ = cv::Mat(rows, cols, type, data.data()).clone();
			}
			else if(dataSize)
			{
				UERROR("Error reading descriptor data for object=%d", id_);
			}
		}

		streamPtr >> words_;

		QByteArray image;
		streamPtr >> image;
		if(!ignoreImage && image.size())
		{
			std::vector<unsigned char> bytes(image.size());
			memcpy(bytes.data(), image.data(), image.size());
			image_ = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
		}

		streamPtr >> rect_;
	}

private:
	int id_;
	cv::Mat image_;
	QRect rect_;
	QString filePath_;
	std::vector<cv::KeyPoint> keypoints_;
	cv::Mat descriptors_;
	QMultiMap<int, int> words_; // <word id, keypoint indexes>
};

} // namespace find_object

#endif /* OBJSIGNATURE_H_ */
