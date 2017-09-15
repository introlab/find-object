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

#include "find_object/Settings.h"

#include "find_object/utilite/ULogger.h"
#include "Vocabulary.h"
#include <QtCore/QVector>
#include <QDataStream>
#include <stdio.h>
#if CV_MAJOR_VERSION < 3
#include <opencv2/gpu/gpu.hpp>
#define CVCUDA cv::gpu
#else
#include <opencv2/core/cuda.hpp>
#define CVCUDA cv::cuda
#ifdef HAVE_OPENCV_CUDAFEATURES2D
#include <opencv2/cudafeatures2d.hpp>
#endif
#endif

namespace find_object {

Vocabulary::Vocabulary()
{
}

Vocabulary::~Vocabulary()
{
}

void Vocabulary::clear()
{
	wordToObjects_.clear();
	notIndexedDescriptors_ = cv::Mat();
	notIndexedWordIds_.clear();

	if(Settings::getGeneral_vocabularyFixed() && Settings::getGeneral_invertedSearch())
	{
		this->update(); // if vocabulary structure has changed

		// If the dictionary is fixed, don't clear indexed descriptors
		return;
	}

	indexedDescriptors_ = cv::Mat();
}

void Vocabulary::save(QDataStream & streamSessionPtr) const
{
	// save index
	streamSessionPtr << wordToObjects_;

	// save words
	qint64 dataSize = indexedDescriptors_.elemSize()*indexedDescriptors_.cols*indexedDescriptors_.rows;
	streamSessionPtr << indexedDescriptors_.rows <<
			indexedDescriptors_.cols <<
			indexedDescriptors_.type() <<
			dataSize;
	streamSessionPtr << QByteArray((char*)indexedDescriptors_.data, dataSize);
}

void Vocabulary::load(QDataStream & streamSessionPtr)
{
	// load index
	streamSessionPtr >> wordToObjects_;

	// load words
	int rows,cols,type;
	qint64 dataSize;
	streamSessionPtr >> rows >> cols >> type >> dataSize;
	QByteArray data;
	streamSessionPtr >> data;
	indexedDescriptors_ = cv::Mat(rows, cols, type, data.data()).clone();

	update();
}

bool Vocabulary::save(const QString & filename) const
{
	// save descriptors
	cv::FileStorage fs(filename.toStdString(), cv::FileStorage::WRITE);
	if(fs.isOpened())
	{
		fs << "Descriptors" << indexedDescriptors_;
		return true;
	}
	else
	{
		UERROR("Failed to open vocabulary file \"%s\"", filename.toStdString().c_str());
	}
	return false;
}

bool Vocabulary::load(const QString & filename)
{
	// save descriptors
	cv::FileStorage fs(filename.toStdString(), cv::FileStorage::READ);
	if(fs.isOpened())
	{
		cv::Mat tmp;
		fs["Descriptors"] >> tmp;

		if(!tmp.empty())
		{
			// clear index
			wordToObjects_.clear();
			indexedDescriptors_ = tmp;
			update();
			return true;
		}
		else
		{
			UERROR("Failed to read \"Descriptors\" matrix field (doesn't exist or is empty) from vocabulary file \"%s\"", filename.toStdString().c_str());
		}
	}
	else
	{
		UERROR("Failed to open vocabulary file \"%s\"", filename.toStdString().c_str());
	}
	return false;
}

QMultiMap<int, int> Vocabulary::addWords(const cv::Mat & descriptorsIn, int objectId)
{
	QMultiMap<int, int> words;
	if (descriptorsIn.empty())
	{
		return words;
	}

	cv::Mat descriptors;
	if(descriptorsIn.type() == CV_8U && Settings::getNearestNeighbor_7ConvertBinToFloat())
	{
		descriptorsIn.convertTo(descriptors, CV_32F);
	}
	else
	{
		descriptors = descriptorsIn;
	}

	if(Settings::getGeneral_vocabularyIncremental() || Settings::getGeneral_vocabularyFixed())
	{
		int k = 2;
		cv::Mat results;
		cv::Mat	dists;

		bool globalSearch = false;
		if(!indexedDescriptors_.empty() && indexedDescriptors_.rows >= (int)k)
		{
			if(indexedDescriptors_.type() != descriptors.type() || indexedDescriptors_.cols != descriptors.cols)
			{
				if(Settings::getGeneral_vocabularyFixed())
				{
					UERROR("Descriptors (type=%d size=%d) to search in vocabulary are not the same type/size as those in the vocabulary (type=%d size=%d)! Empty words returned.",
							descriptors.type(), descriptors.cols, indexedDescriptors_.type(), indexedDescriptors_.cols);
					return words;
				}
				else
				{
					UFATAL("Descriptors (type=%d size=%d) to search in vocabulary are not the same type/size as those in the vocabulary (type=%d size=%d)!",
							descriptors.type(), descriptors.cols, indexedDescriptors_.type(), indexedDescriptors_.cols);
				}
			}

			this->search(descriptors, results, dists, k);

			if( dists.type() == CV_32S )
			{
				cv::Mat temp;
				dists.convertTo(temp, CV_32F);
				dists = temp;
			}

			globalSearch = true;
		}

		if(!Settings::getGeneral_vocabularyFixed())
		{
			notIndexedWordIds_.reserve(notIndexedWordIds_.size() + descriptors.rows);
			notIndexedDescriptors_.reserve(notIndexedDescriptors_.rows + descriptors.rows);
		}
		int matches = 0;
		for(int i = 0; i < descriptors.rows; ++i)
		{
			QMultiMap<float, int> fullResults; // nearest descriptors sorted by distance
			if(notIndexedDescriptors_.rows)
			{
				UASSERT(notIndexedDescriptors_.type() == descriptors.type() && notIndexedDescriptors_.cols == descriptors.cols);

				// Check if this descriptor matches with a word not already added to the vocabulary
				// Do linear search only
				cv::Mat tmpResults;
				cv::Mat	tmpDists;
				if(descriptors.type()==CV_8U)
				{
					//normType – One of NORM_L1, NORM_L2, NORM_HAMMING, NORM_HAMMING2. L1 and L2 norms are
					//			 preferable choices for SIFT and SURF descriptors, NORM_HAMMING should be
					// 			 used with ORB, BRISK and BRIEF, NORM_HAMMING2 should be used with ORB
					// 			 when WTA_K==3 or 4 (see ORB::ORB constructor description).
					int normType = cv::NORM_HAMMING;
					if(Settings::currentDescriptorType().compare("ORB") &&
						(Settings::getFeature2D_ORB_WTA_K()==3 || Settings::getFeature2D_ORB_WTA_K()==4))
					{
						normType = cv::NORM_HAMMING2;
					}

					cv::batchDistance( descriptors.row(i),
									notIndexedDescriptors_,
									tmpDists,
									CV_32S,
									tmpResults,
									normType,
									notIndexedDescriptors_.rows>=k?k:1,
									cv::Mat(),
									0,
									false);
				}
				else
				{
					cv::flann::Index tmpIndex;
#if CV_MAJOR_VERSION == 2 and CV_MINOR_VERSION == 4 and CV_SUBMINOR_VERSION >= 12
					tmpIndex.build(notIndexedDescriptors_, cv::Mat(), cv::flann::LinearIndexParams(), cvflann::FLANN_DIST_L2);
#else
					tmpIndex.build(notIndexedDescriptors_, cv::flann::LinearIndexParams(), cvflann::FLANN_DIST_L2);
#endif
					tmpIndex.knnSearch(descriptors.row(i), tmpResults, tmpDists, notIndexedDescriptors_.rows>1?k:1, cvflann::FLANN_DIST_L2);
				}

				if( tmpDists.type() == CV_32S )
				{
					cv::Mat temp;
					tmpDists.convertTo(temp, CV_32F);
					tmpDists = temp;
				}

				for(int j = 0; j < tmpResults.cols; ++j)
				{
					if(tmpResults.at<int>(0,j) >= 0)
					{
						//printf("local i=%d, j=%d, tmpDist=%f tmpResult=%d\n", i ,j, tmpDists.at<float>(0,j), tmpResults.at<int>(0,j));
						fullResults.insert(tmpDists.at<float>(0,j), notIndexedWordIds_.at(tmpResults.at<int>(0,j)));
					}
				}
			}

			if(globalSearch)
			{
				for(int j=0; j<k; ++j)
				{
					if(results.at<int>(i,j) >= 0)
					{
						//printf("global i=%d, j=%d, dist=%f\n", i ,j, dists.at<float>(i,j));
						fullResults.insert(dists.at<float>(i,j), results.at<int>(i,j));
					}
				}
			}

			bool matched = false;
			if(Settings::getNearestNeighbor_3nndrRatioUsed() &&
			   fullResults.size() >= 2 &&
			   fullResults.begin().key() <= Settings::getNearestNeighbor_4nndrRatio() * (++fullResults.begin()).key())
			{
				matched = true;
			}
			if((matched || !Settings::getNearestNeighbor_3nndrRatioUsed()) &&
			   Settings::getNearestNeighbor_5minDistanceUsed())
			{
				if(fullResults.begin().key() <= Settings::getNearestNeighbor_6minDistance())
				{
					matched = true;
				}
				else
				{
					matched = false;
				}
			}
			if(!matched && !Settings::getNearestNeighbor_3nndrRatioUsed() && !Settings::getNearestNeighbor_5minDistanceUsed())
			{
				matched = true; // no criterion, match to the nearest descriptor
			}

			if(matched)
			{
				words.insert(fullResults.begin().value(), i);
				wordToObjects_.insert(fullResults.begin().value(), objectId);
				++matches;
			}
			else if(!Settings::getGeneral_invertedSearch() || !Settings::getGeneral_vocabularyFixed())
			{
				//concatenate new words
				notIndexedWordIds_.push_back(indexedDescriptors_.rows + notIndexedDescriptors_.rows);
				notIndexedDescriptors_.push_back(descriptors.row(i));
				words.insert(notIndexedWordIds_.back(), i);
				wordToObjects_.insert(notIndexedWordIds_.back(), objectId);
			}
			else
			{
				words.insert(-1, i); // invalid word
			}
		}
	}
	else
	{
		for(int i = 0; i < descriptors.rows; ++i)
		{
			wordToObjects_.insert(indexedDescriptors_.rows + notIndexedDescriptors_.rows+i, objectId);
			words.insert(indexedDescriptors_.rows + notIndexedDescriptors_.rows+i, i);
			notIndexedWordIds_.push_back(indexedDescriptors_.rows + notIndexedDescriptors_.rows+i);
		}

		//just concatenate descriptors
		notIndexedDescriptors_.push_back(descriptors);
	}
	return words;
}

void Vocabulary::update()
{
	if(!notIndexedDescriptors_.empty())
	{
		if(!indexedDescriptors_.empty())
		{
			UASSERT(indexedDescriptors_.cols == notIndexedDescriptors_.cols &&
				 	indexedDescriptors_.type() == notIndexedDescriptors_.type() );
		}
		
		//concatenate descriptors
		indexedDescriptors_.push_back(notIndexedDescriptors_);

		notIndexedDescriptors_ = cv::Mat();
		notIndexedWordIds_.clear();
	}

	if(!indexedDescriptors_.empty() && !Settings::isBruteForceNearestNeighbor())
	{
		cv::flann::IndexParams * params = Settings::createFlannIndexParams();
#if CV_MAJOR_VERSION == 2 and CV_MINOR_VERSION == 4 and CV_SUBMINOR_VERSION >= 12
		flannIndex_.build(indexedDescriptors_, cv::Mat(), *params, Settings::getFlannDistanceType());
#else
		flannIndex_.build(indexedDescriptors_, *params, Settings::getFlannDistanceType());
#endif
		delete params;
	}
}

void Vocabulary::search(const cv::Mat & descriptorsIn, cv::Mat & results, cv::Mat & dists, int k)
{
	if(!indexedDescriptors_.empty())
	{
		cv::Mat descriptors;
		if(descriptorsIn.type() == CV_8U && Settings::getNearestNeighbor_7ConvertBinToFloat())
		{
			descriptorsIn.convertTo(descriptors, CV_32F);
		}
		else
		{
			descriptors = descriptorsIn;
		}

		UASSERT(descriptors.type() == indexedDescriptors_.type() && descriptors.cols == indexedDescriptors_.cols);

		if(Settings::isBruteForceNearestNeighbor())
		{
			std::vector<std::vector<cv::DMatch> > matches;
			if(Settings::getNearestNeighbor_BruteForce_gpu() && CVCUDA::getCudaEnabledDeviceCount())
			{
				CVCUDA::GpuMat newDescriptorsGpu(descriptors);
				CVCUDA::GpuMat lastDescriptorsGpu(indexedDescriptors_);
#if CV_MAJOR_VERSION < 3
				if(indexedDescriptors_.type()==CV_8U)
				{
					CVCUDA::BruteForceMatcher_GPU<cv::Hamming> gpuMatcher;
					gpuMatcher.knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
				}
				else
				{
					CVCUDA::BruteForceMatcher_GPU<cv::L2<float> > gpuMatcher;
					gpuMatcher.knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
				}
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
				cv::Ptr<cv::cuda::DescriptorMatcher> gpuMatcher;
				if(indexedDescriptors_.type()==CV_8U)
				{
					gpuMatcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_HAMMING);
					gpuMatcher->knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
				}
				else
				{
					gpuMatcher = cv::cuda::DescriptorMatcher::createBFMatcher(cv::NORM_L2);
					gpuMatcher->knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
				}
#else
				UERROR("OpenCV3 is not built with CUDAFEATURES2D module, cannot do brute force matching on GPU!");
#endif
#endif
			}
			else
			{
				cv::BFMatcher matcher(indexedDescriptors_.type()==CV_8U?cv::NORM_HAMMING:cv::NORM_L2);
				matcher.knnMatch(descriptors, indexedDescriptors_, matches, k);
			}

			//convert back to matrix style
			results = cv::Mat((int)matches.size(), k, CV_32SC1);
			dists = cv::Mat((int)matches.size(), k, CV_32FC1);
			for(unsigned int i=0; i<matches.size(); ++i)
			{
				for(int j=0; j<k; ++j)
				{
					results.at<int>(i, j) = matches[i].at(j).trainIdx;
					dists.at<float>(i, j) = matches[i].at(j).distance;
				}
			}
		}
		else
		{
			flannIndex_.knnSearch(descriptors, results, dists, k, Settings::getFlannSearchParams());
		}

		if( dists.type() == CV_32S )
		{
			cv::Mat temp;
			dists.convertTo(temp, CV_32F);
			dists = temp;
		}
	}
}

} // namespace find_object
