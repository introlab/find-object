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
#include <stdio.h>
#include <opencv2/gpu/gpu.hpp>

namespace find_object {

Vocabulary::Vocabulary()
{
}

Vocabulary::~Vocabulary()
{
}

void Vocabulary::clear()
{
	indexedDescriptors_ = cv::Mat();
	notIndexedDescriptors_ = cv::Mat();
	wordToObjects_.clear();
	notIndexedWordIds_.clear();
}

void Vocabulary::save(QDataStream & streamPtr) const
{
	if(!indexedDescriptors_.empty() && !wordToObjects_.empty())
	{
		UASSERT(notIndexedDescriptors_.empty() && notIndexedWordIds_.empty());

		// save index
		streamPtr << wordToObjects_;

		// save words
		qint64 dataSize = indexedDescriptors_.elemSize()*indexedDescriptors_.cols*indexedDescriptors_.rows;
		streamPtr << indexedDescriptors_.rows <<
				indexedDescriptors_.cols <<
				indexedDescriptors_.type() <<
				dataSize;
		streamPtr << QByteArray((char*)indexedDescriptors_.data, dataSize);
	}
}

void Vocabulary::load(QDataStream & streamPtr)
{
	// load index
	streamPtr >> wordToObjects_;

	// load words
	int rows,cols,type;
	qint64 dataSize;
	streamPtr >> rows >> cols >> type >> dataSize;
	QByteArray data;
	streamPtr >> data;
	indexedDescriptors_ = cv::Mat(rows, cols, type, data.data()).clone();

	update();
}

QMultiMap<int, int> Vocabulary::addWords(const cv::Mat & descriptors, int objectId, bool incremental)
{
	QMultiMap<int, int> words;
	if (descriptors.empty())
	{
		return words;
	}

	if(incremental)
	{
		int k = 2;
		cv::Mat results;
		cv::Mat	dists;

		bool globalSearch = false;
		if(!indexedDescriptors_.empty() && indexedDescriptors_.rows >= (int)k)
		{
			UASSERT(indexedDescriptors_.type() == descriptors.type() && indexedDescriptors_.cols == descriptors.cols);
			this->search(descriptors, results, dists, k);

			if( dists.type() == CV_32S )
			{
				cv::Mat temp;
				dists.convertTo(temp, CV_32F);
				dists = temp;
			}

			globalSearch = true;
		}

		notIndexedWordIds_.reserve(notIndexedWordIds_.size() + descriptors.rows);
		notIndexedDescriptors_.reserve(notIndexedDescriptors_.rows + descriptors.rows);
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
					tmpIndex.build(notIndexedDescriptors_, cv::flann::LinearIndexParams(), Settings::getFlannDistanceType());
					tmpIndex.knnSearch(descriptors.row(i), tmpResults, tmpDists, notIndexedDescriptors_.rows>1?k:1, Settings::getFlannSearchParams());
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

			bool match = false;
			// Apply NNDR
			if(fullResults.size() >= 2 &&
			   fullResults.begin().key() <= Settings::getNearestNeighbor_4nndrRatio() * (++fullResults.begin()).key())
			{
				match = true;
			}

			if(match)
			{
				words.insert(fullResults.begin().value(), i);
				wordToObjects_.insert(fullResults.begin().value(), objectId);
				++matches;
			}
			else
			{
				//concatenate new words
				notIndexedWordIds_.push_back(indexedDescriptors_.rows + notIndexedDescriptors_.rows);
				notIndexedDescriptors_.push_back(descriptors.row(i));
				words.insert(notIndexedWordIds_.back(), i);
				wordToObjects_.insert(notIndexedWordIds_.back(), objectId);
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
		flannIndex_.build(indexedDescriptors_, *params, Settings::getFlannDistanceType());
		delete params;
	}
}

void Vocabulary::search(const cv::Mat & descriptors, cv::Mat & results, cv::Mat & dists, int k)
{
	if(!indexedDescriptors_.empty())
	{
		UASSERT(descriptors.type() == indexedDescriptors_.type() && descriptors.cols == indexedDescriptors_.cols);

		if(Settings::isBruteForceNearestNeighbor())
		{
			std::vector<std::vector<cv::DMatch> > matches;
			if(Settings::getNearestNeighbor_BruteForce_gpu() && cv::gpu::getCudaEnabledDeviceCount())
			{
				cv::gpu::GpuMat newDescriptorsGpu(descriptors);
				cv::gpu::GpuMat lastDescriptorsGpu(indexedDescriptors_);
				if(indexedDescriptors_.type()==CV_8U)
				{
					cv::gpu::BruteForceMatcher_GPU<cv::Hamming> gpuMatcher;
					gpuMatcher.knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
				}
				else
				{
					cv::gpu::BruteForceMatcher_GPU<cv::L2<float> > gpuMatcher;
					gpuMatcher.knnMatch(newDescriptorsGpu, lastDescriptorsGpu, matches, k);
				}
			}
			else
			{
				cv::BFMatcher matcher(indexedDescriptors_.type()==CV_8U?cv::NORM_HAMMING:cv::NORM_L2);
				matcher.knnMatch(descriptors, indexedDescriptors_, matches, k);
			}

			//convert back to matrix style
			results = cv::Mat(matches.size(), k, CV_32SC1);
			dists = cv::Mat(matches.size(), k, CV_32FC1);
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
