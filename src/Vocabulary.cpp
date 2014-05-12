/*
 * Vocabulary.cpp
 *
 *  Created on: 2014-05-09
 *      Author: mathieu
 */

#include "Vocabulary.h"
#include "Settings.h"
#include <QtCore/QVector>
#include <stdio.h>

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

QMultiMap<int, int> Vocabulary::addWords(const cv::Mat & descriptors, int objectIndex, bool incremental)
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
			Q_ASSERT(indexedDescriptors_.type() == descriptors.type() && indexedDescriptors_.cols == descriptors.cols);
			flannIndex_.knnSearch(descriptors, results, dists, k, Settings::getFlannSearchParams() );

			if( dists.type() == CV_32S )
			{
				cv::Mat temp;
				dists.convertTo(temp, CV_32F);
				dists = temp;
			}

			globalSearch = true;
		}

		QVector<int> newWordIds = notIndexedWordIds_; // index global
		cv::Mat newWords = notIndexedDescriptors_;
		int matches = 0;
		for(int i = 0; i < descriptors.rows; ++i)
		{
			QMap<float, int> fullResults; // nearest descriptors sorted by distance
			if(newWords.rows)
			{
				Q_ASSERT(newWords.type() == descriptors.type() && newWords.cols == descriptors.cols);

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
									newWords,
									tmpDists,
									CV_32S,
									tmpResults,
									normType,
									newWords.rows>=k?k:1,
									cv::Mat(),
									0,
									false);
				}
				else
				{
					cv::flann::Index tmpIndex;
					tmpIndex.build(newWords, cv::flann::LinearIndexParams(), Settings::getFlannDistanceType());
					tmpIndex.knnSearch(descriptors.row(i), tmpResults, tmpDists, newWords.rows>1?k:1, Settings::getFlannSearchParams());
				}

				if( tmpDists.type() == CV_32S )
				{
					cv::Mat temp;
					tmpDists.convertTo(temp, CV_32F);
					tmpDists = temp;
				}

				for(int j = 0; j < (newWords.rows>=k?k:1); ++j)
				{
					if(tmpResults.at<int>(0,j) >= 0)
					{
						//printf("local i=%d, j=%d, tmpDist=%f tmpResult=%d\n", i ,j, tmpDists.at<float>(0,j), tmpResults.at<int>(0,j));
						fullResults.insert(tmpDists.at<float>(0,j), newWordIds.at(tmpResults.at<int>(0,j)));
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
				wordToObjects_.insert(fullResults.begin().value(), objectIndex);
				++matches;
			}
			else
			{
				cv::Mat tmp(newWords.rows+1, descriptors.cols, descriptors.type());
				if(newWords.rows)
				{
					cv::Mat dest(tmp, cv::Range(0, newWords.rows));
					newWords.copyTo(dest);
				}
				cv::Mat dest(tmp, cv::Range(newWords.rows, newWords.rows+1));
				descriptors.row(i).copyTo(dest);
				newWordIds.push_back(indexedDescriptors_.rows + newWords.rows);
				newWords = tmp;
				words.insert(newWordIds.back(), i);
				wordToObjects_.insert(newWordIds.back(), objectIndex);
			}
		}

		//concatenate new words
		if(newWords.rows)
		{
			notIndexedWordIds_ = newWordIds;
			notIndexedDescriptors_ = newWords;
		}
	}
	else
	{
		for(int i = 0; i < descriptors.rows; ++i)
		{
			wordToObjects_.insert(indexedDescriptors_.rows + notIndexedDescriptors_.rows+i, objectIndex);
			words.insert(indexedDescriptors_.rows + notIndexedDescriptors_.rows+i, i);
			notIndexedWordIds_.push_back(indexedDescriptors_.rows + notIndexedDescriptors_.rows+i);
		}

		//just concatenate descriptors
		cv::Mat tmp(notIndexedDescriptors_.rows+descriptors.rows, descriptors.cols, descriptors.type());
		if(notIndexedDescriptors_.rows)
		{
			cv::Mat dest(tmp, cv::Range(0, notIndexedDescriptors_.rows));
			notIndexedDescriptors_.copyTo(dest);
		}
		cv::Mat dest(tmp, cv::Range(notIndexedDescriptors_.rows, notIndexedDescriptors_.rows+descriptors.rows));
		descriptors.copyTo(dest);
		notIndexedDescriptors_ = tmp;
	}

	return words;
}

void Vocabulary::update()
{
	if(!notIndexedDescriptors_.empty())
	{
		Q_ASSERT(indexedDescriptors_.cols == notIndexedDescriptors_.cols &&
				 indexedDescriptors_.type() == notIndexedDescriptors_.type() );

		//concatenate descriptors
		cv::Mat tmp(indexedDescriptors_.rows+notIndexedDescriptors_.rows, notIndexedDescriptors_.cols, notIndexedDescriptors_.type());
		cv::Mat dest(tmp, cv::Range(0, indexedDescriptors_.rows));
		indexedDescriptors_.copyTo(dest);
		dest = cv::Mat(tmp, cv::Range(indexedDescriptors_.rows, indexedDescriptors_.rows+notIndexedDescriptors_.rows));
		notIndexedDescriptors_.copyTo(dest);
		indexedDescriptors_ = tmp;

		notIndexedDescriptors_ = cv::Mat();
		notIndexedWordIds_.clear();
	}

	if(!indexedDescriptors_.empty())
	{
		cv::flann::IndexParams * params = Settings::createFlannIndexParams();
		flannIndex_.build(indexedDescriptors_, *params, Settings::getFlannDistanceType());
		delete params;
	}
}

void Vocabulary::search(const cv::Mat & descriptors, cv::Mat & results, cv::Mat & dists, int k)
{
	Q_ASSERT(notIndexedDescriptors_.empty() && notIndexedWordIds_.size() == 0);

	if(!indexedDescriptors_.empty())
	{
		Q_ASSERT(descriptors.type() == indexedDescriptors_.type() && descriptors.cols == indexedDescriptors_.cols);

		flannIndex_.knnSearch(descriptors, results, dists, k, Settings::getFlannSearchParams());

		if( dists.type() == CV_32S )
		{
			cv::Mat temp;
			dists.convertTo(temp, CV_32F);
			dists = temp;
		}
	}
}
