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
	descriptors_ = cv::Mat();
	wordToObjects_.clear();
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
		cv::Mat results(descriptors.rows, k, CV_32SC1); // results index
		cv::Mat	dists(descriptors.rows, k, CV_32FC1); // Distance results are CV_32FC1

		bool globalSearch = false;
		if(!descriptors_.empty() && descriptors_.rows >= (int)k)
		{
			flannIndex_.knnSearch(descriptors, results, dists, k, Settings::getFlannSearchParams() );
			globalSearch = true;
		}

		QVector<int> newWordsId; // index global
		cv::Mat newWords;
		int matches = 0;
		for(int i = 0; i < descriptors.rows; ++i)
		{
			QMap<float, int> fullResults; // nearest descriptors sorted by distance
			if(newWords.rows)
			{
				// Check if this descriptor matches with a word not already added to the vocabulary
				cv::flann::Index tmpIndex;
				cv::flann::IndexParams * params;
				if(descriptors.type()==CV_8U)
				{
					params = Settings::createFlannIndexParams(); // should be LSH
				}
				else
				{
					params = new cv::flann::LinearIndexParams(); // faster
				}
				tmpIndex.build(newWords, *params, Settings::getFlannDistanceType());
				delete params;
				cv::Mat tmpResults(1, newWords.rows>1?k:1, CV_32SC1); // results index
				cv::Mat	tmpDists(1, newWords.rows>1?k:1, CV_32FC1); // Distance results are CV_32FC1
				tmpIndex.knnSearch(descriptors.row(i), tmpResults, tmpDists, newWords.rows>1?k:1, Settings::getFlannSearchParams());
				for(int j = 0; j < (newWords.rows>1?k:1); ++j)
				{
					fullResults.insert(tmpDists.at<float>(0,j), newWordsId.at(tmpResults.at<int>(0,j)));
				}
			}

			if(globalSearch)
			{
				for(int j=0; j<k; ++j)
				{
					fullResults.insert(dists.at<float>(i,j), results.at<int>(i,j));
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
				newWordsId.push_back(descriptors_.rows + newWords.rows);
				newWords = tmp;
				words.insert(newWordsId.back(), i);
				wordToObjects_.insert(newWordsId.back(), objectIndex);
			}
		}
		//printf("matches = %d\n", matches);

		//concatenate new words
		if(newWords.rows)
		{
			cv::Mat tmp(descriptors_.rows+newWords.rows, descriptors.cols, descriptors.type());
			if(descriptors_.rows)
			{
				cv::Mat dest(tmp, cv::Range(0, descriptors_.rows));
				descriptors_.copyTo(dest);
			}
			cv::Mat dest(tmp, cv::Range(descriptors_.rows, descriptors_.rows+newWords.rows));
			newWords.copyTo(dest);
			descriptors_ = tmp;
		}

		//update
		this->update();
	}
	else
	{
		for(int i = 0; i < descriptors.rows; ++i)
		{
			wordToObjects_.insert(descriptors_.rows+i, objectIndex);
			words.insert(descriptors_.rows+i, i);
		}

		//just concatenate descriptors
		cv::Mat tmp(descriptors_.rows+descriptors.rows, descriptors.cols, descriptors.type());
		if(descriptors_.rows)
		{
			cv::Mat dest(tmp, cv::Range(0, descriptors_.rows));
			descriptors_.copyTo(dest);
		}
		cv::Mat dest(tmp, cv::Range(descriptors_.rows, descriptors_.rows+descriptors.rows));
		descriptors.copyTo(dest);
		descriptors_ = tmp;
	}

	return words;
}

void Vocabulary::update()
{
	if(!descriptors_.empty())
	{
		cv::flann::IndexParams * params = Settings::createFlannIndexParams();
		flannIndex_.build(descriptors_, *params, Settings::getFlannDistanceType());
		delete params;
	}
}

void Vocabulary::search(const cv::Mat & descriptors, cv::Mat & results, cv::Mat & dists, int k)
{
	if(!descriptors_.empty())
	{
		flannIndex_.knnSearch(descriptors, results, dists, k, Settings::getFlannSearchParams());
	}
}
