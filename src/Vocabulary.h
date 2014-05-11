/*
 * Vocabulary.h
 *
 *  Created on: 2014-05-09
 *      Author: mathieu
 */

#ifndef VOCABULARY_H_
#define VOCABULARY_H_

#include <QtCore/QMultiMap>
#include <opencv2/opencv.hpp>

class Vocabulary {
public:
	Vocabulary();
	virtual ~Vocabulary();

	void clear();
	QMultiMap<int, int> addWords(const cv::Mat & descriptors, int objectIndex, bool incremental);
	void update();
	void search(const cv::Mat & descriptors, cv::Mat & results, cv::Mat & dists, int k);
	int size() const {return descriptors_.rows;}
	const QMultiMap<int, int> & wordToObjects() const {return wordToObjects_;}

private:
	cv::flann::Index flannIndex_;
	cv::Mat descriptors_;
	QMultiMap<int, int> wordToObjects_;
};

#endif /* VOCABULARY_H_ */
