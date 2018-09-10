/*
 * Compression.h
 *
 *  Created on: Sep 10, 2018
 *      Author: labm2414
 */

#ifndef SRC_COMPRESSION_H_
#define SRC_COMPRESSION_H_

#include <opencv2/opencv.hpp>

namespace find_object {

std::vector<unsigned char> compressData(const cv::Mat & data);
cv::Mat uncompressData(const unsigned char * bytes, unsigned long size);

}

#endif /* SRC_COMPRESSION_H_ */
