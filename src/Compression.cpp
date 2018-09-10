/*
 * Compression.cpp
 *
 *  Created on: Sep 10, 2018
 *      Author: labm2414
 */

#include <Compression.h>
#include <zlib.h>
#include "find_object/utilite/ULogger.h"

namespace find_object {

std::vector<unsigned char> compressData(const cv::Mat & data)
{
	std::vector<unsigned char> bytes;
	if(!data.empty())
	{
		uLong sourceLen = uLong(data.total())*uLong(data.elemSize());
		uLong destLen = compressBound(sourceLen);
		bytes.resize(destLen);
		int errCode = compress(
						(Bytef *)bytes.data(),
						&destLen,
						(const Bytef *)data.data,
						sourceLen);

		bytes.resize(destLen+3*sizeof(int));
		*((int*)&bytes[destLen]) = data.rows;
		*((int*)&bytes[destLen+sizeof(int)]) = data.cols;
		*((int*)&bytes[destLen+2*sizeof(int)]) = data.type();

		if(errCode == Z_MEM_ERROR)
		{
			UERROR("Z_MEM_ERROR : Insufficient memory.");
		}
		else if(errCode == Z_BUF_ERROR)
		{
			UERROR("Z_BUF_ERROR : The buffer dest was not large enough to hold the uncompressed data.");
		}
	}
	return bytes;
}

cv::Mat uncompressData(const unsigned char * bytes, unsigned long size)
{
	cv::Mat data;
	if(bytes && size>=3*sizeof(int))
	{
		//last 3 int elements are matrix size and type
		int height = *((int*)&bytes[size-3*sizeof(int)]);
		int width = *((int*)&bytes[size-2*sizeof(int)]);
		int type = *((int*)&bytes[size-1*sizeof(int)]);

		data = cv::Mat(height, width, type);
		uLongf totalUncompressed = uLongf(data.total())*uLongf(data.elemSize());

		int errCode = uncompress(
						(Bytef*)data.data,
						&totalUncompressed,
						(const Bytef*)bytes,
						uLong(size));

		if(errCode == Z_MEM_ERROR)
		{
			UERROR("Z_MEM_ERROR : Insufficient memory.");
		}
		else if(errCode == Z_BUF_ERROR)
		{
			UERROR("Z_BUF_ERROR : The buffer dest was not large enough to hold the uncompressed data.");
		}
		else if(errCode == Z_DATA_ERROR)
		{
			UERROR("Z_DATA_ERROR : The compressed data (referenced by source) was corrupted.");
		}
	}
	return data;
}
} /* namespace find_object */
