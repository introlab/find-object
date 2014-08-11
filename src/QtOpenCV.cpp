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

#include "find_object/QtOpenCV.h"
#include <opencv2/core/core_c.h>
#include <stdio.h>

namespace find_object {

QImage cvtCvMat2QImage(const cv::Mat & image, bool isBgr)
{
	QImage qtemp;
	if(!image.empty() && image.depth() == CV_8U)
	{
		if(image.channels()==3)
		{
			const unsigned char * data = image.data;
			if(image.channels() == 3)
			{
				qtemp = QImage(image.cols, image.rows, QImage::Format_RGB32);
				for(int y = 0; y < image.rows; ++y, data += image.cols*image.elemSize())
				{
					for(int x = 0; x < image.cols; ++x)
					{
						QRgb * p = ((QRgb*)qtemp.scanLine (y)) + x;
						if(isBgr)
						{
							*p = qRgb(data[x * image.channels()+2], data[x * image.channels()+1], data[x * image.channels()]);
						}
						else
						{
							*p = qRgb(data[x * image.channels()], data[x * image.channels()+1], data[x * image.channels()+2]);
						}
					}
				}
			}
		}
		else if(image.channels() == 1)
		{
			// mono grayscale
			qtemp = QImage(image.data, image.cols, image.rows, image.cols, QImage::Format_Indexed8).copy();
			QVector<QRgb> my_table;
			for(int i = 0; i < 256; i++) my_table.push_back(qRgb(i,i,i));
			qtemp.setColorTable(my_table);
		}
		else
		{
			printf("Wrong image format, must have 1 or 3 channels\n");
		}
	}
	return qtemp;
}

cv::Mat cvtQImage2CvMat(const QImage & image)
{
	cv::Mat cvImage;
	if(!image.isNull() && image.depth() == 32 && image.format() == QImage::Format_RGB32)
	{
		// assume RGB (3 channels)
		int channels = 3;
		cvImage = cv::Mat(image.height(), image.width(), CV_8UC3);
		unsigned char * data = cvImage.data;
		for(int y = 0; y < image.height(); ++y, data+=cvImage.cols*cvImage.elemSize())
		{
			for(int x = 0; x < image.width(); ++x)
			{
				QRgb rgb = image.pixel(x, y);
				data[x * channels+2] = qRed(rgb); //r
				data[x * channels+1] = qGreen(rgb); //g
				data[x * channels] = qBlue(rgb); //b
			}
		}
	}
	else
	{
		printf("Failed to convert image : depth=%d(!=32) format=%d(!=%d)\n", image.depth(), image.format(), QImage::Format_RGB32);
	}
	return cvImage;
}


QImage cvtIplImage2QImage(const IplImage * image)
{
	QImage qtemp;
	if (image && image->depth == IPL_DEPTH_8U && cvGetSize(image).width>0)
	{
		const char * data = image->imageData;
		qtemp= QImage(image->width, image->height,QImage::Format_RGB32);
		
		for(int y = 0; y < image->height; ++y, data +=image->widthStep )
		{
			for(int x = 0; x < image->width; ++x)
			{
				uint *p = (uint*)qtemp.scanLine (y) + x;
				*p = qRgb(data[x * image->nChannels+2], data[x * image->nChannels+1],data[x * image->nChannels]);
			}
		}
	}
	else if(image && image->depth != IPL_DEPTH_8U)
	{
		printf("Wrong iplImage format, must be 8_bits\n");
	}
	return qtemp;
}

// Returned image must be released explicitly (using cvReleaseImage()).
IplImage * cvtQImage2IplImage(const QImage & image)
{
	IplImage * iplTmp = 0;
	if(!image.isNull() && image.depth() == 32 && image.format() == QImage::Format_RGB32)
	{
		// assume RGB (3 channels)
		int channels = 3;
		iplTmp = cvCreateImage(cvSize(image.width(), image.height()), IPL_DEPTH_8U, channels);
		char * data = iplTmp->imageData;
		for(int y = 0; y < image.height(); ++y, data+=iplTmp->widthStep)
		{
			for(int x = 0; x < image.width(); ++x)
			{
				QRgb rgb = image.pixel(x, y);
				data[x * channels+2] = qRed(rgb); //r
				data[x * channels+1] = qGreen(rgb); //g
				data[x * channels] = qBlue(rgb); //b
			}
		}
	}
	else
	{
		printf("Failed to convert image : depth=%d(!=32) format=%d(!=%d)\n", image.depth(), image.format(), QImage::Format_RGB32);
	}
	return iplTmp;
}

} // namespace find_object
