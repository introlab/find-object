/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "QtOpenCV.h"
#include <opencv2/core/core_c.h>
#include <stdio.h>

QImage cvtCvMat2QImage(const cv::Mat & image)
{
	QImage qtemp;
	if(!image.empty() && image.depth() == CV_8U)
	{
		const unsigned char * data = image.data;
		qtemp = QImage(image.cols, image.rows, QImage::Format_RGB32);
		for(int y = 0; y < image.rows; ++y, data += image.cols*image.elemSize())
		{
			for(int x = 0; x < image.cols; ++x)
			{
				QRgb * p = ((QRgb*)qtemp.scanLine (y)) + x;
				*p = qRgb(data[x * image.channels()+2], data[x * image.channels()+1], data[x * image.channels()]);
			}
		}
	}
	else if(!image.empty() && image.depth() != CV_8U)
	{
		printf("Wrong image format, must be 8_bits\n");
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
		const IplImage test = cvImage;
		printf("%d vs %d\n", cvImage.cols*int(cvImage.elemSize()), test.widthStep);
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
