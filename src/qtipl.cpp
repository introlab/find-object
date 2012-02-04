/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "qtipl.h"
#include <opencv2/core/core_c.h>
#include <stdio.h>

// TODO : support only from gray 8bits ?
QImage Ipl2QImage(const IplImage *newImage)
{
	QImage qtemp;
	if (newImage && newImage->depth == IPL_DEPTH_8U && cvGetSize(newImage).width>0)
	{
		int x;
		int y;
		char* data = newImage->imageData;
		
		qtemp= QImage(newImage->width, newImage->height,QImage::Format_RGB32);
		for( y = 0; y < newImage->height; ++y, data +=newImage->widthStep )
		{
			for( x = 0; x < newImage->width; ++x)
			{
				uint *p = (uint*)qtemp.scanLine (y) + x;
				*p = qRgb(data[x * newImage->nChannels+2], data[x * newImage->nChannels+1],data[x * newImage->nChannels]);
			}
		}
	}
	else
	{
		//Wrong IplImage format
	}
 return qtemp;	
}

IplImage * QImage2Ipl(const QImage & image)
{
	IplImage * iplTmp = 0;
	if(!image.isNull() && image.depth() == 32 && image.format() == QImage::Format_RGB32)
	{
		int x;
		int y;

		// assume RGB (3 channels)
		int channels = 3;
		iplTmp = cvCreateImage(cvSize(image.width(), image.height()), IPL_DEPTH_8U, channels);
		char* data = iplTmp->imageData;
		for( y = 0; y < image.height(); ++y, data+=iplTmp->widthStep)
		{
			for( x = 0; x < image.width(); ++x)
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
		printf("failed to convert image : depth=%d(!=32) format=%d(!=%d)\n", image.depth(), image.format(), QImage::Format_RGB32);
	}
	return iplTmp;
}
