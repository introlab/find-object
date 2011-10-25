
#include "qtipl.h"
#include <opencv2/core/core_c.h>

// TODO : support only from gray 8bits ?
QImage Ipl2QImage(const IplImage *newImage)
{
	QImage qtemp;
	if (newImage && newImage->depth == IPL_DEPTH_8U && cvGetSize(newImage).width>0)
	{
		int x;
		int y;
		char* data = newImage->imageData;
		
		qtemp= QImage(newImage->width, newImage->height,QImage::Format_RGB32 );
		for( y = 0; y < newImage->height; y++, data +=newImage->widthStep )
		{
			for( x = 0; x < newImage->width; x++)
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
