/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef QTOPENCV_H
#define QTOPENCV_H

#include <QtGui/QImage>
#include <opencv2/core/core.hpp>

// Convert OpenCV matrix to QImage
QImage cvtCvMat2QImage(const cv::Mat & image);

// Convert QImage to OpenCV matrix
cv::Mat cvtQImage2CvMat(const QImage & image);

// Convert IplImage to QImage
QImage cvtIplImage2QImage(const IplImage * image);

// Convert QImage to IplImage
IplImage * cvtQImage2IplImage(const QImage & image);

#endif // QTOPENCV_H
