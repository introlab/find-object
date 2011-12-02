/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef QTIPL_H
#define QTIPL_H

#include <QtGui/QImage>
#include <opencv2/core/core.hpp>

QImage Ipl2QImage(const IplImage *newImage);

#endif
