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

#ifndef CAMERA_H_
#define CAMERA_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtGui/QImage>

namespace find_object {

class CameraTcpServer;

class FINDOBJECT_EXP Camera : public QObject {
	Q_OBJECT
public:
	Camera(QObject * parent = 0);
	virtual ~Camera();

	virtual bool start();
	virtual void stop();
	virtual bool isRunning() {return cameraTimer_.isActive();}

	void pause();
	int getTotalFrames();
	int getCurrentFrameIndex();
	int getPort();
	void moveToFrame(int frame);

Q_SIGNALS:
	void imageReceived(const cv::Mat & image);
	void finished();

public Q_SLOTS:
	virtual void updateImageRate();
	virtual void takeImage();

protected:
	void startTimer();
	void stopTimer();

private:
	cv::VideoCapture capture_;
	QTimer cameraTimer_;
	QList<std::string> images_;
	unsigned int currentImageIndex_;
	CameraTcpServer * cameraTcpServer_;
};

} // namespace find_object

#endif /* CAMERA_H_ */
