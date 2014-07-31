/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include <opencv2/highgui/highgui.hpp>
#include <QtCore/QObject>
#include <QtCore/QTimer>
#include <QtGui/QImage>

class CameraTcpClient;

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
	void moveToFrame(int frame);

Q_SIGNALS:
	void imageReceived(const cv::Mat & image);

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
	CameraTcpClient * cameraTcpClient_;
};

#endif /* CAMERA_H_ */
