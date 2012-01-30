/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <opencv2/highgui/highgui.hpp>
#include <QtCore/QObject>
#include <QtCore/QTimer>

class Camera : public QObject {
	Q_OBJECT
public:
	Camera(QObject * parent = 0);
	virtual ~Camera();

	virtual bool start();
	virtual void stop();
	virtual bool isRunning() {return cameraTimer_.isActive();}

	void pause();

signals:
	void imageReceived(const cv::Mat & image);

public slots:
	virtual void updateImageRate();

private slots:
	virtual void takeImage();

protected:
	void startTimer();
	void stopTimer();

private:
	CvCapture * capture_;
	QTimer cameraTimer_;
};

#endif /* CAMERA_H_ */
