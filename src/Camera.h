/*
 * Camera.h
 *
 *  Created on: 2011-10-21
 *      Author: matlab
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

signals:
	void imageReceived(const cv::Mat & image);

public slots:
	void updateImageRate();

private slots:
	void takeImage();

private:
	CvCapture * capture_;
	QTimer cameraTimer_;
};

#endif /* CAMERA_H_ */
