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
#include "Settings.h"

class Camera : public QObject {
	Q_OBJECT
public:
	Camera(int deviceId = Settings::defaultCamera_deviceId(),
			int width = Settings::defaultCamera_imageWidth(),
			int height = Settings::defaultCamera_imageHeight(),
			QObject * parent = 0);
	virtual ~Camera();

	void setDeviceId(int deviceId) {deviceId_=deviceId;}
	void setImageWidth(int imageWidth) {imageWidth_=imageWidth;}
	void setImageHeight(int imageHeight) {imageHeight_=imageHeight;}

	int getDeviceId() const {return deviceId_;}
	int getImageWidth() const {return imageWidth_;}
	int getImageHeight() const {return imageHeight_;}

signals:
	void ready();

public:
	bool init();
	void close();
	IplImage * takeImage();

private:
	CvCapture * capture_;
	int deviceId_;
	int imageWidth_;
	int imageHeight_;
};

#endif /* CAMERA_H_ */
