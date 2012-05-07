/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef ADDOBJECTDIALOG_H_
#define ADDOBJECTDIALOG_H_

#include <QtGui/QDialog>
#include <QtCore/QTimer>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>

class Ui_addObjectDialog;
class ObjWidget;
class Camera;
class KeypointItem;

class AddObjectDialog : public QDialog {

	Q_OBJECT

public:
	AddObjectDialog(Camera * camera, const cv::Mat & image, bool mirrorView, QWidget * parent = 0, Qt::WindowFlags f = 0);
	virtual ~AddObjectDialog();

	// ownership transferred to caller
	ObjWidget * retrieveObject() {ObjWidget * obj = object_; object_=0; return obj;}

private slots:
	void update(const cv::Mat &);
	void next();
	void back();
	void cancel();
	void takePicture();
	void updateNextButton();
	void updateNextButton(const QRect &);
	void changeSelectionMode();

protected:
	virtual void closeEvent(QCloseEvent* event);

private:
	void setState(int state);
	cv::Rect computeROI(const std::vector<cv::KeyPoint> & kpts);
private:
	Ui_addObjectDialog * ui_;
	Camera * camera_;
	ObjWidget * object_;
	cv::Mat cvImage_;

	enum State{kTakePicture, kSelectFeatures, kVerifySelection, kClosing};
	int state_;
	QRect roi_;
};

#endif /* ADDOBJECTDIALOG_H_ */
