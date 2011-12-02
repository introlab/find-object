/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef ADDOBJECTDIALOG_H_
#define ADDOBJECTDIALOG_H_

#include <QtGui/QDialog>
#include <QtCore/QTimer>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core_c.h>

class Ui_addObjectDialog;
class ObjWidget;
class Camera;
class KeypointItem;

class AddObjectDialog : public QDialog {

	Q_OBJECT

public:
	AddObjectDialog(Camera * camera, QList<ObjWidget*> * objects, QWidget * parent = 0, Qt::WindowFlags f = 0);
	virtual ~AddObjectDialog();

private slots:
	void update(const cv::Mat &);
	void next();
	void back();
	void cancel();
	void takePicture();
	void updateNextButton();

protected:
	virtual void closeEvent(QCloseEvent* event);

private:
	void setState(int state);
	CvRect computeROI(const std::vector<cv::KeyPoint> & kpts);
private:
	Ui_addObjectDialog * ui_;
	Camera * camera_;
	QList<ObjWidget*> * objects_;
	IplImage * cvImage_;

	enum State{kTakePicture, kSelectFeatures, kVerifySelection, kClosing};
	int state_;
};

#endif /* ADDOBJECTDIALOG_H_ */
