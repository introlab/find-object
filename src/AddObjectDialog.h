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

#ifndef ADDOBJECTDIALOG_H_
#define ADDOBJECTDIALOG_H_

#include <QtGui/QDialog>
#include <QtCore/QTimer>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>

class Ui_addObjectDialog;

namespace find_object {

class ObjWidget;
class Camera;
class KeypointItem;
class KeypointDetector;
class DescriptorExtractor;
class ObjSignature;

class AddObjectDialog : public QDialog {

	Q_OBJECT

public:
	AddObjectDialog(Camera * camera, const cv::Mat & image, bool mirrorView, QWidget * parent = 0, Qt::WindowFlags f = 0);
	virtual ~AddObjectDialog();

	// ownership transferred to caller
	void retrieveObject(ObjWidget ** widget, ObjSignature ** signature);

private Q_SLOTS:
	void update(const cv::Mat &);
	void next();
	void back();
	void cancel();
	void takePicture();
	void updateNextButton();
	void updateNextButton(const cv::Rect &);
	void changeSelectionMode();

protected:
	virtual void closeEvent(QCloseEvent* event);

private:
	void setState(int state);
	cv::Rect computeROI(const std::vector<cv::KeyPoint> & kpts);
private:
	Ui_addObjectDialog * ui_;
	Camera * camera_;
	ObjWidget * objWidget_;
	ObjSignature * objSignature_;
	cv::Mat cameraImage_;
	cv::Rect roi_;
	KeypointDetector * detector_;
	DescriptorExtractor * extractor_;

	enum State{kTakePicture, kSelectFeatures, kVerifySelection, kClosing};
	int state_;
};

} // namespace find_object

#endif /* ADDOBJECTDIALOG_H_ */
