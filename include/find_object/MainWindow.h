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

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include "find_object/DetectionInfo.h"

#include <QMainWindow>
#include <QtCore/QSet>
#include <QtCore/QTimer>
#include <QtCore/QTime>
#include <QtCore/QMap>
#include <QtCore/QByteArray>

#include <opencv2/opencv.hpp>

namespace rtabmap
{
class PdfPlotCurve;
}

class Ui_mainWindow;
class QLabel;

namespace find_object {

class ObjWidget;
class Camera;
class ParametersToolBox;
class AboutDialog;
class TcpServer;
class KeypointDetector;
class DescriptorExtractor;
class Vocabulary;
class FindObject;

class FINDOBJECT_EXP MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(find_object::FindObject * findObject, find_object::Camera * camera = 0, QWidget * parent = 0);
	virtual ~MainWindow();

	void setSourceImageText(const QString & text);

protected:
	virtual void closeEvent(QCloseEvent * event);
	virtual void keyPressEvent(QKeyEvent *event);

public Q_SLOTS:
	void startProcessing();
	void stopProcessing();
	void pauseProcessing();
	void update(const cv::Mat & image);

private Q_SLOTS:
	void loadSession();
	void saveSession();
	void loadSettings();
	void saveSettings();
	void loadObjects();
	bool saveObjects();
	void loadVocabulary();
	void saveVocabulary();
	void addObjectFromScene();
	void addObjectsFromFiles(const QStringList & fileNames);
	void addObjectsFromFiles();
	void addObjectFromTcp(const cv::Mat & image, int id, const QString & filePath);
	void loadSceneFromFile(const QStringList & fileNames);
	void loadSceneFromFile();
	void setupCameraFromVideoFile();
	void setupCameraFromImagesDirectory();
	void setupCameraFromTcpIp();
	void removeObject(find_object::ObjWidget * object);
	void removeObject(int id);
	void removeAllObjects();
	void updateObjectsSize();
	void updateMirrorView();
	void showHideControls();
	void showObjectsFeatures();
	void hideObjectsFeatures();
	void updateObjects();
	void notifyParametersChanged(const QStringList & param);
	void moveCameraFrame(int frame);
	void rectHovered(int objId);

Q_SIGNALS:
	void objectsFound(const find_object::DetectionInfo &);

private:
	bool loadSettings(const QString & path);
	bool saveSettings(const QString & path) const;
	int loadObjects(const QString & dirPath, bool recursive = false);
	int saveObjects(const QString & dirPath);
	void setupTCPServer();
	int addObjectFromFile(const QString & filePath);
	void showObject(find_object::ObjWidget * obj);
	void updateObjectSize(find_object::ObjWidget * obj);
	void updateVocabulary(const QList<int> & ids = QList<int>());
	void updateObjects(const QList<int> & ids);

private:
	Ui_mainWindow * ui_;
	Camera * camera_;
	FindObject * findObject_;
	rtabmap::PdfPlotCurve * likelihoodCurve_;
	rtabmap::PdfPlotCurve * inliersCurve_;
	AboutDialog * aboutDialog_;
	QMap<int, find_object::ObjWidget*> objWidgets_;
	QTime updateRate_;
	QTime refreshStartTime_;
	int lowestRefreshRate_;
	bool objectsModified_;
	QMap<int, QByteArray> imagesMap_;
	QMap<QString, QVariant> lastObjectsUpdateParameters_; // ParametersMap
	TcpServer * tcpServer_;
	cv::Mat sceneImage_;
};

} // namespace find_object

#endif /* MainWindow_H_ */
