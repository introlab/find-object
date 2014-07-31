/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines

#include <QtGui/QMainWindow>
#include <QtCore/QSet>
#include <QtCore/QTimer>
#include <QtCore/QTime>
#include <QtCore/QMap>
#include <QtCore/QByteArray>

#include <opencv2/opencv.hpp>

class Ui_mainWindow;
class ObjWidget;
class Camera;
class ParametersToolBox;
class QLabel;
class AboutDialog;
class TcpServer;
class KeypointDetector;
class DescriptorExtractor;
class Vocabulary;
class FindObject;

namespace rtabmap
{
class PdfPlotCurve;
}

class FINDOBJECT_EXP MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(FindObject * findObject, Camera * camera = 0, QWidget * parent = 0);
	virtual ~MainWindow();

	void setSourceImageText(const QString & text);

protected:
	virtual void closeEvent(QCloseEvent * event);

public Q_SLOTS:
	void startProcessing();
	void stopProcessing();
	void pauseProcessing();

private Q_SLOTS:
	void loadSettings();
	void saveSettings();
	void loadObjects();
	bool saveObjects();
	void addObjectFromScene();
	void addObjectsFromFiles();
	void loadSceneFromFile();
	void setupCameraFromVideoFile();
	void setupCameraFromImagesDirectory();
	void setupCameraFromTcpIp();
	void removeObject(ObjWidget * object);
	void removeAllObjects();
	void updateObjectsSize();
	void updateMirrorView();
	void showHideControls();
	void update(const cv::Mat & image = cv::Mat());
	void updateObjects();
	void notifyParametersChanged(const QStringList & param);
	void moveCameraFrame(int frame);
	void rectHovered(int objId);

Q_SIGNALS:
	void objectsFound(const QMultiMap<int, QPair<QRect, QTransform> > &);

private:
	bool loadSettings(const QString & path);
	bool saveSettings(const QString & path);
	int loadObjects(const QString & dirPath);
	int saveObjects(const QString & dirPath);
	void setupTCPServer();
	bool addObjectFromFile(const QString & filePath);
	void showObject(ObjWidget * obj);
	void updateObjectSize(ObjWidget * obj);
	void updateVocabulary();

private:
	Ui_mainWindow * ui_;
	Camera * camera_;
	FindObject * findObject_;
	rtabmap::PdfPlotCurve * likelihoodCurve_;
	rtabmap::PdfPlotCurve * inliersCurve_;
	AboutDialog * aboutDialog_;
	QMap<int, ObjWidget*> objWidgets_;
	QTime updateRate_;
	QTime refreshStartTime_;
	int lowestRefreshRate_;
	bool objectsModified_;
	QMap<int, QByteArray> imagesMap_;
	QMap<QString, QVariant> lastObjectsUpdateParameters_; // ParametersMap
	TcpServer * tcpServer_;
	cv::Mat sceneImage_;
};

#endif /* MainWindow_H_ */
