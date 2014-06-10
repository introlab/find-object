/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <QtGui/QMainWindow>
#include <QtCore/QSet>
#include <QtCore/QTimer>
#include <QtCore/QTime>
#include <QtCore/QMap>
#include <QtCore/QByteArray>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include "Vocabulary.h"

class Ui_mainWindow;
class ObjWidget;
class Camera;
class ParametersToolBox;
class QLabel;
class AboutDialog;
class TcpServer;

namespace rtabmap
{
class PdfPlotCurve;
}

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(Camera * camera = 0, const QString & settings = "", QWidget * parent = 0);
	virtual ~MainWindow();

	bool loadSettings(const QString & path);
	bool saveSettings(const QString & path);

	int loadObjects(const QString & dirPath);
	void saveObjects(const QString & dirPath);

	void setSourceImageText(const QString & text);

protected:
	virtual void closeEvent(QCloseEvent * event);

public slots:
	void startProcessing();
	void stopProcessing();
	void pauseProcessing();

private slots:
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
	void update(const cv::Mat & image);
	void updateObjects();
	void notifyParametersChanged(const QStringList & param);
	void moveCameraFrame(int frame);
	void rectHovered(int objId);

signals:
	void objectsFound(const QMultiMap<int, QPair<QRect, QTransform> > &);

private:
	void setupTCPServer();
	void addObjectFromFile(const QString & filePath);
	void showObject(ObjWidget * obj);
	void updateData();
	void updateObjectSize(ObjWidget * obj);

private:
	Ui_mainWindow * ui_;
	Camera * camera_;
	QString settings_;
	rtabmap::PdfPlotCurve * likelihoodCurve_;
	rtabmap::PdfPlotCurve * inliersCurve_;
	AboutDialog * aboutDialog_;
	QList<ObjWidget*> objects_;
	std::vector<cv::Mat> objectsDescriptors_;
	Vocabulary vocabulary_;
	QMap<int, int> dataRange_; // <last id of object's descriptor, id>
	QTime updateRate_;
	QTime refreshStartTime_;
	int lowestRefreshRate_;
	bool objectsModified_;
	QMap<int, QByteArray> imagesMap_;
	TcpServer * tcpServer_;
	QMap<QString, QVariant> lastObjectsUpdateParameters_; // ParametersMap
};

#endif /* MainWindow_H_ */
