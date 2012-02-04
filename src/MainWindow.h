/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef MAINWINDOW_H_
#define MAINWINDOW_H_

#include <QtGui/QMainWindow>
#include <QtCore/QSet>
#include <QtCore/QTimer>
#include <QtCore/QTime>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc_c.h>

class Ui_mainWindow;
class ObjWidget;
class Camera;
class ParametersToolBox;
class QLabel;
class AboutDialog;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(Camera * camera = 0, QWidget * parent = 0);
	virtual ~MainWindow();

	int loadObjects(const QString & dirPath);
	void saveObjects(const QString & dirPath);

	ParametersToolBox * parametersToolBox() const;
	void setSourceImageText(const QString & text);

protected:
	virtual void closeEvent(QCloseEvent * event);

public slots:
	void startProcessing();
	void stopProcessing();
	void pauseProcessing();

private slots:
	void loadObjects();
	bool saveObjects();
	void addObjectFromScene();
	void addObjectsFromFiles();
	void loadSceneFromFile();
	void setupCameraFromVideoFile();
	void removeObject(ObjWidget * object);
	void removeAllObjects();
	void update(const cv::Mat & image);
	void updateObjects();
	void notifyParametersChanged();

signals:
	void objectsFound(const QMap<int, QPair<QRect, QTransform> > &);

private:
	void addObjectFromFile(const QString & filePath);
	void showObject(ObjWidget * obj);
	void updateData();

private:
	Ui_mainWindow * ui_;
	Camera * camera_;
	AboutDialog * aboutDialog_;
	QList<ObjWidget*> objects_;
	cv::Mat dataTree_;
	QList<int> dataRange_;
	QTime updateRate_;
	QTime refreshStartTime_;
	int lowestRefreshRate_;
	bool objectsModified_;
};

#endif /* MainWindow_H_ */
