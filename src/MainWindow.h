

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

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow(Camera * camera = 0, QWidget * parent = 0);
	virtual ~MainWindow();

protected:
	virtual void closeEvent(QCloseEvent * event);

private slots:
	void addObject();
	void startCamera();
	void stopCamera();
	void loadObjects();
	void saveObjects();
	void update(const cv::Mat & image);
	void updateData();
	void removeObject(ObjWidget * object);

private:
	void showObject(ObjWidget * obj);

private:
	Ui_mainWindow * ui_;
	Camera * camera_;
	QList<ObjWidget*> objects_;
	cv::Mat dataTree_;
	QList<int> dataRange_;
	QTime updateRate_;

};

#endif /* MainWindow_H_ */
