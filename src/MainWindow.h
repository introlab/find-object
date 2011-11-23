

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

	void loadObjects(const QString & fileName);
	void saveObjects(const QString & fileName);

protected:
	virtual void closeEvent(QCloseEvent * event);

public slots:
	void startProcessing();
	void stopProcessing();

private slots:
	void loadObjects();
	void saveObjects();
	void addObject();
	void removeObject(ObjWidget * object);
	void update(const cv::Mat & image);

signals:
	void objectsFound(const QMap<int, QPoint> &);

private:
	void showObject(ObjWidget * obj);
	void updateData();

private:
	Ui_mainWindow * ui_;
	Camera * camera_;
	QList<ObjWidget*> objects_;
	cv::Mat dataTree_;
	QList<int> dataRange_;
	QTime updateRate_;
	QTime refreshStartTime_;
	int lowestRefreshRate_;

};

#endif /* MainWindow_H_ */
