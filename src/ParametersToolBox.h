/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef PARAMETERSTOOLBOX_H_
#define PARAMETERSTOOLBOX_H_

#include <QtGui/QToolBox>

class QVBoxLayout;
class QAbstractButton;

class ParametersToolBox: public QToolBox
{
	Q_OBJECT

public:
	ParametersToolBox(QWidget *parent = 0);
	virtual ~ParametersToolBox();

	void setupUi();
	QWidget * getParameterWidget(const QString & key);
	void updateParameter(const QString & key);

private:
	void addParameter(QVBoxLayout * layout, const QString & key, const QVariant & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const QString & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const int & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const double & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const bool & value);
	void addParameter(QVBoxLayout * layout, const QString & name, QWidget * widget);

signals:
	void parametersChanged(const QStringList & name);

private slots:
	void changeParameter();
	void changeParameter(const QString & value);
	void changeParameter(const int & value);
	void resetCurrentPage();
	void resetAllPages();

private:
	QStringList resetPage(int index);
};

#endif /* PARAMETERSTOOLBOX_H_ */
