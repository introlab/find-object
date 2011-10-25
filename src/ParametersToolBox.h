/*
 * ParametersToolbox.h
 *
 *  Created on: 2011-10-22
 *      Author: matlab
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

private:
	void addParameter(QVBoxLayout * layout, const QString & key, const QVariant & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const QString & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const int & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const double & value);
	void addParameter(QVBoxLayout * layout, const QString & key, const bool & value);
	void addParameter(QVBoxLayout * layout, const QString & name, QWidget * widget);

private slots:
	void changeParameter();
	void changeParameter(const QString & value);
	void changeParameter(const int & value);
	void resetCurrentPage();

};

#endif /* PARAMETERSTOOLBOX_H_ */
