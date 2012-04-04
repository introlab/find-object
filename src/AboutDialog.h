/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#ifndef ABOUTDIALOG_H_
#define ABOUTDIALOG_H_

#include <QtGui/QDialog>
#include <QtCore/QUrl>

class Ui_aboutDialog;

class AboutDialog : public QDialog
{
	Q_OBJECT

public:
	AboutDialog(QWidget * parent = 0);

	virtual ~AboutDialog();

private:
	Ui_aboutDialog * ui_;
};


#endif /* ABOUTDIALOG_H_ */
