/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "AboutDialog.h"
#include "ui_aboutDialog.h"

AboutDialog::AboutDialog(QWidget * parent) :
	QDialog(parent)
{
	_ui = new Ui_aboutDialog();
	_ui->setupUi(this);
	_ui->label_version->setText(PROJECT_VERSION);
}

AboutDialog::~AboutDialog()
{
	delete _ui;
}

