/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "AboutDialog.h"
#include "ui_aboutDialog.h"
#include <opencv2/core/version.hpp>

AboutDialog::AboutDialog(QWidget * parent) :
	QDialog(parent)
{
	ui_ = new Ui_aboutDialog();
	ui_->setupUi(this);
	ui_->label_version->setText(PROJECT_VERSION);
	ui_->label_version_opencv->setText(CV_VERSION);
}

AboutDialog::~AboutDialog()
{
	delete ui_;
}

