/*
Copyright (c) 2011-2014, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "find_object/Settings.h"

#include "ParametersToolBox.h"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QLabel>
#include <QGroupBox>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QMessageBox>
#include <stdio.h>
#include "find_object/utilite/ULogger.h"
#include <opencv2/opencv_modules.hpp>

namespace find_object {

ParametersToolBox::ParametersToolBox(QWidget *parent) :
	QToolBox(parent)
{
}

ParametersToolBox::~ParametersToolBox()
{
}

QWidget * ParametersToolBox::getParameterWidget(const QString & key)
{
	return this->findChild<QWidget*>(key);
}

QStringList ParametersToolBox::resetPage(int index)
{
	QStringList paramChanged;
	const QObjectList & children = this->widget(index)->children();
	for(int j=0; j<children.size();++j)
	{
		QString key = children.at(j)->objectName();
		// ignore only the nextObjID setting, to avoid problem with saved objects
		if(key.compare(Settings::kGeneral_nextObjID()) != 0)
		{
			QVariant value = Settings::getDefaultParameters().value(key, QVariant());
			if(value.isValid())
			{
				Settings::setParameter(key, value);

				if(qobject_cast<QComboBox*>(children.at(j)))
				{
					if(((QComboBox*)children.at(j))->currentIndex() != value.toString().split(':').first().toInt())
					{
						((QComboBox*)children.at(j))->setCurrentIndex(value.toString().split(':').first().toInt());
						paramChanged.append(key);
					}
				}
				else if(qobject_cast<QSpinBox*>(children.at(j)))
				{
					if(((QSpinBox*)children.at(j))->value() != value.toInt())
					{
						((QSpinBox*)children.at(j))->setValue(value.toInt());
						paramChanged.append(key);
					}
				}
				else if(qobject_cast<QDoubleSpinBox*>(children.at(j)))
				{
					if(((QDoubleSpinBox*)children.at(j))->value() != value.toDouble())
					{
						((QDoubleSpinBox*)children.at(j))->setValue(value.toDouble());
						paramChanged.append(key);
					}
				}
				else if(qobject_cast<QCheckBox*>(children.at(j)))
				{
					if(((QCheckBox*)children.at(j))->isChecked() != value.toBool())
					{
						((QCheckBox*)children.at(j))->setChecked(value.toBool());
						paramChanged.append(key);
					}
				}
				else if(qobject_cast<QLineEdit*>(children.at(j)))
				{
					if(((QLineEdit*)children.at(j))->text().compare(value.toString()) != 0)
					{
						((QLineEdit*)children.at(j))->setText(value.toString());
						paramChanged.append(key);
					}
				}
			}
		}
	}
	return paramChanged;
}

void ParametersToolBox::resetCurrentPage()
{
	this->blockSignals(true);
	QStringList paramChanged = this->resetPage(this->currentIndex());
	this->blockSignals(false);
	Q_EMIT parametersChanged(paramChanged);
}

void ParametersToolBox::resetAllPages()
{
	QStringList paramChanged;
	this->blockSignals(true);
	for(int i=0; i< this->count(); ++i)
	{
		paramChanged.append(this->resetPage(i));
	}
	this->blockSignals(false);
	Q_EMIT parametersChanged(paramChanged);
}

void ParametersToolBox::updateParametersVisibility()
{
	//show/hide not used parameters
	QComboBox * descriptorBox = this->findChild<QComboBox*>(Settings::kFeature2D_2Descriptor());
	QComboBox * detectorBox = this->findChild<QComboBox*>(Settings::kFeature2D_1Detector());
	if(descriptorBox && detectorBox)
	{
		QString group = Settings::kFeature2D_2Descriptor().split('/').first();
		QWidget * panel = 0;
		for(int i=0; i<this->count(); ++i)
		{
			if(this->widget(i)->objectName().compare(group) == 0)
			{
				panel = this->widget(i);
				break;
			}
		}
		if(panel)
		{
			const QObjectList & objects = panel->children();
			QString descriptorName = descriptorBox->currentText();
			QString detectorName = detectorBox->currentText();

			for(int i=0; i<objects.size(); ++i)
			{
				if(!objects[i]->objectName().isEmpty())
				{
					if(objects[i]->objectName().contains(descriptorName) || objects[i]->objectName().contains(detectorName))
					{
						((QWidget*)objects[i])->setVisible(true);
					}
					else if(objects[i]->objectName().contains("Fast") && detectorName == QString("ORB"))
					{
						((QWidget*)objects[i])->setVisible(true);	// ORB uses some FAST parameters
					}
					else if(!objects[i]->objectName().split('/').at(1).at(0).isDigit())
					{
						((QWidget*)objects[i])->setVisible(false);
					}
				}
			}
		}
	}

	QComboBox * nnBox = this->findChild<QComboBox*>(Settings::kNearestNeighbor_1Strategy());
	if(nnBox)
	{
		QString group = Settings::kNearestNeighbor_1Strategy().split('/').first();
		QWidget * panel = 0;
		for(int i=0; i<this->count(); ++i)
		{
			if(this->widget(i)->objectName().compare(group) == 0)
			{
				panel = this->widget(i);
				break;
			}
		}
		if(panel)
		{
			const QObjectList & objects = panel->children();
			QString nnName = nnBox->currentText();

			for(int i=0; i<objects.size(); ++i)
			{
				if(!objects[i]->objectName().isEmpty())
				{
					if(objects[i]->objectName().contains(nnName))
					{
						((QWidget*)objects[i])->setVisible(true);
					}
					else if(!objects[i]->objectName().split('/').at(1).at(0).isDigit())
					{
						((QWidget*)objects[i])->setVisible(false);
						if(nnBox->currentIndex() < 6 && objects[i]->objectName().split('/').at(1).contains("search"))
						{
							//show flann search parameters
							((QWidget*)objects[i])->setVisible(true);
						}
					}
					else if(objects[i]->objectName().split('/').at(1).contains("Distance_type"))
					{
						// don't show distance when bruteforce is selected
						((QWidget*)objects[i])->setVisible(nnBox->currentIndex() != 6);
					}
				}
			}
		}
	}
}

void ParametersToolBox::setupUi()
{
	this->removeItem(0); // remove dummy page used in .ui
	QWidget * currentItem = 0;
	const ParametersMap & parameters = Settings::getParameters();
	for(ParametersMap::const_iterator iter=parameters.constBegin();
			iter!=parameters.constEnd();
			++iter)
	{
		QStringList splitted = iter.key().split('/');
		QString group = splitted.first();
		QString name = splitted.last();
		if(currentItem == 0 || currentItem->objectName().compare(group) != 0)
		{
			currentItem = new QWidget(this);
			this->addItem(currentItem, group);
			currentItem->setObjectName(group);
			QVBoxLayout * layout = new QVBoxLayout(currentItem);
			currentItem->setLayout(layout);
			layout->setContentsMargins(0,0,0,0);
			layout->setSpacing(0);
			layout->addSpacerItem(new QSpacerItem(0,0, QSizePolicy::Minimum, QSizePolicy::Expanding));

			addParameter(layout, iter.key(), iter.value());
		}
		else
		{
			addParameter((QVBoxLayout*)currentItem->layout(), iter.key(), iter.value());
		}
	}

	updateParametersVisibility();
}

void ParametersToolBox::updateParameter(const QString & key)
{
	QWidget * widget = this->findChild<QWidget*>(key);
	QString type = Settings::getParametersType().value(key);
	if(type.compare("QString") == 0)
	{
		QString value = Settings::getParameter(key).toString();
		if(value.contains(';'))
		{
			// It's a list, just change the index
			QStringList splitted = value.split(':');
			((QComboBox*)widget)->setCurrentIndex(splitted.first().toInt());
		}
		else
		{
			((QLineEdit*)widget)->setText(value);
		}
	}
	else if(type.compare("int") == 0)
	{
		((QSpinBox*)widget)->setValue(Settings::getParameter(key).toInt());
	}
	else if(type.compare("uint") == 0)
	{
		((QSpinBox*)widget)->setValue(Settings::getParameter(key).toInt());
	}
	else if(type.compare("double") == 0)
	{
		((QDoubleSpinBox*)widget)->setValue(Settings::getParameter(key).toDouble());
	}
	else if(type.compare("float") == 0)
	{
		((QDoubleSpinBox*)widget)->setValue(Settings::getParameter(key).toDouble());
	}
	else if(type.compare("bool") == 0)
	{
		((QCheckBox*)widget)->setChecked(Settings::getParameter(key).toBool());
	}
}

void ParametersToolBox::addParameter(QVBoxLayout * layout,
		const QString & key,
		const QVariant & value)
{
	QString type = Settings::getParametersType().value(key);
	if(type.compare("QString") == 0)
	{
		addParameter(layout, key, value.toString());
	}
	else if(type.compare("int") == 0)
	{
		addParameter(layout, key, value.toInt());
	}
	else if(type.compare("uint") == 0)
	{
		addParameter(layout, key, value.toInt());
	}
	else if(type.compare("double") == 0)
	{
		addParameter(layout, key, value.toDouble());
	}
	else if(type.compare("float") == 0)
	{
		addParameter(layout, key, value.toDouble());
	}
	else if(type.compare("bool") == 0)
	{
		addParameter(layout, key, value.toBool());
	}
}

void ParametersToolBox::addParameter(QVBoxLayout * layout,
		const QString & key,
		const QString & value)
{
	if(value.contains(';'))
	{
		QComboBox * widget = new QComboBox(this);
		widget->setObjectName(key);
		QStringList splitted = value.split(':');
		widget->addItems(splitted.last().split(';'));

		if(key.compare(Settings::kFeature2D_1Detector()) == 0)
		{
#if FINDOBJECT_NONFREE == 0
			widget->setItemData(5, 0, Qt::UserRole - 1); // disable SIFT
			widget->setItemData(7, 0, Qt::UserRole - 1); // disable SURF
#endif
#if CV_MAJOR_VERSION < 3
			widget->setItemData(9, 0, Qt::UserRole - 1); // disable AGAST
			widget->setItemData(10, 0, Qt::UserRole - 1); // disable KAZE
			widget->setItemData(11, 0, Qt::UserRole - 1); // disable AKAZE
#else
			widget->setItemData(0, 0, Qt::UserRole - 1); // disable Dense
#ifndef HAVE_OPENCV_XFEATURES2D
			widget->setItemData(6, 0, Qt::UserRole - 1); // disable Star
#endif
#endif
		}
		if(key.compare(Settings::kFeature2D_2Descriptor()) == 0)
		{
#if FINDOBJECT_NONFREE == 0
			widget->setItemData(2, 0, Qt::UserRole - 1); // disable SIFT
			widget->setItemData(3, 0, Qt::UserRole - 1); // disable SURF
#endif
#if CV_MAJOR_VERSION < 3
			widget->setItemData(6, 0, Qt::UserRole - 1); // disable KAZE
			widget->setItemData(7, 0, Qt::UserRole - 1); // disable AKAZE
			widget->setItemData(8, 0, Qt::UserRole - 1); // disable LUCID
			widget->setItemData(9, 0, Qt::UserRole - 1); // disable LATCH
			widget->setItemData(10, 0, Qt::UserRole - 1); // disable DAISY
#else

#ifndef HAVE_OPENCV_XFEATURES2D
			widget->setItemData(0, 0, Qt::UserRole - 1); // disable Brief
			widget->setItemData(5, 0, Qt::UserRole - 1); // disable Freak
			widget->setItemData(8, 0, Qt::UserRole - 1); // disable LUCID
			widget->setItemData(9, 0, Qt::UserRole - 1); // disable LATCH
			widget->setItemData(10, 0, Qt::UserRole - 1); // disable DAISY
#endif
#endif
		}
		if(key.compare(Settings::kNearestNeighbor_1Strategy()) == 0)
		{
#if FINDOBJECT_NONFREE == 0 && CV_MAJOR_VERSION < 3
			// disable FLANN approaches (cannot be used with binary descriptors)
			widget->setItemData(0, 0, Qt::UserRole - 1);
			widget->setItemData(1, 0, Qt::UserRole - 1);
			widget->setItemData(2, 0, Qt::UserRole - 1);
			widget->setItemData(3, 0, Qt::UserRole - 1);
			widget->setItemData(4, 0, Qt::UserRole - 1);
#endif
		}
		if(key.compare(Settings::kHomography_method()) == 0)
		{
#if CV_MAJOR_VERSION < 3
			// disable RHO approach
			widget->setItemData(2, 0, Qt::UserRole - 1);
#endif
		}

		widget->setCurrentIndex(splitted.first().toInt());
		connect(widget, SIGNAL(currentIndexChanged(int)), this, SLOT(changeParameter(int)));
		addParameter(layout, key, widget);
	}
	else
	{
		QLineEdit * widget = new QLineEdit(value, this);
		widget->setObjectName(key);
		connect(widget, SIGNAL(editingFinished()), this, SLOT(changeParameter()));
		addParameter(layout, key, widget);
	}
}

void ParametersToolBox::addParameter(QVBoxLayout * layout,
		const QString & key,
		const double & value)
{
	QDoubleSpinBox * widget = new QDoubleSpinBox(this);
	int decimals = 0;
	int decimalValue = 0;

	QString str = QString::number(Settings::getDefaultParameters().value(key).toDouble());
	str.remove( QRegExp("0+$") );

	if(!str.isEmpty())
	{
		str.replace(',', '.');
		QStringList items = str.split('.');
		if(items.size() == 2)
		{
			decimals = items.back().length();
			decimalValue = items.back().toInt();
		}
	}

	double def = Settings::getDefaultParameters().value(key).toDouble();
	if(def<0.001 || (decimals >= 4 && decimalValue>0))
	{
		widget->setDecimals(5);
		widget->setSingleStep(0.0001);
	}
	else if(def<0.01 || (decimals >= 3 && decimalValue>0))
	{
		widget->setDecimals(4);
		widget->setSingleStep(0.001);
	}
	else if(def<0.1 || (decimals >= 2 && decimalValue>0))
	{
		widget->setDecimals(3);
		widget->setSingleStep(0.01);
	}
	else if(def<1.0 || (decimals >= 1 && decimalValue>0))
	{
		widget->setDecimals(2);
		widget->setSingleStep(0.1);
	}
	else
	{
		widget->setDecimals(1);
	}

	if(def>0.0)
	{
		widget->setMaximum(def*1000000.0);
	}
	else if(def==0.0)
	{
		widget->setMaximum(1000000.0);
	}
	else if(def<0.0)
	{
		widget->setMinimum(def*1000000.0);
		widget->setMaximum(0.0);
	}
	widget->setValue(value);
	widget->setObjectName(key);
	connect(widget, SIGNAL(editingFinished()), this, SLOT(changeParameter()));
	addParameter(layout, key, widget);
}

void ParametersToolBox::addParameter(QVBoxLayout * layout,
		const QString & key,
		const int & value)
{
	QSpinBox * widget = new QSpinBox(this);
	int def = Settings::getDefaultParameters().value(key).toInt();

	if(def>0)
	{
		widget->setMaximum(def*1000000);
	}
	else if(def == 0)
	{
		widget->setMaximum(1000000);
	}
	else if(def<0)
	{
		widget->setMinimum(def*1000000);
		widget->setMaximum(0);
	}
	widget->setValue(value);
	widget->setObjectName(key);
	connect(widget, SIGNAL(editingFinished()), this, SLOT(changeParameter()));
	addParameter(layout, key, widget);
}

void ParametersToolBox::addParameter(QVBoxLayout * layout,
		const QString & key,
		const bool & value)
{
	QCheckBox * widget = new QCheckBox(this);
	widget->setChecked(value);
	widget->setObjectName(key);
	connect(widget, SIGNAL(toggled(bool)), this, SLOT(changeParameter(bool)));
	addParameter(layout, key, widget);
}

void ParametersToolBox::addParameter(QVBoxLayout * layout, const QString & key, QWidget * widget)
{
	QHBoxLayout * hLayout = new QHBoxLayout();
	layout->insertLayout(layout->count()-1, hLayout);
	QString tmp = key.split('/').last();
	if(tmp.at(0).isDigit())
	{
		tmp.remove(0,1);
	}
	QLabel * label = new QLabel(tmp, this);
	label->setObjectName(key+"/label");
	label->setToolTip(QString("<FONT>%1</FONT>").arg(Settings::getDescriptions().value(key, "")));
	label->setTextInteractionFlags(Qt::TextSelectableByMouse);
	hLayout->addWidget(label);
	hLayout->addWidget(widget);
}

void ParametersToolBox::changeParameter(const QString & value)
{
	if(sender())
	{
		Settings::setParameter(sender()->objectName(), value);
		QStringList paramChanged;
		paramChanged.append(sender()->objectName());
		Q_EMIT parametersChanged(paramChanged);
	}
}
void ParametersToolBox::changeParameter()
{
	if(sender())
	{
		QDoubleSpinBox * doubleSpinBox = qobject_cast<QDoubleSpinBox*>(sender());
		QSpinBox * spinBox = qobject_cast<QSpinBox*>(sender());
		QLineEdit * lineEdit = qobject_cast<QLineEdit*>(sender());
		if(doubleSpinBox)
		{
			Settings::setParameter(sender()->objectName(), doubleSpinBox->value());
		}
		else if(spinBox)
		{
			if(spinBox->objectName().compare(Settings::kHomography_minimumInliers()) == 0 &&
			   spinBox->value() < 4)
			{
				Settings::setHomography_minimumInliers(4);
				spinBox->blockSignals(true);
				this->updateParameter(Settings::kHomography_minimumInliers());
				spinBox->blockSignals(false);
			}
			else
			{
				Settings::setParameter(sender()->objectName(), spinBox->value());
			}
		}
		else if(lineEdit)
		{
			Settings::setParameter(sender()->objectName(), lineEdit->text());
		}
		QStringList paramChanged;
		paramChanged.append(sender()->objectName());
		Q_EMIT parametersChanged(paramChanged);
	}
}

void ParametersToolBox::changeParameter(bool value)
{
	if(sender())
	{
		// Workaround as stateChanged(int) is not always emitted, using toggled(bool) instead
		changeParameter(sender(), value?Qt::Checked:Qt::Unchecked);
	}
}

void ParametersToolBox::changeParameter(int value)
{
	if(sender())
	{
		changeParameter(sender(), value);
	}
}

void ParametersToolBox::changeParameter(QObject * sender, int value)
{
	if(sender)
	{
		QStringList paramChanged;
		QComboBox * comboBox = qobject_cast<QComboBox*>(sender);
		QCheckBox * checkBox = qobject_cast<QCheckBox*>(sender);

		bool descriptorChanged = false;
		if(comboBox && comboBox->objectName().compare(Settings::kFeature2D_1Detector()) == 0)
		{
			QComboBox * descriptorBox = (QComboBox*)this->getParameterWidget(Settings::kFeature2D_2Descriptor());
			if(comboBox->objectName().compare(Settings::kFeature2D_1Detector()) == 0 &&
			   comboBox->currentText() != descriptorBox->currentText() &&
			   Settings::getFeature2D_2Descriptor().contains(comboBox->currentText()))
			{
				QMessageBox::StandardButton b = QMessageBox::question(this,
						tr("Use corresponding descriptor type?"),
						tr("Current selected detector type (\"%1\") has its own corresponding descriptor type.\n"
						   "Do you want to use its corresponding descriptor?")
						   .arg(comboBox->currentText()),
						   QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
				if(b == QMessageBox::Yes)
				{
					int index = descriptorBox->findText(comboBox->currentText());
					if(index >= 0)
					{
						QStringList tmp = Settings::getFeature2D_2Descriptor().split(':');
						UASSERT(tmp.size() == 2);
						QString newTmp = QString('0'+index)+":"+tmp.back();
						Settings::setFeature2D_2Descriptor(newTmp);
						descriptorBox->blockSignals(true);
						this->updateParameter(Settings::kFeature2D_2Descriptor());
						descriptorBox->blockSignals(false);
						paramChanged.append(Settings::kFeature2D_2Descriptor());
						descriptorChanged = true;
					}
					else
					{
						UERROR("Combo box detector type not found \"%s\"?!", comboBox->currentText().toStdString().c_str());
					}
				}
			}
		}

		bool nnStrategyChanged = false;
		//verify binary issue with nearest neighbor strategy
		if((comboBox && (comboBox->objectName().compare(Settings::kFeature2D_2Descriptor()) == 0 ||
						 comboBox->objectName().compare(Settings::kNearestNeighbor_1Strategy()) == 0)) ||
		   descriptorChanged ||
		   (checkBox && checkBox->objectName().compare(Settings::kNearestNeighbor_7ConvertBinToFloat()) == 0))
		{
			QComboBox * descriptorBox = (QComboBox*)this->getParameterWidget(Settings::kFeature2D_2Descriptor());
			QComboBox * nnBox = (QComboBox*)this->getParameterWidget(Settings::kNearestNeighbor_1Strategy());
			QComboBox * distBox = (QComboBox*)this->getParameterWidget(Settings::kNearestNeighbor_2Distance_type());
			QCheckBox * binToFloatCheckbox = (QCheckBox*)this->getParameterWidget(Settings::kNearestNeighbor_7ConvertBinToFloat());
			bool isBinaryDescriptor = descriptorBox->currentText().compare("ORB") == 0 ||
									  descriptorBox->currentText().compare("Brief") == 0 ||
									  descriptorBox->currentText().compare("BRISK") == 0 ||
									  descriptorBox->currentText().compare("FREAK") == 0 ||
									  descriptorBox->currentText().compare("AKAZE") == 0 ||
									  descriptorBox->currentText().compare("LATCH") == 0 ||
									  descriptorBox->currentText().compare("LUCID") == 0;
			bool binToFloat = binToFloatCheckbox->isChecked();
			if(isBinaryDescriptor && !binToFloat && nnBox->currentText().compare("Lsh") != 0 && nnBox->currentText().compare("BruteForce") != 0)
			{
				QMessageBox::warning(this,
						tr("Warning"),
						tr("Current selected descriptor type (\"%1\") is binary while nearest neighbor strategy is not (\"%2\").\n"
						   "Falling back to \"BruteForce\" nearest neighbor strategy with Hamming distance (by default).")
						   .arg(descriptorBox->currentText())
						   .arg(nnBox->currentText()));
				QString tmp = Settings::getNearestNeighbor_1Strategy();
				*tmp.begin() = '6'; // set BruteForce
				Settings::setNearestNeighbor_1Strategy(tmp);
				tmp = Settings::getNearestNeighbor_2Distance_type();
				*tmp.begin() = '8'; // set HAMMING
				Settings::setNearestNeighbor_2Distance_type(tmp);
				nnBox->blockSignals(true);
				distBox->blockSignals(true);
				this->updateParameter(Settings::kNearestNeighbor_1Strategy());
				this->updateParameter(Settings::kNearestNeighbor_2Distance_type());
				nnBox->blockSignals(false);
				distBox->blockSignals(false);
				if(sender == nnBox)
				{
					this->updateParametersVisibility();
					return;
				}
				nnStrategyChanged = true;
				paramChanged.append(Settings::kNearestNeighbor_1Strategy());
				paramChanged.append(Settings::kNearestNeighbor_2Distance_type());
			}
			else if((!isBinaryDescriptor || binToFloat) && nnBox->currentText().compare("Lsh") == 0)
			{
				if(binToFloat)
				{
					QMessageBox::warning(this,
							tr("Warning"),
							tr("Current selected descriptor type (\"%1\") is binary, but binary to float descriptors conversion is activated while nearest neighbor strategy is (\"%2\").\n"
							   "Disabling binary to float descriptors conversion and use Hamming distance (by default).")
							   .arg(descriptorBox->currentText())
							   .arg(nnBox->currentText()));

					binToFloatCheckbox->blockSignals(true);
					if(checkBox && checkBox->objectName().compare(Settings::kNearestNeighbor_7ConvertBinToFloat()) == 0)
					{
						Settings::setParameter(checkBox->objectName(), false);
						value = 0;
					}
					else
					{
						paramChanged.append(Settings::kNearestNeighbor_7ConvertBinToFloat());
					}
					this->updateParameter(Settings::kNearestNeighbor_7ConvertBinToFloat());
					binToFloatCheckbox->blockSignals(false);

					QString tmp = Settings::getNearestNeighbor_2Distance_type();
					*tmp.begin() = '8'; // set HAMMING
					Settings::setNearestNeighbor_2Distance_type(tmp);
					distBox->blockSignals(true);
					this->updateParameter(Settings::kNearestNeighbor_2Distance_type());
					distBox->blockSignals(false);
					paramChanged.append(Settings::kNearestNeighbor_2Distance_type());
				}
				else
				{
					QMessageBox::warning(this,
							tr("Warning"),
							tr("Current selected descriptor type (\"%1\") is not binary while nearest neighbor strategy is (\"%2\").\n"
							   "Falling back to \"KDTree\" nearest neighbor strategy with Euclidean_L2 distance (by default).")
							   .arg(descriptorBox->currentText())
							   .arg(nnBox->currentText()));

					QString tmp = Settings::getNearestNeighbor_1Strategy();
					*tmp.begin() = '1'; // set KDTree
					Settings::setNearestNeighbor_1Strategy(tmp);
					tmp = Settings::getNearestNeighbor_2Distance_type();
					*tmp.begin() = '0'; // set EUCLIDEAN_L2
					Settings::setNearestNeighbor_2Distance_type(tmp);
					nnBox->blockSignals(true);
					distBox->blockSignals(true);
					this->updateParameter(Settings::kNearestNeighbor_1Strategy());
					this->updateParameter(Settings::kNearestNeighbor_2Distance_type());
					nnBox->blockSignals(false);
					distBox->blockSignals(false);
					if(sender == nnBox)
					{
						this->updateParametersVisibility();
						return;
					}
					nnStrategyChanged = true;
					paramChanged.append(Settings::kNearestNeighbor_1Strategy());
					paramChanged.append(Settings::kNearestNeighbor_2Distance_type());
				}
			}
		}

		// Distance issue when using nearest neighbor strategy using CV_32F type, though Lsh support all type (doesn't crash at least)
		if(nnStrategyChanged ||
		   (comboBox && (comboBox->objectName().compare(Settings::kNearestNeighbor_1Strategy()) == 0 ||
						 comboBox->objectName().compare(Settings::kNearestNeighbor_2Distance_type()) == 0)))
		{
			QComboBox * nnBox = (QComboBox*)this->getParameterWidget(Settings::kNearestNeighbor_1Strategy());
			QComboBox * distBox = (QComboBox*)this->getParameterWidget(Settings::kNearestNeighbor_2Distance_type());
			if(nnBox->currentText().compare("BruteForce") != 0 && nnBox->currentText().compare("Lsh") != 0 && distBox->currentIndex() > 1)
			{
				QMessageBox::warning(this,
									tr("Warning"),
									tr("Current selected nearest neighbor strategy type (\"%1\") cannot handle distance strategy (\"%2\").\n"
									   "Falling back to \"EUCLIDEAN_L2\" distance strategy (by default).")
									   .arg(nnBox->currentText())
									   .arg(distBox->currentText()));
				QString tmp = Settings::getNearestNeighbor_2Distance_type();
				*tmp.begin() = '0'; // set index
				Settings::setNearestNeighbor_2Distance_type(tmp);
				distBox->blockSignals(true);
				this->updateParameter(Settings::kNearestNeighbor_2Distance_type());
				distBox->blockSignals(false);
				if(sender == distBox)
				{
					this->updateParametersVisibility();
					return;
				}
				paramChanged.append(Settings::kNearestNeighbor_2Distance_type());
			}
		}

		if(comboBox)
		{
			QStringList items;
			for(int i=0; i<comboBox->count(); ++i)
			{
				items.append(comboBox->itemText(i));
			}
			QString merged = QString::number(value) + QString(":") + items.join(";");
			Settings::setParameter(sender->objectName(), merged);
		}

		if(checkBox)
		{
			Settings::setParameter(sender->objectName(), value==Qt::Checked?true:false);
		}

		this->updateParametersVisibility();

		paramChanged.append(sender->objectName());
		Q_EMIT parametersChanged(paramChanged);
	}
}

} // namespace find_object
