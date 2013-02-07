/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "ParametersToolBox.h"
#include "Settings.h"
#include <QtGui/QComboBox>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QLineEdit>
#include <QtGui/QLabel>
#include <QtGui/QGroupBox>
#include <QtGui/QCheckBox>
#include <QtGui/QVBoxLayout>
#include <QtGui/QMessageBox>
#include <stdio.h>

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
	emit parametersChanged(paramChanged);
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
	emit parametersChanged(paramChanged);
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
		widget->setCurrentIndex(splitted.first().toInt());
		connect(widget, SIGNAL(currentIndexChanged(int)), this, SLOT(changeParameter(int)));
		addParameter(layout, key.split('/').last(), widget);
	}
	else
	{
		QLineEdit * widget = new QLineEdit(value, this);
		widget->setObjectName(key);
		connect(widget, SIGNAL(editingFinished()), this, SLOT(changeParameter()));
		addParameter(layout, key.split('/').last(), widget);
	}
}

void ParametersToolBox::addParameter(QVBoxLayout * layout,
		const QString & key,
		const double & value)
{
	QDoubleSpinBox * widget = new QDoubleSpinBox(this);
	double def = Settings::getDefaultParameters().value(key).toDouble();
	if(def<0.001)
	{
		widget->setDecimals(4);
	}
	else if(def<0.01)
	{
		widget->setDecimals(3);
	}

	if(def>=0.0)
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
	addParameter(layout, key.split('/').last(), widget);
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
	addParameter(layout, key.split('/').last(), widget);
}

void ParametersToolBox::addParameter(QVBoxLayout * layout,
		const QString & key,
		const bool & value)
{
	QCheckBox * widget = new QCheckBox(this);
	widget->setChecked(value);
	widget->setObjectName(key);
	connect(widget, SIGNAL(stateChanged(int)), this, SLOT(changeParameter(int)));
	addParameter(layout, key.split('/').last(), widget);
}

void ParametersToolBox::addParameter(QVBoxLayout * layout, const QString & name, QWidget * widget)
{
	QHBoxLayout * hLayout = new QHBoxLayout();
	layout->insertLayout(layout->count()-1, hLayout);
	QString tmp = name;
	if(tmp.at(0).isDigit())
	{
		tmp.remove(0,1);
	}
	hLayout->addWidget(new QLabel(tmp, this));
	hLayout->addWidget(widget);
}

void ParametersToolBox::changeParameter(const QString & value)
{
	if(sender())
	{
		Settings::setParameter(sender()->objectName(), value);
		QStringList paramChanged;
		paramChanged.append(sender()->objectName());
		emit parametersChanged(paramChanged);
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
			Settings::setParameter(sender()->objectName(), spinBox->value());
		}
		else if(lineEdit)
		{
			Settings::setParameter(sender()->objectName(), lineEdit->text());
		}
		QStringList paramChanged;
		paramChanged.append(sender()->objectName());
		emit parametersChanged(paramChanged);
	}
}
void ParametersToolBox::changeParameter(const int & value)
{
	if(sender())
	{
		QStringList paramChanged;
		QComboBox * comboBox = qobject_cast<QComboBox*>(sender());
		QCheckBox * checkBox = qobject_cast<QCheckBox*>(sender());
		if(comboBox)
		{
			bool nnStrategyChanged = false;
			//verify binary issue with nearest neighbor strategy
			if(comboBox->objectName().compare(Settings::kFeature2D_2Descriptor()) == 0 ||
			   comboBox->objectName().compare(Settings::kNearestNeighbor_1Strategy()) == 0)
			{
				QComboBox * descriptorBox = (QComboBox*)this->getParameterWidget(Settings::kFeature2D_2Descriptor());
				QComboBox * nnBox = (QComboBox*)this->getParameterWidget(Settings::kNearestNeighbor_1Strategy());
				bool isBinaryDescriptor = descriptorBox->currentText().compare("ORB") == 0 ||
										  descriptorBox->currentText().compare("Brief") == 0 ||
										  descriptorBox->currentText().compare("BRISK") == 0 ||
										  descriptorBox->currentText().compare("FREAK") == 0;
				if(isBinaryDescriptor && nnBox->currentText().compare("Lsh") != 0)
				{
					QMessageBox::warning(this,
							tr("Warning"),
							tr("Current selected descriptor type (\"%1\") is binary while nearest neighbor strategy is not (\"%2\").\n"
							   "Falling back to \"Lsh\" nearest neighbor strategy (by default).")
							   .arg(descriptorBox->currentText())
							   .arg(nnBox->currentText()));
					QString tmp = Settings::getNearestNeighbor_1Strategy();
					*tmp.begin() = '5'; // set index
					Settings::setNearestNeighbor_1Strategy(tmp);
					nnBox->blockSignals(true);
					this->updateParameter(Settings::kNearestNeighbor_1Strategy());
					nnBox->blockSignals(false);
					if(sender() == nnBox)
					{
						return;
					}
					nnStrategyChanged = true;
					paramChanged.append(Settings::kNearestNeighbor_1Strategy());
				}
				else if(!isBinaryDescriptor && nnBox->currentText().compare("Lsh") == 0)
				{
					QMessageBox::warning(this,
							tr("Warning"),
							tr("Current selected descriptor type (\"%1\") is not binary while nearest neighbor strategy is (\"%2\").\n"
							   "Falling back to \"KDTree\" nearest neighbor strategy (by default).")
							   .arg(descriptorBox->currentText())
							   .arg(nnBox->currentText()));
					QString tmp = Settings::getNearestNeighbor_1Strategy();
					*tmp.begin() = '1'; // set index
					Settings::setNearestNeighbor_1Strategy(tmp);
					nnBox->blockSignals(true);
					this->updateParameter(Settings::kNearestNeighbor_1Strategy());
					nnBox->blockSignals(false);
					if(sender() == nnBox)
					{
						return;
					}
					nnStrategyChanged = true;
					paramChanged.append(Settings::kNearestNeighbor_1Strategy());
				}
			}

			// Distance issue when using nearest neighbor strategy using CV_32F type, though Lsh support all type (doesn't crash at least)
			if(nnStrategyChanged ||
			   comboBox->objectName().compare(Settings::kNearestNeighbor_1Strategy()) == 0 ||
			   comboBox->objectName().compare(Settings::kNearestNeighbor_2Distance_type()) == 0)
			{
				QComboBox * nnBox = (QComboBox*)this->getParameterWidget(Settings::kNearestNeighbor_1Strategy());
				QComboBox * distBox = (QComboBox*)this->getParameterWidget(Settings::kNearestNeighbor_2Distance_type());
				if(nnBox->currentText().compare("Lsh") != 0 && distBox->currentIndex() > 1)
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
					if(sender() == distBox)
					{
						return;
					}
					paramChanged.append(Settings::kNearestNeighbor_2Distance_type());
				}
			}

			QStringList items;
			for(int i=0; i<comboBox->count(); ++i)
			{
				items.append(comboBox->itemText(i));
			}
			QString merged = QString::number(value) + QString(":") + items.join(";");
			Settings::setParameter(sender()->objectName(), merged);
		}
		else if(checkBox)
		{
			Settings::setParameter(sender()->objectName(), value==Qt::Checked?true:false);
		}
		paramChanged.append(sender()->objectName());
		emit parametersChanged(paramChanged);
	}
}
