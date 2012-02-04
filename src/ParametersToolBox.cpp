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

void ParametersToolBox::resetPage(int index)
{
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
				if(qobject_cast<QComboBox*>(children.at(j)))
				{
					((QComboBox*)children.at(j))->setCurrentIndex(value.toString().split(':').first().toInt());
				}
				else if(qobject_cast<QSpinBox*>(children.at(j)))
				{
					((QSpinBox*)children.at(j))->setValue(value.toInt());
				}
				else if(qobject_cast<QDoubleSpinBox*>(children.at(j)))
				{
					((QDoubleSpinBox*)children.at(j))->setValue(value.toDouble());
				}
				else if(qobject_cast<QCheckBox*>(children.at(j)))
				{
					((QCheckBox*)children.at(j))->setChecked(value.toBool());
				}
				else if(qobject_cast<QLineEdit*>(children.at(j)))
				{
					((QLineEdit*)children.at(j))->setText(value.toString());
				}
				Settings::setParameter(key, value);
			}
		}
	}
}

void ParametersToolBox::resetCurrentPage()
{
	this->blockSignals(true);
	this->resetPage(this->currentIndex());
	this->blockSignals(false);
	emit parametersChanged();
}

void ParametersToolBox::resetAllPages()
{
	this->blockSignals(true);
	for(int i=0; i< this->count(); ++i)
	{
		this->resetPage(i);
	}
	this->blockSignals(false);
	emit parametersChanged();
}

void ParametersToolBox::setupUi()
{
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
		((QLineEdit*)widget)->setText(Settings::getParameter(key).toString());
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
	hLayout->addWidget(new QLabel(name, this));
	hLayout->addWidget(widget);
}

void ParametersToolBox::changeParameter(const QString & value)
{
	if(sender())
	{
		Settings::setParameter(sender()->objectName(), value);
		emit parametersChanged();
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
		emit parametersChanged();
	}
}
void ParametersToolBox::changeParameter(const int & value)
{
	if(sender())
	{
		QComboBox * comboBox = qobject_cast<QComboBox*>(sender());
		QCheckBox * checkBox = qobject_cast<QCheckBox*>(sender());
		if(comboBox)
		{
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
		emit parametersChanged();
	}
}
