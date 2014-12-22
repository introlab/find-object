/*
 * ImageDropWidget.cpp
 *
 *  Created on: Dec 22, 2014
 *      Author: mathieu
 */

#include <ImageDropWidget.h>
#include <find_object/Settings.h>
#include <QtGui/QDragEnterEvent>
#include <QtCore/QRegExp>
#include <QtCore/QUrl>

namespace find_object {

ImageDropWidget::ImageDropWidget(QWidget *parent, Qt::WindowFlags flags) :
		QWidget(parent, flags)
{
	setAcceptDrops(true);
}

ImageDropWidget::~ImageDropWidget()
{
}

void ImageDropWidget::dragEnterEvent(QDragEnterEvent *event)
{
	if (event->mimeData()->hasUrls())
	{
		event->acceptProposedAction();
	}
}

void ImageDropWidget::dropEvent(QDropEvent *event)
{
	QStringList fileNames;

	QStringList extensions = Settings::getGeneral_imageFormats().split(" ");

	QList<QUrl> urls = event->mimeData()->urls();
	for(int i=0; i<urls.size(); ++i)
	{
		QString path = urls.at(i).toLocalFile();
		for(int j=0; j<extensions.size(); ++j)
		{
			QRegExp reg(extensions[j]);
			reg.setPatternSyntax(QRegExp::Wildcard);
			if(reg.exactMatch(path))
			{
				fileNames.push_back(path);
				break;
			}
		}
	}

	if(fileNames.size())
	{
		Q_EMIT imagesReceived(fileNames);
	}

	event->acceptProposedAction();
}

}

