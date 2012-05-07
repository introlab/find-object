/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include "Settings.h"
#include "Camera.h"
#include <QtCore/QSettings>
#include <QtCore/QStringList>
#include <QtCore/QDir>
#include <stdio.h>
#include <opencv2/nonfree/features2d.hpp>

ParametersMap Settings::defaultParameters_;
ParametersMap Settings::parameters_;
ParametersType Settings::parametersType_;
Settings Settings::dummyInit_;

QString Settings::workingDirectory()
{
#ifdef WIN32
	return QString("%1/Documents/%2").arg(QDir::homePath()).arg(PROJECT_NAME);
#else
	return QString("%1").arg(QDir::homePath());
#endif
}

QString Settings::iniDefaultPath()
{
#ifdef WIN32
	return QString("%1/Documents/%2/%3").arg(QDir::homePath()).arg(PROJECT_NAME).arg(Settings::iniDefaultFileName());
#else
	return QString("%1/.%2/%3").arg(QDir::homePath()).arg(PROJECT_PREFIX).arg(Settings::iniDefaultFileName());
#endif
}

void Settings::loadSettings(const QString & fileName, QByteArray * windowGeometry)
{
	QString path = fileName;
	if(fileName.isEmpty())
	{
		path = iniDefaultPath();
	}
	QSettings ini(path, QSettings::IniFormat);
	for(ParametersMap::const_iterator iter = defaultParameters_.begin(); iter!=defaultParameters_.end(); ++iter)
	{
		const QString & key = iter.key();
		QVariant value = ini.value(key, QVariant());
		if(value.isValid())
		{
			setParameter(key, value);
		}
	}

	if(windowGeometry)
	{
		QVariant value = ini.value("windowGeometry", QVariant());
		if(value.isValid())
		{
			*windowGeometry = value.toByteArray();
		}
	}

	printf("Settings loaded from %s\n", path.toStdString().c_str());
}

void Settings::saveSettings(const QString & fileName, const QByteArray & windowGeometry)
{
	QString path = fileName;
	if(fileName.isEmpty())
	{
		path = iniDefaultPath();
	}
	QSettings ini(path, QSettings::IniFormat);
	for(ParametersMap::const_iterator iter = parameters_.begin(); iter!=parameters_.end(); ++iter)
	{
		QString type = Settings::getParametersType().value(iter.key());
		if(type.compare("float") == 0)
		{
			ini.setValue(iter.key(), QString::number(iter.value().toFloat(),'g',6));
		}
		else
		{
			ini.setValue(iter.key(), iter.value());
		}
	}
	if(!windowGeometry.isEmpty())
	{
		ini.setValue("windowGeometry", windowGeometry);
	}
	printf("Settings saved to %s\n", path.toStdString().c_str());
}

cv::FeatureDetector * Settings::createFeaturesDetector()
{
	cv::FeatureDetector * detector = 0;
	QString str = getDetector_Type();
	QStringList split = str.split(':');
	if(split.size()==2)
	{
		bool ok = false;
		int index = split.first().toInt(&ok);
		if(ok)
		{
			QStringList strategies = split.last().split(';');
			if(strategies.size() == 8 && index>=0 && index<8)
			{
				switch(index)
				{
				case 0:
					if(strategies.at(index).compare("Dense") == 0)
					{
						detector = new cv::DenseFeatureDetector(
								getDense_initFeatureScale(),
								getDense_featureScaleLevels(),
								getDense_featureScaleMul(),
								getDense_initXyStep(),
								getDense_initImgBound(),
								getDense_varyXyStepWithScale(),
								getDense_varyImgBoundWithScale());
					}
					break;
				case 1:
					if(strategies.at(index).compare("Fast") == 0)
					{
						detector = new cv::FastFeatureDetector(
								getFast_threshold(),
								getFast_nonmaxSuppression());
					}
					break;
				case 2:
					if(strategies.at(index).compare("GFTT") == 0)
					{
						detector = new cv::GFTTDetector(
								getGFTT_maxCorners(),
								getGFTT_qualityLevel(),
								getGFTT_minDistance(),
								getGFTT_blockSize(),
								getGFTT_useHarrisDetector(),
								getGFTT_k());
					}
					break;
				case 3:
					if(strategies.at(index).compare("MSER") == 0)
					{
						detector = new cv::MSER(
								getMSER_delta(),
								getMSER_minArea(),
								getMSER_maxArea(),
								getMSER_maxVariation(),
								getMSER_minDiversity(),
								getMSER_maxEvolution(),
								getMSER_areaThreshold(),
								getMSER_minMargin(),
								getMSER_edgeBlurSize());
					}
					break;
				case 4:
					if(strategies.at(index).compare("ORB") == 0)
					{
						detector = new cv::ORB(
								getORB_nFeatures(),
								getORB_scaleFactor(),
								getORB_nLevels(),
								getORB_edgeThreshold(),
								getORB_firstLevel(),
								getORB_WTA_K(),
								getORB_scoreType(),
								getORB_patchSize());
					}
					break;
				case 5:
					if(strategies.at(index).compare("SIFT") == 0)
					{
						detector = new cv::SIFT(
								getSIFT_nfeatures(),
								getSIFT_nOctaveLayers(),
								getSIFT_contrastThreshold(),
								getSIFT_edgeThreshold(),
								getSIFT_sigma());
					}
					break;
				case 6:
					if(strategies.at(index).compare("Star") == 0)
					{
						detector = new cv::StarFeatureDetector(
								getStar_maxSize(),
								getStar_responseThreshold(),
								getStar_lineThresholdProjected(),
								getStar_lineThresholdBinarized(),
								getStar_suppressNonmaxSize());
					}
					break;
				case 7:
					if(strategies.at(index).compare("SURF") == 0)
					{
						detector = new cv::SURF(
								getSURF_hessianThreshold(),
								getSURF_nOctaves(),
								getSURF_nOctaveLayers(),
								getSURF_extended(),
								getSURF_upright());
					}
					break;
				default:
					break;
				}
			}
		}
	}
	return detector;
}

cv::DescriptorExtractor * Settings::createDescriptorsExtractor()
{
	cv::DescriptorExtractor * extractor = 0;
	QString str = getDescriptor_Type();
	QStringList split = str.split(':');
	if(split.size()==2)
	{
		bool ok = false;
		int index = split.first().toInt(&ok);
		if(ok)
		{
			QStringList strategies = split.last().split(';');
			if(strategies.size() == 4 && index>=0 && index<4)
			{
				switch(index)
				{
				case 0:
					if(strategies.at(index).compare("Brief") == 0)
					{
						extractor = new cv::BriefDescriptorExtractor(
								getBrief_bytes());
					}
					break;
				case 1:
					if(strategies.at(index).compare("ORB") == 0)
					{
						extractor = new cv::ORB(
								getORB_nFeatures(),
								getORB_scaleFactor(),
								getORB_nLevels(),
								getORB_edgeThreshold(),
								getORB_firstLevel(),
								getORB_WTA_K(),
								getORB_scoreType(),
								getORB_patchSize());
					}
					break;
				case 2:
					if(strategies.at(index).compare("SIFT") == 0)
					{
						extractor = new cv::SIFT(
								getSIFT_nfeatures(),
								getSIFT_nOctaveLayers(),
								getSIFT_contrastThreshold(),
								getSIFT_edgeThreshold(),
								getSIFT_sigma());
					}
					break;
				case 3:
					if(strategies.at(index).compare("SURF") == 0)
					{
						extractor = new cv::SURF(
								getSURF_hessianThreshold(),
								getSURF_nOctaves(),
								getSURF_nOctaveLayers(),
								getSURF_extended(),
								getSURF_upright());
					}
					break;
				default:
					break;
				}
			}
		}
	}

	return extractor;
}

QString Settings::currentDetectorType()
{
	int index = Settings::getDetector_Type().split(':').first().toInt();
	return getDetector_Type().split(':').last().split(';').at(index);
}

QString Settings::currentDescriptorType()
{
	int index = Settings::getDescriptor_Type().split(':').first().toInt();
	return getDescriptor_Type().split(':').last().split(';').at(index);
}

