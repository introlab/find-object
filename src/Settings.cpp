/*
 * Settings.cpp
 *
 *  Created on: 2011-10-22
 *      Author: matlab
 */


#include "Settings.h"
#include "Camera.h"
#include <QtCore/QSettings>
#include <QtCore/QStringList>
#include <QtCore/QDir>
#include <stdio.h>

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
	QString str = getDetector_Type().toString();
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
						cv::DenseFeatureDetector::Params params;
						params.initFeatureScale = getDense_initFeatureScale().toFloat();
						params.featureScaleLevels = getDense_featureScaleLevels().toInt();
						params.featureScaleMul = getDense_featureScaleMul().toFloat();
						params.initXyStep = getDense_initXyStep().toInt();
						params.initImgBound = getDense_initImgBound().toInt();
						params.varyXyStepWithScale = getDense_varyXyStepWithScale().toBool();
						params.varyImgBoundWithScale = getDense_varyImgBoundWithScale().toBool();
						detector = new cv::DenseFeatureDetector(params);
					}
					break;
				case 1:
					if(strategies.at(index).compare("Fast") == 0)
					{
						detector = new cv::FastFeatureDetector(
								getFast_threshold().toInt(),
								getFast_nonmaxSuppression().toBool());
					}
					break;
				case 2:
					if(strategies.at(index).compare("GoodFeaturesToTrack") == 0)
					{
						cv::GoodFeaturesToTrackDetector::Params params;
						params.maxCorners = getGoodFeaturesToTrack_maxCorners().toInt();
						params.qualityLevel = getGoodFeaturesToTrack_qualityLevel().toDouble();
						params.minDistance = getGoodFeaturesToTrack_minDistance().toDouble();
						params.blockSize = getGoodFeaturesToTrack_blockSize().toInt();
						params.useHarrisDetector = getGoodFeaturesToTrack_useHarrisDetector().toBool();
						params.k = getGoodFeaturesToTrack_k().toDouble();
						detector = new cv::GoodFeaturesToTrackDetector(params);
					}
					break;
				case 3:
					if(strategies.at(index).compare("Mser") == 0)
					{
						CvMSERParams params = cvMSERParams();
						params.delta = getMser_delta().toInt();
						params.maxArea = getMser_maxArea().toInt();
						params.minArea = getMser_minArea().toInt();
						params.maxVariation = getMser_maxVariation().toFloat();
						params.minDiversity = getMser_minDiversity().toFloat();
						params.maxEvolution = getMser_maxEvolution().toInt();
						params.areaThreshold = getMser_areaThreshold().toDouble();
						params.minMargin = getMser_minMargin().toDouble();
						params.edgeBlurSize = getMser_edgeBlurSize().toInt();
						detector = new cv::MserFeatureDetector(params);
					}
					break;
				case 4:
					if(strategies.at(index).compare("Orb") == 0)
					{
						cv::ORB::CommonParams params;
						params.scale_factor_ = getOrb_scaleFactor().toFloat();
						params.n_levels_ = getOrb_nLevels().toUInt();
						params.first_level_ = getOrb_firstLevel().toUInt();
						params.edge_threshold_ = getOrb_edgeThreshold().toInt();
						detector = new cv::OrbFeatureDetector(
								getOrb_nFeatures().toUInt(),
								params);
					}
					break;
				case 5:
					if(strategies.at(index).compare("Sift") == 0)
					{
						cv::SIFT::DetectorParams detectorParams;
						detectorParams.edgeThreshold = getSift_edgeThreshold().toDouble();
						detectorParams.threshold = getSift_threshold().toDouble();
						cv::SIFT::CommonParams commonParams;
						commonParams.angleMode = getSift_angleMode().toInt();
						commonParams.firstOctave = getSift_firstOctave().toInt();
						commonParams.nOctaveLayers = getSift_nOctaveLayers().toInt();
						commonParams.nOctaves = getSift_nOctaves().toInt();
						detector = new cv::SiftFeatureDetector(
								detectorParams,
								commonParams);
					}
					break;
				case 6:
					if(strategies.at(index).compare("Star") == 0)
					{
						CvStarDetectorParams params = cvStarDetectorParams();
						params.lineThresholdBinarized = getStar_lineThresholdBinarized().toInt();
						params.lineThresholdProjected = getStar_lineThresholdProjected().toInt();
						params.maxSize = getStar_maxSize().toInt();
						params.responseThreshold = getStar_responseThreshold().toInt();
						params.suppressNonmaxSize = getStar_suppressNonmaxSize().toInt();
						detector = new cv::StarFeatureDetector(params);
					}
					break;
				case 7:
					if(strategies.at(index).compare("Surf") == 0)
					{
						detector = new cv::SurfFeatureDetector(
								getSurf_hessianThreshold().toDouble(),
								getSurf_octaves().toInt(),
								getSurf_octaveLayers().toInt(),
								getSurf_upright().toBool());
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
	QString str = getDescriptor_Type().toString();
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
								getBrief_bytes().toInt());
					}
					break;
				case 1:
					if(strategies.at(index).compare("Orb") == 0)
					{
						cv::ORB::CommonParams params;
						params.scale_factor_ = getOrb_scaleFactor().toFloat();
						params.n_levels_ = getOrb_nLevels().toUInt();
						params.first_level_ = getOrb_firstLevel().toUInt();
						params.edge_threshold_ = getOrb_edgeThreshold().toInt();
						extractor = new cv::OrbDescriptorExtractor(params);
					}
					break;
				case 2:
					if(strategies.at(index).compare("Sift") == 0)
					{
						cv::SIFT::DescriptorParams descriptorParams;
						descriptorParams.isNormalize = getSift_isNormalize().toBool();
						descriptorParams.magnification = getSift_magnification().toDouble();
						descriptorParams.recalculateAngles = getSift_recalculateAngles().toBool();
						cv::SIFT::CommonParams commonParams;
						commonParams.angleMode = getSift_angleMode().toInt();
						commonParams.firstOctave = getSift_firstOctave().toInt();
						commonParams.nOctaveLayers = getSift_nOctaveLayers().toInt();
						commonParams.nOctaves = getSift_nOctaves().toInt();
						extractor = new cv::SiftDescriptorExtractor(
								descriptorParams,
								commonParams);
					}
					break;
				case 3:
					if(strategies.at(index).compare("Surf") == 0)
					{
						extractor = new cv::SurfDescriptorExtractor(
								getSurf_octaves().toInt(),
								getSurf_octaveLayers().toInt(),
								getSurf_extended().toBool(),
								getSurf_upright().toBool());
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
	int index = Settings::getDetector_Type().toString().split(':').first().toInt();
	return getDetector_Type().toString().split(':').last().split(';').at(index);
}

QString Settings::currentDescriptorType()
{
	int index = Settings::getDescriptor_Type().toString().split(':').first().toInt();
	return getDescriptor_Type().toString().split(':').last().split(';').at(index);
}

