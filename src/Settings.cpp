/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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
						cv::DenseFeatureDetector::Params params;
						params.initFeatureScale = getDense_initFeatureScale();
						params.featureScaleLevels = getDense_featureScaleLevels();
						params.featureScaleMul = getDense_featureScaleMul();
						params.initXyStep = getDense_initXyStep();
						params.initImgBound = getDense_initImgBound();
						params.varyXyStepWithScale = getDense_varyXyStepWithScale();
						params.varyImgBoundWithScale = getDense_varyImgBoundWithScale();
						detector = new cv::DenseFeatureDetector(params);
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
					if(strategies.at(index).compare("GoodFeaturesToTrack") == 0)
					{
						cv::GoodFeaturesToTrackDetector::Params params;
						params.maxCorners = getGoodFeaturesToTrack_maxCorners();
						params.qualityLevel = getGoodFeaturesToTrack_qualityLevel();
						params.minDistance = getGoodFeaturesToTrack_minDistance();
						params.blockSize = getGoodFeaturesToTrack_blockSize();
						params.useHarrisDetector = getGoodFeaturesToTrack_useHarrisDetector();
						params.k = getGoodFeaturesToTrack_k();
						detector = new cv::GoodFeaturesToTrackDetector(params);
					}
					break;
				case 3:
					if(strategies.at(index).compare("Mser") == 0)
					{
						CvMSERParams params = cvMSERParams();
						params.delta = getMser_delta();
						params.maxArea = getMser_maxArea();
						params.minArea = getMser_minArea();
						params.maxVariation = getMser_maxVariation();
						params.minDiversity = getMser_minDiversity();
						params.maxEvolution = getMser_maxEvolution();
						params.areaThreshold = getMser_areaThreshold();
						params.minMargin = getMser_minMargin();
						params.edgeBlurSize = getMser_edgeBlurSize();
						detector = new cv::MserFeatureDetector(params);
					}
					break;
				case 4:
					if(strategies.at(index).compare("Orb") == 0)
					{
						cv::ORB::CommonParams params;
						params.scale_factor_ = getOrb_scaleFactor();
						params.n_levels_ = getOrb_nLevels();
						params.first_level_ = getOrb_firstLevel();
						params.edge_threshold_ = getOrb_edgeThreshold();
						detector = new cv::OrbFeatureDetector(
								getOrb_nFeatures(),
								params);
					}
					break;
				case 5:
					if(strategies.at(index).compare("Sift") == 0)
					{
						cv::SIFT::DetectorParams detectorParams;
						detectorParams.edgeThreshold = getSift_edgeThreshold();
						detectorParams.threshold = getSift_threshold();
						cv::SIFT::CommonParams commonParams;
						commonParams.angleMode = getSift_angleMode();
						commonParams.firstOctave = getSift_firstOctave();
						commonParams.nOctaveLayers = getSift_nOctaveLayers();
						commonParams.nOctaves = getSift_nOctaves();
						detector = new cv::SiftFeatureDetector(
								detectorParams,
								commonParams);
					}
					break;
				case 6:
					if(strategies.at(index).compare("Star") == 0)
					{
						CvStarDetectorParams params = cvStarDetectorParams();
						params.lineThresholdBinarized = getStar_lineThresholdBinarized();
						params.lineThresholdProjected = getStar_lineThresholdProjected();
						params.maxSize = getStar_maxSize();
						params.responseThreshold = getStar_responseThreshold();
						params.suppressNonmaxSize = getStar_suppressNonmaxSize();
						detector = new cv::StarFeatureDetector(params);
					}
					break;
				case 7:
					if(strategies.at(index).compare("Surf") == 0)
					{
						detector = new cv::SurfFeatureDetector(
								getSurf_hessianThreshold(),
								getSurf_octaves(),
								getSurf_octaveLayers(),
								getSurf_upright());
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
					if(strategies.at(index).compare("Orb") == 0)
					{
						cv::ORB::CommonParams params;
						params.scale_factor_ = getOrb_scaleFactor();
						params.n_levels_ = getOrb_nLevels();
						params.first_level_ = getOrb_firstLevel();
						params.edge_threshold_ = getOrb_edgeThreshold();
						extractor = new cv::OrbDescriptorExtractor(params);
					}
					break;
				case 2:
					if(strategies.at(index).compare("Sift") == 0)
					{
						cv::SIFT::DescriptorParams descriptorParams;
						descriptorParams.isNormalize = getSift_isNormalize();
						descriptorParams.magnification = getSift_magnification();
						descriptorParams.recalculateAngles = getSift_recalculateAngles();
						cv::SIFT::CommonParams commonParams;
						commonParams.angleMode = getSift_angleMode();
						commonParams.firstOctave = getSift_firstOctave();
						commonParams.nOctaveLayers = getSift_nOctaveLayers();
						commonParams.nOctaves = getSift_nOctaves();
						extractor = new cv::SiftDescriptorExtractor(
								descriptorParams,
								commonParams);
					}
					break;
				case 3:
					if(strategies.at(index).compare("Surf") == 0)
					{
						extractor = new cv::SurfDescriptorExtractor(
								getSurf_octaves(),
								getSurf_octaveLayers(),
								getSurf_extended(),
								getSurf_upright());
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

