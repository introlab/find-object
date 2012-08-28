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

#define VERBOSE 0

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

void Settings::loadSettings(const QString & fileName, QByteArray * windowGeometry, QByteArray * windowState)
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
	if(windowState)
	{
		QVariant value = ini.value("windowState", QVariant());
		if(value.isValid())
		{
			*windowState = value.toByteArray();
		}
	}

	printf("Settings loaded from %s\n", path.toStdString().c_str());
}

void Settings::saveSettings(const QString & fileName, const QByteArray & windowGeometry, const QByteArray & windowState)
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
	if(!windowState.isEmpty())
	{
		ini.setValue("windowState", windowState);
	}
	printf("Settings saved to %s\n", path.toStdString().c_str());
}

cv::FeatureDetector * Settings::createFeaturesDetector()
{
	cv::FeatureDetector * detector = 0;
	QString str = getDetector_Descriptor_1Detector();
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
								getDetector_Descriptor_Dense_initFeatureScale(),
								getDetector_Descriptor_Dense_featureScaleLevels(),
								getDetector_Descriptor_Dense_featureScaleMul(),
								getDetector_Descriptor_Dense_initXyStep(),
								getDetector_Descriptor_Dense_initImgBound(),
								getDetector_Descriptor_Dense_varyXyStepWithScale(),
								getDetector_Descriptor_Dense_varyImgBoundWithScale());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "Dense");
					}
					break;
				case 1:
					if(strategies.at(index).compare("Fast") == 0)
					{
						detector = new cv::FastFeatureDetector(
								getDetector_Descriptor_Fast_threshold(),
								getDetector_Descriptor_Fast_nonmaxSuppression());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "Fast");
					}
					break;
				case 2:
					if(strategies.at(index).compare("GFTT") == 0)
					{
						detector = new cv::GFTTDetector(
								getDetector_Descriptor_GFTT_maxCorners(),
								getDetector_Descriptor_GFTT_qualityLevel(),
								getDetector_Descriptor_GFTT_minDistance(),
								getDetector_Descriptor_GFTT_blockSize(),
								getDetector_Descriptor_GFTT_useHarrisDetector(),
								getDetector_Descriptor_GFTT_k());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "GFTT");
					}
					break;
				case 3:
					if(strategies.at(index).compare("MSER") == 0)
					{
						detector = new cv::MSER(
								getDetector_Descriptor_MSER_delta(),
								getDetector_Descriptor_MSER_minArea(),
								getDetector_Descriptor_MSER_maxArea(),
								getDetector_Descriptor_MSER_maxVariation(),
								getDetector_Descriptor_MSER_minDiversity(),
								getDetector_Descriptor_MSER_maxEvolution(),
								getDetector_Descriptor_MSER_areaThreshold(),
								getDetector_Descriptor_MSER_minMargin(),
								getDetector_Descriptor_MSER_edgeBlurSize());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "MSER");
					}
					break;
				case 4:
					if(strategies.at(index).compare("ORB") == 0)
					{
						detector = new cv::ORB(
								getDetector_Descriptor_ORB_nFeatures(),
								getDetector_Descriptor_ORB_scaleFactor(),
								getDetector_Descriptor_ORB_nLevels(),
								getDetector_Descriptor_ORB_edgeThreshold(),
								getDetector_Descriptor_ORB_firstLevel(),
								getDetector_Descriptor_ORB_WTA_K(),
								getDetector_Descriptor_ORB_scoreType(),
								getDetector_Descriptor_ORB_patchSize());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "ORB");
					}
					break;
				case 5:
					if(strategies.at(index).compare("SIFT") == 0)
					{
						detector = new cv::SIFT(
								getDetector_Descriptor_SIFT_nfeatures(),
								getDetector_Descriptor_SIFT_nOctaveLayers(),
								getDetector_Descriptor_SIFT_contrastThreshold(),
								getDetector_Descriptor_SIFT_edgeThreshold(),
								getDetector_Descriptor_SIFT_sigma());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "SIFT");
					}
					break;
				case 6:
					if(strategies.at(index).compare("Star") == 0)
					{
						detector = new cv::StarFeatureDetector(
								getDetector_Descriptor_Star_maxSize(),
								getDetector_Descriptor_Star_responseThreshold(),
								getDetector_Descriptor_Star_lineThresholdProjected(),
								getDetector_Descriptor_Star_lineThresholdBinarized(),
								getDetector_Descriptor_Star_suppressNonmaxSize());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "Star");
					}
					break;
				case 7:
					if(strategies.at(index).compare("SURF") == 0)
					{
						detector = new cv::SURF(
								getDetector_Descriptor_SURF_hessianThreshold(),
								getDetector_Descriptor_SURF_nOctaves(),
								getDetector_Descriptor_SURF_nOctaveLayers(),
								getDetector_Descriptor_SURF_extended(),
								getDetector_Descriptor_SURF_upright());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "SURF");
					}
					break;
				default:
					break;
				}
			}
		}
	}
	if(!detector)
	{
		printf("ERROR: detector strategy not found !? Using default SURF...\n");
		detector = new cv::SURF();
	}
	return detector;
}

cv::DescriptorExtractor * Settings::createDescriptorsExtractor()
{
	cv::DescriptorExtractor * extractor = 0;
	QString str = getDetector_Descriptor_2Descriptor();
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
								getDetector_Descriptor_Brief_bytes());
						if(VERBOSE)printf("Settings::createDescriptorsExtractor() type=%s\n", "Brief");
					}
					break;
				case 1:
					if(strategies.at(index).compare("ORB") == 0)
					{
						extractor = new cv::ORB(
								getDetector_Descriptor_ORB_nFeatures(),
								getDetector_Descriptor_ORB_scaleFactor(),
								getDetector_Descriptor_ORB_nLevels(),
								getDetector_Descriptor_ORB_edgeThreshold(),
								getDetector_Descriptor_ORB_firstLevel(),
								getDetector_Descriptor_ORB_WTA_K(),
								getDetector_Descriptor_ORB_scoreType(),
								getDetector_Descriptor_ORB_patchSize());
						if(VERBOSE)printf("Settings::createDescriptorsExtractor() type=%s\n", "ORB");
					}
					break;
				case 2:
					if(strategies.at(index).compare("SIFT") == 0)
					{
						extractor = new cv::SIFT(
								getDetector_Descriptor_SIFT_nfeatures(),
								getDetector_Descriptor_SIFT_nOctaveLayers(),
								getDetector_Descriptor_SIFT_contrastThreshold(),
								getDetector_Descriptor_SIFT_edgeThreshold(),
								getDetector_Descriptor_SIFT_sigma());
						if(VERBOSE)printf("Settings::createDescriptorsExtractor() type=%s\n", "SIFT");
					}
					break;
				case 3:
					if(strategies.at(index).compare("SURF") == 0)
					{
						extractor = new cv::SURF(
								getDetector_Descriptor_SURF_hessianThreshold(),
								getDetector_Descriptor_SURF_nOctaves(),
								getDetector_Descriptor_SURF_nOctaveLayers(),
								getDetector_Descriptor_SURF_extended(),
								getDetector_Descriptor_SURF_upright());
						if(VERBOSE)printf("Settings::createDescriptorsExtractor() type=%s\n", "SURF");
					}
					break;
				default:
					break;
				}
			}
		}
	}
	if(!extractor)
	{
		printf("ERROR: descriptor strategy not found !? Using default SURF...\n");
		extractor = new cv::SURF();
	}
	return extractor;
}

QString Settings::currentDetectorType()
{
	int index = getDetector_Descriptor_1Detector().split(':').first().toInt();
	return getDetector_Descriptor_1Detector().split(':').last().split(';').at(index);
}

QString Settings::currentDescriptorType()
{
	int index = getDetector_Descriptor_2Descriptor().split(':').first().toInt();
	return getDetector_Descriptor_2Descriptor().split(':').last().split(';').at(index);
}

QString Settings::currentNearestNeighborType()
{
	int index = getNearestNeighbor_1Strategy().split(':').first().toInt();
	return getNearestNeighbor_1Strategy().split(':').last().split(';').at(index);
}

cv::flann::IndexParams * Settings::createFlannIndexParams()
{
	cv::flann::IndexParams * params = 0;
	QString str = getNearestNeighbor_1Strategy();
	QStringList split = str.split(':');
	if(split.size()==2)
	{
		bool ok = false;
		int index = split.first().toInt(&ok);
		if(ok)
		{
			QStringList strategies = split.last().split(';');
			if(strategies.size() == 6 && index>=0 && index<6)
			{
				switch(index)
				{
				case 0:
					if(strategies.at(index).compare("Linear") == 0)
					{
						if(VERBOSE)printf("Settings::getFlannIndexParams() type=%s\n", "Linear");
						params = new cv::flann::LinearIndexParams();
					}
					break;
				case 1:
					if(strategies.at(index).compare("KDTree") == 0)
					{
						if(VERBOSE)printf("Settings::getFlannIndexParams() type=%s\n", "KDTree");
						params = new cv::flann::KDTreeIndexParams(
								getNearestNeighbor_KDTree_trees());
					}
					break;
				case 2:
					if(strategies.at(index).compare("KMeans") == 0)
					{
						cvflann::flann_centers_init_t centers_init = cvflann::FLANN_CENTERS_RANDOM;
						QString str = getNearestNeighbor_KMeans_centers_init();
						QStringList split = str.split(':');
						if(split.size()==2)
						{
							bool ok = false;
							int index = split.first().toInt(&ok);
							if(ok)
							{
								centers_init = (cvflann::flann_centers_init_t)index;
							}
						}
						if(VERBOSE)printf("Settings::getFlannIndexParams() type=%s\n", "KMeans");
						params = new cv::flann::KMeansIndexParams(
								getNearestNeighbor_KMeans_branching(),
								getNearestNeighbor_KMeans_iterations(),
								centers_init,
								getNearestNeighbor_KMeans_cb_index());
					}
					break;
				case 3:
					if(strategies.at(index).compare("Composite") == 0)
					{
						cvflann::flann_centers_init_t centers_init = cvflann::FLANN_CENTERS_RANDOM;
						QString str = getNearestNeighbor_Composite_centers_init();
						QStringList split = str.split(':');
						if(split.size()==2)
						{
							bool ok = false;
							int index = split.first().toInt(&ok);
							if(ok)
							{
								centers_init = (cvflann::flann_centers_init_t)index;
							}
						}
						if(VERBOSE)printf("Settings::getFlannIndexParams() type=%s\n", "Composite");
						params = new cv::flann::CompositeIndexParams(
								getNearestNeighbor_Composite_trees(),
								getNearestNeighbor_Composite_branching(),
								getNearestNeighbor_Composite_iterations(),
								centers_init,
								getNearestNeighbor_Composite_cb_index());
					}
					break;
				case 4:
					if(strategies.at(index).compare("Autotuned") == 0)
					{
						if(VERBOSE)printf("Settings::getFlannIndexParams() type=%s\n", "Autotuned");
						params = new cv::flann::AutotunedIndexParams(
								getNearestNeighbor_Autotuned_target_precision(),
								getNearestNeighbor_Autotuned_build_weight(),
								getNearestNeighbor_Autotuned_memory_weight(),
								getNearestNeighbor_Autotuned_sample_fraction());
					}
					break;
				case 5:
					if(strategies.at(index).compare("Lsh") == 0)
					{
						if(VERBOSE)printf("Settings::getFlannIndexParams() type=%s\n", "Lsh");
						params = new cv::flann::LshIndexParams(
								getNearestNeighbor_Lsh_table_number(),
								getNearestNeighbor_Lsh_key_size(),
								getNearestNeighbor_Lsh_multi_probe_level());

					}
					break;
				default:
					break;
				}
			}
		}
	}
	if(!params)
	{
		printf("ERROR: NN strategy not found !? Using default KDTRee...\n");
		params = new cv::flann::KDTreeIndexParams();
	}
	return params ;
}

cvflann::flann_distance_t Settings::getFlannDistanceType()
{
	cvflann::flann_distance_t distance = cvflann::FLANN_DIST_L2;
	QString str = getNearestNeighbor_2Distance_type();
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
				distance = (cvflann::flann_distance_t)(index+1);
			}
		}
	}
	if(VERBOSE)printf("Settings::getFlannDistanceType() distance=%d\n", distance);
	return distance;
}

cv::flann::SearchParams Settings::getFlannSearchParams()
{
	return cv::flann::SearchParams();
}

