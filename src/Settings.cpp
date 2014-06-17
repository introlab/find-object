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
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/gpu.hpp>

#define VERBOSE 0

ParametersMap Settings::defaultParameters_;
ParametersMap Settings::parameters_;
ParametersType Settings::parametersType_;
DescriptionsMap Settings::descriptions_;
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
			QString str = value.toString();
			if(str.contains(";") && str.size() != getParameter(key).toString().size())
			{
				// If a string list is modified, update the value
				// (assuming that index < 10... one character for index)
				QChar index = str.at(0);
				str = getParameter(key).toString();
				str[0] = index.toAscii();
				value = QVariant(str);
				printf("Updated list of parameter \"%s\"\n", key.toStdString().c_str());
			}
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

class GPUSURF : public cv::Feature2D
{
public:
	GPUSURF(double hessianThreshold,
            int nOctaves=4, int nOctaveLayers=2,
            bool extended=true, bool upright=false) :
		hessianThreshold_(hessianThreshold),
		nOctaves_(nOctaves),
		nOctaveLayers_(nOctaveLayers),
		extended_(extended),
		upright_(upright)
	{
	}
	virtual ~GPUSURF() {}

	void operator()(cv::InputArray img, cv::InputArray mask,
	                    std::vector<cv::KeyPoint>& keypoints,
	                    cv::OutputArray descriptors,
	                    bool useProvidedKeypoints=false) const
	{
		printf("GPUSURF:operator() Don't call this directly!\n");
		exit(-1);
	}
	int descriptorSize() const
	{
		return extended_ ? 128 : 64;
	}
	int descriptorType() const
	{
		return CV_32F;
	}

protected:
    void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat() ) const
    {
    	cv::gpu::GpuMat imgGpu(image);
    	cv::gpu::GpuMat maskGpu(mask);
		cv::gpu::GpuMat keypointsGpu;
		cv::gpu::SURF_GPU surfGpu(hessianThreshold_, nOctaves_, nOctaveLayers_, extended_, 0.01f, upright_);
		surfGpu(imgGpu, maskGpu, keypointsGpu);
		surfGpu.downloadKeypoints(keypointsGpu, keypoints);
    }

    void computeImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors ) const
    {
    	std::vector<float> d;
		cv::gpu::GpuMat imgGpu(image);
		cv::gpu::GpuMat descriptorsGpu;
		cv::gpu::GpuMat keypointsGpu;
		cv::gpu::SURF_GPU surfGpu(hessianThreshold_, nOctaves_, nOctaveLayers_, extended_, 0.01f, upright_);
		surfGpu.uploadKeypoints(keypoints, keypointsGpu);
		surfGpu(imgGpu, cv::gpu::GpuMat(), keypointsGpu, descriptorsGpu, true);
		surfGpu.downloadDescriptors(descriptorsGpu, d);
		unsigned int dim = extended_?128:64;
		descriptors = cv::Mat(d.size()/dim, dim, CV_32F);
		for(int i=0; i<descriptors.rows; ++i)
		{
			float * rowFl = descriptors.ptr<float>(i);
			memcpy(rowFl, &d[i*dim], dim*sizeof(float));
		}
    }

private:
    double hessianThreshold_;
    int nOctaves_;
    int nOctaveLayers_;
    bool extended_;
    bool upright_;
};

cv::FeatureDetector * Settings::createFeaturesDetector()
{
	cv::FeatureDetector * detector = 0;
	QString str = getFeature2D_1Detector();
	QStringList split = str.split(':');
	if(split.size()==2)
	{
		bool ok = false;
		int index = split.first().toInt(&ok);
		if(ok)
		{
			QStringList strategies = split.last().split(';');
			if(strategies.size() == 9 && index>=0 && index<9)
			{
				switch(index)
				{
				case 0:
					if(strategies.at(index).compare("Dense") == 0)
					{
						detector = new cv::DenseFeatureDetector(
								getFeature2D_Dense_initFeatureScale(),
								getFeature2D_Dense_featureScaleLevels(),
								getFeature2D_Dense_featureScaleMul(),
								getFeature2D_Dense_initXyStep(),
								getFeature2D_Dense_initImgBound(),
								getFeature2D_Dense_varyXyStepWithScale(),
								getFeature2D_Dense_varyImgBoundWithScale());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "Dense");
					}
					break;
				case 1:
					if(strategies.at(index).compare("Fast") == 0)
					{
						detector = new cv::FastFeatureDetector(
								getFeature2D_Fast_threshold(),
								getFeature2D_Fast_nonmaxSuppression());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "Fast");
					}
					break;
				case 2:
					if(strategies.at(index).compare("GFTT") == 0)
					{
						detector = new cv::GFTTDetector(
								getFeature2D_GFTT_maxCorners(),
								getFeature2D_GFTT_qualityLevel(),
								getFeature2D_GFTT_minDistance(),
								getFeature2D_GFTT_blockSize(),
								getFeature2D_GFTT_useHarrisDetector(),
								getFeature2D_GFTT_k());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "GFTT");
					}
					break;
				case 3:
					if(strategies.at(index).compare("MSER") == 0)
					{
						detector = new cv::MSER(
								getFeature2D_MSER_delta(),
								getFeature2D_MSER_minArea(),
								getFeature2D_MSER_maxArea(),
								getFeature2D_MSER_maxVariation(),
								getFeature2D_MSER_minDiversity(),
								getFeature2D_MSER_maxEvolution(),
								getFeature2D_MSER_areaThreshold(),
								getFeature2D_MSER_minMargin(),
								getFeature2D_MSER_edgeBlurSize());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "MSER");
					}
					break;
				case 4:
					if(strategies.at(index).compare("ORB") == 0)
					{
						detector = new cv::ORB(
								getFeature2D_ORB_nFeatures(),
								getFeature2D_ORB_scaleFactor(),
								getFeature2D_ORB_nLevels(),
								getFeature2D_ORB_edgeThreshold(),
								getFeature2D_ORB_firstLevel(),
								getFeature2D_ORB_WTA_K(),
								getFeature2D_ORB_scoreType(),
								getFeature2D_ORB_patchSize());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "ORB");
					}
					break;
				case 5:
					if(strategies.at(index).compare("SIFT") == 0)
					{
						detector = new cv::SIFT(
								getFeature2D_SIFT_nfeatures(),
								getFeature2D_SIFT_nOctaveLayers(),
								getFeature2D_SIFT_contrastThreshold(),
								getFeature2D_SIFT_edgeThreshold(),
								getFeature2D_SIFT_sigma());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "SIFT");
					}
					break;
				case 6:
					if(strategies.at(index).compare("Star") == 0)
					{
						detector = new cv::StarFeatureDetector(
								getFeature2D_Star_maxSize(),
								getFeature2D_Star_responseThreshold(),
								getFeature2D_Star_lineThresholdProjected(),
								getFeature2D_Star_lineThresholdBinarized(),
								getFeature2D_Star_suppressNonmaxSize());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "Star");
					}
					break;
				case 7:
					if(strategies.at(index).compare("SURF") == 0)
					{
						if(getFeature2D_SURF_gpu() && cv::gpu::getCudaEnabledDeviceCount())
						{
							detector = new GPUSURF(
									getFeature2D_SURF_hessianThreshold(),
									getFeature2D_SURF_nOctaves(),
									getFeature2D_SURF_nOctaveLayers(),
									getFeature2D_SURF_extended(),
									getFeature2D_SURF_upright());
						}
						else
						{
							detector = new cv::SURF(
								getFeature2D_SURF_hessianThreshold(),
								getFeature2D_SURF_nOctaves(),
								getFeature2D_SURF_nOctaveLayers(),
								getFeature2D_SURF_extended(),
								getFeature2D_SURF_upright());
						}
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "SURF");
					}
					break;
				case 8:
					if(strategies.at(index).compare("BRISK") == 0)
					{
						detector = new cv::BRISK(
								getFeature2D_BRISK_thresh(),
								getFeature2D_BRISK_octaves(),
								getFeature2D_BRISK_patternScale());
						if(VERBOSE)printf("Settings::createFeaturesDetector() type=%s\n", "BRISK");
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
	QString str = getFeature2D_2Descriptor();
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
					if(strategies.at(index).compare("Brief") == 0)
					{
						extractor = new cv::BriefDescriptorExtractor(
								getFeature2D_Brief_bytes());
						if(VERBOSE)printf("Settings::createDescriptorsExtractor() type=%s\n", "Brief");
					}
					break;
				case 1:
					if(strategies.at(index).compare("ORB") == 0)
					{
						extractor = new cv::ORB(
								getFeature2D_ORB_nFeatures(),
								getFeature2D_ORB_scaleFactor(),
								getFeature2D_ORB_nLevels(),
								getFeature2D_ORB_edgeThreshold(),
								getFeature2D_ORB_firstLevel(),
								getFeature2D_ORB_WTA_K(),
								getFeature2D_ORB_scoreType(),
								getFeature2D_ORB_patchSize());
						if(VERBOSE)printf("Settings::createDescriptorsExtractor() type=%s\n", "ORB");
					}
					break;
				case 2:
					if(strategies.at(index).compare("SIFT") == 0)
					{
						extractor = new cv::SIFT(
								getFeature2D_SIFT_nfeatures(),
								getFeature2D_SIFT_nOctaveLayers(),
								getFeature2D_SIFT_contrastThreshold(),
								getFeature2D_SIFT_edgeThreshold(),
								getFeature2D_SIFT_sigma());
						if(VERBOSE)printf("Settings::createDescriptorsExtractor() type=%s\n", "SIFT");
					}
					break;
				case 3:
					if(strategies.at(index).compare("SURF") == 0)
					{
						if(getFeature2D_SURF_gpu() && cv::gpu::getCudaEnabledDeviceCount())
						{
							extractor = new GPUSURF(
									getFeature2D_SURF_hessianThreshold(),
									getFeature2D_SURF_nOctaves(),
									getFeature2D_SURF_nOctaveLayers(),
									getFeature2D_SURF_extended(),
									getFeature2D_SURF_upright());
						}
						else
						{
							extractor = new cv::SURF(
									getFeature2D_SURF_hessianThreshold(),
									getFeature2D_SURF_nOctaves(),
									getFeature2D_SURF_nOctaveLayers(),
									getFeature2D_SURF_extended(),
									getFeature2D_SURF_upright());
						}
						if(VERBOSE)printf("Settings::createDescriptorsExtractor() type=%s\n", "SURF");
					}
					break;
				case 4:
					if(strategies.at(index).compare("BRISK") == 0)
					{
						extractor = new cv::BRISK(
								getFeature2D_BRISK_thresh(),
								getFeature2D_BRISK_octaves(),
								getFeature2D_BRISK_patternScale());
						if(VERBOSE)printf("Settings::createDescriptorsExtractor() type=%s\n", "BRISK");
					}
					break;
				case 5:
					if(strategies.at(index).compare("FREAK") == 0)
					{
						extractor = new cv::FREAK(
								getFeature2D_FREAK_orientationNormalized(),
								getFeature2D_FREAK_scaleNormalized(),
								getFeature2D_FREAK_patternScale(),
								getFeature2D_FREAK_nOctaves());
						if(VERBOSE)printf("Settings::createDescriptorsExtractor() type=%s\n", "FREAK");
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
	int index = getFeature2D_1Detector().split(':').first().toInt();
	return getFeature2D_1Detector().split(':').last().split(';').at(index);
}

QString Settings::currentDescriptorType()
{
	int index = getFeature2D_2Descriptor().split(':').first().toInt();
	return getFeature2D_2Descriptor().split(':').last().split(';').at(index);
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
	return cv::flann::SearchParams(
			getNearestNeighbor_7search_checks(),
			getNearestNeighbor_8search_eps(),
			getNearestNeighbor_9search_sorted());
}

int Settings::getHomographyMethod()
{
	int method = cv::RANSAC;
	QString str = getHomography_method();
	QStringList split = str.split(':');
	if(split.size()==2)
	{
		bool ok = false;
		int index = split.first().toInt(&ok);
		if(ok)
		{
			QStringList strategies = split.last().split(';');
			if(strategies.size() == 2 && index>=0 && index<2)
			{
				switch(method)
				{
				case 0:
					method = cv::LMEDS;
					break;
				default:
					method = cv::RANSAC;
					break;
				}
			}
		}
	}
	if(VERBOSE)printf("Settings::getHomographyMethod() method=%d\n", method);
	return method;
}
