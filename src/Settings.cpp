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

#include "find_object/Camera.h"
#include "find_object/Settings.h"
#include "find_object/utilite/ULogger.h"

#include <QtCore/QSettings>
#include <QtCore/QStringList>
#include <QtCore/QDir>
#include <stdio.h>
#include <opencv2/calib3d/calib3d.hpp>
#if FINDOBJECT_NONFREE == 1
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/gpu.hpp>
#endif
#include <opencv2/gpu/gpu.hpp>

namespace find_object {

ParametersMap Settings::defaultParameters_;
ParametersMap Settings::parameters_;
ParametersType Settings::parametersType_;
DescriptionsMap Settings::descriptions_;
Settings Settings::dummyInit_;
QString Settings::iniPath_;

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

QString Settings::iniPath()
{
	if(!iniPath_.isNull())
	{
		return iniPath_;
	}
	return iniDefaultPath();
}

void Settings::init(const QString & fileName)
{
	iniPath_ = fileName;
	loadSettings(iniPath_);
}

void Settings::loadSettings(const QString & fileName)
{
	QString path = fileName;
	if(fileName.isEmpty())
	{
		path = iniPath();
	}
	if(!path.isEmpty())
	{
		QSettings ini(path, QSettings::IniFormat);
		for(ParametersMap::const_iterator iter = defaultParameters_.begin(); iter!=defaultParameters_.end(); ++iter)
		{
			const QString & key = iter.key();
			QVariant value = ini.value(key, QVariant());
			if(value.isValid())
			{
				QString str = value.toString();
				if(str.contains(";"))
				{
					if(str.size() != getParameter(key).toString().size())
					{
						// If a string list is modified, update the value
						// (assuming that index < 10... one character for index)
						QChar index = str.at(0);
						str = getParameter(key).toString();
						str[0] = index.toAscii();
						value = QVariant(str);
						UINFO("Updated list of parameter \"%s\"", key.toStdString().c_str());
					}
#if FINDOBJECT_NONFREE == 0
					QChar index = str.at(0);
					if(key.compare(Settings::kFeature2D_1Detector()) == 0)
					{
						if(index == '5' || index == '7')
						{
							index = Settings::defaultFeature2D_1Detector().at(0);
							int indexInt = Settings::defaultFeature2D_1Detector().split(':').first().toInt();
							UWARN("Trying to set \"%s\" to SIFT/SURF but Find-Object isn't built "
								  "with the nonfree module from OpenCV. Keeping default combo value: %s.",
								  Settings::kFeature2D_1Detector().toStdString().c_str(),
								  Settings::defaultFeature2D_1Detector().split(':').last().split(";").at(indexInt).toStdString().c_str());
						}
					}
					else if(key.compare(Settings::kFeature2D_2Descriptor()) == 0)
					{
						if(index == '2' || index == '3')
						{
							index = Settings::defaultFeature2D_2Descriptor().at(0);
							int indexInt = Settings::defaultFeature2D_2Descriptor().split(':').first().toInt();
							UWARN("Trying to set \"%s\" to SIFT/SURF but Find-Object isn't built "
								  "with the nonfree module from OpenCV. Keeping default combo value: %s.",
								  Settings::kFeature2D_2Descriptor().toStdString().c_str(),
								  Settings::defaultFeature2D_2Descriptor().split(':').last().split(";").at(indexInt).toStdString().c_str());
						}
					}
					else if(key.compare(Settings::kNearestNeighbor_1Strategy()) == 0)
					{
						if(index <= '4')
						{
							index = Settings::defaultNearestNeighbor_1Strategy().at(0);
							int indexInt = Settings::defaultNearestNeighbor_1Strategy().split(':').first().toInt();
							UWARN("Trying to set \"%s\" to one FLANN approach but Find-Object isn't built "
									  "with the nonfree module from OpenCV and FLANN cannot be used "
									  "with binary descriptors. Keeping default combo value: %s.",
									  Settings::kNearestNeighbor_1Strategy().toStdString().c_str(),
									  Settings::defaultNearestNeighbor_1Strategy().split(':').last().split(";").at(indexInt).toStdString().c_str());
						}
					}
					str = getParameter(key).toString();
					str[0] = index.toAscii();
					value = QVariant(str);
#endif
				}
				setParameter(key, value);
			}
		}
		UINFO("Settings loaded from %s.", path.toStdString().c_str());
	}
	else
	{
		parameters_ = defaultParameters_;
		UINFO("Settings set to defaults.");
	}

	if(cv::gpu::getCudaEnabledDeviceCount() == 0)
	{
#if FINDOBJECT_NONFREE == 1
		Settings::setFeature2D_SURF_gpu(false);
#endif
		Settings::setFeature2D_Fast_gpu(false);
		Settings::setFeature2D_ORB_gpu(false);
		Settings::setNearestNeighbor_BruteForce_gpu(false);
	}
}

void Settings::loadWindowSettings(QByteArray & windowGeometry, QByteArray & windowState, const QString & fileName)
{
	QString path = fileName;
	if(fileName.isEmpty())
	{
		path = iniPath();
	}

	if(!path.isEmpty())
	{
		QSettings ini(path, QSettings::IniFormat);

		QVariant value = ini.value("windowGeometry", QVariant());
		if(value.isValid())
		{
			windowGeometry = value.toByteArray();
		}

		value = ini.value("windowState", QVariant());
		if(value.isValid())
		{
			windowState = value.toByteArray();
		}

		UINFO("Window settings loaded from %s", path.toStdString().c_str());
	}
}

void Settings::saveSettings(const QString & fileName)
{
	QString path = fileName;
	if(fileName.isEmpty())
	{
		path = iniPath();
	}
	if(!path.isEmpty())
	{
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
		UINFO("Settings saved to %s", path.toStdString().c_str());
	}
}

void Settings::saveWindowSettings(const QByteArray & windowGeometry, const QByteArray & windowState, const QString & fileName)
{
	QString path = fileName;
	if(fileName.isEmpty())
	{
		path = iniPath();
	}
	if(!path.isEmpty())
	{
		QSettings ini(path, QSettings::IniFormat);
		if(!windowGeometry.isEmpty())
		{
			ini.setValue("windowGeometry", windowGeometry);
		}
		if(!windowState.isEmpty())
		{
			ini.setValue("windowState", windowState);
		}
		UINFO("Window settings saved to %s", path.toStdString().c_str());
	}
}

class GPUFeature2D
{
public:
	GPUFeature2D() {}
	virtual ~GPUFeature2D() {}

	virtual void detectKeypoints(const cv::Mat & image,
			std::vector<cv::KeyPoint> & keypoints,
			const cv::Mat & mask = cv::Mat()) = 0;

	virtual void computeDescriptors(const cv::Mat & image,
			std::vector<cv::KeyPoint> & keypoints,
			cv::Mat & descriptors) = 0;
};

#if FINDOBJECT_NONFREE == 1
class GPUSURF : public GPUFeature2D
{
public:
	GPUSURF(double hessianThreshold,
            int nOctaves,
            int nOctaveLayers,
            bool extended,
            float keypointsRatio,
            bool upright) :
	surf_(hessianThreshold,
		  nOctaves,
		  nOctaveLayers,
		  extended,
		  keypointsRatio,
		  upright)
	{
	}
	virtual ~GPUSURF() {}

    void detectKeypoints(const cv::Mat & image,
    		std::vector<cv::KeyPoint> & keypoints,
    		const cv::Mat & mask = cv::Mat())
    {
    	cv::gpu::GpuMat imgGpu(image);
    	cv::gpu::GpuMat maskGpu(mask);
		try
		{
			surf_(imgGpu, maskGpu, keypoints);
		}
		catch(cv::Exception &e)
		{
			UERROR("GPUSURF error: %s \n(If something about layer_rows, parameter nOctaves=%d of SURF "
					"is too high for the size of the image (%d,%d).)",
					e.msg.c_str(),
					surf_.nOctaves,
					image.cols,
					image.rows);
		}
    }

    void computeDescriptors( const cv::Mat& image,
    		std::vector<cv::KeyPoint>& keypoints,
    		cv::Mat& descriptors)
    {
    	std::vector<float> d;
		cv::gpu::GpuMat imgGpu(image);
		cv::gpu::GpuMat descriptorsGPU;
		try
		{
			surf_(imgGpu, cv::gpu::GpuMat(), keypoints, descriptorsGPU, true);
    	}
		catch(cv::Exception &e)
		{
			UERROR("GPUSURF error: %s \n(If something about layer_rows, parameter nOctaves=%d of SURF "
					"is too high for the size of the image (%d,%d).)",
					e.msg.c_str(),
					surf_.nOctaves,
					image.cols,
					image.rows);
		}

		// Download descriptors
		if (descriptorsGPU.empty())
			descriptors = cv::Mat();
		else
		{
			UASSERT(descriptorsGPU.type() == CV_32F);
			descriptors = cv::Mat(descriptorsGPU.size(), CV_32F);
			descriptorsGPU.download(descriptors);
		}
    }
private:
    cv::gpu::SURF_GPU surf_; // HACK: static because detectImpl() is const!
};
#endif

class GPUFAST : public GPUFeature2D
{
public:
	GPUFAST(int threshold=Settings::defaultFeature2D_Fast_threshold(),
			bool nonmaxSuppression=Settings::defaultFeature2D_Fast_nonmaxSuppression(),
			double keypointsRatio=Settings::defaultFeature2D_Fast_keypointsRatio()) :
		fast_(threshold,
			  nonmaxSuppression,
			  keypointsRatio)
	{
	}
	virtual ~GPUFAST() {}

protected:
	void detectKeypoints(const cv::Mat & image,
			std::vector<cv::KeyPoint> & keypoints,
			const cv::Mat & mask = cv::Mat())
    {
    	cv::gpu::GpuMat imgGpu(image);
    	cv::gpu::GpuMat maskGpu(mask);
    	fast_(imgGpu, maskGpu, keypoints);
    }
	void computeDescriptors( const cv::Mat& image,
		std::vector<cv::KeyPoint>& keypoints,
		cv::Mat& descriptors)
	{
		UERROR("GPUFAST:computeDescriptors() Should not be used!");
	}

private:
    cv::gpu::FAST_GPU fast_;
};

class GPUORB : public GPUFeature2D
{
public:
	GPUORB(int nFeatures = Settings::defaultFeature2D_ORB_nFeatures(),
			float scaleFactor = Settings::defaultFeature2D_ORB_scaleFactor(),
			int nLevels = Settings::defaultFeature2D_ORB_nLevels(),
			int edgeThreshold = Settings::defaultFeature2D_ORB_edgeThreshold(),
            int firstLevel = Settings::defaultFeature2D_ORB_firstLevel(),
            int WTA_K = Settings::defaultFeature2D_ORB_WTA_K(),
            int scoreType = Settings::defaultFeature2D_ORB_scoreType(),
            int patchSize = Settings::defaultFeature2D_ORB_patchSize(),
            int fastThreshold = Settings::defaultFeature2D_Fast_threshold(),
            bool fastNonmaxSupression = Settings::defaultFeature2D_Fast_nonmaxSuppression()) :
		orb_(nFeatures,
			 scaleFactor,
			 nLevels,
			 edgeThreshold ,
			 firstLevel,
			 WTA_K,
			 scoreType,
			 patchSize)
	{
		orb_.setFastParams(fastThreshold, fastNonmaxSupression);
	}
	virtual ~GPUORB() {}

protected:
	void detectKeypoints(const cv::Mat & image,
			std::vector<cv::KeyPoint> & keypoints,
			const cv::Mat & mask = cv::Mat())
    {
    	cv::gpu::GpuMat imgGpu(image);
    	cv::gpu::GpuMat maskGpu(mask);
    	try
    	{
    		orb_(imgGpu, maskGpu, keypoints);
    	}
    	catch(cv::Exception &e)
		{
    		UERROR("GPUORB error: %s \n(If something about matrix size, the image/object may be too small (%d,%d).)",
					e.msg.c_str(),
					image.cols,
					image.rows);
		}
    }

    void computeDescriptors( const cv::Mat& image,
        		std::vector<cv::KeyPoint>& keypoints,
        		cv::Mat& descriptors)
	{
		std::vector<float> d;
		cv::gpu::GpuMat imgGpu(image);
		cv::gpu::GpuMat descriptorsGPU;
		try
		{
			orb_(imgGpu, cv::gpu::GpuMat(), keypoints, descriptorsGPU); // No option to use provided keypoints!?
		}
		catch(cv::Exception &e)
		{
			UERROR("GPUORB error: %s \n(If something about matrix size, the image/object may be too small (%d,%d).)",
					e.msg.c_str(),
					image.cols,
					image.rows);
		}
		// Download descriptors
		if (descriptorsGPU.empty())
			descriptors = cv::Mat();
		else
		{
			UASSERT(descriptorsGPU.type() == CV_8U);
			descriptors = cv::Mat(descriptorsGPU.size(), CV_8U);
			descriptorsGPU.download(descriptors);
		}
	}
private:
    cv::gpu::ORB_GPU orb_;
};

KeypointDetector * Settings::createKeypointDetector()
{
	cv::FeatureDetector * detector = 0;
	GPUFeature2D * detectorGPU = 0;
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

#if FINDOBJECT_NONFREE == 0
				//check for nonfree stuff
				if(strategies.at(index).compare("SIFT") == 0 ||
				   strategies.at(index).compare("SURF") == 0)
				{
					index = Settings::defaultFeature2D_1Detector().split(':').first().toInt();
					UERROR("Find-Object is not built with OpenCV nonfree module so "
							"SIFT/SURF cannot be used! Using default \"%s\" instead.",
							strategies.at(index).toStdString().c_str());

				}
#endif

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
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("Fast") == 0)
				{
					if(getFeature2D_Fast_gpu() && cv::gpu::getCudaEnabledDeviceCount())
					{
						detectorGPU = new GPUFAST(
								getFeature2D_Fast_threshold(),
								getFeature2D_Fast_nonmaxSuppression());
						UDEBUG("type=%s GPU", strategies.at(index).toStdString().c_str());
					}
					else
					{
						detector = new cv::FastFeatureDetector(
								getFeature2D_Fast_threshold(),
								getFeature2D_Fast_nonmaxSuppression());
						UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
					}
				}
				else if(strategies.at(index).compare("GFTT") == 0)
				{
					detector = new cv::GFTTDetector(
							getFeature2D_GFTT_maxCorners(),
							getFeature2D_GFTT_qualityLevel(),
							getFeature2D_GFTT_minDistance(),
							getFeature2D_GFTT_blockSize(),
							getFeature2D_GFTT_useHarrisDetector(),
							getFeature2D_GFTT_k());
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("MSER") == 0)
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
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("ORB") == 0)
				{
					if(getFeature2D_ORB_gpu() && cv::gpu::getCudaEnabledDeviceCount())
					{
						detectorGPU = new GPUORB(
								getFeature2D_ORB_nFeatures(),
								getFeature2D_ORB_scaleFactor(),
								getFeature2D_ORB_nLevels(),
								getFeature2D_ORB_edgeThreshold(),
								getFeature2D_ORB_firstLevel(),
								getFeature2D_ORB_WTA_K(),
								getFeature2D_ORB_scoreType(),
								getFeature2D_ORB_patchSize(),
								getFeature2D_Fast_threshold(),
								getFeature2D_Fast_nonmaxSuppression());
						UDEBUG("type=%s (GPU)", strategies.at(index).toStdString().c_str());
					}
					else
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
						UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
					}
				}
				else if(strategies.at(index).compare("Star") == 0)
				{
					detector = new cv::StarFeatureDetector(
							getFeature2D_Star_maxSize(),
							getFeature2D_Star_responseThreshold(),
							getFeature2D_Star_lineThresholdProjected(),
							getFeature2D_Star_lineThresholdBinarized(),
							getFeature2D_Star_suppressNonmaxSize());
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("BRISK") == 0)
				{
					detector = new cv::BRISK(
							getFeature2D_BRISK_thresh(),
							getFeature2D_BRISK_octaves(),
							getFeature2D_BRISK_patternScale());
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
#if FINDOBJECT_NONFREE == 1
				else if(strategies.at(index).compare("SIFT") == 0)
				{
					detector = new cv::SIFT(
							getFeature2D_SIFT_nfeatures(),
							getFeature2D_SIFT_nOctaveLayers(),
							getFeature2D_SIFT_contrastThreshold(),
							getFeature2D_SIFT_edgeThreshold(),
							getFeature2D_SIFT_sigma());
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("SURF") == 0)
				{
					if(getFeature2D_SURF_gpu() && cv::gpu::getCudaEnabledDeviceCount())
					{
						detectorGPU = new GPUSURF(
								getFeature2D_SURF_hessianThreshold(),
								getFeature2D_SURF_nOctaves(),
								getFeature2D_SURF_nOctaveLayers(),
								getFeature2D_SURF_extended(),
								getFeature2D_SURF_keypointsRatio(),
								getFeature2D_SURF_upright());
						UDEBUG("type=%s (GPU)", strategies.at(index).toStdString().c_str());
					}
					else
					{
						detector = new cv::SURF(
							getFeature2D_SURF_hessianThreshold(),
							getFeature2D_SURF_nOctaves(),
							getFeature2D_SURF_nOctaveLayers(),
							getFeature2D_SURF_extended(),
							getFeature2D_SURF_upright());
						UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
					}
				}
#endif
			}
		}
	}

	UASSERT(detectorGPU!=0 || detector!=0);
	if(detectorGPU)
	{
		return new KeypointDetector(detectorGPU);
	}
	else
	{
		return new KeypointDetector(detector);
	}
}

DescriptorExtractor * Settings::createDescriptorExtractor()
{
	cv::DescriptorExtractor * extractor = 0;
	GPUFeature2D * extractorGPU = 0;
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

#if FINDOBJECT_NONFREE == 0
				//check for nonfree stuff
				if(strategies.at(index).compare("SIFT") == 0 ||
				   strategies.at(index).compare("SURF") == 0)
				{
					index = Settings::defaultFeature2D_2Descriptor().split(':').first().toInt();
					UERROR("Find-Object is not built with OpenCV nonfree module so "
							"SIFT/SURF cannot be used! Using default \"%s\" instead.",
							strategies.at(index).toStdString().c_str());

				}
#endif

				if(strategies.at(index).compare("Brief") == 0)
				{
					extractor = new cv::BriefDescriptorExtractor(
							getFeature2D_Brief_bytes());
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("ORB") == 0)
				{
					if(getFeature2D_ORB_gpu() && cv::gpu::getCudaEnabledDeviceCount())
					{
						extractorGPU = new GPUORB(
								getFeature2D_ORB_nFeatures(),
								getFeature2D_ORB_scaleFactor(),
								getFeature2D_ORB_nLevels(),
								getFeature2D_ORB_edgeThreshold(),
								getFeature2D_ORB_firstLevel(),
								getFeature2D_ORB_WTA_K(),
								getFeature2D_ORB_scoreType(),
								getFeature2D_ORB_patchSize(),
								getFeature2D_Fast_threshold(),
								getFeature2D_Fast_nonmaxSuppression());
						UDEBUG("type=%s (GPU)", strategies.at(index).toStdString().c_str());
					}
					else
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
						UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
					}
				}
				else if(strategies.at(index).compare("BRISK") == 0)
				{
					extractor = new cv::BRISK(
							getFeature2D_BRISK_thresh(),
							getFeature2D_BRISK_octaves(),
							getFeature2D_BRISK_patternScale());
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("FREAK") == 0)
				{
					extractor = new cv::FREAK(
							getFeature2D_FREAK_orientationNormalized(),
							getFeature2D_FREAK_scaleNormalized(),
							getFeature2D_FREAK_patternScale(),
							getFeature2D_FREAK_nOctaves());
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
#if FINDOBJECT_NONFREE == 1
				else if(strategies.at(index).compare("SIFT") == 0)
				{
					extractor = new cv::SIFT(
							getFeature2D_SIFT_nfeatures(),
							getFeature2D_SIFT_nOctaveLayers(),
							getFeature2D_SIFT_contrastThreshold(),
							getFeature2D_SIFT_edgeThreshold(),
							getFeature2D_SIFT_sigma());
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("SURF") == 0)
				{
					if(getFeature2D_SURF_gpu() && cv::gpu::getCudaEnabledDeviceCount())
					{
						extractorGPU = new GPUSURF(
								getFeature2D_SURF_hessianThreshold(),
								getFeature2D_SURF_nOctaves(),
								getFeature2D_SURF_nOctaveLayers(),
								getFeature2D_SURF_extended(),
								getFeature2D_SURF_keypointsRatio(),
								getFeature2D_SURF_upright());
						UDEBUG("type=%s (GPU)", strategies.at(index).toStdString().c_str());
					}
					else
					{
						extractor = new cv::SURF(
								getFeature2D_SURF_hessianThreshold(),
								getFeature2D_SURF_nOctaves(),
								getFeature2D_SURF_nOctaveLayers(),
								getFeature2D_SURF_extended(),
								getFeature2D_SURF_upright());
						UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
					}
				}
#endif
			}
		}
	}

	UASSERT(extractorGPU!=0 || extractor!=0);
	if(extractorGPU)
	{
		return new DescriptorExtractor(extractorGPU);
	}
	else
	{
		return new DescriptorExtractor(extractor);
	}
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

bool Settings::isBruteForceNearestNeighbor()
{
	bool bruteForce = false;
	QString str = getNearestNeighbor_1Strategy();
	QStringList split = str.split(':');
	if(split.size()==2)
	{
		bool ok = false;
		int index = split.first().toInt(&ok);
		if(ok)
		{
			QStringList strategies = split.last().split(';');
			if(strategies.size() >= 7 && index == 6)
			{
				bruteForce = true;
			}
		}
	}
	return bruteForce;
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
			if(strategies.size() >= 6 && index>=0 && index<6)
			{
				switch(index)
				{
				case 0:
					if(strategies.at(index).compare("Linear") == 0)
					{
						UDEBUG("type=%s", "Linear");
						params = new cv::flann::LinearIndexParams();
					}
					break;
				case 1:
					if(strategies.at(index).compare("KDTree") == 0)
					{
						UDEBUG("type=%s", "KDTree");
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
						UDEBUG("type=%s", "KMeans");
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
						UDEBUG("type=%s", "Composite");
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
						UDEBUG("type=%s", "Autotuned");
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
						UDEBUG("type=%s", "Lsh");
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
		UERROR("NN strategy not found !? Using default KDTRee...");
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
			if(strategies.size() == 9 && index>=0 && index<=8)
			{
				distance = (cvflann::flann_distance_t)(index+1);
			}
		}
	}
	return distance;
}

cv::flann::SearchParams Settings::getFlannSearchParams()
{
	return cv::flann::SearchParams(
			getNearestNeighbor_search_checks(),
			getNearestNeighbor_search_eps(),
			getNearestNeighbor_search_sorted());
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
	UDEBUG("method=%d", method);
	return method;
}

KeypointDetector::KeypointDetector(cv::FeatureDetector * featureDetector) :
	featureDetector_(featureDetector),
	gpuFeature2D_(0)
{
	UASSERT(featureDetector_!=0);
}
KeypointDetector::KeypointDetector(GPUFeature2D * gpuFeature2D) :
	featureDetector_(0),
	gpuFeature2D_(gpuFeature2D)
{
	UASSERT(gpuFeature2D_!=0);
}
void KeypointDetector::detect(const cv::Mat & image,
		std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & mask)
{
	if(featureDetector_)
	{
		featureDetector_->detect(image, keypoints, mask);
	}
	else // assume GPU
	{
		gpuFeature2D_->detectKeypoints(image, keypoints, mask);
	}
}

DescriptorExtractor::DescriptorExtractor(cv::DescriptorExtractor * descriptorExtractor) :
	descriptorExtractor_(descriptorExtractor),
	gpuFeature2D_(0)
{
	UASSERT(descriptorExtractor_!=0);
}
DescriptorExtractor::DescriptorExtractor(GPUFeature2D * gpuFeature2D) :
	descriptorExtractor_(0),
	gpuFeature2D_(gpuFeature2D)
{
	UASSERT(gpuFeature2D_!=0);
}
void DescriptorExtractor::compute(const cv::Mat & image,
		std::vector<cv::KeyPoint> & keypoints,
		cv::Mat & descriptors)
{
	if(descriptorExtractor_)
	{
		descriptorExtractor_->compute(image, keypoints, descriptors);
	}
	else // assume GPU
	{
		gpuFeature2D_->computeDescriptors(image, keypoints, descriptors);
	}
}

} // namespace find_object
