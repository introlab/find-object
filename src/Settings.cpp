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
#include <opencv2/opencv_modules.hpp>

#if CV_MAJOR_VERSION < 3
#include <opencv2/gpu/gpu.hpp>
#define CVCUDA cv::gpu
#else
#include <opencv2/core/cuda.hpp>
#define CVCUDA cv::cuda
#ifdef HAVE_OPENCV_CUDAFEATURES2D
#include <opencv2/cudafeatures2d.hpp>
#endif
#endif

#ifdef HAVE_OPENCV_NONFREE
  #if CV_MAJOR_VERSION == 2 && CV_MINOR_VERSION >=4
  #include <opencv2/nonfree/gpu.hpp>
  #include <opencv2/nonfree/features2d.hpp>
  #endif
#endif
#ifdef HAVE_OPENCV_XFEATURES2D
  #include <opencv2/xfeatures2d.hpp>
  #include <opencv2/xfeatures2d/cuda.hpp>
#endif

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
						str[0] = index.toLatin1();
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
					str[0] = index.toLatin1();
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

	if(CVCUDA::getCudaEnabledDeviceCount() == 0)
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

#if FINDOBJECT_NONFREE == 1
class GPUSURF : public Feature2D
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

	virtual void detect(const cv::Mat & image,
    		std::vector<cv::KeyPoint> & keypoints,
    		const cv::Mat & mask = cv::Mat())
    {
    	CVCUDA::GpuMat imgGpu(image);
    	CVCUDA::GpuMat maskGpu(mask);
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

	virtual void compute( const cv::Mat& image,
    		std::vector<cv::KeyPoint>& keypoints,
    		cv::Mat& descriptors)
    {
    	std::vector<float> d;
		CVCUDA::GpuMat imgGpu(image);
		CVCUDA::GpuMat descriptorsGPU;
		try
		{
			surf_(imgGpu, CVCUDA::GpuMat(), keypoints, descriptorsGPU, true);
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

	virtual void detectAndCompute( const cv::Mat& image,
				std::vector<cv::KeyPoint>& keypoints,
				cv::Mat& descriptors,
				const cv::Mat & mask = cv::Mat())
	{
		std::vector<float> d;
		CVCUDA::GpuMat imgGpu(image);
		CVCUDA::GpuMat descriptorsGPU;
		CVCUDA::GpuMat maskGpu(mask);
		try
		{
			surf_(imgGpu, maskGpu, keypoints, descriptorsGPU, false);
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
#if CV_MAJOR_VERSION < 3
    CVCUDA::SURF_GPU surf_;
#else
    CVCUDA::SURF_CUDA surf_;
#endif
};
#endif

class GPUFAST : public Feature2D
{
public:
	GPUFAST(int threshold=Settings::defaultFeature2D_Fast_threshold(),
			bool nonmaxSuppression=Settings::defaultFeature2D_Fast_nonmaxSuppression(),
#if CV_MAJOR_VERSION < 3
			double keypointsRatio=Settings::defaultFeature2D_Fast_keypointsRatio())
	: fast_(threshold,
			  nonmaxSuppression,
			  keypointsRatio)
#else
			int max_npoints=Settings::defaultFeature2D_Fast_maxNpoints())
#ifdef HAVE_OPENCV_CUDAFEATURES2D
	: fast_(CVCUDA::FastFeatureDetector::create(
			threshold,
			nonmaxSuppression,
			CVCUDA::FastFeatureDetector::TYPE_9_16,
			max_npoints))
#endif
#endif
	{
	}
	virtual ~GPUFAST() {}

	virtual void detect(const cv::Mat & image,
			std::vector<cv::KeyPoint> & keypoints,
			const cv::Mat & mask = cv::Mat())
    {
    	CVCUDA::GpuMat imgGpu(image);
    	CVCUDA::GpuMat maskGpu(mask);
#if CV_MAJOR_VERSION < 3
    	fast_(imgGpu, maskGpu, keypoints);
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
    	CVCUDA::GpuMat keypointsGpu(keypoints);
		fast_->detectAsync(imgGpu, keypointsGpu, maskGpu);
		fast_->convert(keypointsGpu, keypoints);
#endif
#endif
    }
	virtual void compute( const cv::Mat& image,
		std::vector<cv::KeyPoint>& keypoints,
		cv::Mat& descriptors)
	{
		UERROR("GPUFAST:computeDescriptors() Should not be used!");
	}
	virtual void detectAndCompute( const cv::Mat& image,
		std::vector<cv::KeyPoint>& keypoints,
		cv::Mat& descriptors,
		const cv::Mat & mask = cv::Mat())
	{
		UERROR("GPUFAST:detectAndCompute() Should not be used!");
	}

private:
#if CV_MAJOR_VERSION < 3
    CVCUDA::FAST_GPU fast_;
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
    cv::Ptr<CVCUDA::FastFeatureDetector> fast_;
#endif
#endif
};

class GPUORB : public Feature2D
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
#if CV_MAJOR_VERSION < 3
            bool fastNonmaxSupression = Settings::defaultFeature2D_Fast_nonmaxSuppression())
		: orb_(nFeatures,
			 scaleFactor,
			 nLevels,
			 edgeThreshold ,
			 firstLevel,
			 WTA_K,
			 scoreType,
			 patchSize)
#else
			bool blurForDescriptor = Settings::defaultFeature2D_ORB_blurForDescriptor())
#ifdef HAVE_OPENCV_CUDAFEATURES2D
		: orb_(CVCUDA::ORB::create(nFeatures,
			 scaleFactor,
			 nLevels,
			 edgeThreshold ,
			 firstLevel,
			 WTA_K,
			 scoreType,
			 patchSize,
			 fastThreshold,
			 blurForDescriptor))
#endif
#endif
	{
#if CV_MAJOR_VERSION < 3
		orb_.setFastParams(fastThreshold, fastNonmaxSupression);
#endif
	}
	virtual ~GPUORB() {}

	virtual void detect(const cv::Mat & image,
			std::vector<cv::KeyPoint> & keypoints,
			const cv::Mat & mask = cv::Mat())
    {

    	CVCUDA::GpuMat imgGpu(image);
    	CVCUDA::GpuMat maskGpu(mask);

    	try
    	{
#if CV_MAJOR_VERSION < 3
    		orb_(imgGpu, maskGpu, keypoints);
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
    		CVCUDA::GpuMat keypointsGpu;
    		orb_->detectAsync(imgGpu, keypointsGpu, maskGpu);
    		orb_->convert(keypointsGpu, keypoints);
#endif
#endif
    	}
    	catch(cv::Exception &e)
		{
    		UERROR("GPUORB error: %s \n(If something about matrix size, the image/object may be too small (%d,%d).)",
					e.msg.c_str(),
					image.cols,
					image.rows);
		}
    }

	virtual void compute( const cv::Mat& image,
        		std::vector<cv::KeyPoint>& keypoints,
        		cv::Mat& descriptors)
	{
		std::vector<float> d;

		CVCUDA::GpuMat imgGpu(image);
		CVCUDA::GpuMat descriptorsGPU;

		try
		{
#if CV_MAJOR_VERSION < 3
			orb_(imgGpu, CVCUDA::GpuMat(), keypoints, descriptorsGPU); // No option to use provided keypoints!?
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
			UERROR("OpenCV 3 ORB-GPU doesn't support extracting ORB descriptors from already extracted keypoints. "
				   "Use ORB as feature detector too or desactivate ORB-GPU.");
			//orb_->computeAsync(imgGpu, keypoints, descriptorsGPU, true);
#endif
#endif
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

	virtual void detectAndCompute( const cv::Mat& image,
				std::vector<cv::KeyPoint>& keypoints,
				cv::Mat& descriptors,
				const cv::Mat & mask = cv::Mat())
	{
		std::vector<float> d;

		CVCUDA::GpuMat imgGpu(image);
		CVCUDA::GpuMat descriptorsGPU;
		CVCUDA::GpuMat maskGpu(mask);

		try
		{
#if CV_MAJOR_VERSION < 3
			orb_(imgGpu, CVCUDA::GpuMat(), keypoints, descriptorsGPU); // No option to use provided keypoints!?
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
			CVCUDA::GpuMat keypointsGpu;
			orb_->detectAndComputeAsync(imgGpu, maskGpu, keypointsGpu, descriptorsGPU, false);
			orb_->convert(keypointsGpu, keypoints);
#endif
#endif
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
#if CV_MAJOR_VERSION < 3
    CVCUDA::ORB_GPU orb_;
#else
#ifdef HAVE_OPENCV_CUDAFEATURES2D
    cv::Ptr<CVCUDA::ORB> orb_;
#endif
#endif
};

Feature2D * Settings::createKeypointDetector()
{
	Feature2D * feature2D = 0;
	QString str = getFeature2D_1Detector();
	QStringList split = str.split(':');
	if(split.size()==2)
	{
		bool ok = false;
		int index = split.first().toInt(&ok);
		if(ok)
		{
			QStringList strategies = split.last().split(';');

			if(index>=0 && index<strategies.size())
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

#if CV_MAJOR_VERSION < 3
				if(strategies.at(index).compare("AGAST") == 0 ||
				   strategies.at(index).compare("KAZE") == 0 ||
				   strategies.at(index).compare("AKAZE") == 0)
				{
					index = Settings::defaultFeature2D_1Detector().split(':').first().toInt();
					UERROR("Find-Object is built with OpenCV 2 so "
							"AGAST/KAZE/AKAZE cannot be used! Using default \"%s\" instead.",
							strategies.at(index).toStdString().c_str());

				}
#else
				if(strategies.at(index).compare("Dense") == 0)
				{
					index = Settings::defaultFeature2D_1Detector().split(':').first().toInt();
					UERROR("Find-Object is built with OpenCV 3 so "
							"Dense cannot be used! Using default \"%s\" instead.",
							strategies.at(index).toStdString().c_str());

				}
#ifndef HAVE_OPENCV_XFEATURES2D
				if(strategies.at(index).compare("Star") == 0)
				{
					index = Settings::defaultFeature2D_1Detector().split(':').first().toInt();
					UERROR("Find-Object is not built with OpenCV xfeatures2d module so "
							"Star cannot be used! Using default \"%s\" instead.",
							strategies.at(index).toStdString().c_str());

				}
#endif
#endif

				if(strategies.at(index).compare("Dense") == 0)
				{
#if CV_MAJOR_VERSION < 3
					feature2D = new Feature2D(cv::Ptr<cv::FeatureDetector>(new cv::DenseFeatureDetector(
								getFeature2D_Dense_initFeatureScale(),
								getFeature2D_Dense_featureScaleLevels(),
								getFeature2D_Dense_featureScaleMul(),
								getFeature2D_Dense_initXyStep(),
								getFeature2D_Dense_initImgBound(),
								getFeature2D_Dense_varyXyStepWithScale(),
								getFeature2D_Dense_varyImgBoundWithScale())));
#else
					UWARN("Find-Object is not built with OpenCV 2 so Dense cannot be used!");
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("Fast") == 0)
				{
					if(getFeature2D_Fast_gpu() && CVCUDA::getCudaEnabledDeviceCount())
					{
						feature2D = new GPUFAST(
								getFeature2D_Fast_threshold(),
								getFeature2D_Fast_nonmaxSuppression());
						UDEBUG("type=%s GPU", strategies.at(index).toStdString().c_str());
					}
					else
					{
#if CV_MAJOR_VERSION < 3
						feature2D = new Feature2D(cv::Ptr<cv::FeatureDetector>(new cv::FastFeatureDetector(
								getFeature2D_Fast_threshold(),
								getFeature2D_Fast_nonmaxSuppression())));
#else
						feature2D = new Feature2D(cv::FastFeatureDetector::create(
								getFeature2D_Fast_threshold(),
								getFeature2D_Fast_nonmaxSuppression()));
#endif
						UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
					}
				}
				else if(strategies.at(index).compare("AGAST") == 0)
				{
#if CV_MAJOR_VERSION < 3
					UWARN("Find-Object is not built with OpenCV 3 so AGAST cannot be used!");
#else
					feature2D = new Feature2D(cv::AgastFeatureDetector::create(
							getFeature2D_AGAST_threshold(),
							getFeature2D_AGAST_nonmaxSuppression()));
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("GFTT") == 0)
				{
#if CV_MAJOR_VERSION < 3
					feature2D = new Feature2D(cv::Ptr<cv::FeatureDetector>(new cv::GFTTDetector(
							getFeature2D_GFTT_maxCorners(),
							getFeature2D_GFTT_qualityLevel(),
							getFeature2D_GFTT_minDistance(),
							getFeature2D_GFTT_blockSize(),
							getFeature2D_GFTT_useHarrisDetector(),
							getFeature2D_GFTT_k())));
#else
					feature2D = new Feature2D(cv::GFTTDetector::create(
							getFeature2D_GFTT_maxCorners(),
							getFeature2D_GFTT_qualityLevel(),
							getFeature2D_GFTT_minDistance(),
							getFeature2D_GFTT_blockSize(),
							getFeature2D_GFTT_useHarrisDetector(),
							getFeature2D_GFTT_k()));
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("MSER") == 0)
				{
#if CV_MAJOR_VERSION < 3
					feature2D = new Feature2D(cv::Ptr<cv::FeatureDetector>(new cv::MSER(
							getFeature2D_MSER_delta(),
							getFeature2D_MSER_minArea(),
							getFeature2D_MSER_maxArea(),
							getFeature2D_MSER_maxVariation(),
							getFeature2D_MSER_minDiversity(),
							getFeature2D_MSER_maxEvolution(),
							getFeature2D_MSER_areaThreshold(),
							getFeature2D_MSER_minMargin(),
							getFeature2D_MSER_edgeBlurSize())));
#else
					feature2D = new Feature2D(cv::MSER::create(
							getFeature2D_MSER_delta(),
							getFeature2D_MSER_minArea(),
							getFeature2D_MSER_maxArea(),
							getFeature2D_MSER_maxVariation(),
							getFeature2D_MSER_minDiversity(),
							getFeature2D_MSER_maxEvolution(),
							getFeature2D_MSER_areaThreshold(),
							getFeature2D_MSER_minMargin(),
							getFeature2D_MSER_edgeBlurSize()));
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("ORB") == 0)
				{
					if(getFeature2D_ORB_gpu() && CVCUDA::getCudaEnabledDeviceCount())
					{
						feature2D = new GPUORB(
								getFeature2D_ORB_nFeatures(),
								getFeature2D_ORB_scaleFactor(),
								getFeature2D_ORB_nLevels(),
								getFeature2D_ORB_edgeThreshold(),
								getFeature2D_ORB_firstLevel(),
								getFeature2D_ORB_WTA_K(),
								getFeature2D_ORB_scoreType(),
								getFeature2D_ORB_patchSize(),
								getFeature2D_Fast_threshold(),
#if CV_MAJOR_VERSION < 3
								getFeature2D_Fast_nonmaxSuppression());
#else
								getFeature2D_ORB_blurForDescriptor());
#endif
						UDEBUG("type=%s (GPU)", strategies.at(index).toStdString().c_str());
					}
					else
					{
#if CV_MAJOR_VERSION < 3
						feature2D = new Feature2D(cv::Ptr<cv::Feature2D>(new cv::ORB(
								getFeature2D_ORB_nFeatures(),
								getFeature2D_ORB_scaleFactor(),
								getFeature2D_ORB_nLevels(),
								getFeature2D_ORB_edgeThreshold(),
								getFeature2D_ORB_firstLevel(),
								getFeature2D_ORB_WTA_K(),
								getFeature2D_ORB_scoreType(),
								getFeature2D_ORB_patchSize())));
#else
						feature2D = new Feature2D(cv::ORB::create(
								getFeature2D_ORB_nFeatures(),
								getFeature2D_ORB_scaleFactor(),
								getFeature2D_ORB_nLevels(),
								getFeature2D_ORB_edgeThreshold(),
								getFeature2D_ORB_firstLevel(),
								getFeature2D_ORB_WTA_K(),
								getFeature2D_ORB_scoreType(),
								getFeature2D_ORB_patchSize(),
								getFeature2D_Fast_threshold()));
#endif
						UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
					}
				}
				else if(strategies.at(index).compare("Star") == 0)
				{
#if CV_MAJOR_VERSION < 3
					feature2D = new Feature2D(cv::Ptr<cv::FeatureDetector>(new cv::StarFeatureDetector(
								getFeature2D_Star_maxSize(),
								getFeature2D_Star_responseThreshold(),
								getFeature2D_Star_lineThresholdProjected(),
								getFeature2D_Star_lineThresholdBinarized(),
								getFeature2D_Star_suppressNonmaxSize())));
#else
#ifdef HAVE_OPENCV_XFEATURES2D
					feature2D = new Feature2D(cv::xfeatures2d::StarDetector::create(
								getFeature2D_Star_maxSize(),
								getFeature2D_Star_responseThreshold(),
								getFeature2D_Star_lineThresholdProjected(),
								getFeature2D_Star_lineThresholdBinarized(),
								getFeature2D_Star_suppressNonmaxSize()));
#else
					UWARN("Find-Object is not built with OpenCV xfeatures2d module so Star cannot be used!");
#endif
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("BRISK") == 0)
				{
#if CV_MAJOR_VERSION < 3
					feature2D = new Feature2D(cv::Ptr<cv::Feature2D>(new cv::BRISK(
							getFeature2D_BRISK_thresh(),
							getFeature2D_BRISK_octaves(),
							getFeature2D_BRISK_patternScale())));
#else
					feature2D = new Feature2D(cv::BRISK::create(
							getFeature2D_BRISK_thresh(),
							getFeature2D_BRISK_octaves(),
							getFeature2D_BRISK_patternScale()));
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("KAZE") == 0)
				{
#if CV_MAJOR_VERSION < 3
					UWARN("Find-Object is not built with OpenCV 3 so KAZE cannot be used!");
#else
					feature2D = new Feature2D(cv::KAZE::create(
							getFeature2D_KAZE_extended(),
							getFeature2D_KAZE_upright(),
							getFeature2D_KAZE_threshold(),
							getFeature2D_KAZE_nOctaves(),
							getFeature2D_KAZE_nOctaveLayers(),
							cv::KAZE::DIFF_PM_G2)); // FIXME: make a parameter
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("AKAZE") == 0)
				{
#if CV_MAJOR_VERSION < 3
					UWARN("Find-Object is not built with OpenCV 3 so AKAZE cannot be used!");
#else
					feature2D = new Feature2D(cv::AKAZE::create(
							cv::AKAZE::DESCRIPTOR_MLDB, // FIXME: make a parameter
							getFeature2D_AKAZE_descriptorSize(),
							getFeature2D_AKAZE_descriptorChannels(),
							getFeature2D_AKAZE_threshold(),
							getFeature2D_AKAZE_nOctaves(),
							getFeature2D_AKAZE_nOctaveLayers(),
							cv::KAZE::DIFF_PM_G2)); // FIXME: make a parameter
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
#if FINDOBJECT_NONFREE == 1
				else if(strategies.at(index).compare("SIFT") == 0)
				{
#if CV_MAJOR_VERSION < 3
					feature2D = new Feature2D(cv::Ptr<cv::Feature2D>(new cv::SIFT(
							getFeature2D_SIFT_nfeatures(),
							getFeature2D_SIFT_nOctaveLayers(),
							getFeature2D_SIFT_contrastThreshold(),
							getFeature2D_SIFT_edgeThreshold(),
							getFeature2D_SIFT_sigma())));
#else
					feature2D = new Feature2D(cv::xfeatures2d::SIFT::create(
							getFeature2D_SIFT_nfeatures(),
							getFeature2D_SIFT_nOctaveLayers(),
							getFeature2D_SIFT_contrastThreshold(),
							getFeature2D_SIFT_edgeThreshold(),
							getFeature2D_SIFT_sigma()));
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("SURF") == 0)
				{
					if(getFeature2D_SURF_gpu() && CVCUDA::getCudaEnabledDeviceCount())
					{
						feature2D = new GPUSURF(
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
#if CV_MAJOR_VERSION < 3
						feature2D = new Feature2D(cv::Ptr<cv::Feature2D>(new cv::SURF(
							getFeature2D_SURF_hessianThreshold(),
							getFeature2D_SURF_nOctaves(),
							getFeature2D_SURF_nOctaveLayers(),
							getFeature2D_SURF_extended(),
							getFeature2D_SURF_upright())));
#else
						feature2D = new Feature2D(cv::xfeatures2d::SURF::create(
							getFeature2D_SURF_hessianThreshold(),
							getFeature2D_SURF_nOctaves(),
							getFeature2D_SURF_nOctaveLayers(),
							getFeature2D_SURF_extended(),
							getFeature2D_SURF_upright()));
#endif
						UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
					}
				}
#endif
			}
		}
	}

	return feature2D;
}

Feature2D * Settings::createDescriptorExtractor()
{
	Feature2D * feature2D = 0;
	QString str = getFeature2D_2Descriptor();
	QStringList split = str.split(':');
	if(split.size()==2)
	{
		bool ok = false;
		int index = split.first().toInt(&ok);
		if(ok)
		{
			QStringList strategies = split.last().split(';');
			if(index>=0 && index<strategies.size())
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

#if CV_MAJOR_VERSION < 3
				if(strategies.at(index).compare("KAZE") == 0 ||
				   strategies.at(index).compare("AKAZE") == 0)
				{
					index = Settings::defaultFeature2D_2Descriptor().split(':').first().toInt();
					UERROR("Find-Object is built with OpenCV 2 so "
							"KAZE/AKAZE cannot be used! Using default \"%s\" instead.",
							strategies.at(index).toStdString().c_str());

				}
#else
#ifndef HAVE_OPENCV_XFEATURES2D
				if(strategies.at(index).compare("Brief") == 0 ||
				   strategies.at(index).compare("FREAK") == 0 ||
				   strategies.at(index).compare("LUCID") == 0 ||
				   strategies.at(index).compare("LATCH") == 0 ||
				   strategies.at(index).compare("DAISY") == 0)
				{
					index = Settings::defaultFeature2D_2Descriptor().split(':').first().toInt();
					UERROR("Find-Object is not built with OpenCV xfeatures2d module so "
							"Brief/FREAK/LUCID/LATCH/DAISY cannot be used! Using default \"%s\" instead.",
							strategies.at(index).toStdString().c_str());

				}
#endif
#endif

				if(strategies.at(index).compare("Brief") == 0)
				{
#if CV_MAJOR_VERSION < 3
					feature2D = new Feature2D(cv::Ptr<cv::DescriptorExtractor>(new cv::BriefDescriptorExtractor(
								getFeature2D_Brief_bytes())));
#else
#ifdef HAVE_OPENCV_XFEATURES2D
					feature2D = new Feature2D(cv::xfeatures2d::BriefDescriptorExtractor::create(
								getFeature2D_Brief_bytes()));
#else
					UWARN("Find-Object is not built with OpenCV xfeatures2d module so Brief cannot be used!");
#endif
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("ORB") == 0)
				{
					if(getFeature2D_ORB_gpu() && CVCUDA::getCudaEnabledDeviceCount())
					{
						feature2D = new GPUORB(
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
#if CV_MAJOR_VERSION < 3
						feature2D = new Feature2D(cv::Ptr<cv::Feature2D>(new cv::ORB(
								getFeature2D_ORB_nFeatures(),
								getFeature2D_ORB_scaleFactor(),
								getFeature2D_ORB_nLevels(),
								getFeature2D_ORB_edgeThreshold(),
								getFeature2D_ORB_firstLevel(),
								getFeature2D_ORB_WTA_K(),
								getFeature2D_ORB_scoreType(),
								getFeature2D_ORB_patchSize())));
#else
						feature2D = new Feature2D(cv::ORB::create(
								getFeature2D_ORB_nFeatures(),
								getFeature2D_ORB_scaleFactor(),
								getFeature2D_ORB_nLevels(),
								getFeature2D_ORB_edgeThreshold(),
								getFeature2D_ORB_firstLevel(),
								getFeature2D_ORB_WTA_K(),
								getFeature2D_ORB_scoreType(),
								getFeature2D_ORB_patchSize(),
								getFeature2D_Fast_threshold()));
#endif
						UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
					}
				}
				else if(strategies.at(index).compare("BRISK") == 0)
				{
#if CV_MAJOR_VERSION < 3
					feature2D = new Feature2D(cv::Ptr<cv::Feature2D>(new cv::BRISK(
							getFeature2D_BRISK_thresh(),
							getFeature2D_BRISK_octaves(),
							getFeature2D_BRISK_patternScale())));
#else
					feature2D = new Feature2D(cv::BRISK::create(
							getFeature2D_BRISK_thresh(),
							getFeature2D_BRISK_octaves(),
							getFeature2D_BRISK_patternScale()));
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("KAZE") == 0)
				{
#if CV_MAJOR_VERSION < 3
					UWARN("Find-Object is not built with OpenCV 3 so KAZE cannot be used!");
#else
					feature2D = new Feature2D(cv::KAZE::create(
							getFeature2D_KAZE_extended(),
							getFeature2D_KAZE_upright(),
							getFeature2D_KAZE_threshold(),
							getFeature2D_KAZE_nOctaves(),
							getFeature2D_KAZE_nOctaveLayers(),
							cv::KAZE::DIFF_PM_G2)); // FIXME: make a parameter
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("AKAZE") == 0)
				{
#if CV_MAJOR_VERSION < 3
					UWARN("Find-Object is not built with OpenCV 3 so AKAZE cannot be used!");
#else
					feature2D = new Feature2D(cv::AKAZE::create(
							cv::AKAZE::DESCRIPTOR_MLDB, // FIXME: make a parameter
							getFeature2D_AKAZE_descriptorSize(),
							getFeature2D_AKAZE_descriptorChannels(),
							getFeature2D_AKAZE_threshold(),
							getFeature2D_AKAZE_nOctaves(),
							getFeature2D_AKAZE_nOctaveLayers(),
							cv::KAZE::DIFF_PM_G2)); // FIXME: make a parameter
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("FREAK") == 0)
				{
#if CV_MAJOR_VERSION < 3
					feature2D = new Feature2D(cv::Ptr<cv::DescriptorExtractor>(new cv::FREAK(
							getFeature2D_FREAK_orientationNormalized(),
							getFeature2D_FREAK_scaleNormalized(),
							getFeature2D_FREAK_patternScale(),
							getFeature2D_FREAK_nOctaves())));
#else
#ifdef HAVE_OPENCV_XFEATURES2D
					feature2D = new Feature2D(cv::xfeatures2d::FREAK::create(
							getFeature2D_FREAK_orientationNormalized(),
							getFeature2D_FREAK_scaleNormalized(),
							getFeature2D_FREAK_patternScale(),
							getFeature2D_FREAK_nOctaves()));
#else
					UWARN("Find-Object is not built with OpenCV xfeatures2d module so Freak cannot be used!");
#endif
#endif

					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
#ifdef HAVE_OPENCV_XFEATURES2D
				else if(strategies.at(index).compare("LUCID") == 0)
				{
					feature2D = new Feature2D(cv::xfeatures2d::LUCID::create(
							getFeature2D_LUCID_kernel(),
							getFeature2D_LUCID_blur_kernel()));

					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("LATCH") == 0)
				{
					feature2D = new Feature2D(cv::xfeatures2d::LATCH::create(
							getFeature2D_LATCH_bytes(),
							getFeature2D_LATCH_rotationInvariance(),
							getFeature2D_LATCH_half_ssd_size()));

					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("DAISY") == 0)
				{
					feature2D = new Feature2D(cv::xfeatures2d::DAISY::create(
							getFeature2D_DAISY_radius(),
							getFeature2D_DAISY_q_radius(),
							getFeature2D_DAISY_q_theta(),
							getFeature2D_DAISY_q_hist(),
							cv::xfeatures2d::DAISY::NRM_NONE,
							cv::noArray(),
							getFeature2D_DAISY_interpolation(),
							getFeature2D_DAISY_use_orientation()));

					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
#endif
#if FINDOBJECT_NONFREE == 1
				else if(strategies.at(index).compare("SIFT") == 0)
				{
#if CV_MAJOR_VERSION < 3
					feature2D = new Feature2D(cv::Ptr<cv::Feature2D>(new cv::SIFT(
							getFeature2D_SIFT_nfeatures(),
							getFeature2D_SIFT_nOctaveLayers(),
							getFeature2D_SIFT_contrastThreshold(),
							getFeature2D_SIFT_edgeThreshold(),
							getFeature2D_SIFT_sigma())));
#else
					feature2D = new Feature2D(cv::xfeatures2d::SIFT::create(
							getFeature2D_SIFT_nfeatures(),
							getFeature2D_SIFT_nOctaveLayers(),
							getFeature2D_SIFT_contrastThreshold(),
							getFeature2D_SIFT_edgeThreshold(),
							getFeature2D_SIFT_sigma()));
#endif
					UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
				}
				else if(strategies.at(index).compare("SURF") == 0)
				{
					if(getFeature2D_SURF_gpu() && CVCUDA::getCudaEnabledDeviceCount())
					{
						feature2D = new GPUSURF(
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
#if CV_MAJOR_VERSION < 3
						feature2D = new Feature2D(cv::Ptr<cv::Feature2D>(new cv::SURF(
								getFeature2D_SURF_hessianThreshold(),
								getFeature2D_SURF_nOctaves(),
								getFeature2D_SURF_nOctaveLayers(),
								getFeature2D_SURF_extended(),
								getFeature2D_SURF_upright())));
#else
						feature2D = new Feature2D(cv::xfeatures2d::SURF::create(
								getFeature2D_SURF_hessianThreshold(),
								getFeature2D_SURF_nOctaves(),
								getFeature2D_SURF_nOctaveLayers(),
								getFeature2D_SURF_extended(),
								getFeature2D_SURF_upright()));
#endif
						UDEBUG("type=%s", strategies.at(index).toStdString().c_str());
					}
				}
#endif
			}
		}
	}

	return feature2D;
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
#if CV_MAJOR_VERSION >= 3
				case 2:
					method = cv::RHO;
					break;
#endif
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

#if CV_MAJOR_VERSION < 3
Feature2D::Feature2D(cv::Ptr<cv::FeatureDetector> featureDetector) :
	featureDetector_(featureDetector)
{
	UASSERT(!featureDetector_.empty());
}
Feature2D::Feature2D(cv::Ptr<cv::DescriptorExtractor> descriptorExtractor) :
	descriptorExtractor_(descriptorExtractor)
{
	UASSERT(!descriptorExtractor_.empty());
}
#endif
Feature2D::Feature2D(cv::Ptr<cv::Feature2D> feature2D) :
	feature2D_(feature2D)
{
	UASSERT(!feature2D_.empty());
}

void Feature2D::detect(const cv::Mat & image,
		std::vector<cv::KeyPoint> & keypoints,
		const cv::Mat & mask)
{
#if CV_MAJOR_VERSION < 3
	if(!featureDetector_.empty())
	{
		featureDetector_->detect(image, keypoints, mask);
	}
	else
#endif
	if(!feature2D_.empty())
	{
		feature2D_->detect(image, keypoints, mask);
	}
	else
	{
		UERROR("Feature2D not set!?!?");
	}
}

void Feature2D::compute(const cv::Mat & image,
		std::vector<cv::KeyPoint> & keypoints,
		cv::Mat & descriptors)
{
#if CV_MAJOR_VERSION < 3
	if(!descriptorExtractor_.empty())
	{
		descriptorExtractor_->compute(image, keypoints, descriptors);
	}
	else
#endif
	if(!feature2D_.empty())
	{
		feature2D_->compute(image, keypoints, descriptors);
	}
	else
	{
		UERROR("Feature2D not set!?!?");
	}
}

void Feature2D::detectAndCompute(const cv::Mat & image,
		std::vector<cv::KeyPoint> & keypoints,
		cv::Mat & descriptors,
		const cv::Mat & mask)
{
	if(!feature2D_.empty())
	{
#if CV_MAJOR_VERSION < 3
		(*feature2D_)(image, mask, keypoints, descriptors);
#else
		feature2D_->detectAndCompute(image, mask, keypoints, descriptors);
#endif
	}
	else
	{
		UERROR("Cannot use Feature2D::detectAndCompute() if feature2D member is not set.");
	}
}

} // namespace find_object
