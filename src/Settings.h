/*
 * Settings.h
 *
 *  Created on: 2011-10-22
 *      Author: matlab
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include <QtCore/QMap>
#include <QtCore/QVariant>
#include <QtCore/QByteArray>
#include <opencv2/features2d/features2d.hpp>

class Camera;

typedef QMap<QString, QVariant> ParametersMap; // Key, value
typedef QMap<QString, QString> ParametersType; // Key, type

// MACRO BEGIN
#define PARAMETER(PREFIX, NAME, TYPE, DEFAULT_VALUE) \
	public: \
		static QString k##PREFIX##_##NAME() {return QString(#PREFIX "/" #NAME);} \
		static TYPE default##PREFIX##_##NAME() {return DEFAULT_VALUE;} \
		static QString type##PREFIX##_##NAME() {return QString(#TYPE);} \
		static QVariant get##PREFIX##_##NAME() {return parameters_.value(#PREFIX "/" #NAME);} \
		static void set##PREFIX##_##NAME(const TYPE & value) {parameters_[#PREFIX "/" #NAME] = value;} \
	private: \
		class Dummy##PREFIX##_##NAME { \
		public: \
			Dummy##PREFIX##_##NAME() { \
				defaultParameters_.insert(#PREFIX "/" #NAME, QVariant(DEFAULT_VALUE)); \
				parameters_.insert(#PREFIX "/" #NAME, DEFAULT_VALUE); \
				parametersType_.insert(#PREFIX "/" #NAME, #TYPE);} \
		}; \
		Dummy##PREFIX##_##NAME dummy##PREFIX##_##NAME;
// MACRO END

class Settings
{
	PARAMETER(Camera, deviceId, int, 0);
	PARAMETER(Camera, imageWidth, int, 640);
	PARAMETER(Camera, imageHeight, int, 480);
	PARAMETER(Camera, imageRate, int, 2); // Hz

	//List format : [Index:item0;item1;item3;...]
	PARAMETER(Detector, Type, QString, "7:Dense;Fast;GoodFeaturesToTrack;Mser;Orb;Sift;Star;Surf");
	PARAMETER(Descriptor, Type, QString, "3:Brief;Orb;Sift;Surf");

	PARAMETER(Brief, bytes, int, 32);

	PARAMETER(Dense, initFeatureScale, float, cv::DenseFeatureDetector::Params().initFeatureScale);
	PARAMETER(Dense, featureScaleLevels, int, cv::DenseFeatureDetector::Params().featureScaleLevels);
	PARAMETER(Dense, featureScaleMul, float, cv::DenseFeatureDetector::Params().featureScaleMul);
	PARAMETER(Dense, initXyStep, int, cv::DenseFeatureDetector::Params().initXyStep);
	PARAMETER(Dense, initImgBound, int, cv::DenseFeatureDetector::Params().initImgBound);
	PARAMETER(Dense, varyXyStepWithScale, bool, cv::DenseFeatureDetector::Params().varyXyStepWithScale);
	PARAMETER(Dense, varyImgBoundWithScale, bool, cv::DenseFeatureDetector::Params().varyImgBoundWithScale);

	PARAMETER(Fast, threshold, int, 20);
	PARAMETER(Fast, nonmaxSuppression, bool, true);

	PARAMETER(GoodFeaturesToTrack, maxCorners, int, cv::GoodFeaturesToTrackDetector::Params().maxCorners);
	PARAMETER(GoodFeaturesToTrack, qualityLevel, double, cv::GoodFeaturesToTrackDetector::Params().qualityLevel);
	PARAMETER(GoodFeaturesToTrack, minDistance, double, cv::GoodFeaturesToTrackDetector::Params().minDistance);
	PARAMETER(GoodFeaturesToTrack, blockSize, int, cv::GoodFeaturesToTrackDetector::Params().blockSize);
	PARAMETER(GoodFeaturesToTrack, useHarrisDetector, bool, cv::GoodFeaturesToTrackDetector::Params().useHarrisDetector);
	PARAMETER(GoodFeaturesToTrack, k, double, cv::GoodFeaturesToTrackDetector::Params().k);

	PARAMETER(Orb, nFeatures, unsigned int, 700);
	PARAMETER(Orb, scaleFactor, float, cv::ORB::CommonParams().scale_factor_);
	PARAMETER(Orb, nLevels, unsigned int, cv::ORB::CommonParams().n_levels_);
	PARAMETER(Orb, firstLevel, unsigned int, cv::ORB::CommonParams().first_level_);
	PARAMETER(Orb, edgeThreshold, int, cv::ORB::CommonParams().edge_threshold_);

	PARAMETER(Mser, delta, int, cvMSERParams().delta);
	PARAMETER(Mser, minArea, int, cvMSERParams().minArea);
	PARAMETER(Mser, maxArea, int, cvMSERParams().maxArea);
	PARAMETER(Mser, maxVariation, float, cvMSERParams().maxVariation);
	PARAMETER(Mser, minDiversity, float, cvMSERParams().minDiversity);
	PARAMETER(Mser, maxEvolution, int, cvMSERParams().maxEvolution);
	PARAMETER(Mser, areaThreshold, double, cvMSERParams().areaThreshold);
	PARAMETER(Mser, minMargin, double, cvMSERParams().minMargin);
	PARAMETER(Mser, edgeBlurSize, int, cvMSERParams().edgeBlurSize);

	PARAMETER(Sift, threshold, double, cv::SIFT::DetectorParams::GET_DEFAULT_THRESHOLD());
	PARAMETER(Sift, edgeThreshold, double, cv::SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
	PARAMETER(Sift, nOctaves, int, cv::SIFT::CommonParams::DEFAULT_NOCTAVES);
	PARAMETER(Sift, nOctaveLayers, int, cv::SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS);
	PARAMETER(Sift, firstOctave, int, cv::SIFT::CommonParams::DEFAULT_FIRST_OCTAVE);
	PARAMETER(Sift, angleMode, int, cv::SIFT::CommonParams::FIRST_ANGLE);
	PARAMETER(Sift, magnification, double, cv::SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION());
	PARAMETER(Sift, isNormalize, bool, cv::SIFT::DescriptorParams::DEFAULT_IS_NORMALIZE);
	PARAMETER(Sift, recalculateAngles, bool, true);

	PARAMETER(Star, maxSize, int, cvStarDetectorParams().maxSize);
	PARAMETER(Star, responseThreshold, int, cvStarDetectorParams().responseThreshold);
	PARAMETER(Star, lineThresholdProjected, int, cvStarDetectorParams().lineThresholdProjected);
	PARAMETER(Star, lineThresholdBinarized, int, cvStarDetectorParams().lineThresholdBinarized);
	PARAMETER(Star, suppressNonmaxSize, int, cvStarDetectorParams().suppressNonmaxSize);

	PARAMETER(Surf, hessianThreshold, double, 600.0);
	PARAMETER(Surf, octaves, int, 3);
	PARAMETER(Surf, octaveLayers, int, 4);
	PARAMETER(Surf, upright, bool, false);
	PARAMETER(Surf, extended, bool, false);

	PARAMETER(NearestNeighbor, nndrRatio, float, 0.8f); // NNDR RATIO

	PARAMETER(General, nextObjID, unsigned int, 1);

	PARAMETER(Homography, ransacReprojThr, double, 1.0);
	PARAMETER(Homography, minimumInliers, int, 10);

public:
	virtual ~Settings(){}

	static QString workingDirectory();
	static QString iniDefaultPath();
	static QString iniDefaultFileName() {return "config.ini";}

	static void loadSettings(const QString & fileName = QString(), QByteArray * windowGeometry = 0);
	static void saveSettings(const QString & fileName = QString(), const QByteArray & windowGeometry = QByteArray());

	static const ParametersMap & getDefaultParameters() {return defaultParameters_;}
	static const ParametersMap & getParameters() {return parameters_;}
	static const ParametersType & getParametersType() {return parametersType_;}
	static void setParameter(const QString & key, const QVariant & value) {if(parameters_.contains(key))parameters_[key] = value;}
	static void resetParameter(const QString & key) {if(defaultParameters_.contains(key)) parameters_.insert(key, defaultParameters_.value(key));}
	static QVariant getParameter(const QString & key) {return parameters_.value(key, QVariant());}

	static cv::FeatureDetector * createFeaturesDetector();
	static cv::DescriptorExtractor * createDescriptorsExtractor();

	static QString currentDescriptorType();
	static QString currentDetectorType();

	static Camera * createCamera();

private:
	Settings(){}

private:
	static ParametersMap defaultParameters_;
	static ParametersMap parameters_;
	static ParametersType parametersType_;
	static Settings dummyInit_;
};


#endif /* SETTINGS_H_ */
