/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
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

typedef unsigned int uint;

// MACRO BEGIN

#define PARAMETER_GETTER_bool(PREFIX, NAME) \
	static bool get##PREFIX##_##NAME() {return parameters_.value(#PREFIX "/" #NAME).toBool();}
#define PARAMETER_GETTER_int(PREFIX, NAME) \
	static int get##PREFIX##_##NAME() {return parameters_.value(#PREFIX "/" #NAME).toInt();}
#define PARAMETER_GETTER_uint(PREFIX, NAME) \
	static uint get##PREFIX##_##NAME() {return parameters_.value(#PREFIX "/" #NAME).toUInt();}
#define PARAMETER_GETTER_float(PREFIX, NAME) \
	static float get##PREFIX##_##NAME() {return parameters_.value(#PREFIX "/" #NAME).toFloat();}
#define PARAMETER_GETTER_double(PREFIX, NAME) \
	static double get##PREFIX##_##NAME() {return parameters_.value(#PREFIX "/" #NAME).toDouble();}
#define PARAMETER_GETTER_QString(PREFIX, NAME) \
	static QString get##PREFIX##_##NAME() {return parameters_.value(#PREFIX "/" #NAME).toString();}

#define PARAMETER(PREFIX, NAME, TYPE, DEFAULT_VALUE) \
	public: \
		static QString k##PREFIX##_##NAME() {return QString(#PREFIX "/" #NAME);} \
		static TYPE default##PREFIX##_##NAME() {return DEFAULT_VALUE;} \
		static QString type##PREFIX##_##NAME() {return QString(#TYPE);} \
		PARAMETER_GETTER_##TYPE(PREFIX, NAME) \
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
	PARAMETER(Camera, imageWidth, int, 0);
	PARAMETER(Camera, imageHeight, int, 0);
	PARAMETER(Camera, imageRate, int, 2); // Hz
	PARAMETER(Camera, videoFilePath, QString, "");

	//List format : [Index:item0;item1;item3;...]
	PARAMETER(Detector, Type, QString, "7:Dense;Fast;GFTT;MSER;ORB;SIFT;Star;SURF");
	PARAMETER(Descriptor, Type, QString, "3:Brief;ORB;SIFT;SURF");

	PARAMETER(Brief, bytes, int, 32);

	PARAMETER(Dense, initFeatureScale, float, 1.f);
	PARAMETER(Dense, featureScaleLevels, int, 1);
	PARAMETER(Dense, featureScaleMul, float, 0.1f);
	PARAMETER(Dense, initXyStep, int, 6);
	PARAMETER(Dense, initImgBound, int, 0);
	PARAMETER(Dense, varyXyStepWithScale, bool, true);
	PARAMETER(Dense, varyImgBoundWithScale, bool, false);

	PARAMETER(Fast, threshold, int, 10);
	PARAMETER(Fast, nonmaxSuppression, bool, true);

	PARAMETER(GFTT, maxCorners, int, 1000);
	PARAMETER(GFTT, qualityLevel, double, 0.01);
	PARAMETER(GFTT, minDistance, double, 1);
	PARAMETER(GFTT, blockSize, int, 3);
	PARAMETER(GFTT, useHarrisDetector, bool, false);
	PARAMETER(GFTT, k, double, 0.04);

	PARAMETER(ORB, nFeatures, int, 500);
	PARAMETER(ORB, scaleFactor, float,  1.2f);
	PARAMETER(ORB, nLevels, int, 8);
	PARAMETER(ORB, edgeThreshold, int, 31);
	PARAMETER(ORB, firstLevel, int, 0);
	PARAMETER(ORB, WTA_K, int, 2);
	PARAMETER(ORB, scoreType, int, 0);
	PARAMETER(ORB, patchSize, int, 31);

	PARAMETER(MSER, delta, int, 5);
	PARAMETER(MSER, minArea, int, 60);
	PARAMETER(MSER, maxArea, int, 14400);
	PARAMETER(MSER, maxVariation, double, 0.25);
	PARAMETER(MSER, minDiversity, double, 0.2);
	PARAMETER(MSER, maxEvolution, int, 200);
	PARAMETER(MSER, areaThreshold, double, 1.01);
	PARAMETER(MSER, minMargin, double, 0.003);
	PARAMETER(MSER, edgeBlurSize, int, 5);

	PARAMETER(SIFT, nfeatures, int, 0);
	PARAMETER(SIFT, nOctaveLayers, int, 3);
	PARAMETER(SIFT, contrastThreshold, double, 0.04);
	PARAMETER(SIFT, edgeThreshold, double, 10);
	PARAMETER(SIFT, sigma, double, 1.6);

	PARAMETER(Star, maxSize, int, 45);
	PARAMETER(Star, responseThreshold, int, 30);
	PARAMETER(Star, lineThresholdProjected, int, 10);
	PARAMETER(Star, lineThresholdBinarized, int, 8);
	PARAMETER(Star, suppressNonmaxSize, int, 5);

	PARAMETER(SURF, hessianThreshold, double, 600.0);
	PARAMETER(SURF, nOctaves, int, 4);
	PARAMETER(SURF, nOctaveLayers, int, 2);
	PARAMETER(SURF, extended, bool, true);
	PARAMETER(SURF, upright, bool, false);

	PARAMETER(NearestNeighbor, nndrRatioUsed, bool, true);
	PARAMETER(NearestNeighbor, nndrRatio, float, 0.8f);
	PARAMETER(NearestNeighbor, minDistanceUsed, bool, false);
	PARAMETER(NearestNeighbor, minDistance, float, 1.6f);

	PARAMETER(General, autoStartCamera, bool, false);
	PARAMETER(General, autoUpdateObjects, bool, true);
	PARAMETER(General, nextObjID, uint, 1);
	PARAMETER(General, imageFormats, QString, "*.png *.jpg *.bmp *.tiff")
	PARAMETER(General, videoFormats, QString, "*.avi *.m4v")

	PARAMETER(Homography, homographyComputed, bool, true);
	PARAMETER(Homography, ransacReprojThr, double, 1.0);
	PARAMETER(Homography, minimumInliers, uint, 10);

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

private:
	Settings(){}

private:
	static ParametersMap defaultParameters_;
	static ParametersMap parameters_;
	static ParametersType parametersType_;
	static Settings dummyInit_;
};


#endif /* SETTINGS_H_ */
