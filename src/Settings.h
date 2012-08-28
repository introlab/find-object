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
	PARAMETER(Camera, 1deviceId, int, 0);
	PARAMETER(Camera, 2imageWidth, int, 640);
	PARAMETER(Camera, 3imageHeight, int, 480);
	PARAMETER(Camera, 4imageRate, double, 2.0); // Hz
	PARAMETER(Camera, 5mediaPath, QString, "");

	//List format : [Index:item0;item1;item3;...]
	PARAMETER(Detector_Descriptor, 1Detector, QString, "7:Dense;Fast;GFTT;MSER;ORB;SIFT;Star;SURF");
	PARAMETER(Detector_Descriptor, 2Descriptor, QString, "3:Brief;ORB;SIFT;SURF");

	PARAMETER(Detector_Descriptor, Brief_bytes, int, 32);

	PARAMETER(Detector_Descriptor, Dense_initFeatureScale, float, 1.f);
	PARAMETER(Detector_Descriptor, Dense_featureScaleLevels, int, 1);
	PARAMETER(Detector_Descriptor, Dense_featureScaleMul, float, 0.1f);
	PARAMETER(Detector_Descriptor, Dense_initXyStep, int, 6);
	PARAMETER(Detector_Descriptor, Dense_initImgBound, int, 0);
	PARAMETER(Detector_Descriptor, Dense_varyXyStepWithScale, bool, true);
	PARAMETER(Detector_Descriptor, Dense_varyImgBoundWithScale, bool, false);

	PARAMETER(Detector_Descriptor, Fast_threshold, int, 10);
	PARAMETER(Detector_Descriptor, Fast_nonmaxSuppression, bool, true);

	PARAMETER(Detector_Descriptor, GFTT_maxCorners, int, 1000);
	PARAMETER(Detector_Descriptor, GFTT_qualityLevel, double, 0.01);
	PARAMETER(Detector_Descriptor, GFTT_minDistance, double, 1);
	PARAMETER(Detector_Descriptor, GFTT_blockSize, int, 3);
	PARAMETER(Detector_Descriptor, GFTT_useHarrisDetector, bool, false);
	PARAMETER(Detector_Descriptor, GFTT_k, double, 0.04);

	PARAMETER(Detector_Descriptor, ORB_nFeatures, int, 500);
	PARAMETER(Detector_Descriptor, ORB_scaleFactor, float,  1.2f);
	PARAMETER(Detector_Descriptor, ORB_nLevels, int, 8);
	PARAMETER(Detector_Descriptor, ORB_edgeThreshold, int, 31);
	PARAMETER(Detector_Descriptor, ORB_firstLevel, int, 0);
	PARAMETER(Detector_Descriptor, ORB_WTA_K, int, 2);
	PARAMETER(Detector_Descriptor, ORB_scoreType, int, 0);
	PARAMETER(Detector_Descriptor, ORB_patchSize, int, 31);

	PARAMETER(Detector_Descriptor, MSER_delta, int, 5);
	PARAMETER(Detector_Descriptor, MSER_minArea, int, 60);
	PARAMETER(Detector_Descriptor, MSER_maxArea, int, 14400);
	PARAMETER(Detector_Descriptor, MSER_maxVariation, double, 0.25);
	PARAMETER(Detector_Descriptor, MSER_minDiversity, double, 0.2);
	PARAMETER(Detector_Descriptor, MSER_maxEvolution, int, 200);
	PARAMETER(Detector_Descriptor, MSER_areaThreshold, double, 1.01);
	PARAMETER(Detector_Descriptor, MSER_minMargin, double, 0.003);
	PARAMETER(Detector_Descriptor, MSER_edgeBlurSize, int, 5);

	PARAMETER(Detector_Descriptor, SIFT_nfeatures, int, 0);
	PARAMETER(Detector_Descriptor, SIFT_nOctaveLayers, int, 3);
	PARAMETER(Detector_Descriptor, SIFT_contrastThreshold, double, 0.04);
	PARAMETER(Detector_Descriptor, SIFT_edgeThreshold, double, 10);
	PARAMETER(Detector_Descriptor, SIFT_sigma, double, 1.6);

	PARAMETER(Detector_Descriptor, Star_maxSize, int, 45);
	PARAMETER(Detector_Descriptor, Star_responseThreshold, int, 30);
	PARAMETER(Detector_Descriptor, Star_lineThresholdProjected, int, 10);
	PARAMETER(Detector_Descriptor, Star_lineThresholdBinarized, int, 8);
	PARAMETER(Detector_Descriptor, Star_suppressNonmaxSize, int, 5);

	PARAMETER(Detector_Descriptor, SURF_hessianThreshold, double, 600.0);
	PARAMETER(Detector_Descriptor, SURF_nOctaves, int, 4);
	PARAMETER(Detector_Descriptor, SURF_nOctaveLayers, int, 2);
	PARAMETER(Detector_Descriptor, SURF_extended, bool, true);
	PARAMETER(Detector_Descriptor, SURF_upright, bool, false);

	PARAMETER(NearestNeighbor, 1Strategy, QString, "1:Linear;KDTree;KMeans;Composite;Autotuned;Lsh");
	PARAMETER(NearestNeighbor, 2Distance_type, QString, "0:EUCLIDEAN_L2;MANHATTAN_L1;MINKOWSKI;MAX;HIST_INTERSECT;HELLINGER;CHI_SQUARE_CS;KULLBACK_LEIBLER_KL");
	PARAMETER(NearestNeighbor, 3nndrRatioUsed, bool, true);
	PARAMETER(NearestNeighbor, 4nndrRatio, float, 0.8f);
	PARAMETER(NearestNeighbor, 5minDistanceUsed, bool, false);
	PARAMETER(NearestNeighbor, 6minDistance, float, 1.6f);

	PARAMETER(NearestNeighbor, KDTree_trees, int, 4);

	PARAMETER(NearestNeighbor, Composite_trees, int, 4);
	PARAMETER(NearestNeighbor, Composite_branching, int, 32);
	PARAMETER(NearestNeighbor, Composite_iterations, int, 11);
	PARAMETER(NearestNeighbor, Composite_centers_init, QString, "0:RANDOM;GONZALES;KMEANSPP");
	PARAMETER(NearestNeighbor, Composite_cb_index, double, 0.2);

	PARAMETER(NearestNeighbor, Autotuned_target_precision, double, 0.8);
	PARAMETER(NearestNeighbor, Autotuned_build_weight, double, 0.01);
	PARAMETER(NearestNeighbor, Autotuned_memory_weight, double, 0);
	PARAMETER(NearestNeighbor, Autotuned_sample_fraction, double, 0.1);

	PARAMETER(NearestNeighbor, KMeans_branching, int, 32);
	PARAMETER(NearestNeighbor, KMeans_iterations, int, 11);
	PARAMETER(NearestNeighbor, KMeans_centers_init, QString, "0:RANDOM;GONZALES;KMEANSPP");
	PARAMETER(NearestNeighbor, KMeans_cb_index, double, 0.2);

	PARAMETER(NearestNeighbor, Lsh_table_number, int, 20);
	PARAMETER(NearestNeighbor, Lsh_key_size, int, 10);
	PARAMETER(NearestNeighbor, Lsh_multi_probe_level, int, 2);

	PARAMETER(General, autoStartCamera, bool, false);
	PARAMETER(General, autoUpdateObjects, bool, true);
	PARAMETER(General, nextObjID, uint, 1);
	PARAMETER(General, imageFormats, QString, "*.png *.jpg *.bmp *.tiff *.ppm");
	PARAMETER(General, videoFormats, QString, "*.avi *.m4v *.mp4");
	PARAMETER(General, mirrorView, bool, true);
	PARAMETER(General, invertedSearch, bool, false);
	PARAMETER(General, controlsShown, bool, false);

	PARAMETER(Homography, homographyComputed, bool, true);
	PARAMETER(Homography, ransacReprojThr, double, 1.0);
	PARAMETER(Homography, minimumInliers, uint, 10);

public:
	virtual ~Settings(){}

	static QString workingDirectory();
	static QString iniDefaultPath();
	static QString iniDefaultFileName() {return "config.ini";}

	static void loadSettings(const QString & fileName = QString(), QByteArray * windowGeometry = 0, QByteArray * windowState = 0);
	static void saveSettings(const QString & fileName = QString(), const QByteArray & windowGeometry = QByteArray(), const QByteArray & windowState = QByteArray());

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
	static QString currentNearestNeighborType();

	static cv::flann::IndexParams * createFlannIndexParams();
	static cvflann::flann_distance_t getFlannDistanceType();
	static cv::flann::SearchParams getFlannSearchParams();

private:
	Settings(){}

private:
	static ParametersMap defaultParameters_;
	static ParametersMap parameters_;
	static ParametersType parametersType_;
	static Settings dummyInit_;
};


#endif /* SETTINGS_H_ */
