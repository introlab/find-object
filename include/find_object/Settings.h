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

#ifndef SETTINGS_H_
#define SETTINGS_H_

#include "find_object/FindObjectExp.h" // DLL export/import defines
#include "find_object/Version.h" // DLL export/import defines

#include <QtCore/QMap>
#include <QtCore/QVariant>
#include <QtCore/QByteArray>
#include <opencv2/features2d/features2d.hpp>

namespace find_object {

class Feature2D;

typedef QMap<QString, QVariant> ParametersMap; // Key, value
typedef QMap<QString, QString> ParametersType; // Key, type
typedef QMap<QString, QString> DescriptionsMap; // Key, description

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

#define PARAMETER(PREFIX, NAME, TYPE, DEFAULT_VALUE, DESCRIPTION) \
	public: \
		static QString k##PREFIX##_##NAME() {return QString(#PREFIX "/" #NAME);} \
		static TYPE default##PREFIX##_##NAME() {return DEFAULT_VALUE;} \
		static QString type##PREFIX##_##NAME() {return QString(#TYPE);} \
		static QString description##PREFIX##_##NAME() {return QString(DESCRIPTION);} \
		PARAMETER_GETTER_##TYPE(PREFIX, NAME) \
		static void set##PREFIX##_##NAME(const TYPE & value) {parameters_[#PREFIX "/" #NAME] = value;} \
	private: \
		class Dummy##PREFIX##_##NAME { \
		public: \
			Dummy##PREFIX##_##NAME() { \
				defaultParameters_.insert(#PREFIX "/" #NAME, QVariant(DEFAULT_VALUE)); \
				parameters_.insert(#PREFIX "/" #NAME, DEFAULT_VALUE); \
				parametersType_.insert(#PREFIX "/" #NAME, #TYPE); \
				descriptions_.insert(#PREFIX "/" #NAME, DESCRIPTION);} \
		}; \
		Dummy##PREFIX##_##NAME dummy##PREFIX##_##NAME;

#define PARAMETER_COND(PREFIX, NAME, TYPE, COND, DEFAULT_VALUE1, DEFAULT_VALUE2, DESCRIPTION) \
	public: \
		static QString k##PREFIX##_##NAME() {return QString(#PREFIX "/" #NAME);} \
		static TYPE default##PREFIX##_##NAME() {return COND?DEFAULT_VALUE1:DEFAULT_VALUE2;} \
		static QString type##PREFIX##_##NAME() {return QString(#TYPE);} \
		static QString description##PREFIX##_##NAME() {return QString(DESCRIPTION);} \
		PARAMETER_GETTER_##TYPE(PREFIX, NAME) \
		static void set##PREFIX##_##NAME(const TYPE & value) {parameters_[#PREFIX "/" #NAME] = value;} \
	private: \
		class Dummy##PREFIX##_##NAME { \
		public: \
			Dummy##PREFIX##_##NAME() { \
				defaultParameters_.insert(#PREFIX "/" #NAME, QVariant(COND?DEFAULT_VALUE1:DEFAULT_VALUE2)); \
				parameters_.insert(#PREFIX "/" #NAME, COND?DEFAULT_VALUE1:DEFAULT_VALUE2); \
				parametersType_.insert(#PREFIX "/" #NAME, #TYPE); \
				descriptions_.insert(#PREFIX "/" #NAME, DESCRIPTION);} \
		}; \
		Dummy##PREFIX##_##NAME dummy##PREFIX##_##NAME;
// MACRO END

class FINDOBJECT_EXP Settings
{
	PARAMETER(Camera, 1deviceId, int, 0, "Device ID (default 0).");
	PARAMETER(Camera, 2imageWidth, int, 0, "Image width (0 means default width from camera).");
	PARAMETER(Camera, 3imageHeight, int, 0, "Image height (0 means default height from camera).");
	PARAMETER(Camera, 4imageRate, double, 10.0, "Image rate in Hz (0 Hz means as fast as possible)."); // Hz
	PARAMETER(Camera, 5mediaPath, QString, "", "Video file or directory of images. If set, the camera is not used. See General->videoFormats and General->imageFormats for available formats.");
	PARAMETER(Camera, 6useTcpCamera, bool, false, "Use TCP/IP input camera.");
	PARAMETER(Camera, 8port, int, 0, "The images server's port when useTcpCamera is checked. Only one client at the same time is allowed.");
	PARAMETER(Camera, 9queueSize, int, 1, "Maximum images buffered from TCP. If 0, all images are buffered.");

	//List format : [Index:item0;item1;item3;...]

	PARAMETER_COND(Feature2D, 1Detector, QString, FINDOBJECT_NONFREE, "7:Dense;Fast;GFTT;MSER;ORB;SIFT;Star;SURF;BRISK;AGAST;KAZE;AKAZE" , "4:Dense;Fast;GFTT;MSER;ORB;SIFT;Star;SURF;BRISK;AGAST;KAZE;AKAZE", "Keypoint detector.");
	PARAMETER_COND(Feature2D, 2Descriptor, QString, FINDOBJECT_NONFREE, "3:Brief;ORB;SIFT;SURF;BRISK;FREAK;KAZE;AKAZE;LUCID;LATCH;DAISY", "1:Brief;ORB;SIFT;SURF;BRISK;FREAK;KAZE;AKAZE;LUCID;LATCH;DAISY", "Keypoint descriptor.");
	PARAMETER(Feature2D, 3MaxFeatures, int, 0, "Maximum features per image. If the number of features extracted is over this threshold, only X features with the highest response are kept. 0 means all features are kept.");
	PARAMETER(Feature2D, 4Affine, bool, false, "(ASIFT) Extract features on multiple affine transformations of the image.");
	PARAMETER(Feature2D, 5AffineCount, int, 6, "(ASIFT) Higher the value, more affine transformations will be done.");
	PARAMETER(Feature2D, 6SubPix, bool, false, "Refines the corner locations. With SIFT/SURF, features are already subpixel, so no need to activate this.");
	PARAMETER(Feature2D, 7SubPixWinSize, int, 3, "Half of the side length of the search window. For example, if winSize=Size(5,5) , then a 5*2+1 x 5*2+1 = 11 x 11 search window is used.");
	PARAMETER(Feature2D, 8SubPixIterations, int, 30, "The process of corner position refinement stops after X iterations.");
	PARAMETER(Feature2D, 9SubPixEps, float, 0.02f, "The process of corner position refinement stops when the corner position moves by less than epsilon on some iteration.");

	PARAMETER(Feature2D, Brief_bytes, int, 32, "Bytes is a length of descriptor in bytes. It can be equal 16, 32 or 64 bytes.");

#if CV_MAJOR_VERSION < 3
	PARAMETER(Feature2D, Dense_initFeatureScale, float, 1.f, "");
	PARAMETER(Feature2D, Dense_featureScaleLevels, int, 1, "");
	PARAMETER(Feature2D, Dense_featureScaleMul, float, 0.1f, "");
	PARAMETER(Feature2D, Dense_initXyStep, int, 6, "");
	PARAMETER(Feature2D, Dense_initImgBound, int, 0, "");
	PARAMETER(Feature2D, Dense_varyXyStepWithScale, bool, true, "");
	PARAMETER(Feature2D, Dense_varyImgBoundWithScale, bool, false, "");
#endif

	PARAMETER(Feature2D, Fast_threshold, int, 10, "Threshold on difference between intensity of the central pixel and pixels of a circle around this pixel.");
	PARAMETER(Feature2D, Fast_nonmaxSuppression, bool, true, "If true, non-maximum suppression is applied to detected corners (keypoints).");
	PARAMETER(Feature2D, Fast_gpu, bool, false, "GPU-FAST: Use GPU version of FAST. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.");
	PARAMETER(Feature2D, Fast_keypointsRatio, double, 0.05, "Used with FAST GPU (OpenCV 2).");
	PARAMETER(Feature2D, Fast_maxNpoints, int, 5000, "Used with FAST GPU (OpenCV 3).");

	PARAMETER(Feature2D, AGAST_threshold, int, 10, "Threshold on difference between intensity of the central pixel and pixels of a circle around this pixel.");
	PARAMETER(Feature2D, AGAST_nonmaxSuppression, bool, true, "If true, non-maximum suppression is applied to detected corners (keypoints).");

	PARAMETER(Feature2D, KAZE_extended, bool, false, "Set to enable extraction of extended (128-byte) descriptor.");
	PARAMETER(Feature2D, KAZE_upright, bool, false, "Set to enable use of upright descriptors (non rotation-invariant).");
	PARAMETER(Feature2D, KAZE_threshold, float, 0.001f, "Detector response threshold to accept point");
	PARAMETER(Feature2D, KAZE_nOctaves, int, 4, "Maximum octave evolution of the image.");
	PARAMETER(Feature2D, KAZE_nOctaveLayers, int, 4, "Default number of sublevels per scale level.");

	PARAMETER(Feature2D, AKAZE_descriptorSize, int, 0, "Size of the descriptor in bits. 0 -> Full size.");
	PARAMETER(Feature2D, AKAZE_descriptorChannels, int, 3, "Number of channels in the descriptor (1, 2, 3).");
	PARAMETER(Feature2D, AKAZE_threshold, float, 0.001f, "Detector response threshold to accept point.");
	PARAMETER(Feature2D, AKAZE_nOctaves, int, 4, "Maximum octave evolution of the image.");
	PARAMETER(Feature2D, AKAZE_nOctaveLayers, int, 4, "Default number of sublevels per scale level.");

	PARAMETER(Feature2D, GFTT_maxCorners, int, 1000, "Maximum number of corners to return. If there are more corners than are found, the strongest of them is returned.");
	PARAMETER(Feature2D, GFTT_qualityLevel, double, 0.01, "Parameter characterizing the minimal accepted quality of image corners. The parameter value is multiplied by the best corner quality measure, which is the minimal eigenvalue (see cornerMinEigenVal ) or the Harris function response (see cornerHarris ). The corners with the quality measure less than the product are rejected. For example, if the best corner has the quality measure = 1500, and the qualityLevel=0.01 , then all the corners with the quality measure less than 15 are rejected.");
	PARAMETER(Feature2D, GFTT_minDistance, double, 1, "Minimum possible Euclidean distance between the returned corners.");
	PARAMETER(Feature2D, GFTT_blockSize, int, 3, "Size of an average block for computing a derivative covariation matrix over each pixel neighborhood. See cornerEigenValsAndVecs.");
	PARAMETER(Feature2D, GFTT_useHarrisDetector, bool, false, "Parameter indicating whether to use a Harris detector (see cornerHarris) or cornerMinEigenVal.");
	PARAMETER(Feature2D, GFTT_k, double, 0.04, "Free parameter of the Harris detector.");

	PARAMETER(Feature2D, ORB_nFeatures, int, 500, "The maximum number of features to retain.");
	PARAMETER(Feature2D, ORB_scaleFactor, float,  1.2f, "Pyramid decimation ratio, greater than 1. scaleFactor==2 means the classical pyramid, where each next level has 4x less pixels than the previous, but such a big scale factor will degrade feature matching scores dramatically. On the other hand, too close to 1 scale factor will mean that to cover certain scale range you will need more pyramid levels and so the speed will suffer.");
	PARAMETER(Feature2D, ORB_nLevels, int, 8, "The number of pyramid levels. The smallest level will have linear size equal to input_image_linear_size/pow(scaleFactor, nlevels).");
	PARAMETER(Feature2D, ORB_edgeThreshold, int, 31, "This is size of the border where the features are not detected. It should roughly match the patchSize parameter.");
	PARAMETER(Feature2D, ORB_firstLevel, int, 0, "It should be 0 in the current implementation.");
	PARAMETER(Feature2D, ORB_WTA_K, int, 2, "The number of points that produce each element of the oriented BRIEF descriptor. The default value 2 means the BRIEF where we take a random point pair and compare their brightnesses, so we get 0/1 response. Other possible values are 3 and 4. For example, 3 means that we take 3 random points (of course, those point coordinates are random, but they are generated from the pre-defined seed, so each element of BRIEF descriptor is computed deterministically from the pixel rectangle), find point of maximum brightness and output index of the winner (0, 1 or 2). Such output will occupy 2 bits, and therefore it will need a special variant of Hamming distance, denoted as NORM_HAMMING2 (2 bits per bin). When WTA_K=4, we take 4 random points to compute each bin (that will also occupy 2 bits with possible values 0, 1, 2 or 3).");
	PARAMETER(Feature2D, ORB_scoreType, int, 0, "The default HARRIS_SCORE=0 means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is used to retain best nfeatures features); FAST_SCORE=1 is alternative value of the parameter that produces slightly less stable keypoints, but it is a little faster to compute.");
	PARAMETER(Feature2D, ORB_patchSize, int, 31, "size of the patch used by the oriented BRIEF descriptor. Of course, on smaller pyramid layers the perceived image area covered by a feature will be larger.");
	PARAMETER(Feature2D, ORB_gpu, bool, false, "GPU-ORB: Use GPU version of ORB. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.");
	PARAMETER(Feature2D, ORB_blurForDescriptor, bool, false, "GPU-ORB: blurForDescriptor parameter (OpenCV 3).");

	PARAMETER(Feature2D, MSER_delta, int, 5, "");
	PARAMETER(Feature2D, MSER_minArea, int, 60, "");
	PARAMETER(Feature2D, MSER_maxArea, int, 14400, "");
	PARAMETER(Feature2D, MSER_maxVariation, double, 0.25, "");
	PARAMETER(Feature2D, MSER_minDiversity, double, 0.2, "");
	PARAMETER(Feature2D, MSER_maxEvolution, int, 200, "");
	PARAMETER(Feature2D, MSER_areaThreshold, double, 1.01, "");
	PARAMETER(Feature2D, MSER_minMargin, double, 0.003, "");
	PARAMETER(Feature2D, MSER_edgeBlurSize, int, 5, "");

	PARAMETER(Feature2D, SIFT_nfeatures, int, 0, "The number of best features to retain. The features are ranked by their scores (measured in SIFT algorithm as the local contrast).");
	PARAMETER(Feature2D, SIFT_nOctaveLayers, int, 3, "The number of layers in each octave. 3 is the value used in D. Lowe paper. The number of octaves is computed automatically from the image resolution.");
	PARAMETER(Feature2D, SIFT_contrastThreshold, double, 0.04, "The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions. The larger the threshold, the less features are produced by the detector.");
	PARAMETER(Feature2D, SIFT_edgeThreshold, double, 10, "The threshold used to filter out edge-like features. Note that the its meaning is different from the contrastThreshold, i.e. the larger the edgeThreshold, the less features are filtered out (more features are retained).");
	PARAMETER(Feature2D, SIFT_sigma, double, 1.6, "The sigma of the Gaussian applied to the input image at the octave #0. If your image is captured with a weak camera with soft lenses, you might want to reduce the number.");
	PARAMETER(Feature2D, SIFT_rootSIFT, bool, false, "RootSIFT descriptors.");

	PARAMETER(Feature2D, SURF_hessianThreshold, double, 600.0, "Threshold for hessian keypoint detector used in SURF.");
	PARAMETER(Feature2D, SURF_nOctaves, int, 4, "Number of pyramid octaves the keypoint detector will use.");
	PARAMETER(Feature2D, SURF_nOctaveLayers, int, 2, "Number of octave layers within each octave.");
	PARAMETER(Feature2D, SURF_extended, bool, true, "Extended descriptor flag (true - use extended 128-element descriptors; false - use 64-element descriptors).");
	PARAMETER(Feature2D, SURF_upright, bool, false, "Up-right or rotated features flag (true - do not compute orientation of features; false - compute orientation).");
	PARAMETER(Feature2D, SURF_gpu, bool, false, "GPU-SURF: Use GPU version of SURF. This option is enabled only if OpenCV is built with CUDA and GPUs are detected.");
	PARAMETER(Feature2D, SURF_keypointsRatio, float, 0.01f, "Used with SURF GPU.");

	PARAMETER(Feature2D, Star_maxSize, int, 45, "");
	PARAMETER(Feature2D, Star_responseThreshold, int, 30, "");
	PARAMETER(Feature2D, Star_lineThresholdProjected, int, 10, "");
	PARAMETER(Feature2D, Star_lineThresholdBinarized, int, 8, "");
	PARAMETER(Feature2D, Star_suppressNonmaxSize, int, 5, "");

	PARAMETER(Feature2D, BRISK_thresh, int, 30, "FAST/AGAST detection threshold score.");
	PARAMETER(Feature2D, BRISK_octaves, int, 3, "Detection octaves. Use 0 to do single scale.");
	PARAMETER(Feature2D, BRISK_patternScale, float, 1.0f, "Apply this scale to the pattern used for sampling the neighbourhood of a keypoint.");

	PARAMETER(Feature2D, FREAK_orientationNormalized, bool, true, "Enable orientation normalization.");
	PARAMETER(Feature2D, FREAK_scaleNormalized, bool, true, "Enable scale normalization.");
	PARAMETER(Feature2D, FREAK_patternScale, float, 22.0f, "Scaling of the description pattern.");
	PARAMETER(Feature2D, FREAK_nOctaves, int, 4, "Number of octaves covered by the detected keypoints.");

	PARAMETER(Feature2D, LUCID_kernel, int, 1, "Kernel for descriptor construction, where 1=3x3, 2=5x5, 3=7x7 and so forth.");
	PARAMETER(Feature2D, LUCID_blur_kernel, int, 2, "Kernel for blurring image prior to descriptor construction, where 1=3x3, 2=5x5, 3=7x7 and so forth.");

	PARAMETER(Feature2D, LATCH_bytes, int, 32, "Size of the descriptor - can be 64, 32, 16, 8, 4, 2 or 1.");
	PARAMETER(Feature2D, LATCH_rotationInvariance, bool, true, "Whether or not the descriptor should compansate for orientation changes.");
	PARAMETER(Feature2D, LATCH_half_ssd_size, int, 3, "The size of half of the mini-patches size. For example, if we would like to compare triplets of patches of size 7x7x then the half_ssd_size should be (7-1)/2 = 3.");

	PARAMETER(Feature2D, DAISY_radius, float, 15, "Radius of the descriptor at the initial scale.");
	PARAMETER(Feature2D, DAISY_q_radius, int, 3, "Amount of radial range division quantity.");
	PARAMETER(Feature2D, DAISY_q_theta, int, 8, "Amount of angular range division quantity.");
	PARAMETER(Feature2D, DAISY_q_hist, int, 8, "Amount of gradient orientations range division quantity.");
	PARAMETER(Feature2D, DAISY_interpolation, bool, true, "Switch to disable interpolation for speed improvement at minor quality loss.");
	PARAMETER(Feature2D, DAISY_use_orientation, bool, false, "Sample patterns using keypoints orientation, disabled by default.");

	PARAMETER_COND(NearestNeighbor, 1Strategy, QString, FINDOBJECT_NONFREE, "1:Linear;KDTree;KMeans;Composite;Autotuned;Lsh;BruteForce", "6:Linear;KDTree;KMeans;Composite;Autotuned;Lsh;BruteForce", "Nearest neighbor strategy.");
	PARAMETER_COND(NearestNeighbor, 2Distance_type, QString, FINDOBJECT_NONFREE, "0:EUCLIDEAN_L2;MANHATTAN_L1;MINKOWSKI;MAX;HIST_INTERSECT;HELLINGER;CHI_SQUARE_CS;KULLBACK_LEIBLER_KL;HAMMING", "1:EUCLIDEAN_L2;MANHATTAN_L1;MINKOWSKI;MAX;HIST_INTERSECT;HELLINGER;CHI_SQUARE_CS;KULLBACK_LEIBLER_KL;HAMMING", "Distance type.");
	PARAMETER(NearestNeighbor, 3nndrRatioUsed, bool, true, "Nearest neighbor distance ratio approach to accept the best match.");
	PARAMETER(NearestNeighbor, 4nndrRatio, float, 0.8f, "Nearest neighbor distance ratio.");
	PARAMETER(NearestNeighbor, 5minDistanceUsed, bool, false, "Minimum distance with the nearest descriptor to accept a match.");
	PARAMETER(NearestNeighbor, 6minDistance, float, 1.6f, "Minimum distance. You can look at top of this panel where minimum and maximum distances are shown to properly set this parameter depending of the descriptor used.");
	PARAMETER(NearestNeighbor, 7ConvertBinToFloat, bool, false, "Convert binary descriptor to float before quantization, so you can use FLANN strategies with them.");


	PARAMETER(NearestNeighbor, BruteForce_gpu, bool, false, "Brute force GPU");

	PARAMETER(NearestNeighbor, search_checks, int, 32, "The number of times the tree(s) in the index should be recursively traversed. A higher value for this parameter would give better search precision, but also take more time. If automatic configuration was used when the index was created, the number of checks required to achieve the specified precision was also computed, in which case this parameter is ignored.");
	PARAMETER(NearestNeighbor, search_eps, float, 0, "");
	PARAMETER(NearestNeighbor, search_sorted, bool, true, "");

	PARAMETER(NearestNeighbor, KDTree_trees, int, 4, "The number of parallel kd-trees to use. Good values are in the range [1..16].");

	PARAMETER(NearestNeighbor, Composite_trees, int, 4, "The number of parallel kd-trees to use. Good values are in the range [1..16].");
	PARAMETER(NearestNeighbor, Composite_branching, int, 32, "The branching factor to use for the hierarchical k-means tree.");
	PARAMETER(NearestNeighbor, Composite_iterations, int, 11, "The maximum number of iterations to use in the k-means clustering stage when building the k-means tree. A value of -1 used here means that the k-means clustering should be iterated until convergence.");
	PARAMETER(NearestNeighbor, Composite_centers_init, QString, "0:RANDOM;GONZALES;KMEANSPP", "The algorithm to use for selecting the initial centers when performing a k-means clustering step. The possible values are CENTERS_RANDOM (picks the initial cluster centers randomly), CENTERS_GONZALES (picks the initial centers using Gonzales’ algorithm) and CENTERS_KMEANSPP (picks the initial centers using the algorithm suggested in arthur_kmeanspp_2007 ).");
	PARAMETER(NearestNeighbor, Composite_cb_index, double, 0.2, "This parameter (cluster boundary index) influences the way exploration is performed in the hierarchical kmeans tree. When cb_index is zero the next kmeans domain to be explored is chosen to be the one with the closest center. A value greater then zero also takes into account the size of the domain.");

	PARAMETER(NearestNeighbor, Autotuned_target_precision, double, 0.8, "Is a number between 0 and 1 specifying the percentage of the approximate nearest-neighbor searches that return the exact nearest-neighbor. Using a higher value for this parameter gives more accurate results, but the search takes longer. The optimum value usually depends on the application.");
	PARAMETER(NearestNeighbor, Autotuned_build_weight, double, 0.01, "Specifies the importance of the index build time raported to the nearest-neighbor search time. In some applications it’s acceptable for the index build step to take a long time if the subsequent searches in the index can be performed very fast. In other applications it’s required that the index be build as fast as possible even if that leads to slightly longer search times.");
	PARAMETER(NearestNeighbor, Autotuned_memory_weight, double, 0, "Is used to specify the tradeoff between time (index build time and search time) and memory used by the index. A value less than 1 gives more importance to the time spent and a value greater than 1 gives more importance to the memory usage.");
	PARAMETER(NearestNeighbor, Autotuned_sample_fraction, double, 0.1, "Is a number between 0 and 1 indicating what fraction of the dataset to use in the automatic parameter configuration algorithm. Running the algorithm on the full dataset gives the most accurate results, but for very large datasets can take longer than desired. In such case using just a fraction of the data helps speeding up this algorithm while still giving good approximations of the optimum parameters.");

	PARAMETER(NearestNeighbor, KMeans_branching, int, 32, "The branching factor to use for the hierarchical k-means tree.");
	PARAMETER(NearestNeighbor, KMeans_iterations, int, 11, "The maximum number of iterations to use in the k-means clustering stage when building the k-means tree. A value of -1 used here means that the k-means clustering should be iterated until convergence.");
	PARAMETER(NearestNeighbor, KMeans_centers_init, QString, "0:RANDOM;GONZALES;KMEANSPP", "The algorithm to use for selecting the initial centers when performing a k-means clustering step. The possible values are CENTERS_RANDOM (picks the initial cluster centers randomly), CENTERS_GONZALES (picks the initial centers using Gonzales’ algorithm) and CENTERS_KMEANSPP (picks the initial centers using the algorithm suggested in arthur_kmeanspp_2007 ).");
	PARAMETER(NearestNeighbor, KMeans_cb_index, double, 0.2, "This parameter (cluster boundary index) influences the way exploration is performed in the hierarchical kmeans tree. When cb_index is zero the next kmeans domain to be explored is chosen to be the one with the closest center. A value greater then zero also takes into account the size of the domain.");

	PARAMETER(NearestNeighbor, Lsh_table_number, int, 12, "The number of hash tables to use (between 10 and 30 usually).");
	PARAMETER(NearestNeighbor, Lsh_key_size, int, 20, "The size of the hash key in bits (between 10 and 20 usually).");
	PARAMETER(NearestNeighbor, Lsh_multi_probe_level, int, 2, "The number of bits to shift to check for neighboring buckets (0 is regular LSH, 2 is recommended).");

	PARAMETER(General, autoStartCamera, bool, false, "Automatically start the camera when the application is opened.");
	PARAMETER(General, autoUpdateObjects, bool, true, "Automatically update objects on every parameter changes, otherwise you would need to press \"Update objects\" on the objects panel.");
	PARAMETER(General, nextObjID, uint, 1, "Next object ID to use.");
	PARAMETER(General, imageFormats, QString, "*.png *.jpg *.bmp *.tiff *.ppm *.pgm", "Image formats supported.");
	PARAMETER(General, videoFormats, QString, "*.avi *.m4v *.mp4", "Video formats supported.");
	PARAMETER(General, mirrorView, bool, false, "Flip the camera image horizontally (like all webcam applications).");
	PARAMETER(General, invertedSearch, bool, true, "Instead of matching descriptors from the objects to those in a vocabulary created with descriptors extracted from the scene, we create a vocabulary from all the objects' descriptors and we match scene's descriptors to this vocabulary. It is the inverted search mode.");
	PARAMETER(General, controlsShown, bool, false, "Show play/image seek controls (useful with video file and directory of images modes).");
	PARAMETER(General, threads, int, 1, "Number of threads used for objects matching and homography computation. 0 means as many threads as objects. On InvertedSearch mode, multi-threading has only effect on homography computation.");
	PARAMETER(General, multiDetection, bool, false, "Multiple detection of the same object.");
	PARAMETER(General, multiDetectionRadius, int, 30, "Ignore detection of the same object in X pixels radius of the previous detections.");
	PARAMETER(General, port, int, 0, "Port on objects detected are published. If port=0, a port is chosen automatically.")
	PARAMETER(General, autoScroll, bool, true, "Auto scroll to detected object in Objects panel.");
	PARAMETER(General, vocabularyFixed, bool, false, "If the vocabulary is fixed, no new words will be added to it when adding new objects.");
	PARAMETER(General, vocabularyIncremental, bool, false, "The vocabulary is created incrementally. When new objects are added, their descriptors are compared to those already in vocabulary to find if the visual word already exist or not. \"NearestNeighbor/nndrRatio\" and \"NearestNeighbor/minDistance\" are used to compare descriptors.");
	PARAMETER(General, vocabularyUpdateMinWords, int, 2000, "When the vocabulary is incremental (see \"General/vocabularyIncremental\"), after X words added to vocabulary, the internal index is updated with new words. This parameter lets avoiding to reconstruct the whole nearest neighbor index after each time descriptors of an object are added to vocabulary. 0 means no incremental update.");
	PARAMETER(General, sendNoObjDetectedEvents, bool, true, "When there are no objects detected, send an empty object detection event.");
	PARAMETER(General, autoPauseOnDetection, bool, false, "Auto pause the camera when an object is detected.");
	PARAMETER(General, autoScreenshotPath, QString, "", "Path to a directory to save screenshot of the current camera view when there is a detection.");

	PARAMETER(Homography, homographyComputed, bool, true, "Compute homography? On ROS, this is required to publish objects detected.");
	PARAMETER(Homography, method, QString, "1:LMEDS;RANSAC;RHO", "Type of the robust estimation algorithm: least-median algorithm or RANSAC algorithm.");
	PARAMETER(Homography, ransacReprojThr, double, 5.0, "Maximum allowed reprojection error to treat a point pair as an inlier (used in the RANSAC method only). It usually makes sense to set this parameter somewhere in the range of 1 to 10.");
	PARAMETER(Homography, minimumInliers, int, 10, "Minimum inliers to accept the homography. Value must be >= 4.");
	PARAMETER(Homography, ignoreWhenAllInliers, bool, false, "Ignore homography when all features are inliers (sometimes when the homography doesn't converge, it returns the best homography with all features as inliers).");
	PARAMETER(Homography, rectBorderWidth, int, 4, "Homography rectangle border width.");
	PARAMETER(Homography, allCornersVisible, bool, false, "All corners of the detected object must be visible in the scene.");
	PARAMETER(Homography, minAngle, int, 0, "(Degrees) Homography minimum angle. Set 0 to disable. When the angle is very small, this is a good indication that the homography is wrong. A good value is over 60 degrees.");
	PARAMETER(Homography, opticalFlow, bool, false, "Activate optical flow to refine matched features before computing the homography.");
	PARAMETER(Homography, opticalFlowWinSize, int, 16, "Size of the search window at each pyramid level.");
	PARAMETER(Homography, opticalFlowMaxLevel, int, 3, "0-based maximal pyramid level number; if set to 0, pyramids are not used (single level), if set to 1, two levels are used, and so on; if pyramids are passed to input then algorithm will use as many levels as pyramids have but no more than maxLevel.");
	PARAMETER(Homography, opticalFlowIterations, int, 30, "Specifying the termination criteria of the iterative search algorithm (after the specified maximum number of iterations).");
	PARAMETER(Homography, opticalFlowEps, float, 0.01f, "Specifying the termination criteria of the iterative search algorithm (when the search window moves by less than epsilon).");

public:
	virtual ~Settings(){}

	static QString workingDirectory();
	static QString iniDefaultPath();
	static QString iniDefaultFileName() {return "config.ini";}
	static QString iniPath();

	static void init(const QString & fileName);

	static void loadSettings(const QString & fileName = QString());
	static void loadWindowSettings(QByteArray & windowGeometry, QByteArray & windowState, const QString & fileName = QString());
	static void saveSettings(const QString & fileName = QString());
	static void saveWindowSettings(const QByteArray & windowGeometry, const QByteArray & windowState, const QString & fileName = QString());

	static const ParametersMap & getDefaultParameters() {return defaultParameters_;}
	static const ParametersMap & getParameters() {return parameters_;}
	static const ParametersType & getParametersType() {return parametersType_;}
	static const DescriptionsMap & getDescriptions() {return descriptions_;}
	static void setParameter(const QString & key, const QVariant & value) {if(parameters_.contains(key))parameters_[key] = value;}
	static void resetParameter(const QString & key) {if(defaultParameters_.contains(key)) parameters_.insert(key, defaultParameters_.value(key));}
	static QVariant getParameter(const QString & key) {return parameters_.value(key, QVariant());}

	static Feature2D * createKeypointDetector();
	static Feature2D * createDescriptorExtractor();

	static QString currentDescriptorType();
	static QString currentDetectorType();
	static QString currentNearestNeighborType();

	static bool isBruteForceNearestNeighbor();
	static cv::flann::IndexParams * createFlannIndexParams();
	static cvflann::flann_distance_t getFlannDistanceType();
	static cv::flann::SearchParams getFlannSearchParams();

	static int getHomographyMethod();

private:
	Settings(){}

private:
	static ParametersMap defaultParameters_;
	static ParametersMap parameters_;
	static ParametersType parametersType_;
	static DescriptionsMap descriptions_;
	static Settings dummyInit_;
	static QString iniPath_;
};

class Feature2D
{
public:
#if CV_MAJOR_VERSION < 3
	Feature2D(cv::Ptr<cv::FeatureDetector> featureDetector);
	Feature2D(cv::Ptr<cv::DescriptorExtractor> descriptorExtractor);
#endif
	Feature2D(cv::Ptr<cv::Feature2D> feature2D);
	Feature2D() {}
	virtual ~Feature2D() {}

	virtual void detect(const cv::Mat & image,
			std::vector<cv::KeyPoint> & keypoints,
			const cv::Mat & mask = cv::Mat());

	virtual void compute(const cv::Mat & image,
			std::vector<cv::KeyPoint> & keypoints,
			cv::Mat & descriptors);

	virtual void detectAndCompute(const cv::Mat & image,
			std::vector<cv::KeyPoint> & keypoints,
			cv::Mat & descriptors,
			const cv::Mat & mask = cv::Mat());

private:
#if CV_MAJOR_VERSION < 3
	cv::Ptr<cv::FeatureDetector> featureDetector_;
	cv::Ptr<cv::DescriptorExtractor> descriptorExtractor_;
#endif
	cv::Ptr<cv::Feature2D> feature2D_;
};

} // namespace find_object

#endif /* SETTINGS_H_ */
