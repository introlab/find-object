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

#include <stdio.h>
#include <stdlib.h>

// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography
#include <opencv2/opencv_modules.hpp>

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

void showUsage()
{
	printf(
	"\n"
	"Return similarity between two images (the number of similar features between the images).\n"
	"Usage :\n"
	"  ./find_object-similarity [option] object.png scene.png\n"
	"Options: \n"
	"   -inliers            return inliers percentage : inliers / (inliers + outliers)\n"
	"   -quiet              don't show messages\n");

	exit(-1);
}

enum {mTotal, mInliers};

int main(int argc, char * argv[])
{
	bool quiet = false;
	int method = mTotal; //total matches
	if(argc<3)
	{
		printf("Two images required!\n");
		showUsage();
	}
	else if(argc>3)
	{
		for(int i=1; i<argc-2; ++i)
		{
			if(std::string(argv[i]).compare("-inliers") == 0)
			{
				method = mInliers;
			}
			else if(std::string(argv[i]).compare("-quiet") == 0)
			{
				quiet = true;
			}
			else
			{
				printf("Option %s not recognized!", argv[1]);
				showUsage();
			}
		}
	}


	//Load as grayscale
	cv::Mat objectImg = cv::imread(argv[argc-2], cv::IMREAD_GRAYSCALE);
	cv::Mat sceneImg = cv::imread(argv[argc-1], cv::IMREAD_GRAYSCALE);

	int value = 0;
	if(!objectImg.empty() && !sceneImg.empty())
	{
		std::vector<cv::KeyPoint> objectKeypoints;
		std::vector<cv::KeyPoint> sceneKeypoints;
		cv::Mat objectDescriptors;
		cv::Mat sceneDescriptors;

#if CV_MAJOR_VERSION < 3
		////////////////////////////
		// EXTRACT KEYPOINTS
		////////////////////////////
		cv::SIFT sift;
		sift.detect(objectImg, objectKeypoints);
		sift.detect(sceneImg, sceneKeypoints);

		////////////////////////////
		// EXTRACT DESCRIPTORS
		////////////////////////////
		sift.compute(objectImg, objectKeypoints, objectDescriptors);
		sift.compute(sceneImg, sceneKeypoints, sceneDescriptors);
#else
		////////////////////////////
		// EXTRACT KEYPOINTS
		////////////////////////////
#if (CV_MAJOR_VERSION == 4 && CV_MINOR_VERSION <= 3) || (CV_MAJOR_VERSION == 3 && (CV_MINOR_VERSION < 4 || (CV_MINOR_VERSION==4 && CV_SUBMINOR_VERSION<11)))
		cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
#else
        cv::Ptr<cv::SIFT> sift = cv::SIFT::create();
#endif
		sift->detect(objectImg, objectKeypoints);
		sift->detect(sceneImg, sceneKeypoints);

		////////////////////////////
		// EXTRACT DESCRIPTORS
		////////////////////////////
		sift->compute(objectImg, objectKeypoints, objectDescriptors);
		sift->compute(sceneImg, sceneKeypoints, sceneDescriptors);
#endif
		////////////////////////////
		// NEAREST NEIGHBOR MATCHING USING FLANN LIBRARY (included in OpenCV)
		////////////////////////////
		cv::Mat results;
		cv::Mat dists;
		std::vector<std::vector<cv::DMatch> > matches;
		int k=2; // find the 2 nearest neighbors

		// Create Flann KDTree index
		cv::flann::Index flannIndex(sceneDescriptors, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
		results = cv::Mat(objectDescriptors.rows, k, CV_32SC1); // Results index
		dists = cv::Mat(objectDescriptors.rows, k, CV_32FC1); // Distance results are CV_32FC1

		// search (nearest neighbor)
		flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );

		////////////////////////////
		// PROCESS NEAREST NEIGHBOR RESULTS
		////////////////////////////

		// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
		float nndrRatio = 0.6f;
		std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
		std::vector<int> indexes_1, indexes_2; // Used for homography
		std::vector<uchar> outlier_mask;  // Used for homography
		// Check if this descriptor matches with those of the objects

		for(int i=0; i<objectDescriptors.rows; ++i)
		{
			// Apply NNDR
			if(dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
			{
				mpts_1.push_back(objectKeypoints.at(i).pt);
				indexes_1.push_back(i);

				mpts_2.push_back(sceneKeypoints.at(results.at<int>(i,0)).pt);
				indexes_2.push_back(results.at<int>(i,0));
			}
		}

		if(method == mInliers)
		{
			// FIND HOMOGRAPHY
			unsigned int minInliers = 8;
			if(mpts_1.size() >= minInliers)
			{
				cv::Mat H = findHomography(mpts_1,
						mpts_2,
						cv::RANSAC,
						1.0,
						outlier_mask);
				int inliers=0, outliers=0;
				for(unsigned int k=0; k<mpts_1.size();++k)
				{
					if(outlier_mask.at(k))
					{
						++inliers;
					}
					else
					{
						++outliers;
					}
				}
				if(!quiet)
					printf("Total=%d Inliers=%d Outliers=%d\n", (int)mpts_1.size(), inliers, outliers);
				value = (inliers*100) / (inliers+outliers);
			}
		}
		else
		{
			value = (int)mpts_1.size();
		}
	}
	else
	{
		printf("Images are not valid!\n");
		showUsage();
	}
	if(!quiet)
		printf("Similarity = %d\n", value);
	return value;
}
