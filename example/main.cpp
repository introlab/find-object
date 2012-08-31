/*
 * Copyright (C) 2011, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
 */

#include <stdio.h>

// Qt stuff
#include <QtCore/QTime>
#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <QtGui/QGraphicsRectItem>
#include <QtGui/QPen>
#include <QtGui/QColor>

// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography

// From this project (see src folder)
#include "ObjWidget.h"
#include "QtOpenCV.h"

void showUsage()
{
	printf("\n");
	printf("Usage :\n");
	printf("  ./example object.png scene.png\n");
	exit(1);
}

int main(int argc, char * argv[])
{
	if(argc<3)
	{
		showUsage();
	}
	QTime time;

	// GUI stuff
	QApplication app(argc, argv);
	ObjWidget objWidget;
	ObjWidget sceneWidget;

	time.start();
	//Load as grayscale
	cv::Mat objectImg = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
	cv::Mat sceneImg = cv::imread(argv[2], cv::IMREAD_GRAYSCALE);

	if(!objectImg.empty() && !sceneImg.empty())
	{
		printf("Loading images: %d ms\n", time.restart());
		std::vector<cv::KeyPoint> objectKeypoints;
		std::vector<cv::KeyPoint> sceneKeypoints;
		cv::Mat objectDescriptors;
		cv::Mat sceneDescriptors;

		////////////////////////////
		// EXTRACT KEYPOINTS
		////////////////////////////
		// The detector can be any of (see OpenCV features2d.hpp):
		// cv::FeatureDetector * detector = new cv::DenseFeatureDetector();
		// cv::FeatureDetector * detector = new cv::FastFeatureDetector();
		// cv::FeatureDetector * detector = new cv::GFTTDetector();
		// cv::FeatureDetector * detector = new cv::MSER();
		// cv::FeatureDetector * detector = new cv::ORB();
		cv::FeatureDetector * detector = new cv::SIFT();
		// cv::FeatureDetector * detector = new cv::StarFeatureDetector();
		// cv::FeatureDetector * detector = new cv::SURF(600.0);
		detector->detect(objectImg, objectKeypoints);
		printf("Object: %d keypoints detected in %d ms\n", (int)objectKeypoints.size(), time.restart());
		detector->detect(sceneImg, sceneKeypoints);
		printf("Scene: %d keypoints detected in %d ms\n", (int)sceneKeypoints.size(), time.restart());

		////////////////////////////
		// EXTRACT DESCRIPTORS
		////////////////////////////
		// The extractor can be any of (see OpenCV features2d.hpp):
		// cv::DescriptorExtractor * extractor = new cv::BriefDescriptorExtractor();
		// cv::DescriptorExtractor * extractor = new cv::ORB();
		cv::DescriptorExtractor * extractor = new cv::SIFT();
		// cv::DescriptorExtractor * extractor = new cv::SURF(600.0);
		extractor->compute(objectImg, objectKeypoints, objectDescriptors);
		printf("Object: %d descriptors extracted in %d ms\n", objectDescriptors.rows, time.restart());
		extractor->compute(sceneImg, sceneKeypoints, sceneDescriptors);
		printf("Scene: %d descriptors extracted in %d ms\n", sceneDescriptors.rows, time.restart());

		////////////////////////////
		// NEAREST NEIGHBOR MATCHING USING FLANN LIBRARY (included in OpenCV)
		////////////////////////////
		cv::Mat results;
		cv::Mat dists;
		int k=2; // find the 2 nearest neighbors
		if(objectDescriptors.type()==CV_8U)
		{
			// Binary descriptors detected (from ORB or Brief)

			// Create Flann LSH index
			cv::flann::Index flannIndex(sceneDescriptors, cv::flann::LshIndexParams(20, 10, 2));
			printf("Time creating FLANN index = %d ms\n", time.restart());
			results = cv::Mat(objectDescriptors.rows, k, CV_32SC1); // Results index
			dists = cv::Mat(objectDescriptors.rows, k, CV_32FC1); // Distance results are CV_32FC1 ?!?!? NOTE OpenCV doc is not clear about that...

			// search (nearest neighbor)
			flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
			printf("Time nearest neighbor search = %d ms\n", time.restart());
		}
		else
		{
			// assume it is CV_32F

			// Create Flann KDTree index
			cv::flann::Index flannIndex(sceneDescriptors, cv::flann::KDTreeIndexParams());
			printf("Time creating FLANN index = %d ms\n", time.restart());
			results = cv::Mat(objectDescriptors.rows, k, CV_32SC1); // Results index
			dists = cv::Mat(objectDescriptors.rows, k, CV_32FC1); // Distance results are CV_32FC1

			// search (nearest neighbor)
			flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
			printf("Time nearest neighbor search = %d ms\n", time.restart());
		}





		////////////////////////////
		// PROCESS NEAREST NEIGHBOR RESULTS
		////////////////////////////
		// Set gui data
		objWidget.setData(objectKeypoints, objectDescriptors, objectImg, "", "");
		sceneWidget.setData(sceneKeypoints, sceneDescriptors, sceneImg, "", "");

		// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
		float nndrRatio = 0.6;
		std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
		std::vector<int> indexes_1, indexes_2; // Used for homography
		std::vector<uchar> outlier_mask;  // Used for homography
		for(int i=0; i<objectDescriptors.rows; ++i)
		{
			// Check if this descriptor matches with those of the objects
			// Apply NNDR
			if(dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
			{
				mpts_1.push_back(objectKeypoints.at(i).pt);
				indexes_1.push_back(i);

				mpts_2.push_back(sceneKeypoints.at(results.at<int>(i,0)).pt);
				indexes_2.push_back(results.at<int>(i,0));
			}
		}

		// FIND HOMOGRAPHY
		unsigned int minInliers = 8;
		if(mpts_1.size() >= minInliers)
		{
			time.start();
			cv::Mat H = findHomography(mpts_1,
					mpts_2,
					cv::RANSAC,
					1.0,
					outlier_mask);
			printf("Time finding homography = %d ms\n", time.restart());
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
			QTransform hTransform(
			H.at<double>(0,0), H.at<double>(1,0), H.at<double>(2,0),
			H.at<double>(0,1), H.at<double>(1,1), H.at<double>(2,1),
			H.at<double>(0,2), H.at<double>(1,2), H.at<double>(2,2));

			// GUI : Change color and add homography rectangle
			QColor color(Qt::green);
			int alpha = 130;
			color.setAlpha(alpha);
			for(unsigned int k=0; k<mpts_1.size();++k)
			{
				if(outlier_mask.at(k))
				{
					objWidget.setKptColor(indexes_1.at(k), color);
					sceneWidget.setKptColor(indexes_2.at(k), color);
				}
				else
				{
					objWidget.setKptColor(indexes_1.at(k), QColor(255,0,0,alpha));
					sceneWidget.setKptColor(indexes_2.at(k), QColor(255,0,0,alpha));
				}
			}
			QPen rectPen(color);
			rectPen.setWidth(4);
			QGraphicsRectItem * rectItem = new QGraphicsRectItem(objWidget.pixmap().rect());
			rectItem->setPen(rectPen);
			rectItem->setTransform(hTransform);
			sceneWidget.addRect(rectItem);
			printf("Inliers=%d Outliers=%d\n", inliers, outliers);
		}
		else
		{
			printf("Not enough matches (%d) for homography...\n", (int)mpts_1.size());
		}

		// Wait for gui
		objWidget.setGraphicsViewMode(false);
		objWidget.setWindowTitle("Object");
		if(objWidget.pixmap().width() <= 800)
		{
			objWidget.setMinimumSize(objWidget.pixmap().width(), objWidget.pixmap().height());
		}
		else
		{
			objWidget.setMinimumSize(800, 600);
			objWidget.setAutoScale(false);
		}

		sceneWidget.setGraphicsViewMode(false);
		sceneWidget.setWindowTitle("Scene");
		if(sceneWidget.pixmap().width() <= 800)
		{
			sceneWidget.setMinimumSize(sceneWidget.pixmap().width(), sceneWidget.pixmap().height());
		}
		else
		{
			sceneWidget.setMinimumSize(800, 600);
			sceneWidget.setAutoScale(false);
		}

		sceneWidget.show();
		objWidget.show();

		int r = app.exec();
		printf("Closing...\n");

		////////////////////////////
		//Cleanup
		////////////////////////////
		delete detector;
		delete extractor;

		return r;
	}
	else
	{
		printf("Images are not valid!\n");
		showUsage();
	}

	return 1;
}
