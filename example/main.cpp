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
#include <opencv2/calib3d/calib3d.hpp> // for homography

// From this project (see src folder)
#include "ObjWidget.h"

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
	time.start();

	// GUI stuff
	QApplication app(argc, argv);
	ObjWidget objWidget;
	ObjWidget sceneWidget;

	//Load as grayscale
	IplImage * objectImg = cvLoadImage(argv[1], CV_LOAD_IMAGE_GRAYSCALE);
	IplImage * sceneImg = cvLoadImage(argv[2], CV_LOAD_IMAGE_GRAYSCALE);

	if(objectImg && sceneImg)
	{
		std::vector<cv::KeyPoint> objectKeypoints;
		std::vector<cv::KeyPoint> sceneKeypoints;
		cv::Mat objectDescriptors;
		cv::Mat sceneDescriptors;

		////////////////////////////
		// EXTRACT KEYPOINTS
		////////////////////////////
		// The detector can be any of (see OpenCV features2d.hpp):
		// cv::FeatureDetector * detector = new cv::OrbFeatureDetector();
		// cv::FeatureDetector * detector = new cv::FastFeatureDetector();
		// cv::FeatureDetector * detector = new cv::MserFeatureDetector();
		// cv::FeatureDetector * detector = new cv::SiftFeatureDetector();
		// cv::FeatureDetector * detector = new cv::SurfFeatureDetector();
		// cv::FeatureDetector * detector = new cv::StarFeatureDetector();
		cv::FeatureDetector * detector = new cv::SurfFeatureDetector();
		detector->detect(objectImg, objectKeypoints);
		printf("Object: %d keypoints detected in %d ms\n", (int)objectKeypoints.size(), time.restart());
		detector->detect(sceneImg, sceneKeypoints);
		printf("Scene: %d keypoints detected in %d ms\n", (int)sceneKeypoints.size(), time.restart());

		////////////////////////////
		// EXTRACT DESCRIPTORS
		////////////////////////////
		// The extractor can be any of (see OpenCV features2d.hpp):
		// cv::DescriptorExtractor * extractor = new cv::BriefDescriptorExtractor();
		// cv::DescriptorExtractor * extractor = new cv::OrbDescriptorExtractor();
		// cv::DescriptorExtractor * extractor = new cv::SiftDescriptorExtractor();
		// cv::DescriptorExtractor * extractor = new cv::SurfDescriptorExtractor();
		cv::DescriptorExtractor * extractor = new cv::SurfDescriptorExtractor();
		extractor->compute(objectImg, objectKeypoints, objectDescriptors);
		printf("Object: %d descriptors extracted in %d ms\n", objectDescriptors.rows, time.restart());
		extractor->compute(sceneImg, sceneKeypoints, sceneDescriptors);
		printf("Scene: %d descriptors extracted in %d ms\n", sceneDescriptors.rows, time.restart());

		////////////////////////////
		// NEAREST NEIGHBOR MATCHING USING FLANN LIBRARY (included in OpenCV)
		////////////////////////////
		// Format descriptors for Flann
		cv::Mat objectData;
		cv::Mat sceneData;
		if(objectDescriptors.type()!=CV_32F) {
			objectDescriptors.convertTo(objectData, CV_32F); // make sure it's CV_32F
		}
		else {
			objectData = objectDescriptors;
		}
		if(sceneDescriptors.type()!=CV_32F) {
			sceneDescriptors.convertTo(sceneData, CV_32F); // make sure it's CV_32F
		}
		else {
			sceneData = sceneDescriptors;
		}

		// Create Flann index
		cv::flann::Index treeFlannIndex(sceneData, cv::flann::KDTreeIndexParams());
		printf("Time creating FLANN index = %d ms\n", time.restart());

		// search (nearest neighbor)
		int k=2; // find the 2 nearest neighbors
		cv::Mat results(objectData.rows, k, CV_32SC1); // Results index
		cv::Mat dists(objectData.rows, k, CV_32FC1); // Distance results are CV_32FC1
		treeFlannIndex.knnSearch(objectData, results, dists, k, cv::flann::SearchParams() ); // maximum number of leafs checked
		printf("Time nearest neighbor search = %d ms\n", time.restart());

		////////////////////////////
		// PROCESS NEAREST NEIGHBOR RESULTS
		////////////////////////////
		// Set gui data
		objWidget.setData(objectKeypoints, objectDescriptors, objectImg);
		sceneWidget.setData(sceneKeypoints, sceneDescriptors, sceneImg);

		// Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
		float nndrRatio = 0.6;
		std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
		std::vector<int> indexes_1, indexes_2; // Used for homography
		std::vector<uchar> outlier_mask;  // Used for homography
		for(int i=0; i<objectData.rows; ++i)
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
			QGraphicsRectItem * rectItem = new QGraphicsRectItem(objWidget.image().rect());
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
		if(objWidget.image().width() <= 800)
		{
			objWidget.setGeometry(0, 0, objWidget.image().width(), objWidget.image().height());
		}
		else
		{
			objWidget.setGeometry(0, 0, 800, 600);
			objWidget.setAutoScale(false);
		}
		objWidget.show();
		sceneWidget.setGraphicsViewMode(false);
		sceneWidget.setWindowTitle("Scene");
		if(sceneWidget.image().width() <= 800)
		{
			sceneWidget.setGeometry(0, 0, sceneWidget.image().width(), sceneWidget.image().height());
		}
		else
		{
			sceneWidget.setGeometry(0, 0, 800, 600);
			sceneWidget.setAutoScale(false);
		}
		sceneWidget.show();
		int r = app.exec();
		printf("Closing...\n");
		////////////////////////////
		//Cleanup
		////////////////////////////
		delete detector;
		delete extractor;

		if(objectImg) {
			cvReleaseImage(&objectImg);
		}
		if(sceneImg) {
			cvReleaseImage(&sceneImg);
		}

		return r;
	}
	else
	{
		if(objectImg) {
			cvReleaseImage(&objectImg);
		}
		if(sceneImg) {
			cvReleaseImage(&sceneImg);
		}

		printf("Images are not valid!\n");
		showUsage();
	}

	return 1;
}
