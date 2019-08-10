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

#include "FindObjectROS.h"

#include <std_msgs/Float32MultiArray.h>
#include "find_object_2d/ObjectsStamped.h"
#include "find_object_2d/DetectionInfo.h"

#include <cmath>

using namespace find_object;

FindObjectROS::FindObjectROS(QObject * parent) :
	FindObject(true, parent),
	objFramePrefix_("object"),
	usePnP_(true)
{
	ros::NodeHandle pnh("~"); // public
	pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);
	pnh.param("pnp", usePnP_, usePnP_);
	ROS_INFO("object_prefix = %s", objFramePrefix_.c_str());
	ROS_INFO("pnp = %s", usePnP_?"true":"false");

	ros::NodeHandle nh; // public

	pub_ = nh.advertise<std_msgs::Float32MultiArray>("objects", 1);
	pubStamped_ = nh.advertise<find_object_2d::ObjectsStamped>("objectsStamped", 1);
	pubInfo_ = nh.advertise<find_object_2d::DetectionInfo>("info", 1);

	this->connect(this, SIGNAL(objectsFound(const find_object::DetectionInfo &, const QString &, double, const cv::Mat &, float)), this, SLOT(publish(const find_object::DetectionInfo &, const QString &, double, const cv::Mat &, float)));
}

void FindObjectROS::publish(const find_object::DetectionInfo & info, const QString & frameId, double stamp, const cv::Mat & depth, float depthConstant)
{
	// send tf before the message
	if(info.objDetected_.size() && !depth.empty() && depthConstant != 0.0f)
	{
		std::vector<tf::StampedTransform> transforms;
		char multiSubId = 'b';
		int previousId = -1;
		QMultiMap<int, QSize>::const_iterator iterSizes=info.objDetectedSizes_.constBegin();
		for(QMultiMap<int, QTransform>::const_iterator iter=info.objDetected_.constBegin();
			iter!=info.objDetected_.constEnd();
			++iter, ++iterSizes)
		{
			// get data
			int id = iter.key();
			float objectWidth = iterSizes->width();
			float objectHeight = iterSizes->height();

			QString multiSuffix;
			if(id == previousId)
			{
				multiSuffix = QString("_") + multiSubId++;
			}
			else
			{
				multiSubId = 'b';
			}
			previousId = id;

			// Find center of the object
			QPointF center = iter->map(QPointF(objectWidth/2, objectHeight/2));
			cv::Vec3f center3D = this->getDepth(depth,
					center.x()+0.5f, center.y()+0.5f,
					float(depth.cols/2)-0.5f, float(depth.rows/2)-0.5f,
					1.0f/depthConstant, 1.0f/depthConstant);

			cv::Vec3f axisEndX;
			cv::Vec3f axisEndY;
			if(!usePnP_)
			{
				QPointF xAxis = iter->map(QPointF(3*objectWidth/4, objectHeight/2));
				axisEndX = this->getDepth(depth,
						xAxis.x()+0.5f, xAxis.y()+0.5f,
						float(depth.cols/2)-0.5f, float(depth.rows/2)-0.5f,
						1.0f/depthConstant, 1.0f/depthConstant);

				QPointF yAxis = iter->map(QPointF(objectWidth/2, 3*objectHeight/4));
				axisEndY = this->getDepth(depth,
						yAxis.x()+0.5f, yAxis.y()+0.5f,
						float(depth.cols/2)-0.5f, float(depth.rows/2)-0.5f,
						1.0f/depthConstant, 1.0f/depthConstant);
			}

			if((std::isfinite(center3D.val[2]) && usePnP_) ||
				(std::isfinite(center3D.val[0]) && std::isfinite(center3D.val[1]) && std::isfinite(center3D.val[2]) &&
				 std::isfinite(axisEndX.val[0]) && std::isfinite(axisEndX.val[1]) && std::isfinite(axisEndX.val[2]) &&
				 std::isfinite(axisEndY.val[0]) && std::isfinite(axisEndY.val[1]) && std::isfinite(axisEndY.val[2])))
			{
				tf::StampedTransform transform;
				transform.setIdentity();
				transform.child_frame_id_ = QString("%1_%2%3").arg(objFramePrefix_.c_str()).arg(id).arg(multiSuffix).toStdString();
				transform.frame_id_ = frameId.toStdString();
				transform.stamp_.fromSec(stamp);

				tf::Quaternion q;
				if(usePnP_)
				{
					std::vector<cv::Point3f> objectPoints(4);
					std::vector<cv::Point2f> imagePoints(4);

					objectPoints[0] = cv::Point3f(-0.5, -(objectHeight/objectWidth)/2.0f,0);
					objectPoints[1] = cv::Point3f(0.5,-(objectHeight/objectWidth)/2.0f,0);
					objectPoints[2] = cv::Point3f(0.5,(objectHeight/objectWidth)/2.0f,0);
					objectPoints[3] = cv::Point3f(-0.5,(objectHeight/objectWidth)/2.0f,0);

					QPointF pt = iter->map(QPointF(0, 0));
					imagePoints[0] = cv::Point2f(pt.x(), pt.y());
					pt = iter->map(QPointF(objectWidth, 0));
					imagePoints[1] = cv::Point2f(pt.x(), pt.y());
					pt = iter->map(QPointF(objectWidth, objectHeight));
					imagePoints[2] = cv::Point2f(pt.x(), pt.y());
					pt = iter->map(QPointF(0, objectHeight));
					imagePoints[3] = cv::Point2f(pt.x(), pt.y());

					cv::Mat cameraMatrix = cv::Mat::eye(3,3,CV_64FC1);
					cameraMatrix.at<double>(0,0) = 1.0f/depthConstant;
					cameraMatrix.at<double>(1,1) = 1.0f/depthConstant;
					cameraMatrix.at<double>(0,2) = float(depth.cols/2)-0.5f;
					cameraMatrix.at<double>(1,2) = float(depth.rows/2)-0.5f;
					cv::Mat distCoeffs;

					cv::Mat rvec(1,3, CV_64FC1);
					cv::Mat tvec(1,3, CV_64FC1);
					cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);

					cv::Mat R;
					cv::Rodrigues(rvec, R);
					tf::Matrix3x3 rotationMatrix(
							R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
							R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
							R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2));
					rotationMatrix.getRotation(q);
					transform.setOrigin(tf::Vector3(
							tvec.at<double>(0)*(center3D.val[2]/tvec.at<double>(2)),
							tvec.at<double>(1)*(center3D.val[2]/tvec.at<double>(2)),
							tvec.at<double>(2)*(center3D.val[2]/tvec.at<double>(2))));
				}
				else
				{
					tf::Vector3 xAxis(axisEndX.val[0] - center3D.val[0], axisEndX.val[1] - center3D.val[1], axisEndX.val[2] - center3D.val[2]);
					xAxis.normalize();
					tf::Vector3 yAxis(axisEndY.val[0] - center3D.val[0], axisEndY.val[1] - center3D.val[1], axisEndY.val[2] - center3D.val[2]);
					yAxis.normalize();
					tf::Vector3 zAxis = xAxis.cross(yAxis);
					zAxis.normalize();
					tf::Matrix3x3 rotationMatrix(
								xAxis.x(), yAxis.x() ,zAxis.x(),
								xAxis.y(), yAxis.y(), zAxis.y(),
								xAxis.z(), yAxis.z(), zAxis.z());
					rotationMatrix.getRotation(q);
					transform.setOrigin(tf::Vector3(center3D.val[0], center3D.val[1], center3D.val[2]));
				}

				// set x axis going front of the object, with z up and z left
				q *= tf::createQuaternionFromRPY(CV_PI/2.0, CV_PI/2.0, 0);
				transform.setRotation(q.normalized());
				transforms.push_back(transform);
			}
			else
			{
				ROS_WARN("Object %d detected, center 2D at (%f,%f), but invalid depth, cannot set frame \"%s\"! "
						 "(maybe object is too near of the camera or bad depth image)\n",
						id,
						center.x(), center.y(),
						QString("%1_%2").arg(objFramePrefix_.c_str()).arg(id).toStdString().c_str());
			}
		}
		if(transforms.size())
		{
			tfBroadcaster_.sendTransform(transforms);
		}
	}

	if(pub_.getNumSubscribers() || pubStamped_.getNumSubscribers() || pubInfo_.getNumSubscribers())
	{
		std_msgs::Float32MultiArray msg;
		find_object_2d::ObjectsStamped msgStamped;
		find_object_2d::DetectionInfo infoMsg;
		if(pubInfo_.getNumSubscribers())
		{
			infoMsg.ids.resize(info.objDetected_.size());
			infoMsg.widths.resize(info.objDetected_.size());
			infoMsg.heights.resize(info.objDetected_.size());
			infoMsg.filePaths.resize(info.objDetected_.size());
			infoMsg.inliers.resize(info.objDetected_.size());
			infoMsg.outliers.resize(info.objDetected_.size());
			infoMsg.homographies.resize(info.objDetected_.size());
		}
		msg.data = std::vector<float>(info.objDetected_.size()*12);
		msgStamped.objects.data = std::vector<float>(info.objDetected_.size()*12);

		ROS_ASSERT(info.objDetected_.size() == info.objDetectedSizes_.size() &&
				   info.objDetected_.size() == info.objDetectedFilePaths_.size() &&
				   info.objDetected_.size() == info.objDetectedInliersCount_.size() &&
				   info.objDetected_.size() == info.objDetectedOutliersCount_.size());

		int infoIndex=0;
		int i=0;
		QMultiMap<int, QSize>::const_iterator iterSizes=info.objDetectedSizes_.constBegin();
		QMultiMap<int, QString>::const_iterator iterFilePaths=info.objDetectedFilePaths_.constBegin();
		QMultiMap<int, int>::const_iterator iterInliers=info.objDetectedInliersCount_.constBegin();
		QMultiMap<int, int>::const_iterator iterOutliers=info.objDetectedOutliersCount_.constBegin();
		for(QMultiMap<int, QTransform>::const_iterator iter=info.objDetected_.constBegin();
			iter!=info.objDetected_.constEnd();
			++iter, ++iterSizes, ++iterFilePaths, ++infoIndex, ++iterInliers, ++iterOutliers)
		{
			if(pub_.getNumSubscribers() || pubStamped_.getNumSubscribers())
			{
				msg.data[i] = msgStamped.objects.data[i] = iter.key(); ++i;
				msg.data[i] = msgStamped.objects.data[i] = iterSizes->width(); ++i;
				msg.data[i] = msgStamped.objects.data[i] = iterSizes->height(); ++i;
				msg.data[i] = msgStamped.objects.data[i] = iter->m11(); ++i;
				msg.data[i] = msgStamped.objects.data[i] = iter->m12(); ++i;
				msg.data[i] = msgStamped.objects.data[i] = iter->m13(); ++i;
				msg.data[i] = msgStamped.objects.data[i] = iter->m21(); ++i;
				msg.data[i] = msgStamped.objects.data[i] = iter->m22(); ++i;
				msg.data[i] = msgStamped.objects.data[i] = iter->m23(); ++i;
				msg.data[i] = msgStamped.objects.data[i] = iter->m31(); ++i;// dx
				msg.data[i] = msgStamped.objects.data[i] = iter->m32(); ++i;// dy
				msg.data[i] = msgStamped.objects.data[i] = iter->m33(); ++i;
			}

			if(pubInfo_.getNumSubscribers())
			{
				infoMsg.ids[infoIndex].data = iter.key();
				infoMsg.widths[infoIndex].data = iterSizes->width();
				infoMsg.heights[infoIndex].data = iterSizes->height();
				infoMsg.filePaths[infoIndex].data = iterFilePaths.value().toStdString();
				infoMsg.inliers[infoIndex].data = iterInliers.value();
				infoMsg.outliers[infoIndex].data = iterOutliers.value();
				infoMsg.homographies[infoIndex].data.resize(9);
				infoMsg.homographies[infoIndex].data[0] = iter->m11();
				infoMsg.homographies[infoIndex].data[1] = iter->m12();
				infoMsg.homographies[infoIndex].data[2] = iter->m13();
				infoMsg.homographies[infoIndex].data[3] = iter->m21();
				infoMsg.homographies[infoIndex].data[4] = iter->m22();
				infoMsg.homographies[infoIndex].data[5] = iter->m23();
				infoMsg.homographies[infoIndex].data[6] = iter->m31();
				infoMsg.homographies[infoIndex].data[7] = iter->m32();
				infoMsg.homographies[infoIndex].data[8] = iter->m33();
			}
		}
		if(pub_.getNumSubscribers())
		{
			pub_.publish(msg);
		}
		if(pubStamped_.getNumSubscribers())
		{
			// use same header as the input image (for synchronization and frame reference)
			msgStamped.header.frame_id = frameId.toStdString();
			msgStamped.header.stamp.fromSec(stamp);
			pubStamped_.publish(msgStamped);
		}
		if(pubInfo_.getNumSubscribers())
		{
			// use same header as the input image (for synchronization and frame reference)
			infoMsg.header.frame_id = frameId.toStdString();
			infoMsg.header.stamp.fromSec(stamp);
			pubInfo_.publish(infoMsg);
		}
	}
}

cv::Vec3f FindObjectROS::getDepth(const cv::Mat & depthImage,
				   int x, int y,
				   float cx, float cy,
				   float fx, float fy)
{
	if(!(x >=0 && x<depthImage.cols && y >=0 && y<depthImage.rows))
	{
		ROS_ERROR("Point must be inside the image (x=%d, y=%d), image size=(%d,%d)",
				x, y,
				depthImage.cols, depthImage.rows);
		return cv::Vec3f(
				std::numeric_limits<float>::quiet_NaN (),
				std::numeric_limits<float>::quiet_NaN (),
				std::numeric_limits<float>::quiet_NaN ());
	}


	cv::Vec3f pt;

	// Use correct principal point from calibration
	float center_x = cx; //cameraInfo.K.at(2)
	float center_y = cy; //cameraInfo.K.at(5)

	bool isInMM = depthImage.type() == CV_16UC1; // is in mm?

	// Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
	float unit_scaling = isInMM?0.001f:1.0f;
	float constant_x = unit_scaling / fx; //cameraInfo.K.at(0)
	float constant_y = unit_scaling / fy; //cameraInfo.K.at(4)
	float bad_point = std::numeric_limits<float>::quiet_NaN ();

	float depth;
	bool isValid;
	if(isInMM)
	{
		depth = (float)depthImage.at<uint16_t>(y,x);
		isValid = depth != 0.0f;
	}
	else
	{
		depth = depthImage.at<float>(y,x);
		isValid = std::isfinite(depth);
	}

	// Check for invalid measurements
	if (!isValid)
	{
		pt.val[0] = pt.val[1] = pt.val[2] = bad_point;
	}
	else
	{
		// Fill in XYZ
		pt.val[0] = (float(x) - center_x) * depth * constant_x;
		pt.val[1] = (float(y) - center_y) * depth * constant_y;
		pt.val[2] = depth*unit_scaling;
	}
	return pt;
}
