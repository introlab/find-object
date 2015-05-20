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

#include <cmath>

using namespace find_object;

FindObjectROS::FindObjectROS(const std::string & objFramePrefix, QObject * parent) :
	FindObject(parent),
	objFramePrefix_("object")
{
	ros::NodeHandle pnh("~"); // public
	pnh.param("object_prefix", objFramePrefix_, objFramePrefix_);

	ros::NodeHandle nh; // public

	pub_ = nh.advertise<std_msgs::Float32MultiArray>("objects", 1);
	pubStamped_ = nh.advertise<find_object_2d::ObjectsStamped>("objectsStamped", 1);

	this->connect(this, SIGNAL(objectsFound(find_object::DetectionInfo)), this, SLOT(publish(find_object::DetectionInfo)));
}

void FindObjectROS::publish(const find_object::DetectionInfo & info)
{
	// send tf before the message
	if(info.objDetected_.size() && !depth_.empty() && depthConstant_ != 0.0f)
	{
		std::vector<tf::StampedTransform> transforms;
		QMultiMap<int, QSize>::const_iterator iterSizes=info.objDetectedSizes_.constBegin();
		for(QMultiMap<int, QTransform>::const_iterator iter=info.objDetected_.constBegin();
			iter!=info.objDetected_.constEnd();
			++iter, ++iterSizes)
		{
			// get data
			int id = iter.key();
			float objectWidth = iterSizes->width();
			float objectHeight = iterSizes->height();

			// Find center of the object
			QPointF center = iter->map(QPointF(objectWidth/2, objectHeight/2));
			QPointF xAxis = iter->map(QPointF(3*objectWidth/4, objectHeight/2));
			QPointF yAxis = iter->map(QPointF(objectWidth/2, 3*objectHeight/4));

			cv::Vec3f center3D = this->getDepth(depth_,
					center.x()+0.5f, center.y()+0.5f,
					float(depth_.cols/2)-0.5f, float(depth_.rows/2)-0.5f,
					1.0f/depthConstant_, 1.0f/depthConstant_);

			cv::Vec3f axisEndX = this->getDepth(depth_,
					xAxis.x()+0.5f, xAxis.y()+0.5f,
					float(depth_.cols/2)-0.5f, float(depth_.rows/2)-0.5f,
					1.0f/depthConstant_, 1.0f/depthConstant_);

			cv::Vec3f axisEndY = this->getDepth(depth_,
					yAxis.x()+0.5f, yAxis.y()+0.5f,
					float(depth_.cols/2)-0.5f, float(depth_.rows/2)-0.5f,
					1.0f/depthConstant_, 1.0f/depthConstant_);

			if(std::isfinite(center3D.val[0]) && std::isfinite(center3D.val[1]) && std::isfinite(center3D.val[2]) &&
				std::isfinite(axisEndX.val[0]) && std::isfinite(axisEndX.val[1]) && std::isfinite(axisEndX.val[2]) &&
				std::isfinite(axisEndY.val[0]) && std::isfinite(axisEndY.val[1]) && std::isfinite(axisEndY.val[2]))
			{
				tf::StampedTransform transform;
				transform.setIdentity();
				transform.child_frame_id_ = QString("%1_%2").arg(objFramePrefix_.c_str()).arg(id).toStdString();
				transform.frame_id_ = frameId_;
				transform.stamp_ = stamp_;
				transform.setOrigin(tf::Vector3(center3D.val[0], center3D.val[1], center3D.val[2]));

				//set rotation (y inverted)
				tf::Vector3 xAxis(axisEndX.val[0] - center3D.val[0], axisEndX.val[1] - center3D.val[1], axisEndX.val[2] - center3D.val[2]);
				xAxis.normalize();
				tf::Vector3 yAxis(axisEndY.val[0] - center3D.val[0], axisEndY.val[1] - center3D.val[1], axisEndY.val[2] - center3D.val[2]);
				yAxis.normalize();
				tf::Vector3 zAxis = xAxis*yAxis;
				tf::Matrix3x3 rotationMatrix(
							xAxis.x(), yAxis.x() ,zAxis.x(),
							xAxis.y(), yAxis.y(), zAxis.y(),
							xAxis.z(), yAxis.z(), zAxis.z());
				tf::Quaternion q;
				rotationMatrix.getRotation(q);
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

	if(pub_.getNumSubscribers() || pubStamped_.getNumSubscribers())
	{
		std_msgs::Float32MultiArray msg;
		find_object_2d::ObjectsStamped msgStamped;
		msg.data = std::vector<float>(info.objDetected_.size()*12);
		msgStamped.objects.data = std::vector<float>(info.objDetected_.size()*12);
		int i=0;
		QMultiMap<int, QSize>::const_iterator iterSizes=info.objDetectedSizes_.constBegin();
		for(QMultiMap<int, QTransform>::const_iterator iter=info.objDetected_.constBegin();
			iter!=info.objDetected_.constEnd();
			++iter, ++iterSizes)
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
		if(pub_.getNumSubscribers())
		{
			pub_.publish(msg);
		}
		if(pubStamped_.getNumSubscribers())
		{
			// use same header as the input image (for synchronization and frame reference)
			msgStamped.header.frame_id = frameId_;
			msgStamped.header.stamp = stamp_;
			pubStamped_.publish(msgStamped);
		}
	}
}

void FindObjectROS::setDepthData(const std::string & frameId,
		const ros::Time & stamp,
		const cv::Mat & depth,
		float depthConstant)
{
	frameId_ = frameId;
	stamp_ = stamp;
	depth_ = depth;
	depthConstant_ = depthConstant;
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
