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

#ifndef FINDOBJECTROS_H_
#define FINDOBJECTROS_H_

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_broadcaster.h>
#include "find_object/FindObject.h"

#include <std_msgs/msg/float32_multi_array.hpp>
#include "find_object_2d/msg/objects_stamped.hpp"
#include "find_object_2d/msg/detection_info.hpp"

#include <QtCore/QObject>
#include <QtCore/QMultiMap>
#include <QtCore/QPair>
#include <QtCore/QRect>
#include <QtGui/QTransform>

class FindObjectROS : public find_object::FindObject
{
	Q_OBJECT;

public:
	FindObjectROS(rclcpp::Node * node);
	virtual ~FindObjectROS() {}

public Q_SLOTS:
	void publish(const find_object::DetectionInfo & info, const find_object::Header & header, const cv::Mat & depth, float depthConstant);

private:
	cv::Vec3f getDepth(const cv::Mat & depthImage,
					   int x, int y,
					   float cx, float cy,
					   float fx, float fy);

private:
	rclcpp::Node * node_;
	rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
	rclcpp::Publisher<find_object_2d::msg::ObjectsStamped>::SharedPtr pubStamped_;
	rclcpp::Publisher<find_object_2d::msg::DetectionInfo>::SharedPtr pubInfo_;

	std::string objFramePrefix_;
	bool usePnP_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;

};

#endif /* FINDOBJECTROS_H_ */
