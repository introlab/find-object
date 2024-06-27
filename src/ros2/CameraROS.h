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

#ifndef CAMERAROS_H_
#define CAMERAROS_H_

#include <rclcpp/rclcpp.hpp>
#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "find_object/Camera.h"
#include <QtCore/QStringList>

class CameraROS : public find_object::Camera {
	Q_OBJECT
public:
	CameraROS(bool subscribeDepth, rclcpp::Node * node);
	virtual ~CameraROS();
	void setupExecutor(std::shared_ptr<rclcpp::Node> node);

	virtual bool start();
	virtual void stop();

	QStringList subscribedTopics() const;

private Q_SLOTS:
	virtual void takeImage();

private:
	void imgReceivedCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
	void imgDepthReceivedCallback(
			const sensor_msgs::msg::Image::ConstSharedPtr rgbMsg,
			const sensor_msgs::msg::Image::ConstSharedPtr depthMsg,
			const sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfoMsg);

private:
	rclcpp::Node * node_;
	rclcpp::executors::SingleThreadedExecutor executor_;
	bool subscribeDepth_;
	image_transport::Subscriber imageSub_;

	image_transport::SubscriberFilter rgbSub_;
	image_transport::SubscriberFilter depthSub_;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> cameraInfoSub_;

	typedef message_filters::sync_policies::ApproximateTime<
			sensor_msgs::msg::Image,
			sensor_msgs::msg::Image,
			sensor_msgs::msg::CameraInfo> MyApproxSyncPolicy;
	message_filters::Synchronizer<MyApproxSyncPolicy> * approxSync_;

	typedef message_filters::sync_policies::ExactTime<
			sensor_msgs::msg::Image,
			sensor_msgs::msg::Image,
			sensor_msgs::msg::CameraInfo> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
};

#endif /* CAMERAROS_H_ */
