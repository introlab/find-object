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
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#endif

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "find_object/Camera.h"
#include <QtCore/QStringList>

#include <atomic>
#include <thread>

class CameraROS : public find_object::Camera {
	Q_OBJECT
public:
	CameraROS(bool subscribeDepth, rclcpp::Node * node);
	virtual ~CameraROS();
	void setupExecutor(std::shared_ptr<rclcpp::Node> node);

	// The images come from ROS callbacks, run by the executor in a thread of its own
	// (no polling): start() starts it, stop() and pause() stop it.
	virtual bool start();
	virtual void stop();
	virtual void pause();
	virtual bool isRunning() {return running_;}

	QStringList subscribedTopics() const;

Q_SIGNALS:
	// Emitted by handOver() after imageReceived(), see clearBusy().
	void imageHandedOver();

private Q_SLOTS:
	// In the Qt thread, after the slots of imageReceived(): the image is processed.
	void clearBusy();

private:
	// In the executor's thread: whether to process an image with this stamp. Not while
	// the previous one is still waiting for or in processing in the Qt thread (the image
	// is dropped), nor sooner than 1/Camera/4imageRate after the last one accepted,
	// according to their stamps (0 Hz: no limit).
	bool acceptImage(const builtin_interfaces::msg::Time & stamp);
	// In the executor's thread: emits imageReceived(), then imageHandedOver(). Their
	// slots are in the Qt thread, so both are queued, and run in the order they were
	// emitted: clearBusy() after the detection (synchronous in its slot), as long as
	// every slot of imageReceived() is in the Qt thread too.
	void handOver(const cv::Mat & image, const find_object::Header & header, const cv::Mat & depth, float depthConstant);

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

	std::thread spinThread_;
	std::atomic<bool> running_;
	std::atomic<bool> busy_; // an image handed over is not processed yet
	double lastStamp_; // of the last image accepted, used by the executor's thread only
};

#endif /* CAMERAROS_H_ */
