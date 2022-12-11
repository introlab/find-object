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

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <find_object_2d/msg/objects_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <QtCore/QString>

class TfExample : public rclcpp::Node
{
public:
	TfExample() :
		Node("tf_example_node"),
		objFramePrefix_("object")
	{
		tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		//auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
		//	this->get_node_base_interface(),
		//	this->get_node_timers_interface());
		//tfBuffer_->setCreateTimerInterface(timer_interface);
		tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

		targetFrameId_ = this->declare_parameter("target_frame_id", targetFrameId_);
		objFramePrefix_ = this->declare_parameter("object_prefix", objFramePrefix_);

		subs_ = create_subscription<find_object_2d::msg::ObjectsStamped>("objectsStamped", rclcpp::QoS(5).reliability((rmw_qos_reliability_policy_t)1), std::bind(&TfExample::objectsDetectedCallback, this, std::placeholders::_1));
	}

	// Here I synchronize with the ObjectsStamped topic to
	// know when the TF is ready and for which objects
	void objectsDetectedCallback(const find_object_2d::msg::ObjectsStamped::ConstSharedPtr msg)
	{
		if(msg->objects.data.size())
		{
			std::string targetFrameId = targetFrameId_;
			if(targetFrameId.empty())
			{
				targetFrameId = msg->header.frame_id;
			}
			char multiSubId = 'b';
			int previousId = -1;
			for(unsigned int i=0; i<msg->objects.data.size(); i+=12)
			{
				// get data
				int id = (int)msg->objects.data[i];

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

				// "object_1", "object_1_b", "object_1_c", "object_2"
				std::string objectFrameId = QString("%1_%2%3").arg(objFramePrefix_.c_str()).arg(id).arg(multiSuffix).toStdString();

				geometry_msgs::msg::TransformStamped pose;
				try
				{
					// Get transformation from "object_#" frame to target frame
					// The timestamp matches the one sent over TF
					pose = tfBuffer_->lookupTransform(targetFrameId, objectFrameId, tf2_ros::fromMsg(msg->header.stamp));
				}
				catch(tf2::TransformException & ex)
				{
					RCLCPP_WARN(this->get_logger(), "%s",ex.what());
					continue;
				}

				// Here "pose" is the position of the object "id" in target frame.
				RCLCPP_INFO(this->get_logger(), "%s [x,y,z] [x,y,z,w] in \"%s\" frame: [%f,%f,%f] [%f,%f,%f,%f]",
						objectFrameId.c_str(), targetFrameId.c_str(),
						pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z,
						pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w);
			}
		}
	}

private:
	std::string targetFrameId_;
	std::string objFramePrefix_;
	rclcpp::Subscription<find_object_2d::msg::ObjectsStamped>::SharedPtr subs_;
    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
	std::shared_ptr<tf2_ros::TransformListener> tfListener_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TfExample>());
	rclcpp::shutdown();
}
