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
#include <find_object_2d/msg/objects_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#ifdef PRE_ROS_IRON
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif
#include <opencv2/opencv.hpp>
#include <QTransform>
#include <QColor>

class PrintObjects: public rclcpp::Node
{
public:
	PrintObjects() :
		Node("objects_detected")
	{
		image_transport::TransportHints hints(this);

		imagePub_ = image_transport::create_publisher(this, "image_with_objects", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)1).get_rmw_qos_profile());

		// Simple subscriber
		sub_ = create_subscription<std_msgs::msg::Float32MultiArray>("objects", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)1), std::bind(&PrintObjects::objectsDetectedCallback, this, std::placeholders::_1));

		// Synchronized image + objects example
		imageSub_.subscribe(this, "image", hints.getTransport(), rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)1).get_rmw_qos_profile());
		objectsSub_.subscribe(this, "objectsStamped", rclcpp::QoS(1).reliability((rmw_qos_reliability_policy_t)1).get_rmw_qos_profile());

		exactSync_ = new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(10), imageSub_, objectsSub_);
		exactSync_->registerCallback(std::bind(&PrintObjects::imageObjectsDetectedCallback, this, std::placeholders::_1, std::placeholders::_2));
	}

	virtual ~PrintObjects()
	{
		delete exactSync_;
	}

	/**
	 * IMPORTANT :
	 *      Parameter General/MirrorView must be false
	 *      Parameter Homography/homographyComputed must be true
	 */
	void objectsDetectedCallback(
			const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
	{
		printf("---\n");
		const std::vector<float> & data = msg->data;
		if(data.size())
		{
			for(unsigned int i=0; i<data.size(); i+=12)
			{
				// get data
				int id = (int)data[i];
				float objectWidth = data[i+1];
				float objectHeight = data[i+2];

				// Find corners Qt
				QTransform qtHomography(data[i+3], data[i+4], data[i+5],
										data[i+6], data[i+7], data[i+8],
										data[i+9], data[i+10], data[i+11]);

				QPointF qtTopLeft = qtHomography.map(QPointF(0,0));
				QPointF qtTopRight = qtHomography.map(QPointF(objectWidth,0));
				QPointF qtBottomLeft = qtHomography.map(QPointF(0,objectHeight));
				QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth,objectHeight));

				printf("Object %d detected, Qt corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
						id,
						qtTopLeft.x(), qtTopLeft.y(),
						qtTopRight.x(), qtTopRight.y(),
						qtBottomLeft.x(), qtBottomLeft.y(),
						qtBottomRight.x(), qtBottomRight.y());
			}
		}
		else
		{
			printf("No objects detected.\n");
		}
	}
	void imageObjectsDetectedCallback(
			const sensor_msgs::msg::Image::ConstSharedPtr imageMsg,
			const find_object_2d::msg::ObjectsStamped::ConstSharedPtr objectsMsg)
	{
		if(imagePub_.getNumSubscribers() > 0)
		{
			const std::vector<float> & data = objectsMsg->objects.data;
			if(data.size())
			{
				for(unsigned int i=0; i<data.size(); i+=12)
				{
					// get data
					int id = (int)data[i];
					float objectWidth = data[i+1];
					float objectHeight = data[i+2];

					// Find corners OpenCV
					cv::Mat cvHomography(3, 3, CV_32F);
					cvHomography.at<float>(0,0) = data[i+3];
					cvHomography.at<float>(1,0) = data[i+4];
					cvHomography.at<float>(2,0) = data[i+5];
					cvHomography.at<float>(0,1) = data[i+6];
					cvHomography.at<float>(1,1) = data[i+7];
					cvHomography.at<float>(2,1) = data[i+8];
					cvHomography.at<float>(0,2) = data[i+9];
					cvHomography.at<float>(1,2) = data[i+10];
					cvHomography.at<float>(2,2) = data[i+11];
					std::vector<cv::Point2f> inPts, outPts;
					inPts.push_back(cv::Point2f(0,0));
					inPts.push_back(cv::Point2f(objectWidth,0));
					inPts.push_back(cv::Point2f(objectWidth,objectHeight));
					inPts.push_back(cv::Point2f(0,objectHeight));
					inPts.push_back(cv::Point2f(objectWidth/2,objectHeight/2));
					cv::perspectiveTransform(inPts, outPts, cvHomography);

					cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(imageMsg);

					cv_bridge::CvImage img;
					img = *imageDepthPtr;
					std::vector<cv::Point2i> outPtsInt;
					outPtsInt.push_back(outPts[0]);
					outPtsInt.push_back(outPts[1]);
					outPtsInt.push_back(outPts[2]);
					outPtsInt.push_back(outPts[3]);
					QColor color(QColor((Qt::GlobalColor)((id % 10 + 7)==Qt::yellow?Qt::darkYellow:(id % 10 + 7))));
					cv::Scalar cvColor(color.red(), color.green(), color.blue());
					cv::polylines(img.image, outPtsInt, true, cvColor, 3);
					cv::Point2i center = outPts[4];
					cv::putText(img.image, QString("(%1, %2)").arg(center.x).arg(center.y).toStdString(), center, cv::FONT_HERSHEY_SIMPLEX, 0.6, cvColor, 2);
					cv::circle(img.image, center, 1, cvColor, 3);
					imagePub_.publish(img.toImageMsg());
				}
			}
		}
	}

private:
	typedef message_filters::sync_policies::ExactTime<
			sensor_msgs::msg::Image,
			find_object_2d::msg::ObjectsStamped> MyExactSyncPolicy;
	message_filters::Synchronizer<MyExactSyncPolicy> * exactSync_;
	image_transport::Publisher imagePub_;
	image_transport::SubscriberFilter imageSub_;
	message_filters::Subscriber<find_object_2d::msg::ObjectsStamped> objectsSub_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;

};


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PrintObjects>());
	rclcpp::shutdown();

    return 0;
}
