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

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <QTransform>

/**
 * IMPORTANT :
 *      Parameter General/MirrorView must be false
 *      Parameter Homography/homographyComputed must be true
 */
void objectsDetectedCallback(const std_msgs::Float32MultiArray & msg)
{
    printf("---\n");
    if(msg.data.size())
    {
		for(unsigned int i=0; i<msg.data.size(); i+=12)
		{
			// get data
			int id = (int)msg.data[i];
			float objectWidth = msg.data[i+1];
			float objectHeight = msg.data[i+2];

			// Find corners Qt
			QTransform qtHomography(msg.data[i+3], msg.data[i+4], msg.data[i+5],
									msg.data[i+6], msg.data[i+7], msg.data[i+8],
									msg.data[i+9], msg.data[i+10], msg.data[i+11]);

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

			// Example with OpenCV
			if(0)
			{
				// Find corners OpenCV
				cv::Mat cvHomography(3, 3, CV_32F);
				cvHomography.at<float>(0,0) = msg.data[i+3];
				cvHomography.at<float>(1,0) = msg.data[i+4];
				cvHomography.at<float>(2,0) = msg.data[i+5];
				cvHomography.at<float>(0,1) = msg.data[i+6];
				cvHomography.at<float>(1,1) = msg.data[i+7];
				cvHomography.at<float>(2,1) = msg.data[i+8];
				cvHomography.at<float>(0,2) = msg.data[i+9];
				cvHomography.at<float>(1,2) = msg.data[i+10];
				cvHomography.at<float>(2,2) = msg.data[i+11];
				std::vector<cv::Point2f> inPts, outPts;
				inPts.push_back(cv::Point2f(0,0));
				inPts.push_back(cv::Point2f(objectWidth,0));
				inPts.push_back(cv::Point2f(0,objectHeight));
				inPts.push_back(cv::Point2f(objectWidth,objectHeight));
				cv::perspectiveTransform(inPts, outPts, cvHomography);

				printf("Object %d detected, CV corners at (%f,%f) (%f,%f) (%f,%f) (%f,%f)\n",
						id,
						outPts.at(0).x, outPts.at(0).y,
						outPts.at(1).x, outPts.at(1).y,
						outPts.at(2).x, outPts.at(2).y,
						outPts.at(3).x, outPts.at(3).y);
			}
		}
    }
    else
    {
    	printf("No objects detected.\n");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "objects_detected");

    ros::NodeHandle nh;
    ros::Subscriber subs;
    subs = nh.subscribe("objects", 1, objectsDetectedCallback);

    ros::spin();

    return 0;
}
