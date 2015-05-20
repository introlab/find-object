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

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include "find_object/FindObject.h"

#include <QtCore/QObject>
#include <QtCore/QMultiMap>
#include <QtCore/QPair>
#include <QtCore/QRect>
#include <QtGui/QTransform>

class FindObjectROS : public find_object::FindObject
{
	Q_OBJECT;

public:
	FindObjectROS(const std::string & objPrefix, QObject * parent = 0);
	virtual ~FindObjectROS() {}

public Q_SLOTS:
	void publish(const find_object::DetectionInfo & info);

	void setDepthData(const std::string & frameId,
			const ros::Time & stamp,
			const cv::Mat & depth,
			float depthConstant);

private:
	cv::Vec3f getDepth(const cv::Mat & depthImage,
					   int x, int y,
					   float cx, float cy,
					   float fx, float fy);


private:
	ros::Publisher pub_;
	ros::Publisher pubStamped_;

	std::string frameId_;
	ros::Time stamp_;
	cv::Mat depth_;
	float depthConstant_;

	std::string objFramePrefix_;
	tf::TransformBroadcaster tfBroadcaster_;

};

#endif /* FINDOBJECTROS_H_ */
