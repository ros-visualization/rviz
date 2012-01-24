/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "initial_pose_tool.h"
#include "visualization_manager.h"
#include "properties/property.h"
#include "properties/property_manager.h"

#include "ogre_helpers/camera_base.h"
#include "ogre_helpers/arrow.h"
#include "ogre_helpers/qt_ogre_render_window.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <OGRE/OgreRay.h>
#include <OGRE/OgrePlane.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>

#include <tf/transform_listener.h>

namespace rviz
{

InitialPoseTool::InitialPoseTool( const std::string& name, char shortcut_key, VisualizationManager* manager )
: PoseTool( name, shortcut_key, manager )
{
  setTopic("initialpose");
}

InitialPoseTool::~InitialPoseTool()
{
}

void InitialPoseTool::setTopic(const std::string& topic)
{
  topic_ = topic;
  pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic, 1);
}

void InitialPoseTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = manager_->getFixedFrame();
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;

  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::quaternionTFToMsg(quat,
                        pose.pose.pose.orientation);
  pose.pose.covariance[6*0+0] = 0.5 * 0.5;
  pose.pose.covariance[6*1+1] = 0.5 * 0.5;
  pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  ROS_INFO("Setting pose: %.3f %.3f %.3f [frame=%s]", x, y, theta, fixed_frame.c_str());
  pub_.publish(pose);
}

void InitialPoseTool::enumerateProperties(PropertyManager* property_manager, const CategoryPropertyWPtr& parent)
{
  topic_property_ = property_manager->createProperty<StringProperty>("Topic", "Tool " + getName(), boost::bind(&InitialPoseTool::getTopic, this), boost::bind(&InitialPoseTool::setTopic, this, _1), parent, this);
}

}

