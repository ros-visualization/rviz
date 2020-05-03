/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/float_property.h"

#include "rviz/default_plugin/tools/initial_pose_tool.h"

namespace rviz
{
InitialPoseTool::InitialPoseTool()
{
  shortcut_key_ = 'p';

  topic_property_ =
      new StringProperty("Topic", "initialpose", "The topic on which to publish initial pose estimates.",
                         getPropertyContainer(), SLOT(updateTopic()), this);
  std_dev_x_ = new FloatProperty("X std deviation", 0.5, "X standard deviation for initial pose [m]",
                                 getPropertyContainer());
  std_dev_y_ = new FloatProperty("Y std deviation", 0.5, "Y standard deviation for initial pose [m]",
                                 getPropertyContainer());
  std_dev_theta_ =
      new FloatProperty("Theta std deviation", M_PI / 12.0,
                        "Theta standard deviation for initial pose [rad]", getPropertyContainer());
  std_dev_x_->setMin(0);
  std_dev_y_->setMin(0);
  std_dev_theta_->setMin(0);
}

void InitialPoseTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("2D Pose Estimate");
  updateTopic();
}

void InitialPoseTool::updateTopic()
{
  try
  {
    pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(topic_property_->getStdString(), 1);
  }
  catch (const ros::Exception& e)
  {
    ROS_ERROR_STREAM_NAMED("InitialPoseTool", e.what());
  }
}

void InitialPoseTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::PoseWithCovarianceStamped pose;
  pose.header.frame_id = fixed_frame;
  pose.header.stamp = ros::Time::now();
  pose.pose.pose.position.x = x;
  pose.pose.pose.position.y = y;

  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
  pose.pose.covariance[6 * 0 + 0] = std::pow(std_dev_x_->getFloat(), 2);
  pose.pose.covariance[6 * 1 + 1] = std::pow(std_dev_y_->getFloat(), 2);
  pose.pose.covariance[6 * 5 + 5] = std::pow(std_dev_theta_->getFloat(), 2);
  ROS_INFO("Setting pose: %.3f %.3f %.3f [frame=%s]", x, y, theta, fixed_frame.c_str());
  pub_.publish(pose);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::InitialPoseTool, rviz::Tool)
