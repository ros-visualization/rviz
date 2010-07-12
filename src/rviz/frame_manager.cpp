/*
 * Copyright (c) 2009, Willow Garage, Inc.
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

#include "frame_manager.h"
#include "common.h"
#include "display.h"
#include "properties/property.h"

#include <tf/transform_listener.h>
#include <ros/ros.h>

namespace rviz
{

FrameManagerPtr FrameManager::instance()
{
  static FrameManagerWPtr instw;

  FrameManagerPtr inst = instw.lock();
  if (!inst)
  {
    inst.reset(new FrameManager);
    instw = inst;
  }

  return inst;
}

FrameManager::FrameManager()
{
  tf_ = new tf::TransformListener(ros::NodeHandle(), ros::Duration(60), false);
}

FrameManager::~FrameManager()
{
  delete tf_;
}

void FrameManager::update()
{
  boost::mutex::scoped_lock lock(cache_mutex_);
  cache_.clear();
}

void FrameManager::setFixedFrame(const std::string& frame)
{
  boost::mutex::scoped_lock lock(cache_mutex_);
  fixed_frame_ = frame;
  cache_.clear();
}

bool FrameManager::getTransform(const std::string& frame, ros::Time time, Ogre::Vector3& position, Ogre::Quaternion& orientation, bool relative_orientation)
{
  boost::mutex::scoped_lock lock(cache_mutex_);

  position = Ogre::Vector3(9999999, 9999999, 9999999);
  orientation = Ogre::Quaternion::IDENTITY;

  if (fixed_frame_.empty())
  {
    return false;
  }

  M_Cache::iterator it = cache_.find(CacheKey(frame, time, relative_orientation));
  if (it != cache_.end())
  {
    position = it->second.position;
    orientation = it->second.orientation;
    return true;
  }

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0f;

  if (!transform(frame, time, pose, position, orientation, relative_orientation))
  {
    return false;
  }

  cache_.insert(std::make_pair(CacheKey(frame, time, relative_orientation), CacheEntry(position, orientation)));

  return true;
}

bool FrameManager::transform(const std::string& frame, ros::Time time, const geometry_msgs::Pose& pose_msg, Ogre::Vector3& position, Ogre::Quaternion& orientation, bool relative_orientation)
{
  position = Ogre::Vector3::ZERO;
  orientation = Ogre::Quaternion::IDENTITY;

  btQuaternion btorient(pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z, pose_msg.orientation.w);
  if (btorient.x() == 0.0 && btorient.y() == 0.0 && btorient.z() == 0.0 && btorient.w() == 0.0)
  {
    btorient.setW(1.0);
  }

  tf::Stamped<tf::Pose> pose(btTransform(btorient,
                                   btVector3(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z)),
                                   time, frame);
  try
  {
    tf_->transformPose( fixed_frame_, pose, pose );
  }
  catch(tf::TransformException& e)
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s", frame.c_str(), fixed_frame_.c_str(), e.what());
    return false;
  }

  position = Ogre::Vector3(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
  robotToOgre(position);

  btQuaternion quat;
  pose.getBasis().getRotation( quat );
  orientation = Ogre::Quaternion::IDENTITY;

  if (relative_orientation)
  {
    ogreToRobot(orientation);
  }

  orientation = Ogre::Quaternion( quat.w(), quat.x(), quat.y(), quat.z() ) * orientation;
  robotToOgre(orientation);

  return true;
}

bool FrameManager::frameHasProblems(const std::string& frame, ros::Time time, std::string& error)
{
  if (!tf_->frameExists(frame))
  {
    error = "Frame [" + frame + "] does not exist";
    if (frame == fixed_frame_)
    {
      error = "Fixed " + error;
    }
    return true;
  }

  return false;
}

bool FrameManager::transformHasProblems(const std::string& frame, ros::Time time, std::string& error)
{
  std::string tf_error;
  bool transform_succeeded = tf_->canTransform(fixed_frame_, frame, time, &tf_error);
  if (transform_succeeded)
  {
    return false;
  }

  bool ok = true;
  ok = ok && !frameHasProblems(fixed_frame_, time, error);
  ok = ok && !frameHasProblems(frame, time, error);

  if (ok)
  {
    std::stringstream ss;
    ss << "No transform to fixed frame [" << fixed_frame_ << "].  TF error: [" << tf_error << "]";
    error = ss.str();
    ok = false;
  }

  {
    std::stringstream ss;
    ss << "For frame [" << frame << "]: " << error;
    error = ss.str();
  }

  return !ok;
}

std::string getTransformStatusName(const std::string& caller_id)
{
  std::stringstream ss;
  ss << "Transform [sender=" << caller_id << "]";
  return ss.str();
}

std::string FrameManager::discoverFailureReason(const roslib::Header& header, const std::string& caller_id, tf::FilterFailureReason reason)
{
  if (reason == tf::filter_failure_reasons::OutTheBack)
  {
    std::stringstream ss;
    ss << "Message removed because it is too old (frame=[" << header.frame_id << "], stamp=[" << header.stamp << "])";
    return ss.str();
  }
  else
  {
    std::string error;
    if (transformHasProblems(header.frame_id, header.stamp, error))
    {
      return error;
    }
  }

  return "Unknown reason for transform failure";
}

void FrameManager::messageArrived(const roslib::Header& header, const std::string& caller_id, Display* display)
{
  display->setStatus(status_levels::Ok, getTransformStatusName(caller_id), "Transform OK");
}

void FrameManager::messageFailed(const roslib::Header& header, const std::string& caller_id, tf::FilterFailureReason reason, Display* display)
{
  std::string status_name = getTransformStatusName(caller_id);
  std::string status_text = discoverFailureReason(header, caller_id, reason);
  display->setStatus(status_levels::Error, status_name, status_text);
}

}
