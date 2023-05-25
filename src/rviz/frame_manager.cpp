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
#include "display.h"
#include "properties/property.h"

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Float32.h>

namespace rviz
{
FrameManager::FrameManager(std::shared_ptr<tf2_ros::Buffer> tf_buffer,
                           std::shared_ptr<tf2_ros::TransformListener> tf_listener)
{
  assert(!tf_listener || tf_buffer); // tf_listener implies tf_buffer to defined too
  tf_buffer_ =
      tf_buffer ? std::move(tf_buffer) : std::make_shared<tf2_ros::Buffer>(ros::Duration(10 * 60));
  tf_listener_ = tf_listener ?
                     std::move(tf_listener) :
                     std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, ros::NodeHandle(), true);

  setSyncMode(SyncOff);
  setPause(false);
}

FrameManager::~FrameManager()
{
}

void FrameManager::update()
{
  if (pause_)
    return;
  else
  {
    boost::mutex::scoped_lock lock(cache_mutex_);
    cache_.clear();
    switch (sync_mode_)
    {
    case SyncOff: // always use latest time
      sync_time_ = ros::Time::now();
      break;
    case SyncExact: // sync to external source
      // sync_time_ set via syncTime()
      break;
    case SyncApprox:
      // sync_delta is a sliding average of current_delta_, i.e.
      // approximating the average delay of incoming sync messages w.r.t. current time
      sync_delta_ = 0.7 * sync_delta_ + 0.3 * current_delta_;
      // date back sync_time_ to ensure finding TFs that are as old as now() - sync_delta_
      sync_time_ = ros::Time::now() - ros::Duration(sync_delta_);
      break;
    case SyncFrame: // sync to current time
      // date back sync_time_ to ensure finding TFs that are as old as now() - sync_delta_
      sync_time_ = ros::Time::now() - ros::Duration(sync_delta_);
      break;
    }
  }
}

void FrameManager::setFixedFrame(const std::string& frame)
{
  bool should_emit = false;
  {
    boost::mutex::scoped_lock lock(cache_mutex_);
    if (fixed_frame_ != frame)
    {
      fixed_frame_ = frame;
      cache_.clear();
      should_emit = true;
    }
  }
  if (should_emit)
  {
    // This emission must be kept outside of the mutex lock to avoid deadlocks.
    Q_EMIT fixedFrameChanged();
  }
}

void FrameManager::setPause(bool pause)
{
  pause_ = pause;
}

void FrameManager::setSyncMode(SyncMode mode)
{
  sync_mode_ = mode;
  sync_time_ = ros::Time(0);
  sync_delta_ = 0;
  current_delta_ = 0;
}

void FrameManager::syncTime(ros::Time time)
{
  switch (sync_mode_)
  {
  case SyncOff:
  case SyncFrame:
    break;
  case SyncExact:
    sync_time_ = time;
    break;
  case SyncApprox:
    if (time == ros::Time(0))
    {
      current_delta_ = 0;
      return;
    }
    if (ros::Time::now() >= time) // avoid exception due to negative time
    {
      // estimate delay of sync message w.r.t. current time
      current_delta_ = (ros::Time::now() - time).toSec();
    }
    else
    {
      setSyncMode(SyncApprox);
    }
    break;
  }
}

void FrameManager::adjustTime(ros::Time& time)
{
  // we only need to act if we get a zero timestamp, which means "latest"
  if (time != ros::Time())
    return;

  switch (sync_mode_)
  {
  case SyncOff:
    break;
  case SyncFrame:
  case SyncExact:
  case SyncApprox:
    time = sync_time_;
    break;
  }
}


bool FrameManager::getTransform(const std::string& frame,
                                ros::Time time,
                                Ogre::Vector3& position,
                                Ogre::Quaternion& orientation)
{
  adjustTime(time);

  boost::mutex::scoped_lock lock(cache_mutex_);

  position = Ogre::Vector3(9999999, 9999999, 9999999);
  orientation = Ogre::Quaternion::IDENTITY;

  if (fixed_frame_.empty())
  {
    return false;
  }

  M_Cache::iterator it = cache_.find(CacheKey(frame, time));
  if (it != cache_.end())
  {
    position = it->second.position;
    orientation = it->second.orientation;
    return true;
  }

  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0f;

  if (!transform(frame, time, pose, position, orientation))
  {
    return false;
  }

  cache_.insert(std::make_pair(CacheKey(frame, time), CacheEntry(position, orientation)));

  return true;
}

bool FrameManager::transform(const std::string& frame,
                             ros::Time time,
                             const geometry_msgs::Pose& pose_msg,
                             Ogre::Vector3& position,
                             Ogre::Quaternion& orientation)
{
  adjustTime(time);

  position = Ogre::Vector3::ZERO;
  orientation = Ogre::Quaternion::IDENTITY;

  geometry_msgs::Pose pose = pose_msg;
  if (pose.orientation.x == 0.0 && pose.orientation.y == 0.0 && pose.orientation.z == 0.0 &&
      pose.orientation.w == 0.0)
    pose.orientation.w = 1.0;

  // convert pose into fixed_frame_
  try
  {
    tf2::doTransform(pose, pose, tf_buffer_->lookupTransform(fixed_frame_, frame, time));
  }
  catch (const tf2::ExtrapolationException& e)
  {
    if (sync_mode_ == SyncApprox)
      return false;
    // We don't have tf info for sync_time_. Reset sync_time_ to latest available time of frame
    auto tf = tf_buffer_->lookupTransform(frame, frame, ros::Time());
    if (sync_time_ > tf.header.stamp && tf.header.stamp != ros::Time())
    {
      sync_delta_ += (sync_time_ - tf.header.stamp).toSec(); // increase sync delta by observed amount
      sync_time_ = tf.header.stamp;
    }
    return false;
  }
  catch (const std::runtime_error& e)
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s': %s", frame.c_str(),
              fixed_frame_.c_str(), e.what());
    return false;
  }

  position = Ogre::Vector3(pose.position.x, pose.position.y, pose.position.z);
  orientation =
      Ogre::Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

  return true;
}

bool FrameManager::frameHasProblems(const std::string& frame, ros::Time /*time*/, std::string& error)
{
  if (!tf_buffer_->_frameExists(frame))
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
  adjustTime(time);

  std::string tf_error;
  bool transform_succeeded = tf_buffer_->canTransform(fixed_frame_, frame, time, &tf_error);
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

std::string FrameManager::discoverFailureReason(const std::string& frame_id,
                                                const ros::Time& stamp,
                                                const std::string& /*caller_id*/,
                                                tf2_ros::FilterFailureReason reason)
{
  if (reason == tf2_ros::filter_failure_reasons::OutTheBack)
  {
    std::stringstream ss;
    ss << "Message removed because it is too old (frame=[" << frame_id << "], stamp=[" << stamp << "])";
    return ss.str();
  }
  else
  {
    std::string error;
    if (transformHasProblems(frame_id, stamp, error))
    {
      return error;
    }
  }

  return "Unknown reason for transform failure (frame=[" + frame_id + "])";
}

void FrameManager::messageArrived(const std::string& /*frame_id*/,
                                  const ros::Time& /*stamp*/,
                                  const std::string& caller_id,
                                  Display* display)
{
  display->setStatusStd(StatusProperty::Ok, getTransformStatusName(caller_id), "Transform OK");
}

void FrameManager::messageFailedImpl(const std::string& caller_id,
                                     const std::string& status_text,
                                     Display* display)
{
  std::string status_name = getTransformStatusName(caller_id);

  display->setStatusStd(StatusProperty::Error, status_name, status_text);
}

} // namespace rviz
