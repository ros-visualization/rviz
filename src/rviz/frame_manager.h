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

#ifndef RVIZ_FRAME_MANAGER_H
#define RVIZ_FRAME_MANAGER_H

#include <map>

#include <ros/time.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>

#include <boost/thread/mutex.hpp>

#include <geometry_msgs/Pose.h>
#include <roslib/Header.h>

namespace tf
{
class TransformListener;
}

namespace rviz
{

class FrameManager
{
public:
  FrameManager(tf::TransformListener* tf);
  ~FrameManager();

  void setFixedFrame(const std::string& frame);
  bool getTransform(const roslib::Header& header, Ogre::Vector3& position, Ogre::Quaternion& orientation, bool relative_orientation)
  {
    return getTransform(header.frame_id, header.stamp, position, orientation, relative_orientation);
  }

  bool getTransform(const std::string& frame, ros::Time time, Ogre::Vector3& position, Ogre::Quaternion& orientation, bool relative_orientation);

  bool transform(const roslib::Header& header, const geometry_msgs::Pose& pose, Ogre::Vector3& position, Ogre::Quaternion& orientation, bool relative_orientation)
  {
    return transform(header.frame_id, header.stamp, pose, position, orientation, relative_orientation);
  }

  bool transform(const std::string& frame, ros::Time time, const geometry_msgs::Pose& pose, Ogre::Vector3& position, Ogre::Quaternion& orientation, bool relative_orientation);
  void update();

  bool frameHasProblems(const std::string& frame, ros::Time time, std::string& error);
  bool transformHasProblems(const std::string& frame, ros::Time time, std::string& error);

private:
  struct CacheKey
  {
    CacheKey(const std::string& f, ros::Time t, bool r)
    : frame(f)
    , time(t)
    , relative(r)
    {}

    bool operator<(const CacheKey& rhs) const
    {
      if (frame != rhs.frame)
      {
        return frame < rhs.frame;
      }

      if (time != rhs.time)
      {
        return time < rhs.time;
      }

      return relative < rhs.relative;
    }

    std::string frame;
    ros::Time time;
    bool relative;
  };
  struct CacheEntry
  {
    CacheEntry(const Ogre::Vector3& p, const Ogre::Quaternion& o)
    : position(p)
    , orientation(o)
    {}

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
  };
  typedef std::map<CacheKey, CacheEntry > M_Cache;

  boost::mutex cache_mutex_;
  M_Cache cache_;

  tf::TransformListener* tf_;
  std::string fixed_frame_;
};

}

#endif // RVIZ_FRAME_MANAGER_H
